// Copyright 2026 Maik Knof
// Maintainer: Maik Knof <maik.knof@gmx.de>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_snapshot_interfaces/msg/snapshot_status.hpp>
#include "sensor_snapshot/image_snapshot.hpp"
#include "sensor_snapshot/point_cloud_snapshot.hpp"

namespace
{
using namespace std::chrono_literals;
using GetImage = sensor_snapshot_interfaces::srv::GetImage;
using GetCloud = sensor_snapshot_interfaces::srv::GetPointCloud;
using Filter = sensor_snapshot_interfaces::msg::FieldFilter;
using Status = sensor_snapshot_interfaces::msg::SnapshotStatus;

class Components : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    static std::atomic<unsigned> counter{0};
    node_namespace_ = "/snapshot_test_" + std::to_string(++counter);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    options.append_parameter_override("cache.history_size", 3);
    options.arguments({"--ros-args", "-r", "__ns:=" + node_namespace_});
    helper_ = std::make_shared<rclcpp::Node>("test_client", options);
    image_ = std::make_shared<sensor_snapshot::ImageSnapshot>(options);
    cloud_ = std::make_shared<sensor_snapshot::PointCloudSnapshot>(options);
    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 4);
    executor_->add_node(helper_);
    executor_->add_node(image_);
    executor_->add_node(cloud_);
    thread_ = std::thread([this]() {executor_->spin();});
  }

  void TearDown() override
  {
    if (executor_) {
      executor_->cancel();
    }
    if (thread_.joinable()) {
      thread_.join();
    }
    executor_.reset();
    additional_nodes_.clear();
    helper_.reset();
    image_.reset();
    cloud_.reset();
  }

  template<typename ServiceT>
  typename ServiceT::Response::SharedPtr call(
    const std::string & service,
    const typename ServiceT::Request & request = typename ServiceT::Request())
  {
    auto client = helper_->create_client<ServiceT>(service);
    if (!client->wait_for_service(5s)) {
      throw std::runtime_error("Service discovery timed out: " + service);
    }
    auto future = client->async_send_request(std::make_shared<typename ServiceT::Request>(request));
    // This runs on the test thread; the executor remains free to deliver the reply.
    if (future.wait_for(5s) != std::future_status::ready) {
      throw std::runtime_error("Service response timed out: " + service);
    }
    return future.get();
  }

  template<typename MessageT>
  typename rclcpp::Publisher<MessageT>::SharedPtr publisher(const std::string & topic)
  {
    auto result = helper_->create_publisher<MessageT>(topic, rclcpp::SensorDataQoS().keep_last(1));
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (result->get_intra_process_subscription_count() == 0 &&
      std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(5ms);
    }
    if (result->get_intra_process_subscription_count() == 0) {
      throw std::runtime_error("Intra-process subscription did not connect");
    }
    return result;
  }

  template<typename ServiceT>
  typename ServiceT::Response::SharedPtr wait_for_sample(
    const std::string & service,
    uint64_t sequence = 1)
  {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < deadline) {
      auto result = call<ServiceT>(service);
      if (result->sequence >= sequence) {
        return result;
      }
      std::this_thread::sleep_for(5ms);
    }
    throw std::runtime_error("Sensor callback did not populate the cache");
  }

  std::string node_namespace_;
  std::shared_ptr<rclcpp::Node> helper_;
  std::shared_ptr<sensor_snapshot::ImageSnapshot> image_;
  std::shared_ptr<sensor_snapshot::ImageSnapshot> remapped_image_;
  std::shared_ptr<sensor_snapshot::PointCloudSnapshot> cloud_;
  std::vector<std::shared_ptr<rclcpp::Node>> additional_nodes_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread thread_;
};

TEST_F(Components, NamesFollowTopicsWhenLoadingMultipleInstances)
{
  EXPECT_STREQ(image_->get_name(), "snapshot_image");
  EXPECT_STREQ(cloud_->get_name(), "snapshot_points");
  for (const auto & prefix : {"front", "rear"}) {
    const auto topic = std::string(prefix) + "/points";
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    options.arguments({"--ros-args", "-r", "__ns:=" + node_namespace_});
    options.append_parameter_override("input.topic", topic);
    auto instance = std::make_shared<sensor_snapshot::PointCloudSnapshot>(options);
    EXPECT_EQ(std::string(instance->get_name()), "snapshot_" + std::string(prefix) + "_points");
    executor_->add_node(instance);
    additional_nodes_.push_back(instance);
    EXPECT_EQ(call<GetCloud>(topic)->status.code, Status::NO_DATA);
  }
}

TEST_F(Components, ReportsNoDataAndRejectsInvalidAge)
{
  EXPECT_EQ(call<GetImage>("image")->status.code, Status::NO_DATA);
  EXPECT_EQ(call<GetCloud>("points")->status.code, Status::NO_DATA);
  GetImage::Request request;
  request.max_age_seconds = -1;
  EXPECT_EQ(call<GetImage>("image", request)->status.code,
      Status::INVALID_REQUEST);
  request.max_age_seconds = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(call<GetImage>("image", request)->status.code,
      Status::INVALID_REQUEST);
}

TEST_F(Components, ImageServiceCropsPreservesHeaderAndRejectsStaleData)
{
  auto pub = publisher<sensor_msgs::msg::Image>("image");
  auto message = std::make_unique<sensor_msgs::msg::Image>();
  message->header.frame_id = "camera";
  message->header.stamp.sec = 123;
  message->width = 3;
  message->height = 2;
  message->encoding = "mono8";
  message->step = 3;
  message->data = {1, 2, 3, 4, 5, 6};
  pub->publish(std::move(message));
  const auto first = wait_for_sample<GetImage>("image");
  ASSERT_EQ(first->status.code, Status::OK);
  EXPECT_EQ(first->sequence, 1u);

  GetImage::Request request;
  request.region.x_offset = 1;
  request.region.width = 2;
  const auto cropped = call<GetImage>("image", request);
  ASSERT_EQ(cropped->status.code, Status::OK);
  EXPECT_EQ(cropped->image.data, (std::vector<uint8_t>{2, 3, 5, 6}));
  EXPECT_EQ(cropped->image.header, first->image.header);
  EXPECT_EQ(cropped->source_width, 3u);
  EXPECT_EQ(cropped->source_height, 2u);
  EXPECT_EQ(cropped->region.height, 2u);
  EXPECT_EQ(cropped->sequence, first->sequence);

  request.region.width = 100;
  const auto invalid = call<GetImage>("image", request);
  EXPECT_EQ(invalid->status.code, Status::INVALID_REQUEST);
  EXPECT_TRUE(invalid->image.data.empty());
  request = GetImage::Request();
  request.max_age_seconds = 1e-12;
  const auto stale = call<GetImage>("image", request);
  EXPECT_EQ(stale->status.code, Status::STALE);
  EXPECT_TRUE(stale->image.data.empty());
  EXPECT_GT(stale->age_seconds, 0.0);

  auto second = std::make_unique<sensor_msgs::msg::Image>(first->image);
  second->data[0] = 42;
  pub->publish(std::move(second));
  const auto updated = wait_for_sample<GetImage>("image", 2);
  ASSERT_EQ(updated->status.code, Status::OK);
  EXPECT_EQ(updated->image.data[0], 42u);
}

TEST_F(Components, HistorySelectsTimestampAndEvictsOldFrames)
{
  auto pub = publisher<sensor_msgs::msg::Image>("image");
  for (uint64_t sequence = 1; sequence <= 4; ++sequence) {
    auto message = std::make_unique<sensor_msgs::msg::Image>();
    message->header.stamp.sec = sequence * 10;
    message->width = 2;
    message->height = 2;
    message->step = 2;
    message->encoding = "mono8";
    message->data = {static_cast<uint8_t>(sequence * 10), 2, 3, 4};
    pub->publish(std::move(message));
    ASSERT_EQ(wait_for_sample<GetImage>("image", sequence)->status.code, Status::OK);
    if (sequence == 3) {
      GetImage::Request exact;
      exact.stamp.sec = 10;
      EXPECT_EQ(call<GetImage>("image", exact)->sequence, 1u);
    }
  }
  GetImage::Request request;
  request.stamp.sec = 25;
  request.downsample_factor = 2;
  const auto selected = call<GetImage>("image", request);
  ASSERT_EQ(selected->status.code, Status::OK);
  EXPECT_EQ(selected->sequence, 3u);
  EXPECT_EQ(selected->image.header.stamp.sec, 30);
  EXPECT_EQ(selected->image.data, (std::vector<uint8_t>{30}));
  request.max_age_seconds = 1e-12;
  const auto stale = call<GetImage>("image", request);
  EXPECT_EQ(stale->status.code, Status::STALE);
  EXPECT_EQ(stale->sequence, 3u);
  EXPECT_TRUE(stale->image.data.empty());
  request.max_age_seconds = 0.0;
  request.stamp.sec = 10;
  const auto evicted = call<GetImage>("image", request);
  EXPECT_EQ(evicted->status.code, Status::NOT_FOUND);
  EXPECT_TRUE(evicted->image.data.empty());
  request.stamp.sec = 50;
  EXPECT_EQ(call<GetImage>("image", request)->status.code, Status::NOT_FOUND);
  request.stamp.nanosec = 1000000000;
  EXPECT_EQ(call<GetImage>("image", request)->status.code, Status::INVALID_REQUEST);
  EXPECT_EQ(call<GetImage>("image")->sequence, 4u);
}

TEST_F(Components, ServiceNameFollowsNamespacedAndRemappedInputTopic)
{
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  options.arguments({
      "--ros-args", "-r", "__ns:=" + node_namespace_, "-r", "__node:=remapped_snapshot",
      "-r", "image:=front/image"});
  remapped_image_ = std::make_shared<sensor_snapshot::ImageSnapshot>(options);
  executor_->add_node(remapped_image_);
  const auto service_name = node_namespace_ + "/front/image";
  EXPECT_EQ(call<GetImage>(service_name)->status.code, Status::NO_DATA);

  auto pub = publisher<sensor_msgs::msg::Image>("front/image");
  auto message = std::make_unique<sensor_msgs::msg::Image>();
  message->width = 1;
  message->height = 1;
  message->encoding = "mono8";
  message->step = 1;
  message->data = {9};
  pub->publish(std::move(message));
  const auto response = wait_for_sample<GetImage>(service_name);
  EXPECT_EQ(response->status.code, Status::OK);
  EXPECT_EQ(response->image.data, (std::vector<uint8_t>{9}));
}

TEST_F(Components, CloudServiceCombinesFiltersAndSurvivesInvalidRequests)
{
  auto pub = publisher<sensor_msgs::msg::PointCloud2>("points");
  auto message = std::make_unique<sensor_msgs::msg::PointCloud2>();
  message->header.frame_id = "lidar";
  message->width = 2;
  message->height = 2;
  message->point_step = 6;
  message->row_step = 14;
  const uint16_t probe = 1;
  message->is_bigendian = *reinterpret_cast<const uint8_t *>(&probe) == 0;
  sensor_msgs::msg::PointField z;
  z.name = "z";
  z.datatype = sensor_msgs::msg::PointField::FLOAT32;
  z.count = 1;
  auto intensity = z;
  intensity.name = "intensity";
  intensity.offset = 4;
  intensity.datatype = sensor_msgs::msg::PointField::UINT16;
  message->fields = {z, intensity};
  message->data.resize(28, 0xff);
  for (size_t index = 0; index < 4; ++index) {
    auto * point = message->data.data() + (index / 2) * 14 + (index % 2) * 6;
    const float depth = static_cast<float>(index);
    const uint16_t strength = index * 10;
    std::memcpy(point, &depth, sizeof(depth));
    std::memcpy(point + 4, &strength, sizeof(strength));
  }
  pub->publish(std::move(message));
  ASSERT_EQ(wait_for_sample<GetCloud>("points")->status.code,
      Status::OK);

  GetCloud::Request request;
  Filter lower;
  lower.field = "z";
  lower.comparison = Filter::GREATER_OR_EQUAL;
  lower.value = 1.0;
  Filter upper;
  upper.field = "intensity";
  upper.comparison = Filter::LESS;
  upper.value = 30;
  request.filters = {lower, upper};
  auto response = call<GetCloud>("points", request);
  ASSERT_EQ(response->status.code, Status::OK);
  EXPECT_EQ(response->cloud.width, 2u);
  EXPECT_EQ(response->cloud.height, 1u);
  EXPECT_EQ(response->cloud.data.size(), 12u);
  EXPECT_EQ(response->cloud.header.frame_id, "lidar");
  request.filters[0].field = "missing";
  response = call<GetCloud>("points", request);
  EXPECT_EQ(response->status.code, Status::INVALID_REQUEST);
  EXPECT_TRUE(response->cloud.data.empty());
  EXPECT_EQ(call<GetCloud>("points")->status.code, Status::OK);
}
TEST_F(Components, CloudServiceProjectsFieldsAndCombinesVoxelAndStrideSampling)
{
  auto pub = publisher<sensor_msgs::msg::PointCloud2>("points");
  auto message = std::make_unique<sensor_msgs::msg::PointCloud2>();
  message->width = 3;
  message->height = 1;
  message->point_step = 16;
  message->row_step = 48;
  const uint16_t probe = 1;
  message->is_bigendian = *reinterpret_cast<const uint8_t *>(&probe) == 0;
  for (const auto & name : {"x", "y", "z", "intensity"}) {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = message->fields.size() * 4;
    field.datatype = field.name == "intensity" ?
      sensor_msgs::msg::PointField::UINT32 : sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    message->fields.push_back(field);
  }
  message->data.resize(48);
  for (uint32_t index = 0; index < 3; ++index) {
    auto * point = message->data.data() + index * 16;
    const float x = index < 2 ? 0.1f * (index + 1) : 1.1f;
    const float z = 1.0f;
    const uint32_t intensity = index + 1;
    std::memcpy(point, &x, sizeof(x));
    std::memcpy(point + 8, &z, sizeof(z));
    std::memcpy(point + 12, &intensity, sizeof(intensity));
  }
  pub->publish(std::move(message));
  ASSERT_EQ(wait_for_sample<GetCloud>("points")->status.code, Status::OK);
  GetCloud::Request request;
  request.fields = {"intensity"};
  request.voxel_size = 1.0;
  request.downsample_factor = 2;
  const auto response = call<GetCloud>("points", request);
  ASSERT_EQ(response->status.code, Status::OK);
  EXPECT_EQ(response->cloud.fields.size(), 1u);
  EXPECT_EQ(response->cloud.point_step, 4u);
  EXPECT_EQ(response->cloud.width, 2u);
  uint32_t first, second;
  std::memcpy(&first, response->cloud.data.data(), sizeof(first));
  std::memcpy(&second, response->cloud.data.data() + 4, sizeof(second));
  EXPECT_EQ(first, 1u);
  EXPECT_EQ(second, 3u);
}
}  // namespace
