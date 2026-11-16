// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0
#ifndef SENSOR_SNAPSHOT_RECORDER_TEST__FIXTURE_HPP_
#define SENSOR_SNAPSHOT_RECORDER_TEST__FIXTURE_HPP_
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <functional>
#include <future>
#include <mutex>
#include <thread>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "sensor_snapshot_recorder/recorder.hpp"
#include "request_options.hpp"

namespace sensor_snapshot_recorder_test
{
using namespace std::chrono_literals;
using namespace sensor_snapshot_recorder;
using Trigger = std_srvs::srv::Trigger;

template<typename T> auto bag_stamp(const T & message, int) -> decltype(message.recv_timestamp)
{return message.recv_timestamp;}
template<typename T> auto bag_stamp(const T & message, long) -> decltype(message.time_stamp)
{return message.time_stamp;}

class Recorder : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    static unsigned sequence = 0;
    ns_ = "/recorder_test_" + std::to_string(++sequence);
    directory_ = std::filesystem::path(TEST_BAG_DIRECTORY) / std::to_string(sequence);
    std::filesystem::remove_all(directory_);
    helper_ = std::make_shared<rclcpp::Node>("source", ns_);
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(helper_);
  }
  void TearDown() override
  {
    close();
    helper_.reset();
    std::filesystem::remove_all(directory_);
  }
  bool until(std::function<bool()> predicate, std::chrono::milliseconds timeout = 3000ms)
  {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
      if (predicate()) {return true;}
      std::this_thread::sleep_for(5ms);
    }
    return false;
  }
  template<typename Service, typename Fill>
  void provider(const std::string & name, Fill fill)
  {
    using Server = rclcpp::Service<Service>;
    auto server = helper_->create_service<Service>(name,
      [this, fill](typename Server::SharedPtr handle, std::shared_ptr<rmw_request_id_t> header,
      typename Service::Request::SharedPtr request) {
        std::vector<std::function<void()>> ready;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          queued_.push_back([this, handle, header, request, fill] {
              typename Service::Response response;
              fill(*request, response);
              handle->send_response(*header, response);
            });
          ++seen_;
          if (!hold_ && queued_.size() == expected_) {ready.swap(queued_);}
        }
        for (auto & reply : ready) {reply();}
      });
    servers_.push_back(server);
  }
  void providers()
  {
    provider<GetImage>("image", [](const auto &, auto & response) {
        response.image.header.stamp.sec = 11;
        response.image.header.frame_id = "camera";
        response.image.width = 1;response.image.height = 1;
        response.image.step = 3;response.image.encoding = "rgb8";
        response.image.data = {1, 2, 3};
      });
    provider<GetPointCloud>("cloud", [this](const auto & request, auto & response) {
        voxel_seen_ = request.voxel_size;
        fields_seen_ = request.fields;
        response.cloud.header.stamp.sec = 22;
        response.cloud.header.frame_id = "laser";
        response.cloud.width = 1;response.cloud.height = 1;
        response.cloud.point_step = 4;response.cloud.row_step = 4;
        sensor_msgs::msg::PointField field;
        field.name = "x";field.offset = 0;field.datatype = 7;field.count = 1;
        response.cloud.fields = {field};response.cloud.data = {0, 0, 128, 63};
      });
    provider<GetImu>("imu", [this](const auto &, auto & response) {
        response.status.code = bad_ ? 2 : 0;
        response.status.message = bad_ ? "stale IMU" : "";
        response.imu.header.stamp.sec = 33;response.imu.header.frame_id = "imu_link";
        response.imu.linear_acceleration.z = 9.81;
        response.imu.orientation_covariance[0] = -1;
      });
    provider<GetLaserScan>("scan", [this](const auto & request, auto & response) {
        scan_age_seen_ = request.max_age_seconds;
        response.scan.header.stamp.sec = 44;
        response.scan.header.frame_id = "scan_laser";
        response.scan.angle_min = -0.5f;response.scan.angle_max = 0.5f;
        response.scan.angle_increment = 0.5f;response.scan.time_increment = 0.001f;
        response.scan.scan_time = 0.1f;response.scan.range_min = 0.2f;
        response.scan.range_max = 30.0f;
        response.scan.ranges = {1.0f, 2.0f, 3.0f};
        response.scan.intensities = {4.0f, 5.0f, 6.0f};
      });
  }
  rclcpp::NodeOptions options(std::vector<std::string> topics = {})
  {
    if (topics.empty()) {topics = {ns_ + "/image", ns_ + "/cloud", ns_ + "/imu", ns_ + "/scan"};}
    rclcpp::NodeOptions result;
    result.arguments({"--ros-args", "-r", "__ns:=" + ns_});
    result.append_parameter_override("input.topics", topics);
    result.append_parameter_override("output.uri", directory_.string());
    result.append_parameter_override("capture.timeout_seconds", 0.2);
    result.append_parameter_override("discovery.period_seconds", 0.02);
    return result;
  }
  void start(const rclcpp::NodeOptions & opts)
  {
    recorder_ = std::make_shared<SnapshotRecorder>(opts);
    executor_->add_node(recorder_);
    client_ = helper_->create_client<Trigger>("sensor_snapshot_recorder/trigger");
    thread_ = std::thread([this] {executor_->spin();});
    ASSERT_TRUE(client_->wait_for_service(3s));
  }
  auto trigger()
  {
    return client_->async_send_request(std::make_shared<Trigger::Request>());
  }
  Trigger::Response::SharedPtr capture()
  {
    auto result = trigger();
    if (result.wait_for(5s) != std::future_status::ready) {throw std::runtime_error("Trigger hung");}
    return result.get();
  }
  void release()
  {
    std::vector<std::function<void()>> ready;
    {std::lock_guard<std::mutex> lock(mutex_);ready.swap(queued_);}
    for (auto & reply : ready) {reply();}
  }
  void close()
  {
    if (executor_) {executor_->cancel();}
    if (thread_.joinable()) {thread_.join();}
    if (recorder_) {executor_->remove_node(recorder_);recorder_.reset();}
  }
  std::vector<rosbag2_storage::SerializedBagMessageSharedPtr> messages()
  {
    rosbag2_cpp::Reader reader;
    reader.open(directory_.string());
    std::vector<rosbag2_storage::SerializedBagMessageSharedPtr> result;
    while (reader.has_next()) {result.push_back(reader.read_next());}
    return result;
  }

  std::string ns_;
  std::filesystem::path directory_;
  std::shared_ptr<rclcpp::Node> helper_;
  std::shared_ptr<SnapshotRecorder> recorder_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread thread_;
  rclcpp::Client<Trigger>::SharedPtr client_;
  std::vector<rclcpp::ServiceBase::SharedPtr> servers_;
  std::mutex mutex_;
  std::vector<std::function<void()>> queued_;
  std::atomic<size_t> seen_{0};
  std::atomic<bool> hold_{false}, bad_{false};
  size_t expected_{4};
  double voxel_seen_{0}, scan_age_seen_{0};
  std::vector<std::string> fields_seen_;
};
}  // namespace sensor_snapshot_recorder_test
#endif  // SENSOR_SNAPSHOT_RECORDER_TEST__FIXTURE_HPP_
