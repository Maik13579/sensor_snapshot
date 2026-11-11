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

#ifndef SENSOR_SNAPSHOT_TEST__COMPONENT_FIXTURE_HPP_
#define SENSOR_SNAPSHOT_TEST__COMPONENT_FIXTURE_HPP_

#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "sensor_snapshot/image_snapshot.hpp"
#include "sensor_snapshot/point_cloud_snapshot.hpp"
#include "sensor_snapshot/imu_snapshot.hpp"

namespace sensor_snapshot_test
{
using namespace std::chrono_literals;
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
    imu_ = std::make_shared<sensor_snapshot::ImuSnapshot>(options);
    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 4);
    executor_->add_node(helper_);
    executor_->add_node(image_);
    executor_->add_node(cloud_);
    executor_->add_node(imu_);
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
    imu_.reset();
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
  std::shared_ptr<sensor_snapshot::ImuSnapshot> imu_;
  std::vector<std::shared_ptr<rclcpp::Node>> additional_nodes_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread thread_;
};

}  // namespace sensor_snapshot_test
#endif  // SENSOR_SNAPSHOT_TEST__COMPONENT_FIXTURE_HPP_
