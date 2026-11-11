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

#ifndef SENSOR_SNAPSHOT__IMU_SNAPSHOT_HPP_
#define SENSOR_SNAPSHOT__IMU_SNAPSHOT_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_snapshot_interfaces/srv/get_imu.hpp>
#include "sensor_snapshot/message_history.hpp"

namespace sensor_snapshot
{
/// Retains immutable IMU samples; snapshots preserve every field without processing.
class ImuSnapshot : public rclcpp::Node
{
public:
  explicit ImuSnapshot(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using Service = sensor_snapshot_interfaces::srv::GetImu;
  void get_imu(
    const std::shared_ptr<Service::Request> request,
    const std::shared_ptr<Service::Response> response);
  MessageHistory<sensor_msgs::msg::Imu> cache_;
  double default_max_age_seconds_{0.0};
  rclcpp::CallbackGroup::SharedPtr subscription_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  rclcpp::Service<Service>::SharedPtr service_;
};
}  // namespace sensor_snapshot
#endif  // SENSOR_SNAPSHOT__IMU_SNAPSHOT_HPP_
