// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0
#ifndef SENSOR_SNAPSHOT__LASER_SCAN_SNAPSHOT_HPP_
#define SENSOR_SNAPSHOT__LASER_SCAN_SNAPSHOT_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_snapshot_interfaces/srv/get_laser_scan.hpp>
#include "sensor_snapshot/message_history.hpp"

namespace sensor_snapshot
{
/// Retains immutable scans without projecting, filtering or changing beam timing.
class LaserScanSnapshot : public rclcpp::Node
{
public:
  explicit LaserScanSnapshot(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using Service = sensor_snapshot_interfaces::srv::GetLaserScan;
  void get_scan(
    const std::shared_ptr<Service::Request> request,
    const std::shared_ptr<Service::Response> response);
  MessageHistory<sensor_msgs::msg::LaserScan> cache_;
  double default_max_age_seconds_{0.0};
  rclcpp::CallbackGroup::SharedPtr subscription_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Service<Service>::SharedPtr service_;
};
}  // namespace sensor_snapshot
#endif  // SENSOR_SNAPSHOT__LASER_SCAN_SNAPSHOT_HPP_
