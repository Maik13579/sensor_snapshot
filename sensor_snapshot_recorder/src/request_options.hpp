// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0
#ifndef SENSOR_SNAPSHOT_RECORDER__REQUEST_OPTIONS_HPP_
#define SENSOR_SNAPSHOT_RECORDER__REQUEST_OPTIONS_HPP_
#include <rclcpp/rclcpp.hpp>
#include <sensor_snapshot_interfaces/srv/get_image.hpp>
#include <sensor_snapshot_interfaces/srv/get_imu.hpp>
#include <sensor_snapshot_interfaces/srv/get_laser_scan.hpp>
#include <sensor_snapshot_interfaces/srv/get_point_cloud.hpp>

namespace sensor_snapshot_recorder
{
using GetImage = sensor_snapshot_interfaces::srv::GetImage;
using GetImu = sensor_snapshot_interfaces::srv::GetImu;
using GetLaserScan = sensor_snapshot_interfaces::srv::GetLaserScan;
using GetPointCloud = sensor_snapshot_interfaces::srv::GetPointCloud;

// Every option is read once when its typed client is created. The zero stamp is
// deliberately not configurable: a recorder trigger always requests latest data.
GetImage::Request::SharedPtr image_request(rclcpp::Node &, const std::string & topic);
GetImu::Request::SharedPtr imu_request(rclcpp::Node &, const std::string & topic);
GetLaserScan::Request::SharedPtr scan_request(rclcpp::Node &, const std::string & topic);
GetPointCloud::Request::SharedPtr cloud_request(rclcpp::Node &, const std::string & topic);
}  // namespace sensor_snapshot_recorder
#endif  // SENSOR_SNAPSHOT_RECORDER__REQUEST_OPTIONS_HPP_
