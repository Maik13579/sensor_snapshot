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

#ifndef SENSOR_SNAPSHOT__NODE_SETTINGS_HPP_
#define SENSOR_SNAPSHOT__NODE_SETTINGS_HPP_

#include <cstddef>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace sensor_snapshot
{
struct NodeSettings
{
  std::string topic;
  double max_age_seconds;
  size_t history_size;
  rclcpp::QoS subscription_qos{1};
  rclcpp::QoS service_qos{rclcpp::ServicesQoS()};
};

NodeSettings declare_settings(rclcpp::Node & node, const std::string & default_topic);

/// Component-manager parameter overrides are available before the Node constructor.
std::string snapshot_node_name(
  const rclcpp::NodeOptions & options, const std::string & default_topic);
}  // namespace sensor_snapshot
#endif  // SENSOR_SNAPSHOT__NODE_SETTINGS_HPP_
