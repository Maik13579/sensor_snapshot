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

#include "node_settings.hpp"

#include <cmath>
#include <cctype>
#include <limits>
#include <stdexcept>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace sensor_snapshot
{
namespace
{
rcl_interfaces::msg::ParameterDescriptor descriptor(const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor result;
  result.description = description;
  result.read_only = true;  // Endpoints are configured at startup, never silently left stale.
  return result;
}
}  // namespace

std::string snapshot_node_name(
  const rclcpp::NodeOptions & options, const std::string & default_topic)
{
  auto topic = default_topic;
  for (const auto & parameter : options.parameter_overrides()) {
    if (parameter.get_name() == "input.topic") {
      topic = parameter.as_string();
    }
  }
  const auto start = topic.find_first_not_of('/');
  auto name = start == std::string::npos ? std::string{} : topic.substr(start);
  // ROS node names cannot contain topic separators or substitution characters.
  for (auto & character : name) {
    if (!std::isalnum(static_cast<unsigned char>(character)) && character != '_') {
      character = '_';
    }
  }
  return "snapshot_" + name;
}

NodeSettings declare_settings(rclcpp::Node & node, const std::string & default_topic)
{
  NodeSettings settings;
  settings.topic = node.declare_parameter<std::string>(
    "input.topic", default_topic, descriptor("Raw Image or PointCloud2 input topic"));
  const auto reliability = node.declare_parameter<std::string>(
    "input.reliability", "best_effort", descriptor("Input reliability: best_effort or reliable"));

  auto age_descriptor =
    descriptor("Default maximum reception age in seconds; zero disables expiry");
  rcl_interfaces::msg::FloatingPointRange age_range;
  age_range.from_value = 0.0;
  age_range.to_value = std::numeric_limits<double>::max();
  age_descriptor.floating_point_range.push_back(age_range);
  settings.max_age_seconds = node.declare_parameter<double>("cache.max_age_seconds", 0.0,
      age_descriptor);

  auto history_descriptor = descriptor("Number of received messages retained by shared pointer");
  rcl_interfaces::msg::IntegerRange history_range;
  history_range.from_value = 1;
  history_range.to_value = 10000;
  history_descriptor.integer_range.push_back(history_range);
  const auto history_size = node.declare_parameter<int64_t>(
    "cache.history_size", 1, history_descriptor);
  if (history_size < 1 || history_size > 10000) {
    throw std::invalid_argument("cache.history_size must be between 1 and 10000");
  }
  settings.history_size = static_cast<size_t>(history_size);

  auto depth_descriptor =
    descriptor("Service request/response history depth; processing is serialized");
  rcl_interfaces::msg::IntegerRange depth_range;
  depth_range.from_value = 1;
  depth_range.to_value = 100;
  depth_descriptor.integer_range.push_back(depth_range);
  const auto depth = node.declare_parameter<int64_t>("service.qos_depth", 1, depth_descriptor);
  if (depth < 1 || depth > 100) {
    throw std::invalid_argument("service.qos_depth must be between 1 and 100");
  }
  if (settings.topic.empty() ||
    !std::isfinite(settings.max_age_seconds) || settings.max_age_seconds < 0.0)
  {
    throw std::invalid_argument(
        "Topic name must be nonempty and cache age finite/nonnegative");
  }
  settings.subscription_qos = rclcpp::SensorDataQoS().keep_last(1);
  if (reliability == "reliable") {
    settings.subscription_qos.reliable();
  } else if (reliability != "best_effort") {
    throw std::invalid_argument("input.reliability must be best_effort or reliable");
  }
  settings.service_qos.keep_last(static_cast<size_t>(depth));
  return settings;
}
}  // namespace sensor_snapshot
