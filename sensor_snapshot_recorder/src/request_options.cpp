// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0
#include "request_options.hpp"
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>

namespace sensor_snapshot_recorder
{
namespace
{
template<typename T>
T option(rclcpp::Node & node, const std::string & name, T fallback)
{
  if (!node.has_parameter(name)) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return node.declare_parameter<T>(name, fallback, descriptor);
  }
  return node.get_parameter(name).get_value<T>();
}

double nonnegative(rclcpp::Node & node, const std::string & name)
{
  const auto value = option<double>(node, name, 0.0);
  if (!std::isfinite(value) || value < 0) {
    throw std::invalid_argument(name + " must be finite and nonnegative");
  }
  return value;
}

uint32_t integer(rclcpp::Node & node, const std::string & name, int64_t fallback = 0)
{
  const auto value = option<int64_t>(node, name, fallback);
  if (value < fallback || value > std::numeric_limits<uint32_t>::max()) {
    throw std::invalid_argument(name + " is outside its supported integer range");
  }
  return static_cast<uint32_t>(value);
}

sensor_snapshot_interfaces::msg::Region region(rclcpp::Node & node, const std::string & prefix)
{
  sensor_snapshot_interfaces::msg::Region result;
  result.x_offset = integer(node, prefix + ".region.x_offset");
  result.y_offset = integer(node, prefix + ".region.y_offset");
  result.width = integer(node, prefix + ".region.width");
  result.height = integer(node, prefix + ".region.height");
  return result;
}
}  // namespace

GetImage::Request::SharedPtr image_request(rclcpp::Node & node, const std::string & topic)
{
  const auto prefix = "requests." + topic;
  auto request = std::make_shared<GetImage::Request>();
  request->region = region(node, prefix);
  request->downsample_factor = integer(node, prefix + ".downsample_factor", 1);
  request->max_age_seconds = nonnegative(node, prefix + ".max_age_seconds");
  return request;
}

GetImu::Request::SharedPtr imu_request(rclcpp::Node & node, const std::string & topic)
{
  auto request = std::make_shared<GetImu::Request>();
  request->max_age_seconds = nonnegative(node, "requests." + topic + ".max_age_seconds");
  return request;
}

GetPointCloud::Request::SharedPtr cloud_request(rclcpp::Node & node, const std::string & topic)
{
  using Filter = sensor_snapshot_interfaces::msg::FieldFilter;
  const auto prefix = "requests." + topic;
  auto request = std::make_shared<GetPointCloud::Request>();
  request->region = region(node, prefix);
  request->downsample_factor = integer(node, prefix + ".downsample_factor", 1);
  request->max_age_seconds = nonnegative(node, prefix + ".max_age_seconds");
  request->voxel_size = nonnegative(node, prefix + ".voxel_size");
  request->fields = option<std::vector<std::string>>(node, prefix + ".fields", {});

  // ROS parameter arrays cannot contain maps. Named filter blocks avoid that
  // limitation and are evaluated in deterministic lexical order (all are ANDed).
  const auto filter_prefix = prefix + ".filters.";
  std::set<std::string> names;
  for (const auto & [name, value] : node.get_node_parameters_interface()->get_parameter_overrides()) {
    (void)value;
    if (name.compare(0, filter_prefix.size(), filter_prefix) == 0) {
      names.insert(name.substr(filter_prefix.size(), name.find('.', filter_prefix.size()) - filter_prefix.size()));
    }
  }
  const std::map<std::string, uint8_t> comparisons{
    {"LESS", Filter::LESS}, {"LESS_OR_EQUAL", Filter::LESS_OR_EQUAL},
    {"EQUAL", Filter::EQUAL}, {"NOT_EQUAL", Filter::NOT_EQUAL},
    {"GREATER", Filter::GREATER}, {"GREATER_OR_EQUAL", Filter::GREATER_OR_EQUAL}};
  for (const auto & name : names) {
    const auto base = filter_prefix + name;
    Filter filter;
    filter.field = option<std::string>(node, base + ".field", "");
    const auto comparison = option<std::string>(node, base + ".comparison", "GREATER");
    filter.value = option<double>(node, base + ".value", 0.0);
    filter.element_index = integer(node, base + ".element_index");
    if (filter.field.empty() || !comparisons.count(comparison) || !std::isfinite(filter.value)) {
      throw std::invalid_argument("Invalid point-field filter: " + base);
    }
    filter.comparison = comparisons.at(comparison);
    request->filters.push_back(filter);
  }
  return request;
}
}  // namespace sensor_snapshot_recorder
