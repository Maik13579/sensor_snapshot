// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0
#include "sensor_snapshot/laser_scan_snapshot.hpp"
#include "node_settings.hpp"
#include "service_helpers.hpp"

#include <utility>
#include <rclcpp_components/register_node_macro.hpp>

namespace sensor_snapshot
{
LaserScanSnapshot::LaserScanSnapshot(const rclcpp::NodeOptions & options)
: Node(snapshot_node_name(options, "scan"), options)
{
  const auto settings = declare_settings(*this, "scan");
  default_max_age_seconds_ = settings.max_age_seconds;
  cache_.set_capacity(settings.history_size);
  auto groups = snapshot_callback_groups(*this);
  subscription_group_ = groups.first;
  service_group_ = groups.second;
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = subscription_group_;
  subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
    settings.topic, settings.subscription_qos,
    [this](sensor_msgs::msg::LaserScan::ConstSharedPtr message) {
      cache_.store(std::move(message));
    }, subscription_options);
  service_ = create_service<Service>(
    subscription_->get_topic_name(),
    [this](const std::shared_ptr<Service::Request> request,
    const std::shared_ptr<Service::Response> response) {get_scan(request, response);},
    service_qos_argument(settings.service_qos), service_group_);
  RCLCPP_INFO(
    get_logger(), "Caching %s; snapshots at %s", subscription_->get_topic_name(),
    service_->get_service_name());
}

void LaserScanSnapshot::get_scan(
  const std::shared_ptr<Service::Request> request,
  const std::shared_ptr<Service::Response> response)
{
  respond_safely(get_logger(), *response, [&]() {
      const auto source = prepare_sample(
        cache_, request->stamp, request->max_age_seconds, default_max_age_seconds_, *response);
      response->scan = *source;
  });
}
}  // namespace sensor_snapshot

RCLCPP_COMPONENTS_REGISTER_NODE(sensor_snapshot::LaserScanSnapshot)
