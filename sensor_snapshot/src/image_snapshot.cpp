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

#include "sensor_snapshot/image_snapshot.hpp"
#include "node_settings.hpp"
#include "service_helpers.hpp"

#include <utility>
#include <rclcpp_components/register_node_macro.hpp>

namespace sensor_snapshot
{
ImageSnapshot::ImageSnapshot(const rclcpp::NodeOptions & options)
: Node(snapshot_node_name(options, "image"), options)
{
  const auto settings = declare_settings(*this, "image");
  default_max_age_seconds_ = settings.max_age_seconds;
  cache_.set_capacity(settings.history_size);
  subscription_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = subscription_group_;
  subscription_ = create_subscription<sensor_msgs::msg::Image>(
    settings.topic, settings.subscription_qos,
    [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
      cache_.store(std::move(message));
    }, subscription_options);
  service_ = create_service<Service>(
    // Follow the topic after namespaces and remappings have been applied.
    subscription_->get_topic_name(),
    [this](const std::shared_ptr<Service::Request> request,
    const std::shared_ptr<Service::Response> response) {get_image(request, response);},
    settings.service_qos, service_group_);
  RCLCPP_INFO(
    get_logger(), "Caching %s; snapshots at %s", subscription_->get_topic_name(),
    service_->get_service_name());
}

void ImageSnapshot::get_image(
  const std::shared_ptr<Service::Request> request,
  const std::shared_ptr<Service::Response> response)
{
  respond_safely(get_logger(), *response, [&]() {
      const auto source = prepare_sample(
      cache_, request->stamp, request->max_age_seconds, default_max_age_seconds_, *response);
      response->region = resolve_region(request->region, source->width, source->height);
      response->image = crop_image(*source, response->region, request->downsample_factor);
  });
}
}  // namespace sensor_snapshot

RCLCPP_COMPONENTS_REGISTER_NODE(sensor_snapshot::ImageSnapshot)
