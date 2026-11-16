// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0
#include "endpoint.hpp"
#include "request_options.hpp"
#include <optional>
#include <rclcpp/serialization.hpp>
#include <sensor_snapshot_interfaces/msg/snapshot_status.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace sensor_snapshot_recorder
{
namespace
{
template<typename Service, typename Message>
class TypedEndpoint : public Endpoint, public std::enable_shared_from_this<TypedEndpoint<Service, Message>>
{
public:
  using Member = Message Service::Response::*;
  TypedEndpoint(rclcpp::Node & node, const std::string & topic,
    typename Service::Request::SharedPtr request, Member member, std::string type,
    rclcpp::CallbackGroup::SharedPtr group)
  : client_(node.create_client<Service>(topic, service_qos(), group)),
    request_(std::move(request)), member_(member), service_type_(std::move(type)),
    message_type_(rosidl_generator_traits::name<Message>()) {}

  bool ready() const override {return client_->service_is_ready();}
  const std::string & service_type() const override {return service_type_;}
  const std::string & message_type() const override {return message_type_;}
  void cancel() override
  {
    if (pending_) {client_->remove_pending_request(*pending_);pending_.reset();}
  }
  void capture(Reply reply) override
  {
    const auto weak = this->weak_from_this();
    const auto future = client_->async_send_request(request_,
      [weak, reply = std::move(reply)](typename rclcpp::Client<Service>::SharedFuture result) {
        const auto self = weak.lock();
        if (!self) {return;}
        self->pending_.reset();
        try {
          const auto response = result.get();
          if (response->status.code != sensor_snapshot_interfaces::msg::SnapshotStatus::OK) {
            reply(nullptr, "status " + std::to_string(response->status.code) + ": " + response->status.message);
            return;
          }
          auto serialized = std::make_shared<rclcpp::SerializedMessage>();
          rclcpp::Serialization<Message> serialization;
          serialization.serialize_message(&(response.get()->*(self->member_)), serialized.get());
          reply(std::move(serialized), "");
        } catch (const std::exception & error) {reply(nullptr, error.what());}
      });
    pending_ = future.request_id;
  }

private:
  typename rclcpp::Client<Service>::SharedPtr client_;
  typename Service::Request::SharedPtr request_;
  Member member_;
  std::string service_type_, message_type_;
  std::optional<int64_t> pending_;
};
const std::string image_type = "sensor_snapshot_interfaces/srv/GetImage";
const std::string imu_type = "sensor_snapshot_interfaces/srv/GetImu";
const std::string scan_type = "sensor_snapshot_interfaces/srv/GetLaserScan";
const std::string cloud_type = "sensor_snapshot_interfaces/srv/GetPointCloud";
}  // namespace

bool supported_service(const std::string & type)
{
  return type == image_type || type == imu_type || type == cloud_type || type == scan_type;
}

std::shared_ptr<Endpoint> make_endpoint(rclcpp::Node & node, const std::string & topic,
  const std::string & type, rclcpp::CallbackGroup::SharedPtr group)
{
  if (type == image_type) {
    return std::make_shared<TypedEndpoint<GetImage, sensor_msgs::msg::Image>>(
      node, topic, image_request(node, topic), &GetImage::Response::image, type, group);
  }
  if (type == imu_type) {
    return std::make_shared<TypedEndpoint<GetImu, sensor_msgs::msg::Imu>>(
      node, topic, imu_request(node, topic), &GetImu::Response::imu, type, group);
  }
  if (type == scan_type) {
    return std::make_shared<TypedEndpoint<GetLaserScan, sensor_msgs::msg::LaserScan>>(
      node, topic, scan_request(node, topic), &GetLaserScan::Response::scan, type, group);
  }
  return std::make_shared<TypedEndpoint<GetPointCloud, sensor_msgs::msg::PointCloud2>>(
    node, topic, cloud_request(node, topic), &GetPointCloud::Response::cloud, type, group);
}
}  // namespace sensor_snapshot_recorder
