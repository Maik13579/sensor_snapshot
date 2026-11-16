// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0
#ifndef SENSOR_SNAPSHOT_RECORDER__ENDPOINT_HPP_
#define SENSOR_SNAPSHOT_RECORDER__ENDPOINT_HPP_
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/version.h>

namespace sensor_snapshot_recorder
{
inline const auto & service_qos()
{
#if RCLCPP_VERSION_MAJOR < 28
  return rmw_qos_profile_services_default;
#else
  static const rclcpp::ServicesQoS qos;
  return qos;
#endif
}

class Endpoint
{
public:
  using Reply = std::function<void(std::shared_ptr<rclcpp::SerializedMessage>, std::string)>;
  virtual ~Endpoint() = default;
  virtual bool ready() const = 0;
  virtual void capture(Reply) = 0;
  virtual void cancel() = 0;
  virtual const std::string & service_type() const = 0;
  virtual const std::string & message_type() const = 0;
};

bool supported_service(const std::string & type);
std::shared_ptr<Endpoint> make_endpoint(rclcpp::Node &, const std::string & topic,
  const std::string & type, rclcpp::CallbackGroup::SharedPtr);
}  // namespace sensor_snapshot_recorder
#endif  // SENSOR_SNAPSHOT_RECORDER__ENDPOINT_HPP_
