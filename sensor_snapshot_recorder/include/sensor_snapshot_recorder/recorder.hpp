// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0
#ifndef SENSOR_SNAPSHOT_RECORDER__RECORDER_HPP_
#define SENSOR_SNAPSHOT_RECORDER__RECORDER_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace rosbag2_cpp {class Writer;}
namespace sensor_snapshot_recorder
{
class Endpoint;
struct PendingCapture;

/// Records one complete set of latest snapshots per trigger. All callbacks share
/// one mutually-exclusive group; async responses never block that executor group.
class SnapshotRecorder : public rclcpp::Node
{
public:
  explicit SnapshotRecorder(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SnapshotRecorder() override;

private:
  using Trigger = std_srvs::srv::Trigger;
  void refresh_clients();
  void trigger(const std::shared_ptr<rmw_request_id_t> & header);
  void received(uint64_t capture, size_t index,
    std::shared_ptr<rclcpp::SerializedMessage> message, const std::string & error);
  void finish(bool success, std::string message);
  void reply(const std::shared_ptr<rmw_request_id_t> &, bool, const std::string &);

  std::vector<std::string> topics_;
  std::vector<std::shared_ptr<Endpoint>> endpoints_;
  std::map<std::string, std::string> errors_, recorded_types_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::unique_ptr<PendingCapture> active_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Service<Trigger>::SharedPtr trigger_service_;
  rclcpp::TimerBase::SharedPtr discovery_timer_, timeout_timer_;
  double timeout_seconds_{5.0};
  uint64_t next_capture_{1};
  bool storage_failed_{false};
};
}  // namespace sensor_snapshot_recorder
#endif  // SENSOR_SNAPSHOT_RECORDER__RECORDER_HPP_
