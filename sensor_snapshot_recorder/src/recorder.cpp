// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0
#include "sensor_snapshot_recorder/recorder.hpp"
#include "endpoint.hpp"
#include <chrono>
#include <cmath>
#include <filesystem>
#include <set>
#include <stdexcept>
#include <rosbag2_cpp/writer.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace sensor_snapshot_recorder
{
struct PendingCapture
{
  uint64_t id;
  rclcpp::Time stamp;
  std::chrono::steady_clock::time_point deadline;
  std::shared_ptr<rmw_request_id_t> header;
  std::vector<std::shared_ptr<rclcpp::SerializedMessage>> messages;
  size_t remaining;
  std::string errors;
};

SnapshotRecorder::SnapshotRecorder(const rclcpp::NodeOptions & options)
: Node("sensor_snapshot_recorder", options)
{
  rcl_interfaces::msg::ParameterDescriptor fixed;
  fixed.read_only = true;
  auto topics = declare_parameter<std::vector<std::string>>("input.topics", {}, fixed);
  auto uri = declare_parameter<std::string>("output.uri", "", fixed);
  const auto storage = declare_parameter<std::string>("output.storage_id", "mcap", fixed);
  timeout_seconds_ = declare_parameter<double>("capture.timeout_seconds", 5.0, fixed);
  const auto discovery = declare_parameter<double>("discovery.period_seconds", 1.0, fixed);
  if (topics.empty() || !std::isfinite(timeout_seconds_) || timeout_seconds_ <= 0 ||
    !std::isfinite(discovery) || discovery <= 0 || storage.empty())
  {
    throw std::invalid_argument("Set input.topics and finite positive timeout/discovery periods");
  }
  std::set<std::string> unique;
  for (const auto & name : topics) {
    const auto resolved = get_node_base_interface()->resolve_topic_or_service_name(name, true);
    if (!unique.insert(resolved).second) {throw std::invalid_argument("Duplicate topic: " + resolved);}
    topics_.push_back(resolved);
  }
  endpoints_.resize(topics_.size());
  if (uri.empty()) {
    uri = "snapshots_" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
  }
  if (std::filesystem::exists(uri)) {throw std::invalid_argument("Bag output already exists: " + uri);}
  rosbag2_storage::StorageOptions bag_options;
  bag_options.uri = uri;
  bag_options.storage_id = storage;
  writer_ = std::make_unique<rosbag2_cpp::Writer>();
  writer_->open(bag_options, {"cdr", "cdr"});
  group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  trigger_service_ = create_service<Trigger>("~/trigger",
    [this](std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<Trigger::Request>) {
      trigger(header);
    }, service_qos(), group_);
  stop_service_ = create_service<Trigger>("~/stop",
    [this](std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<Trigger::Request>) {
      stop(header);
    }, service_qos(), group_);
  discovery_timer_ = create_wall_timer(std::chrono::duration<double>(discovery),
    [this] {if (!active_ && !stopping_ && !stopped_) {refresh_clients();}}, group_);
  timeout_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] {
      if (active_ && std::chrono::steady_clock::now() >= active_->deadline) {
        finish(false, "Snapshot request timed out; no messages from this trigger were written");
      }
    }, group_);
  refresh_clients();
  RCLCPP_INFO(get_logger(), "Recording triggered snapshots to %s; trigger at %s",
    uri.c_str(), trigger_service_->get_service_name());
}

SnapshotRecorder::~SnapshotRecorder()
{
  for (auto & endpoint : endpoints_) {if (endpoint) {endpoint->cancel();}}
  try {if (writer_) {writer_->close();}}
  catch (const std::exception & error) {RCLCPP_ERROR(get_logger(), "Bag close failed: %s", error.what());}
}

void SnapshotRecorder::set_capture_callback(std::function<void(RecordedFrame)> callback)
{
  capture_callback_ = std::move(callback);
}

void SnapshotRecorder::refresh_clients()
{
  const auto graph = get_service_names_and_types();
  for (size_t i = 0; i < topics_.size(); ++i) {
    const auto & topic = topics_[i];
    const auto found = graph.find(topic);
    if (found == graph.end() || found->second.size() != 1 || !supported_service(found->second[0])) {
      endpoints_[i].reset();
      errors_[topic] = "No unique supported snapshot service";
      continue;
    }
    const auto & type = found->second[0];
    if (endpoints_[i] && endpoints_[i]->service_type() == type) {continue;}
    try {
      auto endpoint = make_endpoint(*this, topic, type, group_);
      if (recorded_types_.count(topic) && recorded_types_[topic] != endpoint->message_type()) {
        throw std::runtime_error("Topic type changed after recording started");
      }
      endpoints_[i] = std::move(endpoint);
      errors_.erase(topic);
      RCLCPP_INFO(get_logger(), "Snapshot client ready: %s [%s]", topic.c_str(), type.c_str());
    } catch (const std::exception & error) {
      endpoints_[i].reset();
      errors_[topic] = error.what();
    }
  }
}

void SnapshotRecorder::trigger(const std::shared_ptr<rmw_request_id_t> & header)
{
  const auto stamp = now();  // The same recorder-side ROS time is used for every bag record.
  if (stopping_ || stopped_) {reply(header, false, "Recording has stopped");return;}
  if (active_) {reply(header, false, "A capture is already in progress");return;}
  if (storage_failed_) {reply(header, false, "Recording stopped after a bag write error");return;}
  refresh_clients();
  for (size_t i = 0; i < endpoints_.size(); ++i) {
    if (!endpoints_[i] || !endpoints_[i]->ready()) {
      reply(header, false, topics_[i] + ": " + (errors_.count(topics_[i]) ? errors_[topics_[i]] : "Service not ready"));
      return;
    }
  }
  const auto id = next_capture_++;
  active_ = std::make_unique<PendingCapture>(PendingCapture{
    id, stamp, std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(timeout_seconds_)),
    header, std::vector<std::shared_ptr<rclcpp::SerializedMessage>>(topics_.size()), topics_.size(), ""});
  const auto weak = weak_from_this();
  // Dispatch every cached request before returning to the executor. No client is
  // synchronously waited on, including in a single-threaded component container.
  for (size_t i = 0; i < endpoints_.size(); ++i) {
    try {
      endpoints_[i]->capture([weak, id, i](auto message, const std::string & error) {
          if (const auto node = weak.lock()) {
            std::static_pointer_cast<SnapshotRecorder>(node)->received(id, i, message, error);
          }
        });
    } catch (const std::exception & error) {received(id, i, nullptr, error.what());}
  }
}

void SnapshotRecorder::received(uint64_t capture, size_t index,
  std::shared_ptr<rclcpp::SerializedMessage> message, const std::string & error)
{
  if (!active_ || active_->id != capture) {return;}  // Discard late replies after timeout.
  if (std::chrono::steady_clock::now() >= active_->deadline) {
    finish(false, "Snapshot request timed out; no messages from this trigger were written");
    return;
  }
  active_->messages[index] = std::move(message);
  if (!error.empty()) {active_->errors += topics_[index] + ": " + error + "; ";}
  if (--active_->remaining != 0) {return;}
  if (!active_->errors.empty()) {finish(false, active_->errors);return;}
  try {
    // All replies are validated and serialized before the first write. Original
    // headers are inside these buffers and are never changed by the recorder.
    for (size_t i = 0; i < topics_.size(); ++i) {
      writer_->write(active_->messages[i], topics_[i], endpoints_[i]->message_type(), active_->stamp);
      recorded_types_[topics_[i]] = endpoints_[i]->message_type();
    }
  } catch (const std::exception & error) {
    storage_failed_ = true;  // Storage failures can leave a partial write; never retry silently.
    finish(false, std::string("Bag write failed; recording stopped: ") + error.what());
    return;
  }
  if (capture_callback_) {
    RecordedFrame frame{capture, active_->stamp, {}};
    for (size_t i = 0; i < topics_.size(); ++i) {
      frame.messages.push_back({topics_[i], endpoints_[i]->message_type(), active_->messages[i]});
    }
    try {capture_callback_(std::move(frame));}
    catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Capture recorded, but its notification failed: %s", error.what());
    }
  }
  finish(true, "Recorded keyframe " + std::to_string(capture) + " (" + std::to_string(topics_.size()) + " topics)");
}

void SnapshotRecorder::finish(bool success, std::string message)
{
  const auto header = active_->header;
  for (auto & endpoint : endpoints_) {if (endpoint) {endpoint->cancel();}}
  // Own the reply text before clearing the capture: callers may pass active_->errors.
  active_.reset();
  reply(header, success, message);
  if (stopping_) {finish_stop();}
}

void SnapshotRecorder::stop(const std::shared_ptr<rmw_request_id_t> & header)
{
  if (stopping_) {
    Trigger::Response response;response.message = "Stop already in progress";
    stop_service_->send_response(*header, response);
    return;
  }
  stop_header_ = header;
  stopping_ = true;
  // A capture already in flight is completed (or times out) before closing.
  if (!active_) {finish_stop();}
}

void SnapshotRecorder::finish_stop()
{
  Trigger::Response response;
  try {
    if (writer_) {writer_->close();writer_.reset();}
    stopped_ = true;
    response.success = true;response.message = "Recording stopped and bag finalized";
  } catch (const std::exception & error) {response.message = error.what();}
  discovery_timer_->cancel();timeout_timer_->cancel();
  stopping_ = false;
  // A failed close must still prevent writes through a potentially closed writer.
  stopped_ = true;
  const auto header = std::move(stop_header_);
  try {stop_service_->send_response(*header, response);}
  catch (const std::exception & error) {RCLCPP_WARN(get_logger(), "Stop reply failed: %s", error.what());}
}

void SnapshotRecorder::reply(const std::shared_ptr<rmw_request_id_t> & header,
  bool success, const std::string & message)
{
  Trigger::Response response;
  response.success = success;
  response.message = message;
  try {trigger_service_->send_response(*header, response);}
  catch (const std::exception & error) {RCLCPP_WARN(get_logger(), "Trigger reply failed: %s", error.what());}
}
}  // namespace sensor_snapshot_recorder
RCLCPP_COMPONENTS_REGISTER_NODE(sensor_snapshot_recorder::SnapshotRecorder)
