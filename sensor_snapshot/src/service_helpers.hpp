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

#ifndef SENSOR_SNAPSHOT__SERVICE_HELPERS_HPP_
#define SENSOR_SNAPSHOT__SERVICE_HELPERS_HPP_

#include <cmath>
#include <exception>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_snapshot_interfaces/msg/snapshot_status.hpp>
#include "sensor_snapshot/message_history.hpp"
#include "sensor_snapshot/processing.hpp"

namespace sensor_snapshot
{
/// Snapshot the pointer and timing together; release the mutex before any processing.
template<typename MessageT, typename ResponseT>
std::shared_ptr<const MessageT> prepare_sample(
  const MessageHistory<MessageT> & cache, const builtin_interfaces::msg::Time & stamp,
  double requested_age, double default_age,
  ResponseT & response)
{
  using Status = sensor_snapshot_interfaces::msg::SnapshotStatus;
  if (!std::isfinite(requested_age) || requested_age < 0.0) {
    throw SnapshotError(Status::INVALID_REQUEST, "max_age_seconds must be finite and nonnegative");
  }
  typename MessageHistory<MessageT>::Sample sample;
  try {
    sample = cache.select(stamp);
  } catch (const std::invalid_argument & error) {
    throw SnapshotError(Status::INVALID_REQUEST, error.what());
  } catch (const std::out_of_range & error) {
    throw SnapshotError(Status::NOT_FOUND, error.what());
  }
  if (!sample.message) {
    throw SnapshotError(Status::NO_DATA, "No sensor message has been received");
  }
  response.sequence = sample.sequence;
  response.source_width = sample.message->width;
  response.source_height = sample.message->height;
  response.age_seconds = std::chrono::duration<double>(
    MessageHistory<MessageT>::Clock::now() - sample.received_at).count();
  const double limit = requested_age == 0.0 ? default_age : requested_age;
  if (limit > 0.0 && response.age_seconds > limit) {
    throw SnapshotError(Status::STALE, "Cached message exceeds the maximum reception age");
  }
  return sample.message;
}

/// Invalid client requests and malformed sensor messages must not kill the container.
template<typename ResponseT, typename FunctionT>
void respond_safely(const rclcpp::Logger & logger, ResponseT & response, FunctionT function)
{
  using Status = sensor_snapshot_interfaces::msg::SnapshotStatus;
  try {
    function();
    response.status.code = Status::OK;
    response.status.message = "OK";
  } catch (const SnapshotError & error) {
    response.status.code = error.status_code();
    response.status.message = error.what();
  } catch (const std::exception & error) {
    response.status.code = Status::INTERNAL_ERROR;
    response.status.message = "Snapshot processing failed";
    RCLCPP_ERROR(logger, "Snapshot processing failed: %s", error.what());
  } catch (...) {
    response.status.code = Status::INTERNAL_ERROR;
    response.status.message = "Unexpected snapshot processing failure";
    RCLCPP_ERROR(logger, "Unexpected snapshot processing failure");
  }
}
}  // namespace sensor_snapshot
#endif  // SENSOR_SNAPSHOT__SERVICE_HELPERS_HPP_
