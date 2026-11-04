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

#ifndef SENSOR_SNAPSHOT__MESSAGE_HISTORY_HPP_
#define SENSOR_SNAPSHOT__MESSAGE_HISTORY_HPP_

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <vector>
#include <builtin_interfaces/msg/time.hpp>

namespace sensor_snapshot
{
/// Fixed-capacity ring of immutable messages; retaining a frame never copies its payload.
template<typename MessageT>
class MessageHistory
{
public:
  using Clock = std::chrono::steady_clock;

  struct Sample
  {
    std::shared_ptr<const MessageT> message;
    Clock::time_point received_at;
    uint64_t sequence{0};
  };

  explicit MessageHistory(size_t capacity = 1)
  : samples_(capacity)
  {
    if (capacity == 0) {
      throw std::invalid_argument("History capacity must be positive");
    }
  }

  /// Startup-only configuration: allocate the pointer slots before subscribing.
  void set_capacity(size_t capacity)
  {
    if (capacity == 0) {
      throw std::invalid_argument("History capacity must be positive");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (sequence_ != 0) {
      throw std::logic_error("History capacity cannot change after receiving messages");
    }
    samples_.resize(capacity);
  }

  void store(std::shared_ptr<const MessageT> message)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto & slot = samples_[next_];
      slot.message.swap(message);
      slot.received_at = Clock::now();
      slot.sequence = ++sequence_;
      next_ = (next_ + 1) % samples_.size();
    }
    // Release the replaced message outside the critical section. No payload copy.
  }

  Sample latest() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_locked();
  }

  /// Zero means latest by receipt. A nonzero stamp selects nearest acquisition time.
  /// Scan timestamps only; messages may arrive out of timestamp order. Ties select
  /// the newest receipt, and requests outside the retained range never clamp.
  Sample select(const builtin_interfaces::msg::Time & stamp) const
  {
    if (stamp.nanosec >= 1000000000u) {
      throw std::invalid_argument("stamp.nanosec must be less than one billion");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (sequence_ == 0 || (stamp.sec == 0 && stamp.nanosec == 0)) {
      return latest_locked();
    }

    const int64_t target = nanoseconds(stamp);
    int64_t minimum = std::numeric_limits<int64_t>::max();
    int64_t maximum = std::numeric_limits<int64_t>::min();
    int64_t nearest_distance = std::numeric_limits<int64_t>::max();
    const Sample * nearest = nullptr;
    for (const auto & sample : samples_) {
      if (!sample.message || sample.message->header.stamp.nanosec >= 1000000000u) {
        continue;
      }
      const int64_t time = nanoseconds(sample.message->header.stamp);
      minimum = std::min(minimum, time);
      maximum = std::max(maximum, time);
      const int64_t distance = time >= target ? time - target : target - time;
      if (distance < nearest_distance ||
        (distance == nearest_distance && nearest && sample.sequence > nearest->sequence))
      {
        nearest = &sample;
        nearest_distance = distance;
      }
    }
    if (!nearest || target < minimum || target > maximum) {
      throw std::out_of_range("Requested timestamp is outside retained history");
    }
    return *nearest;
  }

private:
  static int64_t nanoseconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
  }

  Sample latest_locked() const
  {
    return sequence_ == 0 ? Sample{} : samples_[(next_ + samples_.size() - 1) % samples_.size()];
  }

  mutable std::mutex mutex_;
  std::vector<Sample> samples_;
  size_t next_{0};
  uint64_t sequence_{0};
};
}  // namespace sensor_snapshot
#endif  // SENSOR_SNAPSHOT__MESSAGE_HISTORY_HPP_
