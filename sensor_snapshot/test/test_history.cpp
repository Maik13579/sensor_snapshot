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

#include <gtest/gtest.h>

#include <memory>
#include <thread>
#include <atomic>
#include <sensor_msgs/msg/image.hpp>
#include "sensor_snapshot/message_history.hpp"

namespace
{
using History = sensor_snapshot::MessageHistory<sensor_msgs::msg::Image>;

builtin_interfaces::msg::Time stamp(int32_t seconds, uint32_t nanoseconds = 0)
{
  builtin_interfaces::msg::Time value;
  value.sec = seconds;
  value.nanosec = nanoseconds;
  return value;
}

std::shared_ptr<sensor_msgs::msg::Image> message(int32_t seconds, uint32_t nanoseconds = 0)
{
  auto result = std::make_shared<sensor_msgs::msg::Image>();
  result->header.stamp = stamp(seconds, nanoseconds);
  return result;
}

TEST(History, ZeroStampKeepsLatestBehaviorAndCapacityIsBounded)
{
  History history(2);
  EXPECT_FALSE(history.select(stamp(0)).message);
  auto first = message(1);
  auto second = message(2);
  auto third = message(3);
  history.store(first);
  const auto reader = history.latest();
  history.store(second);
  history.store(third);
  EXPECT_EQ(history.select(stamp(0)).message.get(), third.get());
  EXPECT_EQ(history.select(stamp(2)).message.get(), second.get());
  EXPECT_EQ(history.latest().sequence, 3u);
  EXPECT_THROW(history.select(stamp(1)), std::out_of_range);
  EXPECT_EQ(reader.message.get(), first.get());  // Reader outlives ring eviction.
  EXPECT_THROW(history.set_capacity(3), std::logic_error);
}

TEST(History, NearestSelectionHandlesOutOfOrderStampsAndTies)
{
  History history(4);
  auto later_stamp = message(12);
  auto earlier_stamp = message(10);
  history.store(later_stamp);
  history.store(earlier_stamp);
  // Equidistant stamps prefer the most recently received sample, not the latest stamp.
  EXPECT_EQ(history.select(stamp(11)).message.get(), earlier_stamp.get());
  EXPECT_EQ(history.select(stamp(11, 1)).message.get(), later_stamp.get());
  auto duplicate = message(10);
  history.store(duplicate);
  EXPECT_EQ(history.select(stamp(10)).message.get(), duplicate.get());
  EXPECT_THROW(history.select(stamp(9)), std::out_of_range);
  EXPECT_THROW(history.select(stamp(13)), std::out_of_range);
}

TEST(History, NanosecondsSignedTimeAndInvalidStamps)
{
  History history(3);
  history.store(message(-1, 900000000));
  history.store(message(0, 100000000));
  EXPECT_EQ(history.select(stamp(0, 1)).sequence, 2u);
  EXPECT_EQ(history.select(stamp(-1, 999999999)).sequence, 1u);
  EXPECT_THROW(history.select(stamp(1, 1000000000)), std::invalid_argument);
  EXPECT_THROW(History(0), std::invalid_argument);
  History configurable;
  EXPECT_THROW(configurable.set_capacity(0), std::invalid_argument);
  configurable.set_capacity(3);
  configurable.store(message(1));
  configurable.store(message(2));
  EXPECT_EQ(configurable.select(stamp(1)).sequence, 1u);
}

TEST(History, StoredInvalidStampsDoNotParticipateInTimestampLookup)
{
  History history(3);
  history.store(message(1));
  history.store(message(2, 1000000000));
  history.store(message(3));
  EXPECT_EQ(history.select(stamp(2)).sequence, 3u);
  EXPECT_EQ(history.latest().sequence, 3u);
}

TEST(History, ConcurrentReadersAndWriterKeepSnapshotsConsistent)
{
  History history(8);
  std::atomic<bool> finished{false};
  std::atomic<bool> consistent{true};
  std::thread writer([&]() {
      for (int32_t sequence = 1; sequence <= 1000; ++sequence) {
        history.store(message(sequence));
      }
      finished = true;
    });
  do {
    const auto sample = history.latest();
    if (sample.message &&
      sample.sequence != static_cast<uint64_t>(sample.message->header.stamp.sec))
    {
      consistent = false;
    }
  } while (!finished);
  writer.join();
  EXPECT_TRUE(consistent);
  EXPECT_EQ(history.latest().sequence, 1000u);
}
}  // namespace
