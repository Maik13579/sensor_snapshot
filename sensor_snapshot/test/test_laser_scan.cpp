// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0
#include "component_fixture.hpp"
#include <cmath>
#include <limits>

namespace
{
using sensor_snapshot_test::Components;
using GetScan = sensor_snapshot_interfaces::srv::GetLaserScan;
using Scan = sensor_msgs::msg::LaserScan;
using Status = sensor_snapshot_interfaces::msg::SnapshotStatus;

Scan measurement(int32_t seconds = 10)
{
  Scan scan;
  scan.header.stamp.sec = seconds;
  scan.header.stamp.nanosec = 123;
  scan.header.frame_id = "laser";
  scan.angle_min = -1.0f;
  scan.angle_max = 1.0f;
  scan.angle_increment = 0.5f;
  scan.time_increment = 0.001f;
  scan.scan_time = 0.1f;
  scan.range_min = 0.2f;
  scan.range_max = 30.0f;
  scan.ranges = {0.5f, 1.0f, std::numeric_limits<float>::infinity(), 2.0f, 3.0f};
  scan.intensities = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
  return scan;
}

TEST_F(Components, LaserScanPreservesBeamsTimingAndRepeatedSnapshots)
{
  auto publisher = this->publisher<Scan>("scan");
  const auto original = measurement();
  publisher->publish(std::make_unique<Scan>(original));
  const auto first = wait_for_sample<GetScan>("scan");
  ASSERT_EQ(first->status.code, Status::OK);
  EXPECT_EQ(first->scan, original);
  const auto repeated = call<GetScan>("scan");
  EXPECT_EQ(repeated->scan, original);
  EXPECT_EQ(repeated->sequence, first->sequence);

  // Missing returns and absent intensity data are valid raw measurements.
  auto next = measurement(20);
  next.ranges[0] = std::numeric_limits<float>::quiet_NaN();
  next.intensities.clear();
  publisher->publish(std::make_unique<Scan>(next));
  const auto updated = wait_for_sample<GetScan>("scan", 2);
  ASSERT_EQ(updated->status.code, Status::OK);
  EXPECT_TRUE(std::isnan(updated->scan.ranges[0]));
  EXPECT_TRUE(std::isinf(updated->scan.ranges[2]));
  EXPECT_TRUE(updated->scan.intensities.empty());
  EXPECT_EQ(updated->scan.header, next.header);
  EXPECT_EQ(first->scan, original);
}

TEST_F(Components, LaserScanRejectsInvalidRequestsAndLeavesFailedPayloadEmpty)
{
  EXPECT_EQ(call<GetScan>("scan")->status.code, Status::NO_DATA);
  for (double age : {-1.0, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    GetScan::Request request;
    request.max_age_seconds = age;
    EXPECT_EQ(call<GetScan>("scan", request)->status.code, Status::INVALID_REQUEST);
  }
  GetScan::Request request;
  request.stamp.nanosec = 1000000000;
  EXPECT_EQ(call<GetScan>("scan", request)->status.code, Status::INVALID_REQUEST);
  auto publisher = this->publisher<Scan>("scan");
  publisher->publish(std::make_unique<Scan>(measurement()));
  ASSERT_EQ(wait_for_sample<GetScan>("scan")->status.code, Status::OK);
  request = GetScan::Request();
  request.max_age_seconds = 1e-12;
  const auto stale = call<GetScan>("scan", request);
  EXPECT_EQ(stale->status.code, Status::STALE);
  EXPECT_EQ(stale->scan, Scan());
  EXPECT_EQ(stale->sequence, 1u);
}

TEST_F(Components, LaserScanSelectsHistoryAndRejectsEvictedSamples)
{
  auto publisher = this->publisher<Scan>("scan");
  for (uint64_t sequence = 1; sequence <= 4; ++sequence) {
    publisher->publish(std::make_unique<Scan>(measurement(sequence * 10)));
    ASSERT_EQ(wait_for_sample<GetScan>("scan", sequence)->status.code, Status::OK);
  }
  GetScan::Request request;
  request.stamp = measurement(20).header.stamp;
  EXPECT_EQ(call<GetScan>("scan", request)->scan, measurement(20));
  request.stamp = measurement(25).header.stamp;
  EXPECT_EQ(call<GetScan>("scan", request)->scan, measurement(30));
  for (int32_t seconds : {10, 50}) {
    request.stamp = measurement(seconds).header.stamp;
    const auto missing = call<GetScan>("scan", request);
    EXPECT_EQ(missing->status.code, Status::NOT_FOUND);
    EXPECT_EQ(missing->scan, Scan());
  }
}

TEST_F(Components, LaserScanServiceFollowsRemappingAndDefaultAge)
{
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  options.arguments({"--ros-args", "-r", "__ns:=" + node_namespace_, "-r", "scan:=rear/scan"});
  options.append_parameter_override("cache.max_age_seconds", 1e-12);
  auto instance = std::make_shared<sensor_snapshot::LaserScanSnapshot>(options);
  executor_->add_node(instance);
  additional_nodes_.push_back(instance);
  auto publisher = this->publisher<Scan>("rear/scan");
  publisher->publish(std::make_unique<Scan>(measurement()));
  EXPECT_EQ(wait_for_sample<GetScan>("rear/scan")->status.code, Status::STALE);
  GetScan::Request request;
  request.max_age_seconds = 3600;
  EXPECT_EQ(call<GetScan>("rear/scan", request)->scan, measurement());
}
}  // namespace
