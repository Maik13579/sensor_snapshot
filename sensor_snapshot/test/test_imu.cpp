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

#include "component_fixture.hpp"
#include <limits>

namespace
{
using sensor_snapshot_test::Components;
using GetImu = sensor_snapshot_interfaces::srv::GetImu;
using Status = sensor_snapshot_interfaces::msg::SnapshotStatus;

sensor_msgs::msg::Imu measurement(int32_t seconds = 10)
{
  sensor_msgs::msg::Imu sample;
  sample.header.stamp.sec = seconds;
  sample.header.stamp.nanosec = 123456789;
  sample.header.frame_id = "imu_link";
  sample.orientation.x = .1;
  sample.orientation.y = -.2;
  sample.orientation.z = .3;
  sample.orientation.w = .9;
  sample.angular_velocity.x = .01;
  sample.angular_velocity.y = -.02;
  sample.angular_velocity.z = .03;
  sample.linear_acceleration.x = .2;
  sample.linear_acceleration.y = -.4;
  sample.linear_acceleration.z = 9.7;
  for(size_t i = 0; i < 9; ++i) {
    sample.orientation_covariance[i] = .001 * (i + 1);
    sample.angular_velocity_covariance[i] = .002 * (i + 1);
    sample.linear_acceleration_covariance[i] = .003 * (i + 1);
  }
  // A raw IMU can have no orientation estimate. The marker must stay untouched.
  sample.orientation_covariance[0] = -1;
  return sample;
}

TEST_F(Components, ImuReportsNoDataAndRejectsInvalidAgeAndTimestamp)
{
  EXPECT_EQ(call<GetImu>("imu")->status.code, Status::NO_DATA);
  for(double age : {-1., std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    GetImu::Request request;
    request.max_age_seconds = age;
    EXPECT_EQ(call<GetImu>("imu", request)->status.code, Status::INVALID_REQUEST);
  }
  GetImu::Request request;
  request.stamp.nanosec = 1000000000;
  EXPECT_EQ(call<GetImu>("imu", request)->status.code, Status::INVALID_REQUEST);
  EXPECT_EQ(call<GetImu>("imu")->status.code, Status::NO_DATA);
}

TEST_F(Components, ImuPreservesEveryFieldAndRepeatedRequestsDoNotConsumeTheSample)
{
  auto pub = publisher<sensor_msgs::msg::Imu>("imu");
  const auto original = measurement();
  pub->publish(std::make_unique<sensor_msgs::msg::Imu>(original));
  const auto first = wait_for_sample<GetImu>("imu");
  ASSERT_EQ(first->status.code, Status::OK);
  EXPECT_EQ(first->imu, original);
  EXPECT_EQ(first->sequence, 1u);
  const auto repeated = call<GetImu>("imu");
  EXPECT_EQ(repeated->imu, original);
  EXPECT_EQ(repeated->sequence, first->sequence);
  EXPECT_GE(repeated->age_seconds, first->age_seconds);

  GetImu::Request request;
  request.max_age_seconds = 1e-12;
  const auto stale = call<GetImu>("imu", request);
  EXPECT_EQ(stale->status.code, Status::STALE);
  EXPECT_EQ(stale->sequence, first->sequence);
  EXPECT_EQ(stale->imu, sensor_msgs::msg::Imu());
  EXPECT_GT(stale->age_seconds, 0.);

  auto next = measurement(20);
  next.linear_acceleration.x = 42;
  pub->publish(std::make_unique<sensor_msgs::msg::Imu>(next));
  const auto updated = wait_for_sample<GetImu>("imu", 2);
  ASSERT_EQ(updated->status.code, Status::OK);
  EXPECT_EQ(updated->imu, next);
  EXPECT_EQ(first->imu, original);
}

TEST_F(Components, ImuHistorySelectsSensorTimeAndRejectsEvictedOrOutOfRangeSamples)
{
  auto pub = publisher<sensor_msgs::msg::Imu>("imu");
  for(uint64_t sequence = 1; sequence <= 4; ++sequence) {
    pub->publish(std::make_unique<sensor_msgs::msg::Imu>(measurement(sequence * 10)));
    ASSERT_EQ(wait_for_sample<GetImu>("imu", sequence)->status.code, Status::OK);
  }
  GetImu::Request request;
  request.stamp = measurement(20).header.stamp;
  EXPECT_EQ(call<GetImu>("imu", request)->imu, measurement(20));
  request.stamp = measurement(25).header.stamp;
  const auto nearest = call<GetImu>("imu", request);
  ASSERT_EQ(nearest->status.code, Status::OK);
  EXPECT_EQ(nearest->sequence, 3u);
  EXPECT_EQ(nearest->imu, measurement(30));
  for(int32_t seconds : {10, 50}) {
    request.stamp = measurement(seconds).header.stamp;
    const auto missing = call<GetImu>("imu", request);
    EXPECT_EQ(missing->status.code, Status::NOT_FOUND);
    EXPECT_EQ(missing->imu, sensor_msgs::msg::Imu());
  }
  EXPECT_EQ(call<GetImu>("imu")->imu, measurement(40));
}

TEST_F(Components, ImuServiceFollowsRemappingAndUsesDefaultHistoryAndAgeSettings)
{
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  options.arguments({"--ros-args", "-r", "__ns:=" + node_namespace_, "-r", "imu:=rear/imu"});
  options.append_parameter_override("cache.max_age_seconds", 1e-12);
  auto instance = std::make_shared<sensor_snapshot::ImuSnapshot>(options);
  executor_->add_node(instance);
  additional_nodes_.push_back(instance);
  const auto service = node_namespace_ + "/rear/imu";
  EXPECT_EQ(call<GetImu>(service)->status.code, Status::NO_DATA);
  auto pub = publisher<sensor_msgs::msg::Imu>("rear/imu");
  pub->publish(std::make_unique<sensor_msgs::msg::Imu>(measurement(10)));
  EXPECT_EQ(wait_for_sample<GetImu>(service)->status.code, Status::STALE);
  pub->publish(std::make_unique<sensor_msgs::msg::Imu>(measurement(20)));
  EXPECT_EQ(wait_for_sample<GetImu>(service, 2)->status.code, Status::STALE);
  GetImu::Request request;
  request.max_age_seconds = 3600;
  const auto latest = call<GetImu>(service, request);
  ASSERT_EQ(latest->status.code, Status::OK);
  EXPECT_EQ(latest->imu, measurement(20));
  request.stamp = measurement(10).header.stamp;
  EXPECT_EQ(call<GetImu>(service, request)->status.code, Status::NOT_FOUND);
}
}  // namespace
