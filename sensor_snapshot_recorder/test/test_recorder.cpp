// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0
#include "recorder_fixture.hpp"
#include <rclcpp/serialization.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

namespace sensor_snapshot_recorder_test
{
template<typename Message>
Message decode(const rosbag2_storage::SerializedBagMessageSharedPtr & record)
{
  rclcpp::SerializedMessage serialized(*record->serialized_data);
  Message message;
  rclcpp::Serialization<Message>().deserialize_message(&serialized, &message);
  return message;
}

TEST_F(Recorder, ParallelRequestsKeepHeadersAndUseTheTriggerClockForEveryBagRecord)
{
  providers();hold_ = true;
  auto opts = options();
  opts.append_parameter_override("use_sim_time", true);
  opts.append_parameter_override("capture.timeout_seconds", 3.0);
  opts.append_parameter_override("requests." + ns_ + "/cloud.voxel_size", 0.05);
  opts.append_parameter_override("requests." + ns_ + "/cloud.fields", std::vector<std::string>{"x"});
  opts.append_parameter_override("requests." + ns_ + "/scan.max_age_seconds", 0.75);
  start(opts);
  auto clock = helper_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
  ASSERT_TRUE(until([&] {return clock->get_subscription_count() != 0;}));
  rosgraph_msgs::msg::Clock tick;tick.clock.sec = 123;tick.clock.nanosec = 456;
  clock->publish(tick);
  ASSERT_TRUE(until([&] {return recorder_->now().nanoseconds() == 123000000456LL;}));
  auto first = trigger();
  // None of the providers responds until every request arrived. This proves the
  // recorder does not sequentially wait for a service, even on one executor thread.
  ASSERT_TRUE(until([&] {return seen_ == 4;}));
  tick.clock.sec = 999;clock->publish(tick);
  ASSERT_TRUE(until([&] {return recorder_->now().seconds() > 900;}));
  release();
  ASSERT_EQ(first.wait_for(3s), std::future_status::ready);
  ASSERT_TRUE(first.get()->success);
  EXPECT_DOUBLE_EQ(voxel_seen_, 0.05);
  EXPECT_EQ(fields_seen_, std::vector<std::string>{"x"});
  EXPECT_DOUBLE_EQ(scan_age_seen_, 0.75);
  close();
  const auto records = messages();ASSERT_EQ(records.size(), 4u);
  for (const auto & message : records) {EXPECT_EQ(bag_stamp(*message, 0), 123000000456LL);}
  const auto image = decode<sensor_msgs::msg::Image>(records[0]);
  EXPECT_EQ(image.header.stamp.sec, 11);EXPECT_EQ(image.header.frame_id, "camera");
  EXPECT_EQ(image.data, (std::vector<uint8_t>{1, 2, 3}));
  const auto cloud = decode<sensor_msgs::msg::PointCloud2>(records[1]);
  EXPECT_EQ(cloud.header.stamp.sec, 22);EXPECT_EQ(cloud.data, (std::vector<uint8_t>{0, 0, 128, 63}));
  const auto imu = decode<sensor_msgs::msg::Imu>(records[2]);
  EXPECT_EQ(imu.header.stamp.sec, 33);EXPECT_DOUBLE_EQ(imu.linear_acceleration.z, 9.81);
  EXPECT_DOUBLE_EQ(imu.orientation_covariance[0], -1.0);
  const auto scan = decode<sensor_msgs::msg::LaserScan>(records[3]);
  EXPECT_EQ(scan.header.stamp.sec, 44);EXPECT_EQ(scan.header.frame_id, "scan_laser");
  EXPECT_FLOAT_EQ(scan.angle_increment, 0.5f);EXPECT_FLOAT_EQ(scan.time_increment, 0.001f);
  EXPECT_EQ(scan.ranges, (std::vector<float>{1.0f, 2.0f, 3.0f}));
  EXPECT_EQ(scan.intensities, (std::vector<float>{4.0f, 5.0f, 6.0f}));
}

TEST_F(Recorder, RepeatedTriggersAppendFramesWithCachedRequestOptions)
{
  providers();
  auto opts = options();
  opts.append_parameter_override("requests." + ns_ + "/cloud.voxel_size", 0.05);
  start(opts);
  ASSERT_TRUE(capture()->success);ASSERT_TRUE(capture()->success);
  EXPECT_EQ(seen_, 8u);EXPECT_DOUBLE_EQ(voxel_seen_, 0.05);
  EXPECT_FALSE(recorder_->set_parameter(
    rclcpp::Parameter("requests." + ns_ + "/cloud.voxel_size", 0.1)).successful);
  close();EXPECT_EQ(messages().size(), 8u);
}

TEST_F(Recorder, FailedResponseDoesNotWriteAPartialKeyframe)
{
  providers();bad_ = true;start(options());
  const auto failed = capture();EXPECT_FALSE(failed->success);
  EXPECT_EQ(failed->message, ns_ + "/imu: status 2: stale IMU; ");
  bad_ = false;ASSERT_TRUE(capture()->success);
  close();EXPECT_EQ(messages().size(), 4u);
}

TEST_F(Recorder, TimeoutRejectsOverlapCleansPendingCallsAndDiscardsLateReplies)
{
  providers();hold_ = true;start(options());
  auto first = trigger();ASSERT_TRUE(until([&] {return seen_ == 4;}));
  const auto overlapping = capture();EXPECT_FALSE(overlapping->success);
  EXPECT_NE(overlapping->message.find("already in progress"), std::string::npos);
  ASSERT_EQ(first.wait_for(3s), std::future_status::ready);
  EXPECT_FALSE(first.get()->success);
  release();hold_ = false;
  ASSERT_TRUE(capture()->success);
  close();EXPECT_EQ(messages().size(), 4u);
}

TEST_F(Recorder, MissingServiceDoesNotRequestOrWriteOtherTopics)
{
  providers();start(options({ns_ + "/image", ns_ + "/missing"}));
  const auto failed = capture();EXPECT_FALSE(failed->success);
  EXPECT_NE(failed->message.find("missing"), std::string::npos);EXPECT_EQ(seen_, 0u);
  close();EXPECT_TRUE(messages().empty());
}

TEST_F(Recorder, ServicesCanAppearAfterRecorderStartup)
{
  start(options());EXPECT_FALSE(capture()->success);
  providers();
  Trigger::Response::SharedPtr result;
  ASSERT_TRUE(until([&] {result = capture();return result->success;}));
  close();EXPECT_EQ(messages().size(), 4u);
}

TEST_F(Recorder, RequestOptionsIncludeImageRegionsAndNamedCloudFilters)
{
  auto node = std::make_shared<rclcpp::Node>("request_options", rclcpp::NodeOptions()
    .append_parameter_override("requests./cloud.fields", std::vector<std::string>{"x", "y", "z"})
    .append_parameter_override("requests./cloud.voxel_size", 0.05)
    .append_parameter_override("requests./cloud.filters.height.field", "z")
    .append_parameter_override("requests./cloud.filters.height.comparison", "LESS_OR_EQUAL")
    .append_parameter_override("requests./cloud.filters.height.value", 2.0)
    .append_parameter_override("requests./image.region.x_offset", int64_t(10))
    .append_parameter_override("requests./image.downsample_factor", int64_t(2)));
  auto cloud = cloud_request(*node, "/cloud");
  ASSERT_EQ(cloud->filters.size(), 1u);EXPECT_EQ(cloud->filters[0].field, "z");
  EXPECT_EQ(cloud->filters[0].comparison, 1);EXPECT_DOUBLE_EQ(cloud->voxel_size, 0.05);
  auto image = image_request(*node, "/image");
  EXPECT_EQ(image->region.x_offset, 10u);EXPECT_EQ(image->downsample_factor, 2u);
  EXPECT_EQ(image->stamp.sec, 0);EXPECT_EQ(cloud->stamp.sec, 0);
}

TEST_F(Recorder, InvalidConfigurationAndExistingOutputAreRejected)
{
  EXPECT_THROW(std::make_shared<SnapshotRecorder>(), std::invalid_argument);
  auto opts = options({ns_ + "/imu", ns_ + "/imu"});
  EXPECT_THROW(std::make_shared<SnapshotRecorder>(opts), std::invalid_argument);
  std::filesystem::create_directories(directory_);
  EXPECT_THROW(std::make_shared<SnapshotRecorder>(options()), std::invalid_argument);
}

TEST_F(Recorder, CaptureNotificationRetainsRecordedPayloadsAfterStop)
{
  providers();std::promise<RecordedFrame> notification;
  auto notified = notification.get_future();
  start(options(), [&notification](RecordedFrame frame) {notification.set_value(std::move(frame));});
  ASSERT_TRUE(capture()->success);
  ASSERT_EQ(notified.wait_for(3s), std::future_status::ready);
  const auto frame = notified.get();ASSERT_EQ(frame.messages.size(), 4u);
  for (const auto & message : frame.messages) {
    ASSERT_GT(message.data->size(), 0u);
    EXPECT_GT(message.data->capacity(), 0u);
  }
  auto stop = helper_->create_client<Trigger>("sensor_snapshot_recorder/stop");
  ASSERT_TRUE(stop->wait_for_service(3s));
  auto stopped = stop->async_send_request(std::make_shared<Trigger::Request>());
  ASSERT_EQ(stopped.wait_for(3s), std::future_status::ready);EXPECT_TRUE(stopped.get()->success);
  EXPECT_TRUE(std::filesystem::exists(directory_ / "metadata.yaml"));
  EXPECT_FALSE(capture()->success);
  close();
  sensor_msgs::msg::Imu imu;
  rclcpp::Serialization<sensor_msgs::msg::Imu>().deserialize_message(frame.messages[2].data.get(), &imu);
  EXPECT_DOUBLE_EQ(imu.linear_acceleration.z, 9.81);
  EXPECT_EQ(frame.messages[2].topic, ns_ + "/imu");
  EXPECT_EQ(messages().size(), 4u);
}

TEST_F(Recorder, StopWaitsForPendingCaptureBeforeClosingTheWriter)
{
  providers();hold_ = true;start(options());
  auto capture = trigger();ASSERT_TRUE(until([this] {return seen_ == 4;}));
  auto stop = helper_->create_client<Trigger>("sensor_snapshot_recorder/stop");
  ASSERT_TRUE(stop->wait_for_service(3s));
  auto stopped = stop->async_send_request(std::make_shared<Trigger::Request>());
  EXPECT_EQ(stopped.wait_for(20ms), std::future_status::timeout);
  release();
  ASSERT_EQ(capture.wait_for(3s), std::future_status::ready);EXPECT_TRUE(capture.get()->success);
  ASSERT_EQ(stopped.wait_for(3s), std::future_status::ready);EXPECT_TRUE(stopped.get()->success);
  EXPECT_FALSE(this->capture()->success);
  close();EXPECT_EQ(messages().size(), 4u);
}

TEST_F(Recorder, FailedCaptureDoesNotSendNotification)
{
  providers();bad_ = true;std::atomic<unsigned> notified{0};
  start(options(), [&notified](RecordedFrame) {++notified;});
  EXPECT_FALSE(capture()->success);EXPECT_EQ(notified, 0u);
}
}  // namespace sensor_snapshot_recorder_test
