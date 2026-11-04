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

#include <algorithm>
#include <array>
#include <cstring>
#include <limits>
#include <memory>
#include <vector>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_snapshot_interfaces/msg/snapshot_status.hpp>
#include "sensor_snapshot/message_history.hpp"
#include "sensor_snapshot/processing.hpp"

namespace
{
using Filter = sensor_snapshot_interfaces::msg::FieldFilter;
using Region = sensor_snapshot_interfaces::msg::Region;
using Status = sensor_snapshot_interfaces::msg::SnapshotStatus;
using Field = sensor_msgs::msg::PointField;

Region rectangle(uint32_t x, uint32_t y, uint32_t width, uint32_t height)
{
  Region region;
  region.x_offset = x;
  region.y_offset = y;
  region.width = width;
  region.height = height;
  return region;
}

Filter condition(const std::string & field, uint8_t comparison, double value, uint32_t index = 0)
{
  Filter filter;
  filter.field = field;
  filter.comparison = comparison;
  filter.value = value;
  filter.element_index = index;
  return filter;
}

Field field(const std::string & name, uint32_t offset, uint8_t datatype, uint32_t count = 1)
{
  Field result;
  result.name = name;
  result.offset = offset;
  result.datatype = datatype;
  result.count = count;
  return result;
}

template<typename T>
void write_number(uint8_t * destination, T value, bool bigendian)
{
  std::array<uint8_t, sizeof(T)> bytes;
  std::memcpy(bytes.data(), &value, sizeof(T));
  const uint16_t probe = 1;
  if (bigendian != (*reinterpret_cast<const uint8_t *>(&probe) == 0)) {
    std::reverse(bytes.begin(), bytes.end());
  }
  std::memcpy(destination, bytes.data(), sizeof(T));
}

sensor_msgs::msg::PointCloud2 make_cloud(bool bigendian = false)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "sensor";
  cloud.header.stamp.sec = 123;
  cloud.width = 3;
  cloud.height = 2;
  cloud.point_step = 24;
  cloud.row_step = cloud.width * cloud.point_step + 5;
  cloud.is_bigendian = bigendian;
  cloud.is_dense = true;
  cloud.fields = {
    field("x", 0, Field::FLOAT32), field("intensity", 4, Field::UINT16),
    field("ring", 6, Field::INT8), field("features", 7, Field::FLOAT64, 2)};
  cloud.data.resize(cloud.row_step * cloud.height, 0xab);
  for (uint32_t y = 0; y < cloud.height; ++y) {
    for (uint32_t x = 0; x < cloud.width; ++x) {
      const auto index = y * cloud.width + x;
      auto * point = cloud.data.data() + y * cloud.row_step + x * cloud.point_step;
      write_number(point, static_cast<float>(index), bigendian);
      write_number(point + 4, static_cast<uint16_t>(index * 10), bigendian);
      write_number(point + 6, static_cast<int8_t>(static_cast<int>(index) - 2), bigendian);
      write_number(point + 7, static_cast<double>(index) + 0.25, bigendian);
      write_number(point + 15, static_cast<double>(index) + 0.75, bigendian);
    }
  }
  return cloud;
}

template<typename FunctionT>
void expect_status(FunctionT function, uint8_t expected)
{
  try {
    function();
    FAIL() << "Expected a SnapshotError";
  } catch (const sensor_snapshot::SnapshotError & error) {
    EXPECT_EQ(error.status_code(), expected) << error.what();
  }
}

TEST(Region, DefaultsOffsetsAndOverflow)
{
  const auto full = sensor_snapshot::resolve_region(Region{}, 640, 480);
  EXPECT_EQ(full.width, 640u);
  EXPECT_EQ(full.height, 480u);
  const auto remainder = sensor_snapshot::resolve_region(rectangle(10, 20, 0, 0), 640, 480);
  EXPECT_EQ(remainder.width, 630u);
  EXPECT_EQ(remainder.height, 460u);
  expect_status([]() {
      sensor_snapshot::resolve_region(rectangle(1, 0, UINT32_MAX, 1), 640, 480);
  }, Status::INVALID_REQUEST);
  expect_status([]() {
      sensor_snapshot::resolve_region(rectangle(640, 0, 0, 0), 640, 480);
  }, Status::INVALID_REQUEST);
  EXPECT_EQ(sensor_snapshot::resolve_region(Region{}, 0, 1).width, 0u);
}

TEST(Cache, RetainsPointerAndOldReadersAcrossReplacement)
{
  sensor_snapshot::MessageHistory<sensor_msgs::msg::Image> cache;
  EXPECT_FALSE(cache.latest().message);
  auto first = std::make_shared<sensor_msgs::msg::Image>();
  first->data = {1, 2, 3};
  cache.store(first);
  const auto reader = cache.latest();
  EXPECT_EQ(reader.message.get(), first.get());
  EXPECT_EQ(reader.sequence, 1u);
  cache.store(std::make_shared<sensor_msgs::msg::Image>());
  EXPECT_EQ(cache.latest().sequence, 2u);
  EXPECT_EQ(reader.message->data, (std::vector<uint8_t>{1, 2, 3}));
}

TEST(Cache, ReleasesReplacedMessageOutsideTheLock)
{
  sensor_snapshot::MessageHistory<sensor_msgs::msg::Image> cache;
  bool deleted = false;
  auto old = std::shared_ptr<const sensor_msgs::msg::Image>(
    new sensor_msgs::msg::Image(), [&](const auto * message) {
      // Re-entering the cache would deadlock if destruction held its mutex.
      EXPECT_EQ(cache.latest().sequence, 2u);
      deleted = true;
      delete message;
    });
  cache.store(std::move(old));
  cache.store(std::make_shared<sensor_msgs::msg::Image>());
  EXPECT_TRUE(deleted);
}

sensor_msgs::msg::Image make_image()
{
  sensor_msgs::msg::Image image;
  image.width = 3;
  image.height = 2;
  image.encoding = "16UC1";
  image.is_bigendian = true;
  image.step = 8;  // Six pixel bytes plus two padding bytes per row.
  image.header.frame_id = "camera";
  image.header.stamp.sec = 42;
  image.data = {0, 1, 0, 2, 0, 3, 99, 99, 0, 4, 0, 5, 0, 6, 99, 99};
  return image;
}

TEST(Image, CropsPaddedDepthRowsWithoutConvertingBytes)
{
  const auto source = make_image();
  const auto crop = sensor_snapshot::crop_image(source, rectangle(1, 0, 2, 2));
  EXPECT_EQ(crop.width, 2u);
  EXPECT_EQ(crop.height, 2u);
  EXPECT_EQ(crop.step, 4u);
  EXPECT_TRUE(crop.is_bigendian);
  EXPECT_EQ(crop.header, source.header);
  EXPECT_EQ(crop.encoding, source.encoding);
  EXPECT_EQ(crop.data, (std::vector<uint8_t>{0, 2, 0, 3, 0, 5, 0, 6}));
  EXPECT_EQ(sensor_snapshot::crop_image(source, Region{}), source);
}

TEST(Image, CropsMultichannelAndChecksMalformedBuffers)
{
  auto source = make_image();
  source.width = 2;
  source.encoding = "rgb8";
  const auto crop = sensor_snapshot::crop_image(source, rectangle(1, 0, 1, 2));
  EXPECT_EQ(crop.step, 3u);
  EXPECT_EQ(crop.data, (std::vector<uint8_t>{2, 0, 3, 5, 0, 6}));
  source.data.pop_back();
  expect_status([&]() {sensor_snapshot::crop_image(source, Region{});}, Status::INVALID_DATA);
  source = make_image();
  source.step = 1;
  expect_status([&]() {
      sensor_snapshot::crop_image(source, rectangle(0, 0, 1, 1));
  }, Status::INVALID_DATA);
}

TEST(Image, RejectsUnsupportedCropsButAllowsOpaqueFullSnapshots)
{
  for (const auto & encoding : {"unknown", "yuv422", "nv21"}) {
    auto source = make_image();
    source.encoding = encoding;
    EXPECT_EQ(sensor_snapshot::crop_image(source, Region{}), source);
    expect_status([&]() {
        sensor_snapshot::crop_image(source, rectangle(0, 0, 1, 1));
    }, Status::UNSUPPORTED);
  }
  auto bayer = make_image();
  bayer.encoding = "bayer_rggb16";
  expect_status([&]() {
      sensor_snapshot::crop_image(bayer, rectangle(1, 0, 1, 1));
  }, Status::UNSUPPORTED);
  EXPECT_EQ(sensor_snapshot::crop_image(bayer, rectangle(2, 0, 1, 1)).width, 1u);
}

TEST(Cloud, PreservesFullCloudAndOrganizedCropIncludingCustomFields)
{
  const auto source = make_cloud();
  EXPECT_EQ(sensor_snapshot::filter_point_cloud(source, Region{}, {}), source);
  const auto crop = sensor_snapshot::filter_point_cloud(source, rectangle(1, 0, 2, 2), {});
  EXPECT_EQ(crop.width, 2u);
  EXPECT_EQ(crop.height, 2u);
  EXPECT_EQ(crop.row_step, 48u);
  EXPECT_EQ(crop.fields, source.fields);
  EXPECT_EQ(crop.header, source.header);
  std::vector<uint8_t> expected;
  for (size_t row = 0; row < 2; ++row) {
    const auto begin = source.data.begin() + row * source.row_step + source.point_step;
    expected.insert(expected.end(), begin, begin + 2 * source.point_step);
  }
  EXPECT_EQ(crop.data, expected);
}

TEST(Cloud, AppliesAndFiltersOnlyInsideRectangleForBothByteOrders)
{
  for (const bool bigendian : {false, true}) {
    const auto source = make_cloud(bigendian);
    const auto output = sensor_snapshot::filter_point_cloud(
      source, rectangle(1, 0, 2, 2),
      {condition("x", Filter::GREATER, 1), condition("intensity", Filter::LESS_OR_EQUAL, 40),
        condition("ring", Filter::GREATER_OR_EQUAL, 0)});
    // ROI contains indices 1, 2, 4, 5. Only indices 2 and 4 pass all comparisons.
    EXPECT_EQ(output.width, 2u);
    EXPECT_EQ(output.height, 1u);
    EXPECT_EQ(output.fields, source.fields);
    EXPECT_EQ(output.is_bigendian, bigendian);
    std::vector<uint8_t> expected;
    for (const size_t offset : {2 * source.point_step, source.row_step + source.point_step}) {
      expected.insert(expected.end(), source.data.begin() + offset,
        source.data.begin() + offset + source.point_step);
    }
    EXPECT_EQ(output.data, expected);
  }
}

TEST(Cloud, AllComparisonOperatorsAndArrayElements)
{
  const auto source = make_cloud();
  const std::array<uint32_t, 6> counts{2, 3, 1, 5, 4, 3};
  for (uint8_t comparison = 0; comparison < counts.size(); ++comparison) {
    const auto output = sensor_snapshot::filter_point_cloud(
      source, Region{}, {condition("x", comparison, 2.0)});
    EXPECT_EQ(output.width, counts[comparison]);
  }
  const auto output = sensor_snapshot::filter_point_cloud(
    source, Region{}, {condition("features", Filter::EQUAL, 3.75, 1)});
  EXPECT_EQ(output.width, 1u);
}

template<typename T>
void check_numeric_type(uint8_t datatype)
{
  for (const bool bigendian : {false, true}) {
    sensor_msgs::msg::PointCloud2 source;
    source.width = 2;
    source.height = 1;
    source.point_step = 2 * sizeof(T) + 2;
    source.row_step = 2 * source.point_step;
    source.is_bigendian = bigendian;
    source.fields = {field("value", 1, datatype, 2)};  // Deliberately unaligned.
    source.data.resize(source.row_step);
    write_number(source.data.data() + 1, static_cast<T>(1), bigendian);
    write_number(source.data.data() + 1 + sizeof(T), static_cast<T>(9), bigendian);
    write_number(source.data.data() + source.point_step + 1, static_cast<T>(2), bigendian);
    write_number(source.data.data() + source.point_step + 1 + sizeof(T), static_cast<T>(8),
        bigendian);
    const auto output = sensor_snapshot::filter_point_cloud(
      source, Region{}, {condition("value", Filter::GREATER, 8, 1)});
    EXPECT_EQ(output.width, 1u);
    EXPECT_EQ(output.data,
        std::vector<uint8_t>(source.data.begin(), source.data.begin() + source.point_step));
  }
}

TEST(Cloud, SupportsAllEightNumericTypesIncludingUnalignedFields)
{
  check_numeric_type<int8_t>(Field::INT8);
  check_numeric_type<uint8_t>(Field::UINT8);
  check_numeric_type<int16_t>(Field::INT16);
  check_numeric_type<uint16_t>(Field::UINT16);
  check_numeric_type<int32_t>(Field::INT32);
  check_numeric_type<uint32_t>(Field::UINT32);
  check_numeric_type<float>(Field::FLOAT32);
  check_numeric_type<double>(Field::FLOAT64);
}

TEST(Cloud, RejectsNonFiniteFieldValuesEvenForNotEqual)
{
  auto source = make_cloud();
  write_number(source.data.data(), std::numeric_limits<float>::quiet_NaN(), false);
  write_number(source.data.data() + source.point_step, std::numeric_limits<float>::infinity(),
      false);
  source.is_dense = false;
  const auto output = sensor_snapshot::filter_point_cloud(
    source, Region{}, {condition("x", Filter::NOT_EQUAL, -100)});
  EXPECT_EQ(output.width, 4u);
  EXPECT_FALSE(output.is_dense);
}

TEST(Cloud, EmptyInputsAndNoMatchesAreSuccessful)
{
  auto source = make_cloud();
  const auto none = sensor_snapshot::filter_point_cloud(
    source, Region{}, {condition("x", Filter::LESS, -100)});
  EXPECT_EQ(none.width, 0u);
  EXPECT_EQ(none.height, 1u);
  EXPECT_EQ(none.row_step, 0u);
  EXPECT_TRUE(none.data.empty());
  source.width = 0;
  source.height = 1;
  source.row_step = 0;
  source.data.clear();
  EXPECT_EQ(sensor_snapshot::filter_point_cloud(source, Region{}, {}), source);
  EXPECT_TRUE(sensor_snapshot::filter_point_cloud(
      source, Region{}, {condition("x", Filter::GREATER, 0)}).data.empty());
}

TEST(Cloud, InvalidRequestsAndMalformedLayoutsHaveExplicitStatuses)
{
  auto source = make_cloud();
  for (const auto & invalid : {
      condition("missing", Filter::EQUAL, 0), condition("x", 255, 0),
      condition("x", Filter::EQUAL, 0, 1),
      condition("x", Filter::EQUAL, std::numeric_limits<double>::quiet_NaN())})
  {
    expect_status([&]() {
        sensor_snapshot::filter_point_cloud(source, Region{}, {invalid});
    }, Status::INVALID_REQUEST);
  }
  source.data.pop_back();
  expect_status([&]() {
      sensor_snapshot::filter_point_cloud(source, Region{}, {});
      }, Status::INVALID_DATA);
  source = make_cloud();
  source.fields[0].offset = source.point_step;
  expect_status([&]() {
      sensor_snapshot::filter_point_cloud(source, Region{}, {});
      }, Status::INVALID_DATA);
  source = make_cloud();
  source.fields[0].name = source.fields[1].name;
  expect_status([&]() {
      sensor_snapshot::filter_point_cloud(source, Region{}, {});
      }, Status::INVALID_DATA);
}
}  // namespace
