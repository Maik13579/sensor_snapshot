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

#include <array>
#include <algorithm>
#include <cstring>
#include <limits>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_snapshot_interfaces/msg/snapshot_status.hpp>
#include "sensor_snapshot/processing.hpp"

namespace
{
using Field = sensor_msgs::msg::PointField;
using Filter = sensor_snapshot_interfaces::msg::FieldFilter;
using Region = sensor_snapshot_interfaces::msg::Region;
using Status = sensor_snapshot_interfaces::msg::SnapshotStatus;

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
void write(uint8_t * pointer, T value, bool bigendian)
{
  std::array<uint8_t, sizeof(T)> bytes;
  std::memcpy(bytes.data(), &value, sizeof(T));
  const uint16_t endian_probe = 1;
  const bool host_bigendian = *reinterpret_cast<const uint8_t *>(&endian_probe) == 0;
  if (bigendian != host_bigendian) {
    std::reverse(bytes.begin(), bytes.end());
  }
  std::memcpy(pointer, bytes.data(), bytes.size());
}

sensor_msgs::msg::PointCloud2 cloud(bool bigendian = false)
{
  sensor_msgs::msg::PointCloud2 result;
  result.width = 3;
  result.height = 3;
  result.point_step = 28;
  result.row_step = result.width * result.point_step + 5;
  result.is_bigendian = bigendian;
  result.is_dense = true;
  result.header.frame_id = "sensor";
  result.fields = {
    field("x", 0, Field::FLOAT32), field("y", 4, Field::FLOAT32),
    field("z", 8, Field::FLOAT32), field("intensity", 12, Field::UINT16),
    field("features", 14, Field::INT32, 2)};
  result.data.resize(result.row_step * result.height, 0xab);
  for (uint32_t row = 0; row < result.height; ++row) {
    for (uint32_t column = 0; column < result.width; ++column) {
      auto * point = result.data.data() + row * result.row_step + column * result.point_step;
      const auto index = row * result.width + column;
      write(point, static_cast<float>(column) * 0.1f, bigendian);
      write(point + 4, static_cast<float>(row) * 0.1f, bigendian);
      write(point + 8, 1.0f, bigendian);
      write(point + 12, static_cast<uint16_t>(index), bigendian);
      write(point + 14, static_cast<int32_t>(index), bigendian);
      write(point + 18, -static_cast<int32_t>(index), bigendian);
    }
  }
  return result;
}

Filter greater(const std::string & field_name, double threshold)
{
  Filter result;
  result.field = field_name;
  result.comparison = Filter::GREATER;
  result.value = threshold;
  return result;
}

template<typename FunctionT>
void expect_status(FunctionT function, uint8_t expected)
{
  try {
    function();
    FAIL() << "Expected SnapshotError";
  } catch (const sensor_snapshot::SnapshotError & error) {
    EXPECT_EQ(error.status_code(), expected) << error.what();
  }
}

TEST(Sampling, ImageStrideIsAnchoredAtCropOriginAndRoundsDimensionsUp)
{
  sensor_msgs::msg::Image source;
  source.width = 5;
  source.height = 3;
  source.step = 7;
  source.encoding = "mono8";
  source.data = {0, 1, 2, 3, 4, 99, 99, 5, 6, 7, 8, 9, 99, 99, 10, 11, 12, 13, 14, 99, 99};
  auto sampled = sensor_snapshot::crop_image(source, Region{}, 2);
  EXPECT_EQ(sampled.width, 3u);
  EXPECT_EQ(sampled.height, 2u);
  EXPECT_EQ(sampled.data, (std::vector<uint8_t>{0, 2, 4, 10, 12, 14}));
  Region region;
  region.x_offset = 1;
  region.width = 3;
  sampled = sensor_snapshot::crop_image(source, region, 2);
  EXPECT_EQ(sampled.data, (std::vector<uint8_t>{1, 3, 11, 13}));
  EXPECT_EQ(sensor_snapshot::crop_image(source, Region{}, UINT32_MAX).data,
    (std::vector<uint8_t>{0}));
  expect_status([&]() {sensor_snapshot::crop_image(source, Region{}, 0);}, Status::INVALID_REQUEST);
  source.encoding = "bayer_rggb8";
  expect_status([&]() {sensor_snapshot::crop_image(source, Region{}, 2);}, Status::UNSUPPORTED);
}

TEST(Sampling, ImageStridePreservesMultibytePixels)
{
  sensor_msgs::msg::Image source;
  source.width = 3;
  source.height = 1;
  source.step = 8;
  source.encoding = "16UC1";
  source.is_bigendian = true;
  source.data = {0, 1, 0, 2, 0, 3, 99, 99};
  const auto output = sensor_snapshot::crop_image(source, Region{}, 2);
  EXPECT_EQ(output.data, (std::vector<uint8_t>{0, 1, 0, 3}));
  EXPECT_TRUE(output.is_bigendian);
}

TEST(Sampling, CloudProjectionReordersPacksAndPreservesArrayBytes)
{
  for (const bool bigendian : {false, true}) {
    const auto source = cloud(bigendian);
    const auto output = sensor_snapshot::filter_point_cloud(
      source, Region{}, {}, {"features", "intensity"});
    EXPECT_EQ(output.width, source.width);
    EXPECT_EQ(output.height, source.height);
    EXPECT_EQ(output.fields.size(), 2u);
    EXPECT_EQ(output.fields[0].name, "features");
    EXPECT_EQ(output.fields[0].offset, 0u);
    EXPECT_EQ(output.fields[0].count, 2u);
    EXPECT_EQ(output.fields[1].offset, 8u);
    EXPECT_EQ(output.point_step, 10u);
    EXPECT_EQ(output.row_step, 30u);
    EXPECT_EQ(output.is_bigendian, bigendian);
    std::vector<uint8_t> expected;
    for (uint32_t row = 0; row < source.height; ++row) {
      for (uint32_t column = 0; column < source.width; ++column) {
        const auto point = source.data.begin() + row * source.row_step + column * source.point_step;
        expected.insert(expected.end(), point + 14, point + 22);
        expected.insert(expected.end(), point + 12, point + 14);
      }
    }
    EXPECT_EQ(output.data, expected);
  }
}

TEST(Sampling, CloudStrideProjectionAndFiltersUseTheOriginalRecord)
{
  const auto source = cloud();
  const auto output = sensor_snapshot::filter_point_cloud(
    source, Region{}, {greater("intensity", 3)}, {"features"}, 2);
  // Sampled indices are 0,2,6,8; a filter on an omitted field keeps only 6 and 8.
  EXPECT_EQ(output.width, 2u);
  EXPECT_EQ(output.height, 1u);
  EXPECT_EQ(output.point_step, 8u);
  std::vector<uint8_t> expected;
  for (const size_t column : {0u, 2u}) {
    const auto begin = source.data.begin() + 2 * source.row_step + column * source.point_step + 14;
    expected.insert(expected.end(), begin, begin + 8);
  }
  EXPECT_EQ(output.data, expected);
  const auto organized = sensor_snapshot::filter_point_cloud(source, Region{}, {}, {"intensity"},
      2);
  EXPECT_EQ(organized.width, 2u);
  EXPECT_EQ(organized.height, 2u);
  EXPECT_EQ(organized.data.size(), 8u);
  auto unorganized = source;
  unorganized.height = 1;
  unorganized.data.resize(unorganized.row_step);
  EXPECT_EQ(sensor_snapshot::filter_point_cloud(unorganized, Region{}, {}, {}, 2).width, 2u);
}

TEST(Sampling, InvalidSelectedFieldsAndStrideAreRejectedEvenForEmptyClouds)
{
  auto source = cloud();
  for (const auto & fields : {std::vector<std::string>{"missing"},
      std::vector<std::string>{"x", "x"}})
  {
    expect_status([&]() {
        sensor_snapshot::filter_point_cloud(source, Region{}, {}, fields);
    }, Status::INVALID_REQUEST);
  }
  expect_status([&]() {
      sensor_snapshot::filter_point_cloud(source, Region{}, {}, {}, 0);
  }, Status::INVALID_REQUEST);
  source.width = 0;
  source.row_step = 0;
  source.data.clear();
  const auto empty = sensor_snapshot::filter_point_cloud(source, Region{}, {}, {"intensity"}, 2);
  EXPECT_TRUE(empty.data.empty());
  EXPECT_EQ(empty.point_step, 2u);
  expect_status([&]() {
      sensor_snapshot::filter_point_cloud(source, Region{}, {}, {"missing"});
  }, Status::INVALID_REQUEST);
}

TEST(Voxel, KeepsTheFirstPassingOriginalPointAndCanOmitXYZ)
{
  for (const bool bigendian : {false, true}) {
    const auto source = cloud(bigendian);
    const auto output = sensor_snapshot::filter_point_cloud(
      source, Region{}, {greater("intensity", 0)}, {"intensity"}, 1, 1.0);
    EXPECT_EQ(output.width, 1u);
    EXPECT_EQ(output.height, 1u);
    EXPECT_EQ(output.point_step, 2u);
    std::array<uint8_t, 2> expected;
    write(expected.data(), static_cast<uint16_t>(1), bigendian);
    EXPECT_EQ(output.data, (std::vector<uint8_t>{expected[0], expected[1]}));
  }
}

TEST(Voxel, SeparatesNegativeCellsAndSkipsNonFiniteCoordinates)
{
  auto source = cloud();
  source.height = 1;
  source.data.resize(source.row_step);
  write(source.data.data(), -0.1f, false);
  write(source.data.data() + source.point_step, 0.1f, false);
  write(source.data.data() + 2 * source.point_step, std::numeric_limits<float>::quiet_NaN(), false);
  const auto output = sensor_snapshot::filter_point_cloud(source, Region{}, {}, {}, 1, 1.0);
  EXPECT_EQ(output.width, 2u);
  EXPECT_EQ(output.data.size(), 2 * source.point_step);
}

TEST(Voxel, ValidatesSizeCoordinateFieldsAndIndexOverflow)
{
  auto source = cloud();
  for (const double invalid : {-1.0, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), std::numeric_limits<double>::denorm_min()})
  {
    expect_status([&]() {
        sensor_snapshot::filter_point_cloud(source, Region{}, {}, {}, 1, invalid);
    }, Status::INVALID_REQUEST);
  }
  source.fields[0].name = "not_x";
  expect_status([&]() {
      sensor_snapshot::filter_point_cloud(source, Region{}, {}, {}, 1, 1.0);
  }, Status::INVALID_REQUEST);
}
}  // namespace
