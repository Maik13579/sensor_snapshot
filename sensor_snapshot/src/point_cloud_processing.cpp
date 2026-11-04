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

#include "sensor_snapshot/processing.hpp"
#include "point_cloud_region_iterator.hpp"
#include "sampling.hpp"
#include "voxel_filter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <unordered_set>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_snapshot_interfaces/msg/snapshot_status.hpp>

namespace sensor_snapshot
{
namespace
{
using Field = sensor_msgs::msg::PointField;
using Filter = sensor_snapshot_interfaces::msg::FieldFilter;
using Status = sensor_snapshot_interfaces::msg::SnapshotStatus;
using Reader = double (*)(const uint8_t *, bool);

size_t field_size(uint8_t datatype)
{
  switch (datatype) {
    case Field::INT8: case Field::UINT8: return 1;
    case Field::INT16: case Field::UINT16: return 2;
    case Field::INT32: case Field::UINT32: case Field::FLOAT32: return 4;
    case Field::FLOAT64: return 8;
    default: throw SnapshotError(Status::UNSUPPORTED, "Unsupported PointField datatype");
  }
}

void validate_cloud(const sensor_msgs::msg::PointCloud2 & cloud)
{
  const uint64_t row_bytes = static_cast<uint64_t>(cloud.width) * cloud.point_step;
  if ((cloud.width != 0 && (cloud.height == 0 || cloud.point_step == 0)) ||
    row_bytes > cloud.row_step ||
    static_cast<uint64_t>(cloud.row_step) * cloud.height > cloud.data.size())
  {
    throw SnapshotError(Status::INVALID_DATA, "Cloud dimensions/strides exceed its data buffer");
  }
  std::unordered_set<std::string> names;
  for (const auto & field : cloud.fields) {
    if (field.count == 0 || field.name.empty() || !names.insert(field.name).second ||
      static_cast<uint64_t>(field.offset) + field_size(field.datatype) * field.count >
      cloud.point_step)
    {
      throw SnapshotError(Status::INVALID_DATA, "Invalid/duplicate cloud field: " + field.name);
    }
  }
}

template<typename T>
double read_number(const uint8_t * data, bool swap_bytes)
{
  // memcpy avoids unaligned accesses and strict-aliasing violations in packed fields.
  std::array<uint8_t, sizeof(T)> bytes;
  std::memcpy(bytes.data(), data, sizeof(T));
  if (swap_bytes) {
    std::reverse(bytes.begin(), bytes.end());
  }
  T value;
  std::memcpy(&value, bytes.data(), sizeof(T));
  return static_cast<double>(value);
}

Reader reader_for(uint8_t datatype)
{
  switch (datatype) {
    case Field::INT8: return read_number<int8_t>;
    case Field::UINT8: return read_number<uint8_t>;
    case Field::INT16: return read_number<int16_t>;
    case Field::UINT16: return read_number<uint16_t>;
    case Field::INT32: return read_number<int32_t>;
    case Field::UINT32: return read_number<uint32_t>;
    case Field::FLOAT32: return read_number<float>;
    case Field::FLOAT64: return read_number<double>;
    default: throw SnapshotError(Status::UNSUPPORTED, "Unsupported filter field datatype");
  }
}

struct CompiledFilter
{
  size_t offset;
  Reader read;
  uint8_t comparison;
  double threshold;

  bool passes(const uint8_t * point, bool swap_bytes) const
  {
    const double value = read(point + offset, swap_bytes);
    if (!std::isfinite(value)) {
      return false;
    }
    switch (comparison) {
      case Filter::LESS: return value < threshold;
      case Filter::LESS_OR_EQUAL: return value <= threshold;
      case Filter::EQUAL: return value == threshold;
      case Filter::NOT_EQUAL: return value != threshold;
      case Filter::GREATER_OR_EQUAL: return value >= threshold;
      case Filter::GREATER: return value > threshold;
      default: return false;  // Rejected when compiling the request.
    }
  }
};

std::vector<CompiledFilter> compile_filters(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::vector<Filter> & filters)
{
  std::vector<CompiledFilter> compiled;
  compiled.reserve(filters.size());
  for (const auto & filter : filters) {
    const auto field = std::find_if(
      cloud.fields.begin(), cloud.fields.end(),
      [&filter](const Field & candidate) {return candidate.name == filter.field;});
    if (field == cloud.fields.end()) {
      throw SnapshotError(Status::INVALID_REQUEST, "Filter field not found: " + filter.field);
    }
    if (filter.comparison > Filter::GREATER || !std::isfinite(filter.value) ||
      filter.element_index >= field->count)
    {
      throw SnapshotError(Status::INVALID_REQUEST,
            "Invalid comparison/value/element for: " + filter.field);
    }
    compiled.push_back({
          field->offset + field_size(field->datatype) * filter.element_index,
          reader_for(field->datatype), filter.comparison, filter.value});
  }
  return compiled;
}

struct CopySpan
{
  size_t offset;
  size_t bytes;
};

struct Projection
{
  std::vector<Field> fields;
  std::vector<CopySpan> copies;
  uint32_t point_step{0};
};

const Field & find_field(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  const auto found = std::find_if(
    cloud.fields.begin(), cloud.fields.end(),
    [&name](const Field & field) {return field.name == name;});
  if (found == cloud.fields.end()) {
    throw SnapshotError(Status::INVALID_REQUEST, "Cloud field not found: " + name);
  }
  return *found;
}

Projection compile_projection(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::vector<std::string> & requested)
{
  Projection projection;
  if (requested.empty()) {
    // Empty means all original bytes, including point padding and field offsets.
    projection.fields = cloud.fields;
    projection.point_step = cloud.point_step;
    projection.copies.push_back({0, cloud.point_step});
    return projection;
  }

  std::unordered_set<std::string> names;
  for (const auto & name : requested) {
    if (!names.insert(name).second) {
      throw SnapshotError(Status::INVALID_REQUEST, "Duplicate selected field: " + name);
    }
    auto field = find_field(cloud, name);
    const size_t bytes = field_size(field.datatype) * field.count;
    if (bytes > std::numeric_limits<uint32_t>::max() - projection.point_step) {
      throw SnapshotError(Status::INVALID_REQUEST, "Selected fields exceed the point size limit");
    }
    projection.copies.push_back({field.offset, bytes});
    field.offset = projection.point_step;
    projection.fields.push_back(std::move(field));
    projection.point_step += static_cast<uint32_t>(bytes);
  }
  return projection;
}

struct Coordinate
{
  size_t offset{0};
  Reader read{nullptr};

  double value(const uint8_t * point, bool swap_bytes) const
  {
    return read(point + offset, swap_bytes);
  }
};

std::array<Coordinate, 3> voxel_coordinates(const sensor_msgs::msg::PointCloud2 & cloud)
{
  std::array<Coordinate, 3> coordinates;
  const std::array<std::string, 3> names{"x", "y", "z"};
  for (size_t index = 0; index < names.size(); ++index) {
    const auto & field = find_field(cloud, names[index]);
    coordinates[index] = {field.offset, reader_for(field.datatype)};
  }
  return coordinates;
}
}  // namespace

sensor_msgs::msg::PointCloud2 filter_point_cloud(
  const sensor_msgs::msg::PointCloud2 & source,
  const sensor_snapshot_interfaces::msg::Region & requested,
  const std::vector<sensor_snapshot_interfaces::msg::FieldFilter> & filters,
  const std::vector<std::string> & fields, uint32_t downsample_factor, double voxel_size)
{
  validate_cloud(source);
  const auto region = resolve_region(requested, source.width, source.height);
  const auto width = sampled_extent(region.width, downsample_factor);
  const auto height = sampled_extent(region.height, downsample_factor);
  const auto compiled = compile_filters(source, filters);
  auto projection = compile_projection(source, fields);
  VoxelFilter voxels(voxel_size);
  const auto coordinates = voxels.enabled() ? voxel_coordinates(source) : std::array<Coordinate,
      3>{};
  const bool compact = !filters.empty() || voxels.enabled();
  const bool original_records = fields.empty() && downsample_factor == 1 && !compact;
  const bool full_cloud = region.x_offset == 0 && region.y_offset == 0 &&
    region.width == source.width && region.height == source.height;
  if (original_records && full_cloud) {
    return source;
  }

  sensor_msgs::msg::PointCloud2 output;
  output.header = source.header;
  output.fields = std::move(projection.fields);
  output.is_bigendian = source.is_bigendian;
  output.point_step = projection.point_step;
  // Conservatively retain false even if filtering happens to remove all invalid XYZ.
  output.is_dense = source.is_dense;
  if (original_records) {
    output.width = region.width;
    output.height = region.height;
    output.row_step = region.width * source.point_step;
    output.data.resize(static_cast<size_t>(output.row_step) * output.height);
    for (uint32_t row = 0; row < region.height && output.row_step != 0; ++row) {
      const size_t offset = static_cast<size_t>(region.y_offset + row) * source.row_step +
        static_cast<size_t>(region.x_offset) * source.point_step;
      std::memcpy(
        output.data.data() + static_cast<size_t>(row) * output.row_step,
        source.data.data() + offset, output.row_step);
    }
    return output;
  }

  const uint64_t count = static_cast<uint64_t>(width) * height;
  const uint64_t row_bytes = static_cast<uint64_t>(width) * output.point_step;
  const auto row_limit = std::numeric_limits<uint32_t>::max();
  if ((!compact && row_bytes > row_limit) ||
    (compact && output.point_step != 0 && count > row_limit / output.point_step))
  {
    throw SnapshotError(Status::INVALID_REQUEST,
        "Selected cloud exceeds the output row size limit");
  }
  output.width = compact ? 0 : width;
  output.height = compact ? 1 : height;
  output.data.reserve(static_cast<size_t>(count * output.point_step));
  const uint16_t endian_probe = 1;
  const bool host_is_bigendian = *reinterpret_cast<const uint8_t *>(&endian_probe) == 0;
  const bool swap_bytes = source.is_bigendian != host_is_bigendian;
  for (PointCloudRegionIterator point(source, region, downsample_factor); point; ++point) {
    // Offsets and numeric readers were resolved once, outside this high-volume loop.
    bool keep = true;
    for (const auto & filter : compiled) {
      if (!filter.passes(*point, swap_bytes)) {
        keep = false;
        break;
      }
    }
    if (!keep) {
      continue;
    }
    if (voxels.enabled() && !voxels.keep(
        coordinates[0].value(*point, swap_bytes),
        coordinates[1].value(*point, swap_bytes),
        coordinates[2].value(*point, swap_bytes)))
    {
      continue;
    }
    // Project only after testing the original point: omitted fields may still be
    // needed for comparisons or XYZ voxel assignment. Preserve their byte order.
    for (const auto & span : projection.copies) {
      output.data.insert(output.data.end(), *point + span.offset,
          *point + span.offset + span.bytes);
    }
    if (compact) {
      ++output.width;
    }
  }
  output.row_step =
    compact ? static_cast<uint32_t>(output.data.size()) : static_cast<uint32_t>(row_bytes);
  return output;
}
}  // namespace sensor_snapshot
