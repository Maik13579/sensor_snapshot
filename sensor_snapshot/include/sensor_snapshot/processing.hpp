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

#ifndef SENSOR_SNAPSHOT__PROCESSING_HPP_
#define SENSOR_SNAPSHOT__PROCESSING_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_snapshot_interfaces/msg/field_filter.hpp>
#include <sensor_snapshot_interfaces/msg/region.hpp>

namespace sensor_snapshot
{
/// Carries a stable service status without exposing exceptions across ROS callbacks.
class SnapshotError : public std::runtime_error
{
public:
  SnapshotError(uint8_t status_code, const std::string & message)
  : std::runtime_error(message), status_code_(status_code) {}
  uint8_t status_code() const {return status_code_;}

private:
  uint8_t status_code_;
};

/// Resolve zero extents to the source edge and reject out-of-bounds rectangles.
sensor_snapshot_interfaces::msg::Region resolve_region(
  const sensor_snapshot_interfaces::msg::Region & requested, uint32_t width, uint32_t height);

/// Copy a crop directly into its result, optionally sampling every Nth pixel.
sensor_msgs::msg::Image crop_image(
  const sensor_msgs::msg::Image & source,
  const sensor_snapshot_interfaces::msg::Region & requested, uint32_t downsample_factor = 1);

/// Iterate only the rectangle; resolve filter field offsets once per request.
sensor_msgs::msg::PointCloud2 filter_point_cloud(
  const sensor_msgs::msg::PointCloud2 & source,
  const sensor_snapshot_interfaces::msg::Region & requested,
  const std::vector<sensor_snapshot_interfaces::msg::FieldFilter> & filters,
  const std::vector<std::string> & fields = {}, uint32_t downsample_factor = 1,
  double voxel_size = 0.0);
}  // namespace sensor_snapshot
#endif  // SENSOR_SNAPSHOT__PROCESSING_HPP_
