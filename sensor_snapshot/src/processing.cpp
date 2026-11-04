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
#include <sensor_snapshot_interfaces/msg/snapshot_status.hpp>

namespace sensor_snapshot
{
sensor_snapshot_interfaces::msg::Region resolve_region(
  const sensor_snapshot_interfaces::msg::Region & requested, uint32_t width, uint32_t height)
{
  using Status = sensor_snapshot_interfaces::msg::SnapshotStatus;
  // Empty clouds may legitimately have width = 0. The default still selects all.
  if (requested.x_offset > width || requested.y_offset > height) {
    throw SnapshotError(Status::INVALID_REQUEST, "Region offset exceeds source dimensions");
  }
  auto region = requested;
  if (region.width == 0) {
    region.width = width - region.x_offset;
  }
  if (region.height == 0) {
    region.height = height - region.y_offset;
  }
  // Subtraction avoids overflow with very large request extents.
  if (region.width > width - region.x_offset || region.height > height - region.y_offset) {
    throw SnapshotError(Status::INVALID_REQUEST, "Region extends beyond source dimensions");
  }
  if ((region.width == 0 || region.height == 0) && width != 0 && height != 0) {
    throw SnapshotError(Status::INVALID_REQUEST, "Region must contain at least one point/pixel");
  }
  return region;
}
}  // namespace sensor_snapshot
