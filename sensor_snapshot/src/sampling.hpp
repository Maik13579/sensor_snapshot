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

#ifndef SENSOR_SNAPSHOT__SAMPLING_HPP_
#define SENSOR_SNAPSHOT__SAMPLING_HPP_

#include <cstdint>
#include <sensor_snapshot_interfaces/msg/snapshot_status.hpp>
#include "sensor_snapshot/processing.hpp"

namespace sensor_snapshot
{
/// Number of samples when visiting 0, factor, 2*factor, ... inside an extent.
inline uint32_t sampled_extent(uint32_t extent, uint32_t factor)
{
  if (factor == 0) {
    throw SnapshotError(
      sensor_snapshot_interfaces::msg::SnapshotStatus::INVALID_REQUEST,
      "downsample_factor must be at least one");
  }
  return extent == 0 ? 0 : 1 + (extent - 1) / factor;
}
}  // namespace sensor_snapshot
#endif  // SENSOR_SNAPSHOT__SAMPLING_HPP_
