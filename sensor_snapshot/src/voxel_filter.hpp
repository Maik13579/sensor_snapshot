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

#ifndef SENSOR_SNAPSHOT__VOXEL_FILTER_HPP_
#define SENSOR_SNAPSHOT__VOXEL_FILTER_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <unordered_set>
#include <sensor_snapshot_interfaces/msg/snapshot_status.hpp>
#include "sensor_snapshot/processing.hpp"

namespace sensor_snapshot
{
/// Retain the first original point in each voxel; no averaging of custom fields.
class VoxelFilter
{
public:
  explicit VoxelFilter(double size)
  : size_(size)
  {
    if (!std::isfinite(size) || size < 0.0) {
      throw SnapshotError(
        sensor_snapshot_interfaces::msg::SnapshotStatus::INVALID_REQUEST,
        "voxel_size must be finite and nonnegative");
    }
  }

  bool enabled() const {return size_ > 0.0;}

  bool keep(double x, double y, double z)
  {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      return false;
    }
    return occupied_.insert({cell(x), cell(y), cell(z)}).second;
  }

private:
  using Key = std::array<int64_t, 3>;

  struct Hash
  {
    size_t operator()(const Key & key) const
    {
      size_t hash = 0;
      for (const auto coordinate : key) {
        hash ^= std::hash<int64_t>{}(coordinate) + 0x9e3779b9u + (hash << 6) + (hash >> 2);
      }
      return hash;
    }
  };

  int64_t cell(double coordinate) const
  {
    // floor, rather than truncation, keeps negative-coordinate voxels consistent.
    const double index = std::floor(coordinate / size_);
    const double lower = static_cast<double>(std::numeric_limits<int64_t>::min());
    if (!std::isfinite(index) || index < lower || index >= -lower) {
      throw SnapshotError(
        sensor_snapshot_interfaces::msg::SnapshotStatus::INVALID_REQUEST,
        "voxel_size is too small for the cloud coordinates");
    }
    return static_cast<int64_t>(index);
  }

  double size_;
  std::unordered_set<Key, Hash> occupied_;
};
}  // namespace sensor_snapshot
#endif  // SENSOR_SNAPSHOT__VOXEL_FILTER_HPP_
