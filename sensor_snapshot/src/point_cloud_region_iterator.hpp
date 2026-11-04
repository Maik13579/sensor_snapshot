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

#ifndef SENSOR_SNAPSHOT__POINT_CLOUD_REGION_ITERATOR_HPP_
#define SENSOR_SNAPSHOT__POINT_CLOUD_REGION_ITERATOR_HPP_

#include <cstddef>
#include <cstdint>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_snapshot_interfaces/msg/region.hpp>
#include "sampling.hpp"

namespace sensor_snapshot
{
/// Byte iterator over a validated rectangle, skipping source row padding.
/// Unlike a typed PointCloud2ConstIterator, it handles padded rows and allows
/// each requested field to have a different numeric datatype and byte order.
class PointCloudRegionIterator
{
public:
  PointCloudRegionIterator(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const sensor_snapshot_interfaces::msg::Region & region, uint32_t factor = 1)
  : cloud_(cloud), factor_(factor),
    columns_(sampled_extent(region.width, factor)),
    rows_(sampled_extent(region.height, factor))
  {
    if (columns_ != 0 && rows_ != 0) {
      first_ = cloud.data.data() + static_cast<size_t>(region.y_offset) * cloud.row_step +
        static_cast<size_t>(region.x_offset) * cloud.point_step;
      current_ = first_;
    }
  }

  explicit operator bool() const {return row_ < rows_ && columns_ != 0;}
  const uint8_t * operator*() const {return current_;}

  PointCloudRegionIterator & operator++()
  {
    if (++column_ < columns_) {
      current_ += static_cast<size_t>(factor_) * cloud_.point_step;
    } else {
      column_ = 0;
      ++row_;
      if (row_ < rows_) {
        current_ = first_ + static_cast<size_t>(row_) * factor_ * cloud_.row_step;
      }
    }
    return *this;
  }

private:
  const sensor_msgs::msg::PointCloud2 & cloud_;
  uint32_t factor_;
  uint32_t columns_;
  uint32_t rows_;
  const uint8_t * first_{nullptr};
  const uint8_t * current_{nullptr};
  uint32_t column_{0};
  uint32_t row_{0};
};
}  // namespace sensor_snapshot
#endif  // SENSOR_SNAPSHOT__POINT_CLOUD_REGION_ITERATOR_HPP_
