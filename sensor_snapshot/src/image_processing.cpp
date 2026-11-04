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
#include "sampling.hpp"

#include <cstring>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_snapshot_interfaces/msg/snapshot_status.hpp>

namespace sensor_snapshot
{
namespace
{
using Status = sensor_snapshot_interfaces::msg::SnapshotStatus;

bool is_yuv(const std::string & encoding)
{
  // These formats share chroma between pixels or use multiple planes. A generic
  // bytes-per-pixel crop would silently corrupt them, so only full snapshots pass.
  return encoding == "yuv422" || encoding == "yuv422_yuy2" || encoding == "uyvy" ||
         encoding == "yuyv" || encoding == "nv12" || encoding == "nv21" ||
         encoding == "nv24" || encoding == "nv42";
}
}  // namespace

sensor_msgs::msg::Image crop_image(
  const sensor_msgs::msg::Image & source,
  const sensor_snapshot_interfaces::msg::Region & requested, uint32_t downsample_factor)
{
  if (source.width == 0 || source.height == 0 || source.step == 0 ||
    static_cast<uint64_t>(source.step) * source.height > source.data.size())
  {
    throw SnapshotError(Status::INVALID_DATA, "Image dimensions/stride exceed its data buffer");
  }
  const auto region = resolve_region(requested, source.width, source.height);
  const auto width = sampled_extent(region.width, downsample_factor);
  const auto height = sampled_extent(region.height, downsample_factor);
  const bool full_image = region.x_offset == 0 && region.y_offset == 0 &&
    region.width == source.width && region.height == source.height;
  if (full_image && downsample_factor == 1) {
    // Preserve padding and opaque encodings when no pixel interpretation is needed.
    return source;
  }
  if (is_yuv(source.encoding)) {
    throw SnapshotError(Status::UNSUPPORTED, "YUV cropping/downsampling is unsupported");
  }
  if (sensor_msgs::image_encodings::isBayer(source.encoding)) {
    if ((region.x_offset % 2) != 0 || (region.y_offset % 2) != 0) {
      throw SnapshotError(Status::UNSUPPORTED,
        "Bayer crops require even x/y offsets to keep the encoding");
    }
    if (downsample_factor != 1) {
      throw SnapshotError(Status::UNSUPPORTED, "Bayer downsampling is unsupported");
    }
  }

  int channels;
  int bit_depth;
  try {
    channels = sensor_msgs::image_encodings::numChannels(source.encoding);
    bit_depth = sensor_msgs::image_encodings::bitDepth(source.encoding);
  } catch (const std::runtime_error &) {
    throw SnapshotError(Status::UNSUPPORTED,
      "Unknown image encoding for cropping: " + source.encoding);
  }
  const uint64_t pixel_bytes = static_cast<uint64_t>(channels) * (bit_depth / 8);
  if (channels <= 0 || bit_depth <= 0 || bit_depth % 8 != 0 ||
    pixel_bytes * source.width > source.step)
  {
    throw SnapshotError(Status::INVALID_DATA, "Image row stride is too small for its encoding");
  }

  sensor_msgs::msg::Image output;
  output.header = source.header;
  output.encoding = source.encoding;
  output.is_bigendian = source.is_bigendian;
  output.width = width;
  output.height = height;
  output.step = static_cast<uint32_t>(pixel_bytes * width);
  output.data.resize(static_cast<size_t>(output.step) * height);
  for (uint32_t row = 0; row < height; ++row) {
    const size_t source_row = region.y_offset + static_cast<size_t>(row) * downsample_factor;
    const auto * input = source.data.data() + source_row * source.step +
      static_cast<size_t>(region.x_offset) * pixel_bytes;
    auto * destination = output.data.data() + static_cast<size_t>(row) * output.step;
    if (downsample_factor == 1) {
      std::memcpy(destination, input, output.step);
    } else {
      // Nearest-neighbor/stride sampling copies original pixel bytes without conversion.
      for (uint32_t column = 0; column < width; ++column) {
        std::memcpy(
          destination + column * pixel_bytes,
          input + static_cast<size_t>(column) * downsample_factor * pixel_bytes, pixel_bytes);
      }
    }
  }
  return output;
}
}  // namespace sensor_snapshot
