# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
"""Small messages with known bytes, including padding and array-valued fields."""

import struct
import sys
from pathlib import Path

# colcon invokes python -m pytest from the source package. Exercise the installed
# CLI as ros2 loads it, rather than shadowing it with source.
source_root = Path(__file__).resolve().parents[1]
sys.path[:] = [entry for entry in sys.path if Path(entry).resolve() != source_root]

import pytest
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField


@pytest.fixture
def image():
    message = Image(height=2, width=2, encoding='rgb8', step=6)
    message.header.frame_id = 'camera'
    message.header.stamp.sec = 42
    message.data = bytes([255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 255])
    return message


@pytest.fixture
def cloud():
    message = PointCloud2(height=2, width=2, point_step=32, row_step=68, is_bigendian=True)
    message.header.frame_id = 'laser'
    message.header.stamp.sec = 10
    message.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='ring', offset=8, datatype=PointField.UINT16, count=1),
        PointField(name='normal', offset=12, datatype=PointField.FLOAT32, count=3),
        PointField(name='wall_id', offset=24, datatype=PointField.INT32, count=1),
    ]
    data = bytearray([0xaa] * (message.row_step * message.height))
    for index in range(4):
        offset = index // 2 * message.row_step + index % 2 * message.point_step
        struct.pack_into('>f', data, offset, index + 0.5)
        struct.pack_into('>H', data, offset + 8, index + 7)
        struct.pack_into('>fffi', data, offset + 12, 1.0, 2.0, 3.0, index - 1)
    message.data = bytes(data)
    return message


@pytest.fixture
def imu():
    message = Imu()
    message.header.frame_id = 'imu_link'
    message.linear_acceleration.z = 9.81
    message.orientation_covariance[0] = -1.0
    return message
