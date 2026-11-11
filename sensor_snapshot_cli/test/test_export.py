# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
import struct

import cv2
import numpy as np
import pytest
from sensor_msgs.msg import Image, PointCloud2, PointField
import yaml

from sensor_snapshot_cli.images import show_image
from sensor_snapshot_cli.output import save_snapshot
from sensor_snapshot_cli.pcd import save_pcd


def test_pcd_retains_organized_dimensions_array_fields_and_labels(tmp_path, cloud):
    path = tmp_path / 'cloud.pcd'
    save_pcd(path, cloud)
    header, data = path.read_bytes().split(b'DATA binary\n')
    assert b'FIELDS x ring normal wall_id\n' in header
    assert b'SIZE 4 2 4 4\n' in header
    assert b'TYPE F U F I\n' in header
    assert b'COUNT 1 1 3 1\n' in header
    assert b'WIDTH 2\nHEIGHT 2\n' in header
    assert b'POINTS 4\n' in header
    expected = b''.join(struct.pack('<fHfffi', i + 0.5, i + 7, 1., 2., 3., i - 1)
                        for i in range(4))
    # PCL versions can leave zero padding after the declared binary data. Readers
    # use POINTS and SIZE/COUNT; verify those bytes without depending on file padding.
    assert data[:len(expected)] == expected


@pytest.mark.parametrize('datatype,code,value', [
    (1, 'b', -8), (2, 'B', 250), (3, 'h', -300), (4, 'H', 65000),
    (5, 'i', -100000), (6, 'I', 4000000000), (7, 'f', 1.25), (8, 'd', 2.5),
])
def test_pcd_handles_all_standard_datatypes(tmp_path, datatype, code, value):
    width = struct.calcsize(code)
    cloud = PointCloud2(height=1, width=1, point_step=width, row_step=width)
    cloud.fields = [PointField(name='value', offset=0, datatype=datatype, count=1)]
    cloud.data = struct.pack('<' + code, value)
    path = tmp_path / 'cloud.pcd'
    save_pcd(path, cloud)
    assert struct.unpack_from('<' + code, path.read_bytes().split(b'DATA binary\n')[1])[0] == value


def test_pcd_packed_rgb_bits_are_not_numerically_converted(tmp_path):
    cloud = PointCloud2(height=1, width=1, point_step=4, row_step=4, is_bigendian=True)
    cloud.fields = [PointField(name='rgb', offset=0, datatype=PointField.FLOAT32, count=1)]
    cloud.data = struct.pack('>I', 0xff123456)
    path = tmp_path / 'cloud.pcd'
    save_pcd(path, cloud)
    assert struct.unpack_from('<I', path.read_bytes().split(b'DATA binary\n')[1])[0] == 0xff123456


@pytest.mark.parametrize('datatype,code,value', [(9, 'q', -2**50), (10, 'Q', 2**60), (11, '?', True)])
def test_newer_ros_point_datatypes_when_supported_by_pcl(tmp_path, datatype, code, value):
    cloud = PointCloud2(height=1, width=1, point_step=struct.calcsize(code),
                        row_step=struct.calcsize(code))
    cloud.fields = [PointField(name='value', offset=0, datatype=datatype, count=1)]
    cloud.data = struct.pack('<' + code, value)
    path = tmp_path / 'new_type.pcd'
    try:
        save_pcd(path, cloud)
    except ValueError as error:
        if not hasattr(PointField, 'INT64') and 'Installed PCL does not support' in str(error):
            # Older ROS releases do not define these datatypes; older PCL can
            # reject them explicitly. Lyrical defines them and must support them.
            return
        raise
    assert struct.unpack_from('<' + code, path.read_bytes().split(b'DATA binary\n')[1])[0] == value


def test_empty_pcd_and_invalid_layouts(tmp_path, cloud):
    cloud.width = 0
    cloud.height = 1
    cloud.row_step = 0
    cloud.data = b''
    with pytest.raises(ValueError, match='nonempty cloud'):
        save_pcd(tmp_path / 'cloud.pcd', cloud)
    cloud.width = 1
    cloud.fields[0].offset = 1000
    with pytest.raises(ValueError, match='exceeds point_step'):
        save_pcd(tmp_path / 'cloud.pcd', cloud)


@pytest.mark.parametrize('failure', ['short_data', 'short_row', 'duplicate', 'count', 'type', 'name'])
def test_bad_cloud_does_not_replace_existing_output(tmp_path, cloud, failure):
    destination = tmp_path / 'cloud.pcd'
    destination.write_bytes(b'original')
    if failure == 'short_data':
        cloud.data = b''
    elif failure == 'short_row':
        cloud.row_step = 1
    elif failure == 'duplicate':
        cloud.fields[1].name = cloud.fields[0].name
    elif failure == 'count':
        cloud.fields[0].count = 0
    elif failure == 'type':
        cloud.fields[0].datatype = 255
    else:
        cloud.fields[0].name = 'invalid name'
    with pytest.raises(ValueError):
        save_snapshot(destination, cloud, 'cloud', force=True)
    assert destination.read_bytes() == b'original'
    assert sorted(p.name for p in tmp_path.iterdir()) == ['cloud.pcd']


def test_png_colors_and_overwrite_protection(tmp_path, image):
    path = tmp_path / 'rgb.png'
    save_snapshot(path, image, 'image')
    pixels = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    assert pixels.tolist() == [[[0, 0, 255], [0, 255, 0]], [[255, 0, 0], [255, 255, 255]]]
    with pytest.raises(ValueError, match='already exists'):
        save_snapshot(path, image, 'image')
    save_snapshot(path, image, 'image', force=True)


def test_uint16_png_handles_padded_big_endian_rows(tmp_path):
    image = Image(height=2, width=2, encoding='mono16', step=6, is_bigendian=True)
    image.data = struct.pack('>HHHHHH', 100, 200, 65535, 300, 400, 65535)
    path = tmp_path / 'depth.png'
    save_snapshot(path, image, 'image')
    pixels = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    assert pixels.dtype == np.uint16
    assert pixels.tolist() == [[100, 200], [300, 400]]


def test_rgb_png_with_padding_not_divisible_by_pixel_size(tmp_path):
    image = Image(height=2, width=1, encoding='rgb8', step=4)
    image.data = bytes([255, 0, 0, 99, 0, 255, 0, 99])
    path = tmp_path / 'padded.png'
    save_snapshot(path, image, 'image')
    assert cv2.imread(str(path)).tolist() == [[[0, 0, 255]], [[0, 255, 0]]]


def test_float_depth_tiff_retains_values_and_png_rejects_loss(tmp_path):
    pixels = np.array([[0.125, 1.75], [np.nan, np.inf]], dtype=np.float32)
    image = Image(height=2, width=2, encoding='32FC1', step=8, data=pixels.tobytes())
    path = tmp_path / 'depth.tiff'
    save_snapshot(path, image, 'image')
    actual = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    np.testing.assert_equal(actual, pixels)
    with pytest.raises(ValueError, match='PNG requires'):
        save_snapshot(tmp_path / 'depth.png', image, 'image')
    assert not (tmp_path / 'depth.png').exists()


def test_imu_yaml_is_complete_and_formats_are_checked(tmp_path, imu, image):
    path = tmp_path / 'imu.yaml'
    save_snapshot(path, imu, 'imu')
    result = yaml.safe_load(path.read_text())
    assert result['header']['frame_id'] == 'imu_link'
    assert result['linear_acceleration']['z'] == 9.81
    assert result['orientation_covariance'][0] == -1.0
    with pytest.raises(ValueError, match='image output requires'):
        save_snapshot(tmp_path / 'image.pcd', image, 'image')


def test_preview_requires_display(monkeypatch, image):
    monkeypatch.delenv('DISPLAY', raising=False)
    monkeypatch.delenv('WAYLAND_DISPLAY', raising=False)
    with pytest.raises(ValueError, match='requires a graphical display'):
        show_image(image)


def test_preview_closes_on_escape(monkeypatch, image):
    monkeypatch.setenv('DISPLAY', ':test')
    frames, closed = [], []
    monkeypatch.setattr(cv2, 'namedWindow', lambda *args: None)
    monkeypatch.setattr(cv2, 'imshow', lambda title, data: frames.append(data.copy()))
    monkeypatch.setattr(cv2, 'getWindowProperty', lambda *args: 1)
    monkeypatch.setattr(cv2, 'waitKey', lambda *args: 27)
    monkeypatch.setattr(cv2, 'destroyAllWindows', lambda: closed.append(True))
    show_image(image)
    assert len(frames) == 1 and closed == [True]
    assert frames[0][0, 0].tolist() == [0, 0, 255]
