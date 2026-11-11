# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
import struct

import cv2
import numpy as np
import pytest
from sensor_msgs.msg import Image

from sensor_snapshot_cli.images import image_array, show_image


def test_color_order_and_padding():
    image = Image(height=2, width=1, encoding='rgb8', step=4)
    image.data = bytes([255, 0, 0, 99, 0, 255, 0, 99])
    assert image_array(image).tolist() == [[[0, 0, 255]], [[0, 255, 0]]]


def test_big_endian_uint16_with_padded_rows():
    image = Image(height=2, width=2, encoding='mono16', step=6, is_bigendian=True)
    image.data = struct.pack('>HHHHHH', 100, 200, 65535, 300, 400, 65535)
    pixels = image_array(image)
    assert pixels.dtype == np.uint16
    assert pixels.tolist() == [[100, 200], [300, 400]]


def test_float_depth_values_are_only_scaled_for_display():
    pixels = np.array([[0.125, 1.75], [np.nan, np.inf]], dtype=np.float32)
    image = Image(height=2, width=2, encoding='32FC1', step=8, data=pixels.tobytes())
    np.testing.assert_equal(image_array(image), pixels)


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
