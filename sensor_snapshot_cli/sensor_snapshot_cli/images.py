# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
"""Single frozen image preview; imported only when --show-image is requested."""

import os


def image_array(message):
    """Interpret image layout and color order without modifying the ROS message."""
    import cv2
    from cv_bridge import CvBridge
    import numpy as np

    bridge = CvBridge()
    encoding = message.encoding.lower()
    dtype, channels = bridge.encoding_to_dtype_with_channels(message.encoding)
    dtype = np.dtype(dtype).newbyteorder('>' if message.is_bigendian else '<')
    if not message.width or not message.height:
        raise ValueError('Cannot preview an empty image')
    if message.step < message.width * channels * dtype.itemsize:
        raise ValueError('Image step is smaller than its pixel data')
    if len(message.data) < message.height * message.step:
        raise ValueError('Image data is shorter than step * height')
    shape = ((message.height, message.width) if channels == 1 else
             (message.height, message.width, channels))
    strides = ((message.step, dtype.itemsize) if channels == 1 else
               (message.step, channels * dtype.itemsize, dtype.itemsize))
    # Interpret strides explicitly: older cv_bridge releases differ in their
    # handling of row padding, and NumPy 2 removed ndarray.newbyteorder().
    data = np.ndarray(shape, dtype=dtype, buffer=memoryview(message.data), strides=strides)
    data = np.ascontiguousarray(data, dtype=dtype.newbyteorder('='))
    if encoding.startswith('bayer_') or encoding in ('yuv422', 'yuv422_yuy2', 'uyvy', 'yuyv'):
        packed = bridge.cv2_to_imgmsg(data, encoding=message.encoding)
        target = 'bgr16' if encoding.startswith('bayer_') and encoding.endswith('16') else 'bgr8'
        data = bridge.imgmsg_to_cv2(packed, desired_encoding=target)
    elif encoding in ('rgb8', 'rgb16'):
        data = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
    elif encoding in ('rgba8', 'rgba16'):
        data = cv2.cvtColor(data, cv2.COLOR_RGBA2BGRA)
    return data


def show_image(message):
    """Display the already received sample; closing the window ends the preview."""
    if os.name == 'posix' and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        raise ValueError('--show-image requires a graphical display (DISPLAY or WAYLAND_DISPLAY)')
    import cv2
    import numpy as np

    data = image_array(message)
    if data.dtype != np.uint8:
        finite = np.isfinite(data)
        preview = np.zeros(data.shape, dtype=np.uint8)
        if finite.any():
            low, high = float(data[finite].min()), float(data[finite].max())
            if high > low:
                preview[finite] = ((data[finite].astype(np.float64) - low)
                                   * (255.0 / (high - low))).astype(np.uint8)
        data = preview
    title = f'Snapshot: {message.header.frame_id or "image"} (Esc / Q to close)'
    try:
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        cv2.imshow(title, data)
        while cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) >= 1:
            if cv2.waitKey(30) & 0xff in (27, ord('q'), ord('Q')):
                break
    finally:
        cv2.destroyAllWindows()
