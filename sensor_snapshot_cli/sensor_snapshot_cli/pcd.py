# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
"""Normalize ROS point layouts for PCL without dropping non-XYZ fields."""

import re


# sensor_msgs/PointField datatype -> NumPy code. Keep array-valued fields intact.
DATATYPES = {
    1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2',
    5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8',
    9: 'i8', 10: 'u8', 11: '?',  # Added to sensor_msgs/PointField in newer ROS releases.
}


def save_pcd(path, cloud):
    """Pack declared fields; let PCL write the file rather than implementing PCD."""
    import numpy as np
    try:
        from . import _pcl_io
    except ImportError as error:
        raise ValueError(
            'PCL I/O adapter is unavailable. Install dependencies with '
            'rosdep install --from-paths src -riy from your workspace root, then '
            'rebuild sensor_snapshot_cli and source install/setup.bash') from error

    if not cloud.fields:
        raise ValueError('Cannot export a cloud without fields')
    if not cloud.width * cloud.height:
        raise ValueError('PCL export requires a nonempty cloud')
    names, formats, offsets, packed_fields = [], [], [], []
    for field in cloud.fields:
        if (not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', field.name)
                or field.name == '_' or field.name in names):
            raise ValueError(f'Invalid or duplicate PCD field name: {field.name!r}')
        if field.datatype not in DATATYPES or field.count < 1:
            raise ValueError(f'Unsupported datatype/count for PCD field {field.name}')
        code = DATATYPES[field.datatype]
        size = np.dtype(code).itemsize
        if field.offset + size * field.count > cloud.point_step:
            raise ValueError(f'PCD field {field.name} exceeds point_step')
        names.append(field.name)
        source_type = np.dtype(('>' if cloud.is_bigendian else '<') + code)
        formats.append(source_type if field.count == 1 else np.dtype((source_type, (field.count,))))
        offsets.append(field.offset)
        packed_fields.append((field.name, '<' + code) if field.count == 1 else
                             (field.name, '<' + code, (field.count,)))
    if cloud.row_step < cloud.width * cloud.point_step:
        raise ValueError('Cloud row_step is smaller than its point data')
    if len(cloud.data) < cloud.row_step * cloud.height:
        raise ValueError('Cloud data is shorter than row_step * height')
    dtype = np.dtype({
        'names': names, 'formats': formats, 'offsets': offsets, 'itemsize': cloud.point_step})
    packed = np.empty((cloud.height, cloud.width), dtype=np.dtype(packed_fields))
    if cloud.width * cloud.height:
        source = np.ndarray(
            (cloud.height, cloud.width), dtype=dtype, buffer=memoryview(cloud.data),
            strides=(cloud.row_step, cloud.point_step))
        for name in names:
            packed[name] = source[name]
    fields = [(field.name, packed.dtype.fields[field.name][1], field.datatype, field.count)
              for field in cloud.fields]
    _pcl_io.save(str(path), cloud.width, cloud.height, packed.dtype.itemsize,
                 fields, memoryview(packed).cast('B'))
