# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
"""Validate formats and commit complete output files without partial overwrites."""

from contextlib import contextmanager
import os
from pathlib import Path
import tempfile


EXTENSIONS = {'image': ('.png', '.tif', '.tiff'), 'cloud': ('.pcd',), 'imu': ('.yaml', '.yml')}


def validate_output(path, kind, force=False):
    path = Path(path).expanduser()
    if path.suffix.lower() not in EXTENSIONS[kind]:
        raise ValueError(f'{kind} output requires one of: {", ".join(EXTENSIONS[kind])}')
    if path.exists() and not force:
        raise ValueError(f'Output already exists: {path} (use --force to replace it)')
    if not path.parent.is_dir():
        raise ValueError(f'Output directory does not exist: {path.parent}')
    return path


@contextmanager
def output_file(path, force):
    """Only expose a completed file; failed encoding leaves the destination alone."""
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix=f'.{path.name}.', delete=False) as f:
            temporary = Path(f.name)
        yield temporary
        if force:
            os.replace(temporary, path)
        else:
            os.link(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def save_snapshot(path, message, kind, force=False):
    path = validate_output(path, kind, force)
    with output_file(path, force) as temporary:
        if kind == 'cloud':
            from .pcd import save_pcd
            save_pcd(temporary, message)
        elif kind == 'image':
            from .images import encode_image
            temporary.write_bytes(encode_image(message, path.suffix.lower()))
        else:
            from rosidl_runtime_py import message_to_yaml
            temporary.write_text(message_to_yaml(message), encoding='utf-8')
    return path
