# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
import argparse
import os
from pathlib import Path
import subprocess
import sys

import pytest
import yaml

from sensor_snapshot_cli.command import SensorSnapshotCommand
from sensor_snapshot_cli.discovery import snapshot_services, SnapshotNameCompleter
from sensor_snapshot_cli.text import format_message, select_field


def test_field_selection_and_ros_sequences(imu, cloud):
    assert select_field(imu, 'linear_acceleration.z') == 9.81
    assert yaml.safe_load(format_message(select_field(imu, 'linear_acceleration')))['z'] == 9.81
    assert yaml.safe_load(format_message(cloud.fields))[0]['name'] == 'x'
    assert yaml.safe_load(format_message(imu.orientation_covariance))[0] == -1.0
    for name in ('missing', 'linear_acceleration.missing', 'header..stamp', '__class__'):
        with pytest.raises(ValueError, match='Invalid field'):
            select_field(imu, name)


def test_echo_truncation_and_full_output(image):
    assert '...' in format_message(image, truncate_length=2)
    assert '...' not in format_message(image, truncate_length=None)
    assert isinstance(yaml.safe_load(format_message(image, no_arr=True))['data'], str)
    assert yaml.safe_load(format_message(image, no_str=True))['header']['frame_id'] != 'camera'


def test_discovery_filters_types_and_sorts():
    graph = [('/z', ['sensor_snapshot_interfaces/srv/GetImu']),
             ('/other', ['std_srvs/srv/Empty']),
             ('/a', ['sensor_snapshot_interfaces/srv/GetImage'])]
    assert [name for name, _ in snapshot_services(graph)] == ['/a', '/z']


def test_arguments_attach_completion_and_validate_numbers():
    parser = argparse.ArgumentParser()
    SensorSnapshotCommand().add_arguments(parser, 'ros2 sensor_snapshot')
    action = next(action for action in parser._actions if action.dest == 'service')
    assert isinstance(action.completer, SnapshotNameCompleter)
    for option, value in [('--timeout', 'nan'), ('--timeout', '0'),
                          ('--discovery-time', '-1'), ('--truncate-length', '-1')]:
        with pytest.raises(SystemExit):
            parser.parse_args([option, value])


def test_packaging_configuration_does_not_need_native_build_tools(tmp_path):
    # Reproduce the GitLab pybuild configure stage with pkg-config/compilers absent.
    package_root = Path(__file__).resolve().parents[1]
    result = subprocess.run(
        [sys.executable, 'setup.py', 'config'], cwd=package_root,
        env=dict(os.environ, PATH=str(tmp_path)), capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
