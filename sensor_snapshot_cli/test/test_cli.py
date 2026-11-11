# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
"""Exercise real ROS graph discovery and RPCs without any sensor hardware."""

import argparse
from collections import Counter
import os
import subprocess
import threading
import time

import cv2
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from sensor_snapshot_interfaces.msg import SnapshotStatus
from sensor_snapshot_interfaces.srv import GetImage, GetImu, GetPointCloud
from std_srvs.srv import Empty
import yaml

from sensor_snapshot_cli.command import SensorSnapshotCommand
from sensor_snapshot_cli.discovery import SnapshotNameCompleter


@pytest.fixture
def server(image, cloud, imu):
    context = Context()
    rclpy.init(args=[], context=context)
    node = rclpy.create_node('_snapshot_test_server', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    calls, requests = Counter(), {}

    def callback(kind, payload):
        def respond(request, response):
            calls[kind] += 1
            requests[kind] = request
            response.sequence = calls[kind]
            setattr(response, kind, payload)
            return response
        return respond

    services = [
        node.create_service(GetImage, '/cli_test/image', callback('image', image)),
        node.create_service(GetPointCloud, '/cli_test/points', callback('cloud', cloud)),
        node.create_service(GetImu, '/cli_test/imu', callback('imu', imu)),
        node.create_service(Empty, '/cli_test/other', lambda request, response: response),
    ]

    def stale(request, response):
        response.status.code = SnapshotStatus.STALE
        response.status.message = 'Cached sample expired'
        return response

    def slow(request, response):
        time.sleep(0.25)
        return response

    services.extend([
        node.create_service(GetImage, '/cli_test/stale', stale),
        node.create_service(GetImu, '/cli_test/slow', slow),
    ])
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    yield calls, requests
    executor.shutdown()
    thread.join(timeout=5)
    node.destroy_node()
    context.try_shutdown()


def run(*arguments):
    command = SensorSnapshotCommand()
    parser = argparse.ArgumentParser()
    command.add_arguments(parser, 'ros2 sensor_snapshot')
    args = parser.parse_args(['--discovery-time', '1.5', '--timeout', '1', *map(str, arguments)])
    return command.main(parser=parser, args=args)


def test_list_and_completion_only_offer_snapshot_services(server, capsys):
    assert run() == 0
    output = capsys.readouterr().out
    assert '/cli_test/image [sensor_snapshot_interfaces/srv/GetImage]' in output
    assert '/cli_test/imu ' in output and '/cli_test/points ' in output
    assert '/cli_test/other' not in output
    assert SnapshotNameCompleter()('/cli_test/im', argparse.Namespace()) == [
        '/cli_test/image', '/cli_test/imu']
    assert sum(server[0].values()) == 0


def test_installed_command_and_shell_completion_protocol(server, tmp_path):
    help_result = subprocess.run(
        ['ros2', 'sensor_snapshot', '--help'], capture_output=True, text=True, timeout=10)
    assert help_result.returncode == 0, help_result.stderr
    assert '--show-image' in help_result.stdout
    result = subprocess.run(
        ['ros2', 'sensor_snapshot', '/cli_test/imu', '--field', 'linear_acceleration.z'],
        capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert yaml.safe_load(result.stdout) == 9.81
    completion_file = tmp_path / 'completion.txt'
    command_line = 'ros2 sensor_snapshot /cli_test/im'
    environment = dict(os.environ, _ARGCOMPLETE='1', COMP_LINE=command_line,
                       COMP_POINT=str(len(command_line)), _ARGCOMPLETE_IFS='\013',
                       _ARGCOMPLETE_SUPPRESS_SPACE='1',
                       _ARGCOMPLETE_STDOUT_FILENAME=str(completion_file))
    result = subprocess.run(['ros2'], env=environment, capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert sorted(completion_file.read_text().split('\013')) == ['/cli_test/image', '/cli_test/imu']


def test_imu_field_response_and_request_options(server, capsys):
    assert run('/cli_test/imu', '--field', 'linear_acceleration.z',
               '--request', '{max_age_seconds: 0.5}') == 0
    assert yaml.safe_load(capsys.readouterr().out) == 9.81
    assert server[1]['imu'].max_age_seconds == 0.5
    assert run('/cli_test/imu', '--response', '--field', 'sequence') == 0
    assert yaml.safe_load(capsys.readouterr().out) == 2


def test_save_and_preview_use_one_response_and_field_does_not_limit_export(
        server, tmp_path, monkeypatch, capsys):
    from sensor_snapshot_cli import images
    previews = []
    monkeypatch.setattr(images, 'show_image', previews.append)
    path = tmp_path / 'frame.png'
    assert run('/cli_test/image', '--output', path, '--show-image',
               '--field', 'header.frame_id') == 0
    assert server[0]['image'] == 1
    assert len(previews) == 1 and previews[0].header.frame_id == 'camera'
    assert cv2.imread(str(path)).shape == (2, 2, 3)
    output = capsys.readouterr()
    assert yaml.safe_load(output.out) == 'camera'
    assert 'Saved image snapshot' in output.err


def test_cloud_and_imu_export(server, tmp_path, capsys):
    path = tmp_path / 'cloud.pcd'
    assert run('/cli_test/points', '--output', path,
               '--request', '{voxel_size: 0.05, fields: [x, ring]}') == 0
    assert b'FIELDS x ring normal wall_id' in path.read_bytes()
    assert server[1]['cloud'].voxel_size == 0.05
    assert list(server[1]['cloud'].fields) == ['x', 'ring']
    path = tmp_path / 'imu.yaml'
    assert run('/cli_test/imu', '--output', path) == 0
    assert yaml.safe_load(path.read_text())['linear_acceleration']['z'] == 9.81
    assert capsys.readouterr().out == ''


@pytest.mark.parametrize('arguments,error', [
    (['/cli_test/stale'], 'Cached sample expired'),
    (['/cli_test/missing', '--discovery-time', '0.05'], 'No snapshot service'),
    (['/cli_test/other'], 'not a sensor snapshot service'),
    (['/cli_test/slow', '--timeout', '0.01'], 'Timed out requesting'),
    (['/cli_test/imu', '--show-image'], 'only available for image'),
    (['/cli_test/image', '--field', 'bad.field'], 'Invalid field'),
    (['/cli_test/imu', '--request', '{missing: 1}'], 'Invalid snapshot request'),
    (['--request', '[]'], 'YAML mapping'),
])
def test_failures_are_nonzero_and_never_create_output(server, tmp_path, capsys, arguments, error):
    path = tmp_path / ('out.yaml' if any(name in arguments for name in
                                       ('/cli_test/imu', '/cli_test/slow')) else 'out.png')
    assert run(*arguments, '--output', path) == 1
    assert error in capsys.readouterr().err
    assert not path.exists()
