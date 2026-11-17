# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
"""Exercise real ROS graph discovery and RPCs without any sensor hardware."""

import argparse
from collections import Counter
import subprocess
import threading
import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from sensor_snapshot_interfaces.msg import SnapshotStatus
from sensor_snapshot_interfaces.srv import GetImage, GetImu, GetLaserScan, GetPointCloud
from std_srvs.srv import Empty
import yaml

from sensor_snapshot_cli.command import SensorSnapshotCommand
from sensor_snapshot_cli.discovery import SnapshotNameCompleter


@pytest.fixture
def server(image, cloud, imu, scan):
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
        node.create_service(GetLaserScan, '/cli_test/scan', callback('scan', scan)),
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
    if arguments and arguments[0] == 'list':
        values = ['list', '--discovery-time', '1.5', *map(str, arguments[1:])]
    else:
        values = ['get', '--discovery-time', '1.5', '--timeout', '1', *map(str, arguments)]
    args = parser.parse_args(values)
    return command.main(parser=parser, args=args)


def test_list_and_completion_only_offer_snapshot_services(server, capsys):
    assert run('list') == 0
    output = capsys.readouterr().out
    assert '/cli_test/image [sensor_snapshot_interfaces/srv/GetImage]' in output
    assert '/cli_test/imu ' in output and '/cli_test/points ' in output
    assert '/cli_test/scan [sensor_snapshot_interfaces/srv/GetLaserScan]' in output
    assert '/cli_test/other' not in output
    assert SnapshotNameCompleter()('/cli_test/im', argparse.Namespace()) == [
        '/cli_test/image', '/cli_test/imu']
    assert SnapshotNameCompleter()('/cli_test/sc', argparse.Namespace()) == ['/cli_test/scan']
    assert sum(server[0].values()) == 0


def test_installed_command(server):
    help_result = subprocess.run(
        ['ros2', 'sensor_snapshot', '--help'], capture_output=True, text=True, timeout=10)
    assert help_result.returncode == 0, help_result.stderr
    assert '{get,list,load,unload,record}' in help_result.stdout
    assert '--output' not in help_result.stdout and '--force' not in help_result.stdout
    bare = subprocess.run(['ros2', 'sensor_snapshot'],
                          capture_output=True, text=True, timeout=10)
    assert bare.returncode == 0 and bare.stdout == help_result.stdout
    get_help = subprocess.run(['ros2', 'sensor_snapshot', 'get', '--help'],
                              capture_output=True, text=True, timeout=10)
    assert get_help.returncode == 0 and '--show-image' in get_help.stdout
    result = subprocess.run(
        ['ros2', 'sensor_snapshot', 'get', '/cli_test/imu', '--field', 'linear_acceleration.z'],
        capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert yaml.safe_load(result.stdout) == 9.81


def test_imu_field_response_and_request_options(server, capsys):
    assert run('/cli_test/imu', '--field', 'linear_acceleration.z',
               '--request', '{max_age_seconds: 0.5}') == 0
    assert yaml.safe_load(capsys.readouterr().out) == 9.81
    assert server[1]['imu'].max_age_seconds == 0.5
    assert run('/cli_test/imu', '--response', '--field', 'sequence') == 0
    assert yaml.safe_load(capsys.readouterr().out) == 2


def test_laser_scan_payload_request_and_installed_command(server, capsys):
    assert run('/cli_test/scan', '--field', 'ranges',
               '--request', '{max_age_seconds: 0.5, stamp: {sec: 99}}') == 0
    assert yaml.safe_load(capsys.readouterr().out) == [1.0, float('inf'), 3.0]
    assert server[1]['scan'].max_age_seconds == 0.5
    assert server[1]['scan'].stamp.sec == 99
    result = subprocess.run(
        ['ros2', 'sensor_snapshot', 'get', '/cli_test/scan', '--field', 'header'],
        capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    header = yaml.safe_load(result.stdout)
    assert header['frame_id'] == 'laser_scan' and header['stamp']['sec'] == 99


def test_preview_uses_one_response_and_can_print_a_field(server, monkeypatch, capsys):
    from sensor_snapshot_cli import images
    previews = []
    monkeypatch.setattr(images, 'show_image', previews.append)
    assert run('/cli_test/image', '--show-image', '--field', 'header.frame_id') == 0
    assert server[0]['image'] == 1
    assert len(previews) == 1 and previews[0].header.frame_id == 'camera'
    output = capsys.readouterr()
    assert yaml.safe_load(output.out) == 'camera'
    assert output.err == ''


def test_cloud_request_options_and_imu_text(server, capsys):
    assert run('/cli_test/points', '--field', 'header.frame_id',
               '--request', '{voxel_size: 0.05, fields: [x, ring]}') == 0
    assert yaml.safe_load(capsys.readouterr().out) == 'laser'
    assert server[1]['cloud'].voxel_size == 0.05
    assert list(server[1]['cloud'].fields) == ['x', 'ring']
    assert run('/cli_test/imu') == 0
    assert yaml.safe_load(capsys.readouterr().out)['linear_acceleration']['z'] == 9.81


@pytest.mark.parametrize('arguments,error', [
    (['/cli_test/stale'], 'Cached sample expired'),
    (['/cli_test/missing', '--discovery-time', '0.05'], 'No snapshot service'),
    (['/cli_test/other'], 'not a sensor snapshot service'),
    (['/cli_test/slow', '--timeout', '0.01'], 'Timed out requesting'),
    (['/cli_test/imu', '--show-image'], 'only available for image'),
    (['/cli_test/scan', '--show-image'], 'only available for image'),
    (['/cli_test/image', '--field', 'bad.field'], 'Invalid field'),
    (['/cli_test/imu', '--request', '{missing: 1}'], 'Invalid snapshot request'),
    (['/cli_test/imu', '--request', '[]'], 'YAML mapping'),
])
def test_failures_are_nonzero(server, capsys, arguments, error):
    assert run(*arguments) == 1
    result = capsys.readouterr()
    assert error in result.err
    assert result.out == ''
