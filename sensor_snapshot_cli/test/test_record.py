# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
"""Check the installed record command, terminal input, YAML and bag contents."""
import argparse
import os
import pty
import signal
import subprocess
import time

import pytest
from std_srvs.srv import Trigger
import yaml

from sensor_snapshot_cli.command import SensorSnapshotCommand
from sensor_snapshot_cli.discovery import snapshot_node
from sensor_snapshot_cli.record import recorder_command
from test_cli import server  # noqa: F401 - shared live snapshot service fixture


def wait_until(predicate, timeout=10):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError('Timed out waiting for the record command')


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()
        raise


def read_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {topic.name: get_message(topic.type) for topic in reader.get_all_topics_and_types()}
    result = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        result.append((topic, deserialize_message(data, types[topic]), stamp))
    return result


def test_space_captures_frames_and_ctrl_c_finalizes_bag(server, tmp_path):
    destination = tmp_path / 'keyframes'
    log_path = tmp_path / 'record.log'
    master, slave = pty.openpty()
    with log_path.open('w') as log:
        process = subprocess.Popen([
            'ros2', 'sensor_snapshot', 'record', '/cli_test/image', '/cli_test/imu', '/cli_test/scan',
            '-o', str(destination)], stdin=slave, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        try:
            wait_until(lambda: 'Space: capture keyframe' in log_path.read_text())
            os.write(master, b' ')
            wait_until(lambda: 'Recorded keyframe 1' in log_path.read_text())
            os.write(master, b' ')
            wait_until(lambda: 'Recorded keyframe 2' in log_path.read_text())
        finally:
            stop(process)
            os.close(master)
            os.close(slave)
    assert process.returncode == 0, log_path.read_text()
    records = read_bag(destination)
    assert len(records) == 6
    assert records[0][2] == records[1][2] == records[2][2]
    assert records[3][2] == records[4][2] == records[5][2] and records[3][2] > records[0][2]
    assert records[0][1].header.stamp.sec == 42
    assert records[0][1].header.frame_id == 'camera'
    assert records[1][1].linear_acceleration.z == 9.81
    scan = records[2][1]
    assert scan.header.stamp.sec == 99 and scan.header.frame_id == 'laser_scan'
    assert list(scan.ranges) == [1.0, float('inf'), 3.0]
    assert list(scan.intensities) == [4.0, 5.0, 6.0]
    assert scan.time_increment == pytest.approx(0.001)
    assert server[0]['image'] == 2 and server[0]['imu'] == 2 and server[0]['scan'] == 2


def test_yaml_path_per_topic_options_and_external_trigger(server, tmp_path):
    destination = tmp_path / 'override_bag'
    configured_destination = tmp_path / 'unused_bag'
    parameter_file = tmp_path / 'recorder.yaml'
    parameter_file.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
        'input.topics': ['/cli_test/points'], 'output.uri': str(configured_destination),
        'requests': {'/cli_test/points': {'voxel_size': 0.05, 'fields': ['x']}},
    }}}))
    log_path = tmp_path / 'record.log'
    with log_path.open('w') as log:
        process = subprocess.Popen([
            'ros2', 'sensor_snapshot', 'record', str(parameter_file),
            '-o', str(destination), '--disable-keyboard-controls'], stdin=subprocess.DEVNULL,
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            wait_until(lambda: 'Trigger service:' in log_path.read_text())
            with snapshot_node() as (node, executor):
                client = node.create_client(Trigger, '/sensor_snapshot_recorder/trigger')
                assert client.wait_for_service(timeout_sec=5)
                for _ in range(2):
                    future = client.call_async(Trigger.Request())
                    executor.spin_until_future_complete(future, timeout_sec=5)
                    assert future.done() and future.result().success
        finally:
            stop(process)
    assert process.returncode == 0, log_path.read_text()
    assert not configured_destination.exists()
    assert server[1]['cloud'].voxel_size == 0.05
    assert list(server[1]['cloud'].fields) == ['x']
    records = read_bag(destination)
    assert len(records) == 2
    assert all(record[0] == '/cli_test/points' for record in records)
    assert all(record[1].header.frame_id == 'laser' for record in records)


def test_record_parser_and_missing_configuration(tmp_path):
    parser = argparse.ArgumentParser()
    SensorSnapshotCommand().add_arguments(parser, 'ros2 sensor_snapshot', argv=['record'])
    args = parser.parse_args(['record'])
    with pytest.raises(ValueError, match='Specify snapshot topics'):
        recorder_command(args)
    args = parser.parse_args(['record', str(tmp_path / 'missing.yaml')])
    with pytest.raises(ValueError, match='does not exist'):
        recorder_command(args)
    parameter_file = tmp_path / 'recorder.yaml'
    parameter_file.write_text('/**:\n  ros__parameters:\n    input.topics: [/imu/data]\n')
    args = parser.parse_args(['record', '--params-file', str(parameter_file)])
    command = recorder_command(args)
    index = command.index('--params-file')
    assert command[index + 1] == str(parameter_file.resolve())
