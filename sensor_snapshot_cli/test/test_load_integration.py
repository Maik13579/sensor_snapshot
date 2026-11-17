# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
"""Load real C++ components into externally owned containers without hardware."""

import os
from pathlib import Path
import signal
import subprocess
import threading

from ament_index_python.packages import get_package_prefix
from composition_interfaces.srv import ListNodes, LoadNode
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu

from sensor_snapshot_cli.discovery import snapshot_node
from sensor_snapshot_cli.load import ContainerNameCompleter


@pytest.fixture
def container(tmp_path):
    name = f'snapshot_cli_test_{os.getpid()}'
    prefix = Path(get_package_prefix('rclcpp_components'))
    executable = prefix / 'lib/rclcpp_components/component_container_mt'
    log_path = tmp_path / 'container.log'
    with log_path.open('w') as log:
        process = subprocess.Popen([str(executable), '--ros-args', '-r', f'__node:={name}'],
                                   stdout=log, stderr=subprocess.STDOUT)
        try:
            with snapshot_node() as (node, executor):
                client = node.create_client(ListNodes, f'/{name}/_container/list_nodes')
                assert client.wait_for_service(timeout_sec=10)
            yield f'/{name}', process
        finally:
            before_shutdown = process.poll()
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            # Pytest includes captured teardown output when a test fails.
            print(f'Container exit code before teardown: {before_shutdown}')
            print('Container log:\n' + '\n'.join(log_path.read_text().splitlines()[-100:]))


def call(node, executor, kind, service, request):
    client = node.create_client(kind, service)
    try:
        assert client.wait_for_service(timeout_sec=5), f'Service unavailable: {service}'
        future = client.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=10)
        assert future.done()
        return future.result()
    finally:
        node.destroy_client(client)


def members(container):
    with snapshot_node() as (node, executor):
        return call(node, executor, ListNodes, f'{container}/_container/list_nodes',
                    ListNodes.Request()).full_node_names


@pytest.mark.parametrize('operation', ['load', 'unload'])
def test_installed_default_interactive_menu_invalid_input_and_exit(operation):
    result = subprocess.run(['ros2', 'sensor_snapshot', operation], input='bad\n0\n',
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    title = 'Choose a topic type:' if operation == 'load' else 'Choose a snapshot type to unload:'
    assert result.stdout.count(title) == 2
    assert 'Invalid choice.' in result.stdout and '0  Exit' in result.stdout


def test_installed_loader_discovers_real_publisher_container_and_leaves_it_running(container):
    name, process = container
    topic = f'{name}/image'
    assert name in ContainerNameCompleter()(name, None)
    # The synthetic image publisher runs in the C++ container, with no camera device.
    with snapshot_node() as (node, executor):
        request = LoadNode.Request(package_name='image_tools', plugin_name='image_tools::Cam2Image',
                                   node_name='synthetic_camera', node_namespace=name,
                                   remap_rules=[f'image:={topic}'])
        request.parameters = [Parameter('burger_mode', value=True).to_parameter_msg()]
        response = call(node, executor, LoadNode, f'{name}/_container/load_node', request)
        assert response.success, response.error_message
    result = subprocess.run(['ros2', 'sensor_snapshot', 'load', topic, '--timeout', '10'],
                            capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr
    assert f'loading into {name}' in result.stdout and '[READY]' in result.stdout
    assert any('snapshot_' in member for member in members(name))
    grouped = subprocess.run(['ros2', 'sensor_snapshot', 'list', '--show-container'],
                             capture_output=True, text=True, timeout=10)
    assert grouped.returncode == 0, grouped.stderr
    assert name in grouped.stdout.splitlines()
    assert f'  2  {topic} [sensor_snapshot_interfaces/srv/GetImage]' in grouped.stdout
    repeated = subprocess.run(['ros2', 'sensor_snapshot', 'load', topic, '--timeout', '5'],
                              capture_output=True, text=True, timeout=10)
    assert repeated.returncode == 0, repeated.stderr
    assert '[SKIP]' in repeated.stdout
    assert len(members(name)) == 2
    removed = subprocess.run(['ros2', 'sensor_snapshot', 'unload', topic, '--timeout', '5'],
                             capture_output=True, text=True, timeout=10)
    assert removed.returncode == 0, removed.stdout + removed.stderr
    assert '[UNLOADED]' in removed.stdout
    assert process.poll() is None, 'Container exited after unloading the snapshot'
    assert members(name) == [f'{name}/synthetic_camera']
    assert process.poll() is None


def test_external_publisher_uses_only_configured_fallback_without_message_traffic(container):
    name, process = container
    context = Context()
    rclpy.init(args=[], context=context)
    node = rclpy.create_node(f'load_test_publisher_{os.getpid()}', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    topic = f'{name}/imu'
    publisher = node.create_publisher(Imu, topic, 1)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        result = subprocess.run([
            'ros2', 'sensor_snapshot', 'load', topic, '--fallback-container', name,
            '--timeout', '8'], capture_output=True, text=True, timeout=13)
        assert result.returncode == 0, result.stdout + result.stderr
        assert '[FALLBACK]' in result.stdout and '[READY]' in result.stdout
        assert len(members(name)) == 1
        assert process.poll() is None
    finally:
        executor.shutdown()
        thread.join(timeout=5)
        node.destroy_publisher(publisher)
        node.destroy_node()
        context.try_shutdown()
