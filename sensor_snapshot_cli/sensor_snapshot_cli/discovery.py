# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
"""Live service discovery, bounded one-shot calls and shell completion."""

from contextlib import contextmanager
import os
import time


SERVICE_PAYLOADS = {
    'sensor_snapshot_interfaces/srv/GetImage': 'image',
    'sensor_snapshot_interfaces/srv/GetPointCloud': 'cloud',
    'sensor_snapshot_interfaces/srv/GetImu': 'imu',
}


def snapshot_services(graph):
    """Keep only supported services, sorting names/types for stable output."""
    return sorted(
        (name, sorted(set(types) & SERVICE_PAYLOADS.keys()))
        for name, types in graph if set(types) & SERVICE_PAYLOADS.keys())


@contextmanager
def snapshot_node():
    """Own our context so completion and repeated invocations clean up reliably."""
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor

    context = Context()
    node = None
    executor = None
    try:
        rclpy.init(args=[], context=context)
        node = rclpy.create_node(
            f'_sensor_snapshot_cli_{os.getpid()}', context=context,
            start_parameter_services=False)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        yield node, executor
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        context.try_shutdown()


def discover(node, executor, timeout, service=None):
    """Allow graph discovery time; a named service returns as soon as it appears."""
    deadline = time.monotonic() + timeout
    while True:
        graph = node.get_service_names_and_types()
        if service is not None:
            for name, types in graph:
                if name == service:
                    if len(types) != 1:
                        raise ValueError(f'Service {name} advertises multiple types: {types}')
                    if types[0] not in SERVICE_PAYLOADS:
                        raise ValueError(f'{name} is not a sensor snapshot service ({types[0]})')
                    return types[0]
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            if service is not None:
                raise ValueError(f'No snapshot service found at {service}')
            return snapshot_services(graph)
        executor.spin_once(timeout_sec=min(0.05, remaining))


def request_snapshot(node, executor, service, service_type, values, timeout):
    """Call exactly once; unsuccessful responses never become usable payloads."""
    from rosidl_runtime_py import set_message_fields
    from rosidl_runtime_py.utilities import get_service
    from sensor_snapshot_interfaces.msg import SnapshotStatus

    service_class = get_service(service_type)
    request = service_class.Request()
    try:
        set_message_fields(request, values)
    except (AttributeError, TypeError, ValueError, AssertionError) as error:
        raise ValueError(f'Invalid snapshot request: {error}') from error
    client = node.create_client(service_class, service)
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            raise ValueError(f'Timed out waiting for {service}')
        future = client.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=timeout)
        if not future.done():
            future.cancel()
            raise ValueError(f'Timed out requesting a snapshot from {service}')
        response = future.result()
        if response is None:
            raise ValueError(f'No response from {service}')
        if response.status.code != SnapshotStatus.OK:
            raise ValueError(
                f'Snapshot failed (status {response.status.code}): {response.status.message}')
        return response
    finally:
        node.destroy_client(client)


class SnapshotNameCompleter:
    """argcomplete hook: complete service names matching the snapshot protocols."""

    def __call__(self, prefix, parsed_args, **kwargs):
        try:
            with snapshot_node() as (node, executor):
                services = discover(node, executor, timeout=0.5)
            return [name for name, _ in services if name.startswith(prefix)]
        except (Exception, KeyboardInterrupt):
            # Completion must never print exceptions into an interactive shell.
            return []
