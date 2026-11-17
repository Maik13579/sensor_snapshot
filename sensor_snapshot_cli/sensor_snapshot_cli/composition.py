# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
"""Nonblocking component service calls and publisher/container discovery."""

from dataclasses import dataclass

from composition_interfaces.srv import ListNodes, LoadNode, UnloadNode
from rclpy.parameter import Parameter


@dataclass(frozen=True)
class SnapshotType:
    label: str
    plugin: str
    service: str


SNAPSHOT_TYPES = {
    f'sensor_msgs/msg/{message}': SnapshotType(
        message, f'sensor_snapshot::{component}',
        f'sensor_snapshot_interfaces/srv/{service}')
    for message, component, service in (
        ('LaserScan', 'LaserScanSnapshot', 'GetLaserScan'),
        ('Image', 'ImageSnapshot', 'GetImage'),
        ('PointCloud2', 'PointCloudSnapshot', 'GetPointCloud'),
        ('Imu', 'ImuSnapshot', 'GetImu'),
    )
}


def topic_publisher(node, topic):
    """Require one supported publisher endpoint; receiving messages is unnecessary."""
    publishers = node.get_publishers_info_by_topic(topic)
    if not publishers:
        return None
    if len(publishers) != 1:
        raise ValueError(f'multiple publishers found ({len(publishers)}); expected exactly one')
    publisher = publishers[0]
    if publisher.topic_type not in SNAPSHOT_TYPES:
        raise ValueError(f'unsupported topic type: {publisher.topic_type}')
    return publisher


def publisher_name(publisher):
    return f'{publisher.node_namespace.rstrip("/")}/{publisher.node_name}'


def container_names(services):
    """Find managers by their typed services, including hidden _container names."""
    suffix = '/_container/list_nodes'
    return sorted(name[:-len(suffix)] for name, types in services.items()
                  if name.endswith(suffix) and 'composition_interfaces/srv/ListNodes' in types)


class ServiceCall:
    """Advance a service call without blocking other topics or extending a deadline."""

    def __init__(self, node, service_type, name, request, deadline):
        self.node = node
        self.name = name
        self.request = request
        self.deadline = deadline
        self.client = node.create_client(service_type, name)
        self.future = None
        self.response = None
        self.error = None
        self.done = False

    def tick(self, now):
        if self.done:
            return
        try:
            if self.future is not None and self.future.done():
                self.response = self.future.result()
                if self.response is None:
                    self.error = f'no response from {self.name}'
                self.done = True
            elif now >= self.deadline:
                self.error = f'timed out calling {self.name}'
                self.done = True
            elif self.future is None:
                if self.client.service_is_ready():
                    self.future = self.client.call_async(self.request)
        except Exception as error:
            self.error = f'{self.name}: {error}'
            self.done = True
        if self.done:
            self.close()

    def close(self):
        if self.future is not None and not self.future.done():
            self.future.cancel()
        if self.client is not None:
            self.node.destroy_client(self.client)
            self.client = None


class ContainerIndex:
    """Refresh manager membership asynchronously while other topics can load."""

    def __init__(self, node):
        self.node = node
        self.calls = {}
        self.members = {}
        self.components = {}

    def refresh(self, services, now, deadline):
        names = container_names(services)
        for name in list(self.calls):
            if name not in names:
                self.calls.pop(name).close()
                self.members.pop(name, None)
                self.components.pop(name, None)
        for name in names:
            if name not in self.calls or self.calls[name].done:
                self.calls[name] = ServiceCall(
                    self.node, ListNodes, f'{name}/_container/list_nodes', ListNodes.Request(),
                    min(deadline, now + 1.0))

    def tick(self, now):
        for name, call in self.calls.items():
            call.tick(now)
            if call.done:
                self.members[name] = (() if call.error else call.response.full_node_names)
                self.components[name] = ([] if call.error else list(zip(
                    call.response.full_node_names, call.response.unique_ids)))

    @property
    def pending(self):
        return any(not call.done for call in self.calls.values())

    def owners(self, publisher):
        name = publisher_name(publisher)
        return sorted(container for container, members in self.members.items() if name in members)

    def close(self):
        for call in self.calls.values():
            call.close()


def load_component(node, topic, snapshot_type, container, deadline):
    request = LoadNode.Request()
    request.package_name = 'sensor_snapshot'
    request.plugin_name = snapshot_type.plugin
    request.parameters = [Parameter('input.topic', value=topic).to_parameter_msg()]
    request.extra_arguments = [
        Parameter('use_intra_process_comms', value=True).to_parameter_msg()]
    return ServiceCall(node, LoadNode, f'{container}/_container/load_node', request, deadline)


def unload_component(node, container, unique_id, deadline):
    return ServiceCall(node, UnloadNode, f'{container}/_container/unload_node',
                       UnloadNode.Request(unique_id=unique_id), deadline)


def component_services(node, index):
    """Enumerate each component's services once, tolerating components that disappear."""
    from rclpy.node import NodeNameNonExistentError

    for container, components in index.components.items():
        for full_name, unique_id in components:
            namespace, _, name = full_name.rpartition('/')
            try:
                services = dict(node.get_service_names_and_types_by_node(name, namespace or '/'))
            except NodeNameNonExistentError:
                continue  # Components may disappear between the two graph queries.
            yield container, unique_id, full_name, services


def service_components(node, index, topic, service_type):
    """Find component IDs through their actual service ownership, rather than node names."""
    return [(container, uid, name)
            for container, uid, name, services in component_services(node, index)
            if service_type in services.get(topic, ())]
