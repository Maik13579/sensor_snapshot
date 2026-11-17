# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
"""Deterministic graph/RPC scenarios, including slow and missing topics."""

import argparse
from contextlib import contextmanager
from types import SimpleNamespace

from composition_interfaces.srv import ListNodes, LoadNode, UnloadNode
import pytest

from sensor_snapshot_cli.command import SensorSnapshotCommand
from sensor_snapshot_cli.composition import SNAPSHOT_TYPES
from sensor_snapshot_cli import interactive, load


def publisher(kind='Image', name='camera', namespace='/sensors'):
    return SimpleNamespace(topic_type=f'sensor_msgs/msg/{kind}',
                           node_name=name, node_namespace=namespace)


class Future:
    def __init__(self, graph, response, delay):
        self.graph, self.response = graph, response
        self.ready_at = graph.now + delay
        self.cancelled = False

    def done(self):
        return self.cancelled or self.graph.now >= self.ready_at

    def result(self):
        return self.response

    def cancel(self):
        self.cancelled = True


class Client:
    def __init__(self, graph, kind, name):
        self.graph, self.kind, self.name = graph, kind, name

    def service_is_ready(self):
        return self.name in self.graph.services

    def call_async(self, request):
        self.graph.requests.append((self.graph.now, self.name, request))
        container = self.name.split('/_container/')[0]
        if self.kind is ListNodes:
            names = self.graph.members[container]
            response = ListNodes.Response(
                full_node_names=names,
                unique_ids=[self.graph.component_ids[container][name] for name in names])
            return Future(self.graph, response, self.graph.list_delay)
        result = self.graph.results.get(container, '')
        if self.kind is UnloadNode:
            response = UnloadNode.Response(success=not result, error_message=result)
            if not result:
                name = next(name for name, uid in self.graph.component_ids[container].items()
                            if uid == request.unique_id)
                for topic in self.graph.owned_services.get(name, {}):
                    self.graph.at(self.graph.now + self.graph.service_delay,
                                  lambda topic=topic: self.graph.services.pop(topic, None))
                self.graph.members[container].remove(name)
            return Future(self.graph, response, self.graph.load_delays.get(container, 0.0))
        response = LoadNode.Response(success=not result, error_message=result)
        if not result and self.graph.create_services:
            topic = request.parameters[0].value.string_value
            kind = next(kind for kind in SNAPSHOT_TYPES.values()
                        if kind.plugin == request.plugin_name)
            self.graph.at(self.graph.now + self.graph.service_delay,
                          lambda: self.graph.services.update({topic: [kind.service]}))
        return Future(self.graph, response, self.graph.load_delays.get(container, 0.0))


class Graph:
    def __init__(self):
        self.now = 0.0
        self.services, self.publishers, self.members = {}, {}, {}
        self.component_ids, self.owned_services = {}, {}
        self.results, self.load_delays, self.requests, self.events = {}, {}, [], []
        self.create_services, self.service_delay, self.list_delay = True, 0.0, 0.0

    def get_service_names_and_types(self):
        return list(self.services.items())

    def get_publishers_info_by_topic(self, topic):
        return self.publishers.get(topic, [])

    def get_topic_names_and_types(self):
        return [(name, sorted({pub.topic_type for pub in pubs}))
                for name, pubs in self.publishers.items()]

    def get_service_names_and_types_by_node(self, name, namespace):
        full_name = f'{namespace.rstrip("/")}/{name}'
        return list(self.owned_services.get(full_name, {}).items())

    def create_client(self, kind, name):
        return Client(self, kind, name)

    def destroy_client(self, client):
        pass

    def at(self, when, action):
        self.events.append((when, action))

    def spin_once(self, timeout_sec):
        self.now += timeout_sec
        for when, action in list(self.events):
            if when <= self.now:
                self.events.remove((when, action))
                action()

    def container(self, name, members=()):
        self.members[name] = list(members)
        self.component_ids[name] = {member: index for index, member in enumerate(members, 1)}
        self.services.update({
            f'{name}/_container/list_nodes': ['composition_interfaces/srv/ListNodes'],
            f'{name}/_container/load_node': ['composition_interfaces/srv/LoadNode'],
            f'{name}/_container/unload_node': ['composition_interfaces/srv/UnloadNode'],
        })

    def loads(self):
        return [(stamp, service, request) for stamp, service, request in self.requests
                if isinstance(request, LoadNode.Request)]


@pytest.fixture
def graph(monkeypatch):
    graph = Graph()
    monkeypatch.setattr(load.time, 'monotonic', lambda: graph.now)
    return graph


def test_loads_ready_topics_while_others_are_missing_and_uses_one_deadline(graph, capsys):
    graph.container('/camera_container', ['/sensors/camera'])
    graph.publishers['/image'] = [publisher()]
    graph.at(2.5, lambda: graph.publishers.update({'/imu': [publisher('Imu')]}))
    loader = load.Loader(graph, graph, ['/missing', '/image', '/imu'], 4, ignore_errors=True)
    assert loader.run() == 0
    assert graph.now == pytest.approx(4)
    assert graph.loads()[0][0] < 1
    assert graph.loads()[1][0] >= 2.5
    assert '[READY] /image' in capsys.readouterr().out
    assert [topic.stage for topic in loader.topics] == ['failed', 'done', 'done']


@pytest.mark.parametrize('kind', ['Image', 'LaserScan', 'PointCloud2', 'Imu'])
def test_plugin_mapping_parameters_and_intra_process(graph, kind):
    graph.container('/sensors/container', ['/sensors/camera'])
    graph.publishers['/data'] = [publisher(kind)]
    assert load.Loader(graph, graph, ['/data', '/data'], 5).run() == 0
    assert len(graph.loads()) == 1
    _, service, request = graph.loads()[0]
    assert service == '/sensors/container/_container/load_node'
    assert request.package_name == 'sensor_snapshot'
    assert request.plugin_name == SNAPSHOT_TYPES[f'sensor_msgs/msg/{kind}'].plugin
    assert request.parameters[0].name == 'input.topic'
    assert request.parameters[0].value.string_value == '/data'
    assert request.extra_arguments[0].name == 'use_intra_process_comms'
    assert request.extra_arguments[0].value.bool_value


@pytest.mark.parametrize('publishers,expected', [
    ([publisher(), publisher()], 'multiple publishers'),
    ([publisher('String')], 'unsupported topic type'),
])
def test_invalid_topics_fail_or_are_logged_and_skipped(graph, capsys, publishers, expected):
    graph.publishers['/bad'] = publishers
    with pytest.raises(load.TopicError, match=expected):
        load.Loader(graph, graph, ['/bad'], 3).run()
    assert not graph.loads()
    assert load.Loader(graph, graph, ['/bad'], 3, ignore_errors=True).run() == 0
    output = capsys.readouterr()
    assert expected in output.err and '[SKIP] /bad' in output.out


def test_skips_existing_snapshot_without_publisher_and_logs_conflicts(graph, capsys):
    graph.services['/image'] = ['sensor_snapshot_interfaces/srv/GetImage']
    assert load.Loader(graph, graph, ['/image'], 3).run() == 0
    assert '[SKIP] /image: snapshot service already exists' in capsys.readouterr().out
    graph.services['/image'] = ['std_srvs/srv/Trigger']
    with pytest.raises(load.TopicError, match='conflicting service'):
        load.Loader(graph, graph, ['/image'], 3).run()


def test_optional_fallback_handles_missing_owner_and_explicit_rejection(graph):
    graph.publishers['/image'] = [publisher()]
    with pytest.raises(load.TopicError, match='no discoverable component container'):
        load.Loader(graph, graph, ['/image'], 5).run()
    graph.container('/primary', ['/sensors/camera'])
    graph.container('/fallback')
    graph.results['/primary'] = 'plugin unavailable'
    assert load.Loader(graph, graph, ['/image'], 5, fallback='/fallback').run() == 0
    assert [service for _, service, _ in graph.loads()] == [
        '/primary/_container/load_node', '/fallback/_container/load_node']


def test_explicit_target_overrides_owner_and_fallback_does_not_get_created(graph):
    graph.publishers['/image'] = [publisher()]
    graph.container('/primary', ['/sensors/camera'])
    graph.container('/selected')
    assert load.Loader(graph, graph, ['/image'], 5, target='/selected').run() == 0
    assert graph.loads()[0][1] == '/selected/_container/load_node'
    graph.services.pop('/image')
    graph.results['/primary'] = 'load rejected'
    with pytest.raises(load.TopicError, match='global timeout'):
        load.Loader(graph, graph, ['/image'], 5, fallback='/does_not_exist').run()
    assert '/does_not_exist' not in graph.members


def test_fallback_can_appear_later_within_the_global_timeout(graph):
    graph.publishers['/image'] = [publisher()]
    graph.at(2.5, lambda: graph.container('/fallback'))
    assert load.Loader(graph, graph, ['/image'], 5, fallback='/fallback').run() == 0
    assert 2.5 <= graph.loads()[0][0] < 5


def test_slow_load_does_not_block_other_topics_and_is_not_retried(graph, capsys):
    graph.container('/slow', ['/sensors/slow'])
    graph.container('/fast', ['/sensors/fast'])
    graph.publishers.update({'/slow': [publisher(name='slow')], '/fast': [publisher(name='fast')]})
    graph.load_delays['/slow'] = 20
    graph.service_delay = 20
    # Only the fast request exposes its service before the deadline.
    graph.at(0.5, lambda: graph.services.update({
        '/fast': ['sensor_snapshot_interfaces/srv/GetImage']}))
    assert load.Loader(graph, graph, ['/slow', '/fast'], 3, ignore_errors=True).run() == 0
    assert len(graph.loads()) == 2
    assert all(stamp < 1 for stamp, _, _ in graph.loads())
    output = capsys.readouterr()
    assert '[READY] /fast' in output.out and 'global timeout' in output.err
    assert graph.now == pytest.approx(3)


def test_service_confirmation_is_required_and_missing_topics_fail_at_deadline(graph):
    graph.container('/container', ['/sensors/camera'])
    graph.publishers['/image'] = [publisher()]
    graph.create_services = False
    with pytest.raises(load.TopicError, match='snapshot service did not become available'):
        load.Loader(graph, graph, ['/image'], 2).run()
    assert graph.now == pytest.approx(2)
    with pytest.raises(load.TopicError, match='no publisher found'):
        load.Loader(graph, graph, ['/missing'], 2).run()
    assert graph.now == pytest.approx(4)


def test_ignore_errors_continues_loading_good_topics(graph, capsys):
    graph.publishers.update({'/bad': [publisher('String')], '/good': [publisher()]})
    graph.container('/fallback')
    assert load.Loader(graph, graph, ['/bad', '/good'], 3, fallback='/fallback',
                       ignore_errors=True).run() == 0
    assert len(graph.loads()) == 1
    assert '[READY] /good' in capsys.readouterr().out


def test_numbered_menus_redraw_after_invalid_input_and_support_back(monkeypatch, capsys):
    answers = iter(['x', '-1', '9', '2', '3'])
    monkeypatch.setattr('builtins.input', lambda prompt: next(answers))
    assert interactive.choose('Topics', ['/a', '/b']) == 1
    assert interactive.choose('Topics', ['/a', '/b']) is None
    assert capsys.readouterr().out.count('Invalid choice.') == 3


def test_interactive_menu_prioritizes_owner_and_loads_user_choice(graph, monkeypatch, capsys):
    graph.container('/z_owner', ['/sensors/camera'])
    graph.container('/a_other')
    graph.publishers['/image'] = [publisher()]
    # Image -> topic -> second container -> Back -> Exit.
    answers = iter(['2', '1', '2', '2', '0'])
    monkeypatch.setattr('builtins.input', lambda prompt: next(answers))
    assert interactive.run(graph, graph, 3) == 0
    assert graph.loads()[0][1] == '/a_other/_container/load_node'
    output = capsys.readouterr().out
    assert "1  /z_owner  [publisher's container]" in output
    assert '2  /a_other' in output and '[READY] /image' in output


def test_interactive_error_returns_to_menu_and_eof_exits(graph, monkeypatch, capsys):
    graph.publishers['/scan'] = [publisher('LaserScan'), publisher('LaserScan')]
    answers = iter(['1', '1', '2', '0'])
    monkeypatch.setattr('builtins.input', lambda prompt: next(answers))
    assert interactive.run(graph, graph, 3) == 0
    assert 'multiple publishers' in capsys.readouterr().err
    monkeypatch.setattr('builtins.input', lambda prompt: (_ for _ in ()).throw(EOFError()))
    assert interactive.run(graph, graph, 3) == 0


@pytest.mark.parametrize('choices', [['2', '0'], ['2', '1', '0']])
def test_zero_exits_from_topic_and_container_menus(graph, monkeypatch, choices):
    graph.publishers['/image'] = [publisher()]
    graph.container('/container', ['/sensors/camera'])
    answers = iter(choices)
    monkeypatch.setattr('builtins.input', lambda prompt: next(answers))
    assert interactive.run(graph, graph, 3) == 0
    assert not graph.loads()


def test_load_parser_defaults_validation_and_completion_routing(monkeypatch):
    command = SensorSnapshotCommand()
    parser = argparse.ArgumentParser()
    command.add_arguments(parser, 'ros2 sensor_snapshot', argv=['load'])
    args = parser.parse_args(['load', '/image', '--fallback-container', '/container'])
    assert args.timeout == 15 and args.operation == 'load' and not args.interactive
    assert parser.parse_args(['load', '-i']).interactive
    for value in ('0', '-1', 'nan', 'inf'):
        with pytest.raises(SystemExit):
            parser.parse_args(['load', '/image', '--timeout', value])
    calls = []

    @contextmanager
    def context():
        yield None, None

    monkeypatch.setattr(load, 'snapshot_node', context)
    monkeypatch.setattr(interactive, 'run', lambda *args: calls.append(args) or 0)
    assert load.run(parser.parse_args(['load'])) == 0
    assert len(calls) == 1
    with pytest.raises(ValueError, match='chooses its own'):
        load.run(parser.parse_args(['load', '-i', '/image']))


def test_topic_and_container_completion_use_bounded_graph_queries_without_rpcs(graph, monkeypatch):
    graph.container('/camera_container')
    graph.container('/other_container')
    graph.publishers['/image'] = [publisher()]
    graph.publishers['/unsupported'] = [publisher('String')]

    @contextmanager
    def context():
        yield graph, graph

    monkeypatch.setattr(load, 'snapshot_node', context)
    assert load.ContainerNameCompleter()('/camera', None) == ['/camera_container']
    assert load.TopicNameCompleter()('/', None) == ['/image']
    assert graph.now < 1.2
    assert not graph.requests
