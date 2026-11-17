# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
"""Only unload the component that owns the selected snapshot service."""

import argparse
from contextlib import contextmanager

from composition_interfaces.srv import UnloadNode
import pytest

from sensor_snapshot_cli.command import SensorSnapshotCommand
from sensor_snapshot_cli.discovery import SnapshotNameCompleter
from sensor_snapshot_cli import interactive, load, unload
from test_load import Graph


@pytest.fixture
def graph(monkeypatch):
    result = Graph()
    monkeypatch.setattr(load.time, 'monotonic', lambda: result.now)
    return result


def snapshot(graph, topic='/image', container='/container', component='/custom_snapshot'):
    graph.container(container, ['/sensor_driver', component])
    service_type = 'sensor_snapshot_interfaces/srv/GetImage'
    graph.services[topic] = [service_type]
    graph.owned_services[component] = {topic: [service_type]}


def requests(graph):
    return [(name, request) for _, name, request in graph.requests
            if isinstance(request, UnloadNode.Request)]


def test_unloads_actual_service_owner_without_a_publisher_or_assumed_node_name(graph, capsys):
    snapshot(graph)
    assert unload.Unloader(graph, graph, ['/image', '/image'], 3).run() == 0
    assert requests(graph)[0][0] == '/container/_container/unload_node'
    assert requests(graph)[0][1].unique_id == 2
    assert len(requests(graph)) == 1
    assert graph.members['/container'] == ['/sensor_driver']
    assert '/image' not in graph.services
    assert '[UNLOADED] /image' in capsys.readouterr().out


def test_rejection_and_unknown_component_owner_are_errors(graph):
    snapshot(graph)
    graph.results['/container'] = 'cannot unload'
    with pytest.raises(load.TopicError, match='cannot unload'):
        unload.Unloader(graph, graph, ['/image'], 3).run()
    graph.members['/container'] = []
    with pytest.raises(load.TopicError, match='no discoverable component container'):
        unload.Unloader(graph, graph, ['/image'], 2).run()


def test_ignore_errors_skips_wrong_services_and_missing_topics_and_unloads_others(graph, capsys):
    snapshot(graph)
    graph.services['/other'] = ['std_srvs/srv/Trigger']
    assert unload.Unloader(graph, graph, ['/other', '/missing', '/image'], 3,
                           ignore_errors=True).run() == 0
    output = capsys.readouterr()
    assert '[UNLOADED] /image' in output.out
    assert '[SKIP] /other' in output.out and '[SKIP] /missing' in output.out
    assert 'not a unique snapshot service' in output.err and 'not found' in output.err
    assert graph.now == pytest.approx(3)


def test_ambiguous_component_service_ownership_fails(graph):
    snapshot(graph)
    graph.container('/second', ['/duplicate_snapshot'])
    graph.owned_services['/duplicate_snapshot'] = dict(graph.owned_services['/custom_snapshot'])
    with pytest.raises(load.TopicError, match='multiple components'):
        unload.Unloader(graph, graph, ['/image'], 3).run()
    assert not requests(graph)


def test_waits_for_service_removal_and_obeys_the_global_deadline(graph):
    snapshot(graph)
    graph.service_delay = 10
    with pytest.raises(load.TopicError, match='snapshot service did not disappear'):
        unload.Unloader(graph, graph, ['/image'], 2).run()
    assert graph.now == pytest.approx(2)
    assert len(requests(graph)) == 1


def test_interactive_unload_returns_to_menu(graph, monkeypatch, capsys):
    snapshot(graph)
    answers = iter(['2', '1', '1', '0'])  # Image, snapshot, Back from empty list, Exit.
    monkeypatch.setattr('builtins.input', lambda prompt: next(answers))
    assert interactive.run(graph, graph, 3, unload=True) == 0
    assert len(requests(graph)) == 1
    output = capsys.readouterr().out
    assert '[UNLOADED] /image' in output and 'No Image topics found.' in output


def test_unload_parser_completion_and_default_interactive_mode(monkeypatch):
    parser = argparse.ArgumentParser()
    SensorSnapshotCommand().add_arguments(parser, 'ros2 sensor_snapshot')
    commands = next(action for action in parser._actions
                    if isinstance(action, argparse._SubParsersAction))
    topic_action = next(action for action in commands.choices['unload']._actions
                        if action.dest == 'topics')
    assert isinstance(topic_action.completer, SnapshotNameCompleter)
    assert parser.parse_args(['unload', '-i']).interactive
    calls = []

    @contextmanager
    def context():
        yield None, None

    monkeypatch.setattr(unload, 'snapshot_node', context)
    monkeypatch.setattr(interactive, 'run', lambda *args, **kwargs: calls.append(kwargs) or 0)
    assert unload.run(parser.parse_args(['unload'])) == 0
    assert calls == [{'unload': True}]
    with pytest.raises(ValueError, match='chooses its own'):
        unload.run(parser.parse_args(['unload', '-i', '/image']))


def test_unload_all_discovers_and_removes_every_snapshot(graph, monkeypatch):
    snapshot(graph, '/first/image', '/first', '/first_snapshot')
    snapshot(graph, '/second/image', '/second', '/second_snapshot')

    @contextmanager
    def context():
        yield graph, graph

    monkeypatch.setattr(unload, 'snapshot_node', context)
    parser = argparse.ArgumentParser()
    SensorSnapshotCommand().add_arguments(parser, 'ros2 sensor_snapshot')
    assert unload.run(parser.parse_args(['unload', '-a'])) == 0
    assert '/first/image' not in graph.services and '/second/image' not in graph.services
    assert len(requests(graph)) == 2
    assert graph.members['/first'] == graph.members['/second'] == ['/sensor_driver']
    with pytest.raises(ValueError, match='cannot be combined'):
        unload.run(parser.parse_args(['unload', '--all', '/first/image']))
    with pytest.raises(SystemExit):
        parser.parse_args(['unload', '-a', '-i'])


def test_unload_all_without_snapshots_logs_a_skip(graph, monkeypatch, capsys):
    @contextmanager
    def context():
        yield graph, graph

    monkeypatch.setattr(unload, 'snapshot_node', context)
    parser = argparse.ArgumentParser()
    SensorSnapshotCommand().add_arguments(parser, 'ros2 sensor_snapshot')
    assert unload.run(parser.parse_args(['unload', '--all'])) == 0
    assert '[SKIP] all: no snapshot services found' in capsys.readouterr().out
    assert not requests(graph)


def test_unload_all_discovery_uses_the_same_global_deadline(graph, monkeypatch):
    graph.services['/unmanaged'] = ['sensor_snapshot_interfaces/srv/GetImage']

    @contextmanager
    def context():
        yield graph, graph

    monkeypatch.setattr(unload, 'snapshot_node', context)
    parser = argparse.ArgumentParser()
    SensorSnapshotCommand().add_arguments(parser, 'ros2 sensor_snapshot')
    assert unload.run(parser.parse_args(['unload', '-a', '--timeout', '3'])) == 1
    assert graph.now == pytest.approx(3)
