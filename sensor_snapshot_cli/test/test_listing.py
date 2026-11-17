# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
"""Grouped listings show actual owning containers, component IDs and unknown owners."""

import pytest

from sensor_snapshot_cli import listing
from test_load import Graph


@pytest.fixture
def graph(monkeypatch):
    result = Graph()
    monkeypatch.setattr(listing.time, 'monotonic', lambda: result.now)
    return result


def test_grouped_listing_only_shows_snapshots_and_separates_unknown_owners(graph, capsys):
    image = 'sensor_snapshot_interfaces/srv/GetImage'
    scan = 'sensor_snapshot_interfaces/srv/GetLaserScan'
    graph.container('/z_container', ['/driver', '/custom_image_snapshot'])
    graph.container('/a_container', ['/scan_snapshot'])
    graph.owned_services['/custom_image_snapshot'] = {'/image': [image]}
    graph.owned_services['/scan_snapshot'] = {'/scan': [scan]}
    snapshots = [('/image', [image]), ('/scan', [scan]), ('/unmanaged', [image])]
    listing.show_containers(graph, graph, snapshots, 3)
    assert capsys.readouterr().out.splitlines() == [
        '/a_container', f'  1  /scan [{scan}]',
        '/z_container', f'  2  /image [{image}]',
        'Container not identified:', f'  /unmanaged [{image}]',
    ]


def test_grouped_listing_tolerates_unresponsive_containers_within_timeout(graph, capsys):
    graph.container('/unresponsive', ['/snapshot'])
    graph.list_delay = 10
    service_type = 'sensor_snapshot_interfaces/srv/GetImage'
    listing.show_containers(graph, graph, [('/image', [service_type])], 0.3)
    assert graph.now == pytest.approx(0.3)
    assert 'Container not identified:' in capsys.readouterr().out
