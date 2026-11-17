# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
"""Flat snapshot listing or a view grouped by owning component container."""

from collections import defaultdict
import time

from .composition import component_services, ContainerIndex
from .discovery import discover, snapshot_node


def show_containers(node, executor, snapshots, timeout):
    index = ContainerIndex(node)
    deadline = time.monotonic() + timeout
    try:
        index.refresh(dict(node.get_service_names_and_types()), time.monotonic(), deadline)
        while index.pending and time.monotonic() < deadline:
            index.tick(time.monotonic())
            if index.pending:
                executor.spin_once(timeout_sec=max(0.0, min(0.05, deadline - time.monotonic())))
        index.tick(time.monotonic())
        available = dict(snapshots)
        groups = defaultdict(set)
        located = set()
        for container, uid, _, services in component_services(node, index):
            for topic, types in services.items():
                if topic in available and set(types) & set(available[topic]):
                    groups[container].add((uid, topic))
                    located.add(topic)
        for container, entries in sorted(groups.items()):
            print(container)
            for uid, topic in sorted(entries):
                print(f'  {uid}  {topic} [{", ".join(available[topic])}]')
        unlocated = sorted(available.keys() - located)
        if unlocated:
            print('Container not identified:')
            for topic in unlocated:
                print(f'  {topic} [{", ".join(available[topic])}]')
    finally:
        index.close()


def run(args):
    with snapshot_node() as (node, executor):
        snapshots = discover(node, executor, args.discovery_time)
        if args.show_container:
            show_containers(node, executor, snapshots, args.timeout)
        else:
            for name, types in snapshots:
                print(f'{name} [{", ".join(types)}]')
    return 0
