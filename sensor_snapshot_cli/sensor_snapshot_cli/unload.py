# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
"""Remove snapshot components identified by the services they actually provide."""

from dataclasses import dataclass
import time

from .composition import ContainerIndex, service_components, unload_component
from .discovery import discover, SERVICE_PAYLOADS, SnapshotNameCompleter, snapshot_node
from .load import log, normalize, TopicError


def add_arguments(parser, positive_seconds):
    topics = parser.add_argument('topics', nargs='*', metavar='TOPIC',
                                 help='Existing snapshots to unload')
    topics.completer = SnapshotNameCompleter()
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('-i', '--interactive', action='store_true',
                       help='Choose existing snapshots using numbered menus')
    modes.add_argument('-a', '--all', dest='all_snapshots', action='store_true',
                       help='Unload all discovered snapshot components')
    parser.add_argument('--timeout', type=positive_seconds, default=15.0,
                        help='Global timeout for the whole batch (default: 15 seconds)')
    parser.add_argument('--ignore-errors', action='store_true',
                        help='Log and skip failed topics, continue and exit successfully')


@dataclass
class Snapshot:
    name: str
    stage: str = 'discovery'
    service_type: str = ''
    call: object = None
    target: str = ''
    waiting_logged: bool = False
    next_lookup: float = 0.0


class Unloader:
    def __init__(self, node, executor, topics, timeout, ignore_errors=False):
        self.node = node
        self.executor = executor
        self.snapshots = [Snapshot(name) for name in dict.fromkeys(topics)]
        self.deadline = time.monotonic() + timeout
        self.ignore_errors = ignore_errors
        self.index = ContainerIndex(node)

    def fail(self, snapshot, message):
        log('ERROR', snapshot.name, message)
        snapshot.stage = 'failed'
        if not self.ignore_errors:
            raise TopicError(message)
        log('SKIP', snapshot.name, 'continuing because --ignore-errors is set')

    def observe_graph(self, now):
        services = dict(self.node.get_service_names_and_types())
        for snapshot in self.pending():
            types = services.get(snapshot.name)
            if not types:
                if snapshot.stage in ('unloading', 'verification'):
                    log('UNLOADED', snapshot.name, 'snapshot service removed')
                    snapshot.stage = 'done'
                elif snapshot.stage == 'container':
                    log('SKIP', snapshot.name, 'snapshot service no longer exists')
                    snapshot.stage = 'done'
                elif not snapshot.waiting_logged:
                    log('WAIT', snapshot.name, 'waiting for snapshot service')
                    snapshot.waiting_logged = True
                continue
            if len(types) != 1 or types[0] not in SERVICE_PAYLOADS:
                self.fail(snapshot, f'not a unique snapshot service: {", ".join(types)}')
                continue
            if snapshot.service_type and snapshot.service_type != types[0]:
                self.fail(snapshot, 'snapshot service type changed during unloading')
                continue
            snapshot.service_type = types[0]
            if snapshot.stage == 'discovery':
                snapshot.stage = 'container'
                log('WAIT', snapshot.name, 'locating snapshot component')
        if now < self.deadline and any(s.stage == 'container' for s in self.snapshots):
            self.index.refresh(services, now, self.deadline)

    def advance(self, now):
        if now >= self.deadline:
            return
        self.index.tick(now)
        for snapshot in self.pending():
            lookup_ready = snapshot.stage == 'container' and not self.index.pending
            if lookup_ready and now >= snapshot.next_lookup:
                matches = service_components(
                    self.node, self.index, snapshot.name, snapshot.service_type)
                if len(matches) > 1:
                    self.fail(snapshot, 'multiple components provide this snapshot service')
                    continue
                if matches:
                    container, unique_id, node_name = matches[0]
                    snapshot.target = container
                    log('UNLOAD', snapshot.name, f'unloading {node_name} from {container}')
                    snapshot.call = unload_component(
                        self.node, container, unique_id, self.deadline)
                    snapshot.stage = 'unloading'
                elif not self.index.pending:
                    snapshot.next_lookup = now + 1.0
            if snapshot.stage == 'unloading':
                snapshot.call.tick(now)
                if not snapshot.call.done:
                    continue
                if snapshot.call.error:
                    self.fail(snapshot, snapshot.call.error)
                elif not snapshot.call.response.success:
                    self.fail(snapshot, snapshot.call.response.error_message or 'unload rejected')
                else:
                    snapshot.stage = 'verification'

    def pending(self):
        return [snapshot for snapshot in self.snapshots if snapshot.stage not in ('done', 'failed')]

    def run(self):
        next_poll = 0.0
        try:
            while self.pending():
                now = time.monotonic()
                if now >= next_poll or now >= self.deadline:
                    self.observe_graph(now)
                    next_poll = now + 1.0
                if now >= self.deadline:
                    reasons = {
                        'discovery': 'snapshot service not found',
                        'container': 'snapshot has no discoverable component container',
                        'unloading': 'unload response or service removal still pending',
                        'verification': 'snapshot service did not disappear',
                    }
                    for snapshot in self.pending():
                        self.fail(snapshot, f'global timeout: {reasons[snapshot.stage]}')
                    break
                self.advance(time.monotonic())
                if self.pending():
                    self.executor.spin_once(timeout_sec=max(
                        0.0, min(0.05, self.deadline - time.monotonic())))
            return 0
        finally:
            self.index.close()
            for snapshot in self.snapshots:
                if snapshot.call is not None:
                    snapshot.call.close()


def run(args):
    if args.all_snapshots and args.topics:
        raise ValueError('--all cannot be combined with topic arguments')
    interactive_mode = args.interactive or (not args.topics and not args.all_snapshots)
    if args.interactive and args.topics:
        raise ValueError('Interactive mode chooses its own topics')
    try:
        with snapshot_node() as (node, executor):
            if interactive_mode:
                from .interactive import run as interactive_run
                return interactive_run(node, executor, args.timeout, unload=True)
            timeout = args.timeout
            if args.all_snapshots:
                deadline = time.monotonic() + timeout
                topics = [name for name, _ in discover(node, executor, min(1.0, timeout))]
                if not topics:
                    log('SKIP', 'all', 'no snapshot services found')
                    return 0
                timeout = max(0.0, deadline - time.monotonic())
            else:
                topics = [normalize(node, name) for name in args.topics]
            return Unloader(node, executor, topics, timeout, args.ignore_errors).run()
    except TopicError:
        return 1
