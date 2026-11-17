# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
"""Incrementally load snapshot components with one deadline for the entire batch."""

from dataclasses import dataclass
import sys
import time

from rclpy.expand_topic_name import expand_topic_name

from .composition import (
    ContainerIndex, container_names, load_component, SNAPSHOT_TYPES, topic_publisher,
)
from .discovery import SERVICE_PAYLOADS, snapshot_node


class TopicNameCompleter:
    def names(self, node):
        return [name for name, types in node.get_topic_names_and_types()
                if set(types) & SNAPSHOT_TYPES.keys()]

    def __call__(self, prefix, parsed_args, **kwargs):
        try:
            with snapshot_node() as (node, executor):
                deadline = time.monotonic() + 0.5
                while time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.05)
                return sorted(name for name in self.names(node) if name.startswith(prefix))
        except (Exception, KeyboardInterrupt):
            return []


class ContainerNameCompleter(TopicNameCompleter):
    """Complete manager names from graph metadata, without component service calls."""

    def names(self, node):
        return container_names(dict(node.get_service_names_and_types()))


def add_arguments(parser, positive_seconds):
    topics = parser.add_argument(
        'topics', nargs='*', metavar='TOPIC',
        help='Topics to equip with snapshot services')
    topics.completer = TopicNameCompleter()
    parser.add_argument('-i', '--interactive', action='store_true',
                        help='Choose topics and target containers using numbered menus')
    fallback = parser.add_argument(
        '--fallback-container', metavar='CONTAINER',
        help='Existing container to use when loading beside the publisher fails')
    fallback.completer = ContainerNameCompleter()
    parser.add_argument('--timeout', type=positive_seconds, default=15.0,
                        help='Global startup timeout for the whole batch (default: 15 seconds)')
    parser.add_argument('--ignore-errors', action='store_true',
                        help='Log and skip failed topics, continue and exit successfully')


def log(event, topic, message):
    print(f'[{event}] {topic}: {message}',
          file=sys.stderr if event == 'ERROR' else sys.stdout, flush=True)


class TopicError(ValueError):
    """A topic failure that has already been logged."""


@dataclass
class Topic:
    name: str
    publisher: object = None
    snapshot_type: object = None
    call: object = None
    target: str = ''
    stage: str = 'publisher'
    waiting_logged: bool = False
    fallback_used: bool = False


class Loader:
    def __init__(self, node, executor, topics, timeout, fallback=None, ignore_errors=False,
                 target=None):
        self.node = node
        self.executor = executor
        self.topics = [Topic(name) for name in dict.fromkeys(topics)]
        self.deadline = time.monotonic() + timeout
        self.fallback = fallback
        self.ignore_errors = ignore_errors
        self.target = target
        self.index = ContainerIndex(node)

    def fail(self, topic, message):
        log('ERROR', topic.name, message)
        topic.stage = 'failed'
        if not self.ignore_errors:
            raise TopicError(message)
        log('SKIP', topic.name, 'continuing because --ignore-errors is set')

    def observe_service(self, topic, services):
        types = services.get(topic.name)
        if not types:
            return False
        expected = topic.snapshot_type.service if topic.snapshot_type else None
        conflict = len(types) != 1 or types[0] not in SERVICE_PAYLOADS
        if expected and expected != types[0]:
            conflict = True
        if conflict:
            self.fail(topic, f'conflicting service types: {", ".join(types)}')
        else:
            event = 'SKIP' if topic.stage in ('publisher', 'container') else 'READY'
            message = ('snapshot service already exists' if event == 'SKIP'
                       else 'snapshot service available')
            log(event, topic.name, message)
            topic.stage = 'done'
        return True

    def observe_graph(self, now):
        services = dict(self.node.get_service_names_and_types())
        for topic in self.topics:
            if topic.stage in ('done', 'failed') or self.observe_service(topic, services):
                continue
            if topic.stage not in ('publisher', 'container'):
                continue
            try:
                topic.publisher = topic_publisher(self.node, topic.name)
                if topic.publisher is not None:
                    topic.snapshot_type = SNAPSHOT_TYPES[topic.publisher.topic_type]
                    topic.stage = 'container'
                else:
                    topic.stage = 'publisher'
                    if not topic.waiting_logged:
                        log('WAIT', topic.name, 'waiting for publisher')
                        topic.waiting_logged = True
            except ValueError as error:
                self.fail(topic, str(error))
        if not self.target and any(topic.stage == 'container' for topic in self.topics):
            self.index.refresh(services, now, self.deadline)

    def start_load(self, topic, container):
        topic.target = container
        topic.fallback_used = container == self.fallback
        log('LOAD', topic.name, f'loading into {container}')
        topic.call = load_component(
            self.node, topic.name, topic.snapshot_type, container, self.deadline)
        topic.stage = 'loading'

    def fallback_or_fail(self, topic, message, now):
        if self.fallback and not topic.fallback_used and now < self.deadline:
            log('FALLBACK', topic.name, f'{message}; trying {self.fallback}')
            self.start_load(topic, self.fallback)
        else:
            self.fail(topic, message)

    def advance(self, now):
        if now >= self.deadline:
            return
        self.index.tick(now)
        for topic in self.topics:
            if topic.stage == 'container':
                owners = self.index.owners(topic.publisher) if not self.target else []
                if len(owners) > 1:
                    self.fail(topic, f'ambiguous publisher container: {", ".join(owners)}')
                    continue
                target = self.target or (owners[0] if owners else None)
                if target:
                    self.start_load(topic, target)
                elif not self.index.pending:
                    self.fallback_or_fail(
                        topic, 'publisher has no discoverable component container', now)
            if topic.stage == 'loading':
                topic.call.tick(now)
                if not topic.call.done:
                    continue
                if topic.call.error:
                    self.fallback_or_fail(topic, topic.call.error, now)
                elif not topic.call.response.success:
                    self.fallback_or_fail(
                        topic, topic.call.response.error_message or 'load rejected', now)
                else:
                    topic.stage = 'service'

    def pending(self):
        return [topic for topic in self.topics if topic.stage not in ('done', 'failed')]

    def run(self):
        next_poll = 0.0
        try:
            while self.pending():
                now = time.monotonic()
                if now >= self.deadline:
                    services = dict(self.node.get_service_names_and_types())
                    for topic in self.pending():
                        if not self.observe_service(topic, services):
                            reason = ('no publisher found' if topic.stage == 'publisher'
                                      else 'snapshot service did not become available')
                            self.fail(topic, f'global timeout: {reason}')
                    break
                if now >= next_poll:
                    self.observe_graph(now)
                    next_poll = now + 1.0
                self.advance(time.monotonic())
                if self.pending():
                    self.executor.spin_once(timeout_sec=max(
                        0.0, min(0.05, self.deadline - time.monotonic())))
            return 0
        finally:
            self.index.close()
            for topic in self.topics:
                if topic.call is not None:
                    topic.call.close()


def normalize(node, name):
    return expand_topic_name(name, node.get_name(), node.get_namespace())


def run(args):
    interactive_mode = args.interactive or not args.topics
    if interactive_mode and (args.topics or args.fallback_container):
        raise ValueError('Interactive mode chooses its own topics and target containers')
    try:
        with snapshot_node() as (node, executor):
            if interactive_mode:
                from .interactive import run as interactive_run
                return interactive_run(node, executor, args.timeout)
            topics = [normalize(node, name) for name in args.topics]
            fallback = normalize(node, args.fallback_container) if args.fallback_container else None
            return Loader(node, executor, topics, args.timeout, fallback, args.ignore_errors).run()
    except TopicError:
        return 1
