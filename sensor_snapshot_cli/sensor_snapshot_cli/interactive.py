# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
"""Numbered topic/container menus for ros2 sensor_snapshot load --interactive."""

import time

from .composition import ContainerIndex, container_names, SNAPSHOT_TYPES, topic_publisher
from .discovery import SERVICE_PAYLOADS
from .load import Loader, log, TopicError


class InteractiveExit(Exception):
    """Exit all menus when the user enters zero."""


def choose(title, labels, back='Back'):
    """Redraw invalid choices, return for Back, and exit all menus for zero or EOF."""
    while True:
        print(f'\n{title}', flush=True)
        options = [*labels, back] if back != 'Exit' else labels
        for number, label in enumerate(options, 1):
            print(f'{number}  {label}', flush=True)
        print('0  Exit', flush=True)
        try:
            value = int(input('Choose a number: '))
        except EOFError:
            # EOF exits the whole program rather than cycling through parent menus.
            raise
        except ValueError:
            print('Invalid choice.', flush=True)
            continue
        if value == 0:
            raise InteractiveExit()
        if 1 <= value <= len(labels):
            return value - 1
        if back != 'Exit' and value == len(labels) + 1:
            return None
        print('Invalid choice.', flush=True)


def spin_for(node, executor, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=max(0.0, min(0.05, deadline - time.monotonic())))


def choose_container(node, executor, topic, timeout):
    services = dict(node.get_service_names_and_types())
    if topic in services:
        if len(services[topic]) == 1 and services[topic][0] in SERVICE_PAYLOADS:
            log('SKIP', topic, 'snapshot service already exists')
        else:
            log('ERROR', topic, 'a conflicting service already exists')
        return
    publisher = topic_publisher(node, topic)
    if publisher is None:
        log('ERROR', topic, 'no publisher found')
        return
    deadline = time.monotonic() + timeout
    index = ContainerIndex(node)
    try:
        index.refresh(services, time.monotonic(), deadline)
        while index.pending and time.monotonic() < deadline:
            index.tick(time.monotonic())
            executor.spin_once(timeout_sec=max(0.0, min(0.05, deadline - time.monotonic())))
        owners = index.owners(publisher)
        containers = sorted(container_names(services), key=lambda name: (name not in owners, name))
    finally:
        index.close()
    if not containers:
        log('ERROR', topic, 'no component containers found')
        return
    labels = [name + ("  [publisher's container]" if name in owners else '') for name in containers]
    selection = choose(f'Load {topic} into:', labels)
    if selection is not None:
        Loader(node, executor, [topic], timeout, target=containers[selection]).run()


def run(node, executor, timeout, unload=False):
    try:
        spin_for(node, executor, min(1.0, timeout))
        types = list(SNAPSHOT_TYPES.items())
        while True:
            title = 'Choose a snapshot type to unload:' if unload else 'Choose a topic type:'
            selected = choose(title, [kind.label for _, kind in types], back='Exit')
            if selected is None:
                return 0
            message_type, kind = types[selected]
            while True:
                graph = (node.get_service_names_and_types() if unload
                         else node.get_topic_names_and_types())
                expected = kind.service if unload else message_type
                topics = sorted(name for name, advertised in graph if expected in advertised)
                if not topics:
                    print(f'No {kind.label} topics found.', flush=True)
                selected_topic = choose(f'{kind.label} topics:', topics)
                if selected_topic is None:
                    break
                topic = topics[selected_topic]
                try:
                    if unload:
                        from .unload import Unloader
                        Unloader(node, executor, [topic], timeout).run()
                    else:
                        choose_container(node, executor, topic, timeout)
                except TopicError:
                    pass  # The loader already printed the error; allow another selection.
                except ValueError as error:
                    log('ERROR', topic, str(error))
    except (EOFError, InteractiveExit):
        return 0
