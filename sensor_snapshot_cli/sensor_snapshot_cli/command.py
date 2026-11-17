# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
"""Explicit get, list, load, unload and record subcommands for sensor snapshots."""

import argparse
import math
import sys

from ros2cli.command import CommandExtension

from .discovery import (
    discover, request_snapshot, SERVICE_PAYLOADS, SnapshotNameCompleter, snapshot_node,
)
from .text import format_message, select_field


def positive_seconds(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError('Expected a finite positive number of seconds')
    return result


def nonnegative_integer(value):
    result = int(value)
    if result < 0:
        raise argparse.ArgumentTypeError('Expected a nonnegative integer')
    return result


class SensorSnapshotCommand(CommandExtension):
    """Load, unload, discover, inspect, preview or record sensor snapshots."""

    def add_arguments(self, parser, cli_name, *, argv=None):
        from . import load, record, unload
        self._parser = parser
        parser.epilog = 'Project: https://github.com/Maik13579/sensor_snapshot'
        commands = parser.add_subparsers(dest='operation', metavar='{get,list,load,unload,record}')
        get_parser = commands.add_parser('get', help='Request one snapshot from a topic')
        self.add_get_arguments(get_parser)
        list_parser = commands.add_parser('list', help='List available snapshot services')
        list_parser.add_argument('--discovery-time', type=positive_seconds, default=1.0,
                                 help='Maximum discovery time (default: 1 second)')
        list_parser.add_argument('--show-container', action='store_true',
                                 help='Group snapshots by container and show component IDs')
        list_parser.add_argument('--timeout', type=positive_seconds, default=5.0,
                                 help='Maximum container lookup time (default: 5 seconds)')
        load.add_arguments(commands.add_parser('load', help='Load snapshot components'),
                           positive_seconds)
        unload.add_arguments(commands.add_parser('unload', help='Unload existing snapshots'),
                             positive_seconds)
        record.add_arguments(commands.add_parser('record', help='Record triggered snapshots'),
                             positive_seconds)

    def add_get_arguments(self, parser):
        argument = parser.add_argument(
            'service', metavar='TOPIC', help='Snapshot service/topic name')
        argument.completer = SnapshotNameCompleter()
        parser.add_argument('--field', help='Print a payload field, using dots for nested fields')
        parser.add_argument('--response', action='store_true',
                            help='Print the full service response, including status/sequence/age')
        parser.add_argument('--show-image', action='store_true', help='Open a frozen image preview')
        parser.add_argument('--request', default='{}', metavar='YAML',
                            help='Service request fields, e.g. "{voxel_size: 0.05}"')
        parser.add_argument('--timeout', type=positive_seconds, default=5.0,
                            help='Timeout for service availability/response (default: 5 seconds)')
        parser.add_argument('--discovery-time', type=positive_seconds, default=1.0,
                            help='Maximum discovery time (default: 1 second)')
        parser.add_argument('--full-length', '-f', action='store_true',
                            help='Print complete arrays and strings')
        parser.add_argument('--truncate-length', '-l', type=nonnegative_integer, default=128,
                            help='Printed array/string limit; 0 is unlimited (default: 128)')
        parser.add_argument('--no-arr', action='store_true',
                            help='Abbreviate array fields in printed messages')
        parser.add_argument('--no-str', action='store_true',
                            help='Abbreviate string fields in printed messages')
        parser.add_argument('--flow-style', action='store_true',
                            help='Print YAML collections inline')

    def main(self, *, parser, args):
        try:
            if args.operation is None:
                self._parser.print_help()
                return 0
            if args.operation == 'load':
                from .load import run
                return run(args)
            if args.operation == 'record':
                from .record import run
                return run(args)
            if args.operation == 'unload':
                from .unload import run
                return run(args)
            if args.operation == 'list':
                from .listing import run
                return run(args)
            return self.get_snapshot(args)
        except KeyboardInterrupt:
            return 130
        except Exception as error:
            print(f'sensor_snapshot: {error}', file=sys.stderr)
            return 1

    def get_snapshot(self, args):
        import yaml

        values = yaml.safe_load(args.request)
        if not isinstance(values, dict):
            raise ValueError('--request must be a YAML mapping')
        with snapshot_node() as (node, executor):
            from rclpy.expand_topic_name import expand_topic_name

            name = expand_topic_name(args.service, node.get_name(), node.get_namespace())
            service_type = discover(node, executor, args.discovery_time, name)
            kind = SERVICE_PAYLOADS[service_type]
            if args.show_image and kind != 'image':
                raise ValueError('--show-image is only available for image snapshots')
            response = request_snapshot(node, executor, name, service_type, values, args.timeout)
        # No ROS context remains alive while a preview is open.
        message = getattr(response, kind)
        selected = select_field(response if args.response else message, args.field)
        should_print = not args.show_image or args.field or args.response
        if should_print:
            text = format_message(
                selected, truncate_length=None if args.full_length or args.truncate_length == 0
                else args.truncate_length, no_arr=args.no_arr, no_str=args.no_str,
                flow_style=args.flow_style)
            print(text, end='')
        if args.show_image:
            from .images import show_image
            show_image(message)
        return 0
