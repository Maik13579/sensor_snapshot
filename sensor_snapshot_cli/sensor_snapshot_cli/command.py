# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
"""ros2 sensor_snapshot: discover, inspect and preview a single cached sample."""

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
    """Discover, inspect or preview one image, point-cloud or IMU snapshot."""

    def add_arguments(self, parser, cli_name, *, argv=None):
        argument = parser.add_argument(
            'service', nargs='?', help='Snapshot service/topic name; omit to list available snapshots')
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
                            help='Maximum printed array/string length; 0 is unlimited (default: 128)')
        parser.add_argument('--no-arr', action='store_true', help='Abbreviate array fields in printed messages')
        parser.add_argument('--no-str', action='store_true', help='Abbreviate string fields in printed messages')
        parser.add_argument('--flow-style', action='store_true', help='Print YAML collections inline')

    def main(self, *, parser, args):
        try:
            return self.run(args)
        except KeyboardInterrupt:
            return 130
        except Exception as error:
            print(f'sensor_snapshot: {error}', file=sys.stderr)
            return 1

    def run(self, args):
        import yaml

        values = yaml.safe_load(args.request)
        if not isinstance(values, dict):
            raise ValueError('--request must be a YAML mapping')
        if not args.service and (args.field or args.show_image or args.response or values):
            raise ValueError('A snapshot service name is required for these options')
        with snapshot_node() as (node, executor):
            if not args.service:
                for name, types in discover(node, executor, args.discovery_time):
                    print(f'{name} [{", ".join(types)}]')
                return 0
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
