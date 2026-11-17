# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
"""Terminal controller for the C++ recorder; sensor payloads stay in that process."""

from contextlib import contextmanager
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import termios
import tty

from .discovery import SnapshotNameCompleter, snapshot_node


def add_arguments(parser, positive_seconds):
    topics = parser.add_argument('topics', nargs='*', metavar='TOPIC_OR_YAML',
                                help='Snapshot topics, or a recorder YAML parameter file')
    topics.completer = SnapshotNameCompleter()
    parser.add_argument('--params-file', help='ROS parameter YAML file for the recorder')
    parser.add_argument('-o', '--output', metavar='BAG', help='Bag directory; existing bags are rejected')
    parser.add_argument('-s', '--storage', help='Rosbag storage plugin (default: mcap)')
    parser.add_argument('--timeout', type=positive_seconds,
                        help='Override the recorder snapshot-response timeout')
    parser.add_argument('--node-name', default='sensor_snapshot_recorder',
                        help='Recorder node name (default: sensor_snapshot_recorder)')
    parser.add_argument('--disable-keyboard-controls', action='store_true',
                        help='Capture only through the recorder trigger service')


def recorder_command(args):
    """Build an argument vector, never a shell command, preserving YAML options."""
    from ament_index_python.packages import get_package_prefix
    import yaml

    topics = []
    parameter_file = args.params_file
    for item in args.topics:
        if Path(item).suffix.lower() in ('.yaml', '.yml'):
            if parameter_file:
                raise ValueError('Specify only one recorder YAML file')
            parameter_file = item
        else:
            topics.append(item)
    if not topics and not parameter_file:
        raise ValueError('Specify snapshot topics or a recorder YAML file')
    if parameter_file and not Path(parameter_file).is_file():
        raise ValueError(f'Recorder YAML file does not exist: {parameter_file}')
    # Node names do not accept slashes. Topics themselves can be fully qualified.
    import re
    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', args.node_name):
        raise ValueError('--node-name must be a valid ROS node name')
    prefix = get_package_prefix('sensor_snapshot_recorder')
    executable = Path(prefix) / 'lib/sensor_snapshot_recorder/sensor_snapshot_recorder_node'
    command = [str(executable), '--ros-args', '-r', f'__node:={args.node_name}']
    if parameter_file:
        command.extend(['--params-file', str(Path(parameter_file).resolve())])
    overrides = {}
    if topics:
        overrides['input.topics'] = topics
    if args.output is not None:
        overrides['output.uri'] = args.output
    if args.storage is not None:
        overrides['output.storage_id'] = args.storage
    if args.timeout is not None:
        overrides['capture.timeout_seconds'] = args.timeout
    for key, value in overrides.items():
        encoded = yaml.safe_dump(value, default_flow_style=True).strip().removesuffix('...').strip()
        command.extend(['-p', f'{key}:={encoded}'])
    return command


@contextmanager
def keyboard(enabled):
    """Keep Ctrl+C as a signal and always restore the caller's terminal settings."""
    saved = None
    if enabled:
        saved = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())
        changed = termios.tcgetattr(sys.stdin.fileno())
        changed[3] &= ~termios.ECHO
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, changed)
    try:
        yield
    finally:
        if saved is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, saved)


def stop(process):
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def run(args):
    from std_srvs.srv import Trigger

    command = recorder_command(args)
    process = subprocess.Popen(command, start_new_session=True)
    service = f'/{args.node_name}/trigger'
    interactive = sys.stdin.isatty() and not args.disable_keyboard_controls
    try:
        with snapshot_node() as (node, executor), keyboard(interactive):
            client = node.create_client(Trigger, service)
            while not client.wait_for_service(timeout_sec=0.1):
                if process.poll() is not None:
                    raise ValueError(f'Recorder exited during startup (code {process.returncode})')
            print(f'Trigger service: {service}', flush=True)
            print('Space: capture keyframe | Ctrl+C: stop and finalize bag' if interactive else
                  'Keyboard controls disabled; use the trigger service. Ctrl+C stops recording.', flush=True)
            pending = None
            while process.poll() is None:
                executor.spin_once(timeout_sec=0.05)
                if pending is not None and pending.done():
                    response = pending.result()
                    print(response.message, file=sys.stdout if response.success else sys.stderr, flush=True)
                    pending = None
                if interactive and select.select([sys.stdin], [], [], 0)[0]:
                    key = os.read(sys.stdin.fileno(), 1)
                    if not key:
                        break
                    if key == b' ' and pending is None:
                        pending = client.call_async(Trigger.Request())
            if process.poll() not in (None, 0):
                raise ValueError(f'Recorder exited with code {process.returncode}')
    except KeyboardInterrupt:
        pass
    finally:
        stop(process)
    return 0
