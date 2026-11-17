# sensor_snapshot_cli

The `sensor_snapshot_cli` package adds the `ros2 sensor_snapshot` command for
loading, unloading, discovering, inspecting, previewing and recording snapshots. The
[snapshot components](../sensor_snapshot/README.md) cache sensor data and
provide the services used by these commands.

## Commands

| Command | Purpose |
| --- | --- |
| `ros2 sensor_snapshot get TOPIC` | Request and display one snapshot |
| `ros2 sensor_snapshot list` | List available snapshot services |
| `ros2 sensor_snapshot list --show-container` | Group snapshots by owning component container |
| `ros2 sensor_snapshot load TOPIC ...` | Load snapshot components onto sensor topics |
| `ros2 sensor_snapshot load` | Choose topics and containers interactively |
| `ros2 sensor_snapshot unload TOPIC ...` | Unload existing snapshot components |
| `ros2 sensor_snapshot unload -a` | Unload all discovered snapshots |
| `ros2 sensor_snapshot unload` | Choose existing snapshots to unload interactively |
| `ros2 sensor_snapshot record TOPIC ...` | Record triggered snapshot batches |

`ros2 sensor_snapshot` without a subcommand prints help. Use a subcommand's
`--help` option for its arguments.

## Load snapshots onto running sensors

```bash
# Discover types and load alongside the publishing components.
ros2 sensor_snapshot load /camera/image_raw /scan /imu/data

# Use an externally started container for standalone publishers or rejected loads.
ros2 sensor_snapshot load /camera/image_raw /scan \
  --fallback-container /snapshot_container --timeout 15

# Log and skip individual topic errors while continuing with the other topics.
ros2 sensor_snapshot load /scan /missing --ignore-errors

# Choose a topic type, topic and existing target container using numbered menus.
ros2 sensor_snapshot load
```

The loader polls the live ROS graph once per second and processes each topic as
soon as its publisher appears. One global `--timeout` (default: 15 seconds) covers
publisher discovery, component loading and snapshot-service confirmation for the
whole batch. It does not wait for sensor messages. Exactly one publisher and a
supported type (`Image`, `PointCloud2`, `Imu` or `LaserScan`) are required.

Existing snapshot services are logged and skipped. Publisher containers are
identified using their component-list services. Loading is attempted there first;
if no owner can be found or a load is rejected, `--fallback-container` is used when
provided. The loader never starts or owns a container. Snapshot components enable
intra-process communication; the publishing sensor's configuration is unchanged.
The command exits after the requested snapshot services appear. Components remain
loaded in the externally managed containers after the command exits, including
components loaded before a later topic fails.

Errors normally stop the command with a nonzero status. `--ignore-errors` logs and
skips failed topics, continues loading the others, and returns success. Missing
publishers are skipped only when the global deadline expires. A load request whose
response remains pending can run until that deadline; it is not submitted again,
because a timeout does not prove that the container rejected it.

Omitting topic arguments starts interactive mode; `-i` / `--interactive` remains
available as an explicit alias. Interactive mode lists the four supported topic types, their discovered topics,
and all discovered component containers. The publisher's container is listed
first and marked. The chosen container is the explicit target. Invalid input
redraws the menu. `0` exits from every menu, and Back is a separate choice in
submenus. EOF or Ctrl+C also ends the program. Successes, existing-service skips
and errors return to the topic list so more topics can be selected. Each selected
load has its own global timeout;
time spent choosing menu entries is unrestricted. Positional topics and
`--fallback-container` are not accepted with `-i`.

For a sensor launch file, add an action such as:

```python
from launch.actions import ExecuteProcess

load_snapshots = ExecuteProcess(
    cmd=['ros2', 'sensor_snapshot', 'load', '/camera/image_raw', '/scan',
         '--fallback-container', '/snapshot_container'],
    output='screen',
)
```

Add the action to the launch description alongside the sensor and any fallback
container. Discovery handles sensors that start later within the timeout.

## Unload existing snapshots

```bash
ros2 sensor_snapshot unload /camera/image_raw /scan
ros2 sensor_snapshot unload -a
ros2 sensor_snapshot unload /scan /missing --timeout 15 --ignore-errors
ros2 sensor_snapshot unload
```

The topic arguments complete to existing snapshot services. The unloader finds
the component that actually provides each service and unloads it by its container
ID. Custom node names are supported. The sensor driver and its container remain
running; no publisher is needed to remove an existing snapshot.

`-a` / `--all` discovers all existing snapshot services and unloads their
components. It cannot be combined with topic arguments or `-i`. An empty
selection is logged and succeeds. Discovery consumes part of the same global
timeout as unloading.

One global `--timeout` (default: 15 seconds) covers service discovery, component
ownership lookup, unloading and confirmation that the services have disappeared.
Ambiguous owners, rejected unloads and snapshots without a discoverable component
container are errors. `--ignore-errors` logs and skips individual failures while
continuing with the other topics and returns success.

Omitting topic arguments starts interactive unloading; `-i` is also supported.
Interactive unloading lists the four snapshot types and their existing snapshot
services. Selecting one unloads its component and returns to the list. Errors are
displayed and return to the menu. `0` exits from every menu, and Back returns to
the previous menu.

## Shell completion

Normal ROS 2 shell completion offers subcommands and flags. `get`, `record` and
`unload` complete existing snapshot service names; `load` completes supported
sensor topics, and `--fallback-container` completes discovered component manager
names. Dynamic completion uses bounded graph discovery (about 0.5 seconds) and
does not request snapshots or query component membership.

## Request snapshots

```bash
# List available snapshot services and their types.
ros2 sensor_snapshot list

# Group snapshots by their owning component container.
ros2 sensor_snapshot list --show-container

# Print one payload, or a dotted field inside it.
ros2 sensor_snapshot get /imu/data --field linear_acceleration
ros2 sensor_snapshot get /camera/image_raw --field header
ros2 sensor_snapshot get /scan --field ranges

# Preview one frozen image.
ros2 sensor_snapshot get /camera/image_raw --show-image

# Request a filtered cloud and print its header.
ros2 sensor_snapshot get /camera/points --field header --request '{voxel_size: 0.05}'

# Inspect response metadata rather than the payload.
ros2 sensor_snapshot get /imu/data --response --field age_seconds
```

The service/topic argument has tab completion for all four snapshot service types;
unrelated services are excluded. It uses the normal ROS 2 shell completion setup
provided when sourcing the installation. Names are service names (the snapshot
components deliberately use the same names as their input topics).

Each `get` invocation makes one request. `--show-image` suppresses the default message
dump; explicit `--field`/`--response` still prints. Failed requests return a nonzero
exit status. Discovery and RPC waits are bounded by `--discovery-time` and
`--timeout`. `--request` accepts a YAML mapping of any supported request fields.
Echo-style output supports `--no-arr`, `--no-str`, `--full-length`,
`--truncate-length` and `--flow-style`.

Preview requires a graphical display and closes with Esc, Q or the window close
button. Non-uint8 images are scaled for display only. There is no cloud, IMU or scan
visualization.

`list --show-container` prints each container followed by its snapshot services,
including the container-specific component IDs, for example:

```text
/camera_container
  2  /camera/image_raw [sensor_snapshot_interfaces/srv/GetImage]
  3  /camera/points [sensor_snapshot_interfaces/srv/GetPointCloud]
/lidar_container
  1  /scan [sensor_snapshot_interfaces/srv/GetLaserScan]
```

Only containers with discovered snapshots are included. Snapshots without an
identified container appear under `Container not identified:`. Graph discovery
uses `--discovery-time` (default: 1 second); `--timeout` bounds container lookup
(default: 5 seconds).

## Triggered bag recording

Record latest snapshots from the listed topics into one bag:

```bash
ros2 sensor_snapshot record /velodyne/points /imu/data -o keyframes
```

Press **Space** to capture one keyframe. **Ctrl+C** stops recording and finalizes
the bag. Topic arguments have tab completion for available snapshot services.
`-o` chooses the bag directory and `-s` chooses the storage format (default: MCAP).
Existing bag directories are rejected. With no `-o`, a new name is generated.

The command also accepts a ROS parameter YAML file:

```bash
ros2 sensor_snapshot record recorder.yaml
ros2 sensor_snapshot record --params-file recorder.yaml
```

For example:

```yaml
/**:
  ros__parameters:
    input.topics: [/velodyne/points, /imu/data]
    output.uri: keyframes
    requests:
      /velodyne/points:
        voxel_size: 0.05
        fields: [x, y, z]
        max_age_seconds: 0.5
      /imu/data:
        max_age_seconds: 0.5
```

Command-line topics and bag options override their YAML equivalents. Request
options live under `requests.<fully-qualified-topic>`. They are prepared once
when the service's typed client is created and reused for each trigger. Images
support `region`, `downsample_factor` and `max_age_seconds`. Clouds also support
`fields`, `voxel_size` and named `filters` blocks; the example configuration shows
how to express field comparisons. IMU and laser scan requests support `max_age_seconds`.

The [recorder package](../sensor_snapshot_recorder/README.md) documents capture
semantics, ROS parameters, standalone launch and the trigger/stop services.
The [interfaces package](../sensor_snapshot_interfaces/README.md) documents the
snapshot request fields accepted by `--request` and per-topic recorder settings.
