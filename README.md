# sensor_snapshot

C++ components that retain image/cloud/IMU messages by shared pointer and return
snapshots on demand. Python clients receive only the data they request.
The snapshot service uses the same name as its input topic.
Node names use `snapshot_<topic>` with slashes replaced by underscores:
`/camera/points` becomes `snapshot_camera_points`.
Standalone containers use `snapshot_<first-topic>_container`.

## Launch

Point cloud only:

```bash
ros2 launch sensor_snapshot snapshot.launch.py cloud_topic:=/camera/points history_size:=30
```

Image and cloud components in an existing sensor container:

```bash
ros2 launch sensor_snapshot snapshot.launch.py \
  container:=/sensor_container \
  image_topic:=/camera/image_raw cloud_topic:=/camera/points
```

IMU only, loaded alongside its driver:

```bash
ros2 launch sensor_snapshot snapshot.launch.py \
  container:=/imu_container imu_topic:=/imu/data_raw
```

Image, cloud and IMU topics can also be enabled together. Each empty topic disables
its component; all default to empty. Without `container`,
a multithreaded container is created when needed. For intra-process delivery, load
snapshots beside the sensor and enable intra-process communication on the sensor
as well. Publisher ownership and other subscribers can still introduce copies.

| Launch argument | Default | Meaning |
| --- | --- | --- |
| `image_topic` | Empty | Image input and `GetImage` service name |
| `cloud_topic` | Empty | Cloud input and `GetPointCloud` service name |
| `imu_topic` | Empty | IMU input and `GetImu` service name |
| `container` | Empty | Existing multithreaded container; empty creates one |
| `namespace` | Empty | Component namespace |
| `history_size` | `1` | Received frames retained per component, 1–10000 |
| `max_age_seconds` | `0.0` | Default reception-age limit; zero disables expiry |
| `reliability` | `best_effort` | Input reliability: `best_effort` or `reliable` |
| `service_qos_depth` | `1` | Service history depth, 1–100 |

## Command-line snapshots

The `sensor_snapshot_cli` package adds the `ros2 sensor_snapshot` command.

```bash
# List available snapshot services and their types.
ros2 sensor_snapshot

# Print one payload, or a dotted field inside it.
ros2 sensor_snapshot /imu/data --field linear_acceleration
ros2 sensor_snapshot /camera/image_raw --field header

# Save and/or preview the same frozen image.
ros2 sensor_snapshot /camera/image_raw --output frame.png
ros2 sensor_snapshot /camera/image_raw --show-image --output frame.png

# Save the returned cloud, including all its fields, as binary PCD.
ros2 sensor_snapshot /camera/points --output cloud.pcd
ros2 sensor_snapshot /camera/points --output cloud.pcd --request '{voxel_size: 0.05}'

# Save a complete IMU measurement as YAML.
ros2 sensor_snapshot /imu/data --output imu.yaml

# Inspect response metadata rather than the payload.
ros2 sensor_snapshot /imu/data --response --field age_seconds
```

The service/topic argument has tab completion for the three snapshot service types;
unrelated services are excluded. It uses the normal ROS 2 shell completion setup
provided when sourcing the installation. Names are service names (the snapshot
components deliberately use the same names as their input topics).

Each invocation makes one request. `--field` affects only printed output, never the
saved payload. `--output` or `--show-image` suppresses the default message dump;
explicit `--field`/`--response` still prints. Existing files require `--force`.
Failed requests return a nonzero exit status and do not write a file. Discovery and
RPC waits are bounded by `--discovery-time` and `--timeout`. `--request` accepts a
YAML mapping of any supported request fields. Echo-style output supports
`--no-arr`, `--no-str`, `--full-length`, `--truncate-length` and `--flow-style`.

Image export supports PNG for uint8/uint16 pixels and TIFF for uint8/uint16 or
single-channel float32/float64 depth. Color order is converted for the codec;
floating-point depth is never silently converted to PNG. Preview requires a
graphical display and closes with Esc, Q or the window close button.
Non-uint8 previews are scaled for display only; saved measurements retain their
values. There is no cloud or IMU visualization.

Clouds are exported as binary PCD. Organized dimensions and every standard PointField
(including COUNT, RGB, intensity and labels) are retained; endian differences and
layout padding are normalized. Empty or invalid clouds are rejected. PCD/image
files are payload exports and do not retain the complete ROS header; IMU YAML does.

## Python snippets

Create the clients once in your node's `__init__`. Keep them in a separate callback
group from the timer/subscription callback that makes the request.

```python
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_snapshot_interfaces.srv import GetImage, GetPointCloud, GetImu

self.snapshot_group = ReentrantCallbackGroup()
self.image_client = self.create_client(
    GetImage, '/camera/image_raw', callback_group=self.snapshot_group)
self.cloud_client = self.create_client(
    GetPointCloud, '/camera/points', callback_group=self.snapshot_group)
self.imu_client = self.create_client(
    GetImu, '/imu/data_raw', callback_group=self.snapshot_group)
```

Run your node with a multithreaded executor:

```python
from rclpy.executors import MultiThreadedExecutor

executor = MultiThreadedExecutor(num_threads=2)
executor.add_node(node)
executor.spin()
```

### Cloud request

Inside your existing callback:

```python
from sensor_snapshot_interfaces.msg import FieldFilter

client = self.cloud_client
request = GetPointCloud.Request()
request.fields = ['x', 'y', 'z']  # Empty returns all fields.
request.voxel_size = 0.05
request.filters = [
    FieldFilter(field='z', comparison=FieldFilter.GREATER, value=0.2),
    FieldFilter(field='z', comparison=FieldFilter.LESS_OR_EQUAL, value=3.0),
    FieldFilter(field='intensity', comparison=FieldFilter.GREATER_OR_EQUAL, value=20.0),
]
```

### Image request

```python
from sensor_snapshot_interfaces.msg import Region

client = self.image_client
request = GetImage.Request()
request.region = Region(x_offset=100, y_offset=100, width=320, height=240)
request.downsample_factor = 2
request.max_age_seconds = 0.5
```

### IMU request

```python
client = self.imu_client
request = GetImu.Request()  # Latest received sample; no filtering or averaging.
request.max_age_seconds = 0.5
```

The response's `imu` is an unchanged `sensor_msgs/Imu`: acceleration, angular
velocity, orientation and all covariance fields retain their original values.
An unavailable orientation (`orientation_covariance[0] == -1`) stays unavailable;
the component does not estimate orientation, normalize acceleration or detect
stationarity. The original acquisition timestamp and frame ID are preserved.

### Send any request

`rclpy` uses `client.call(...)` for a synchronous request. The other executor thread
handles the response while your application callback waits.

```python
from sensor_snapshot_interfaces.msg import SnapshotStatus

if not client.service_is_ready():
    return
response = client.call(request, timeout_sec=2.0)
if response is None:
    self.get_logger().warning('Snapshot request timed out')
    return
if response.status.code != SnapshotStatus.OK:
    self.get_logger().warning(response.status.message)
    return
```

Use `response.cloud`, `response.image` or `response.imu` for the matching request.
Check `status` before using the payload; failed IMU requests leave it at its default
value rather than providing a valid measurement.

### Select a historical frame

Set the timestamp before sending any request:

```python
request.stamp = detection.header.stamp
```

Enable enough history with `history_size` at launch. A zero stamp selects the
latest received message. Other stamps select the closest sensor timestamp within
retained history; ties prefer the newest receipt. Outside that range, the service
returns `NOT_FOUND`. The returned header contains the actual selected timestamp.

## Request options

| Field | Default | Meaning |
| --- | --- | --- |
| `region` | All zeros | Pixel/point-index rectangle; zero width/height extends to the source edge |
| `downsample_factor` | `1` | Sample every Nth column/row, anchored at the region origin |
| `fields` (cloud) | Empty | Return all original fields; otherwise pack selected fields in list order |
| `filters` (cloud) | Empty | AND conditions: `LESS`, `LESS_OR_EQUAL`, `EQUAL`, `NOT_EQUAL`, `GREATER_OR_EQUAL`, `GREATER` |
| `voxel_size` (cloud) | `0.0` | Disabled; positive values keep the first passing original point per XYZ voxel |
| `stamp` | Zero | Latest receipt, or select by sensor acquisition timestamp |
| `max_age_seconds` | `0.0` | Use the server default; positive values set a request-specific reception-age limit |

Cloud processing order: region → stride sampling → comparisons → voxels → field
selection. Point/voxel filters produce a compact cloud (`height=1`); without them,
the sampled rectangle stays organized. Zero matches is a successful empty cloud.
Comparisons are exact; non-finite tested values fail. `element_index` selects an
array element in a field whose `count` exceeds one.

Image sampling copies original pixels without interpolation. YUV cropping/sampling
and Bayer sampling are unsupported; Bayer crops require even x/y offsets.
CameraInfo is not returned, so account for crop origin and sampling in your geometry.

IMU requests support only `stamp` and `max_age_seconds`; region, sampling, field
selection, voxel and comparison options apply to image/cloud requests only.

An empty cache returns `NO_DATA`; an expired sample returns `STALE`. `age_seconds`
is measured since subscription receipt at selection time, separately from the sensor
header stamp. `sequence` identifies repeated snapshots. History retains whole frames;
service responses still incur copying and transport into Python. IMU samples use
the same bounded history, timestamp-selection, sequence and freshness semantics.
