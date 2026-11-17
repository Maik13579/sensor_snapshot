# sensor_snapshot_interfaces

ROS 2 messages and services shared by the
[snapshot components](../sensor_snapshot/README.md),
[CLI](../sensor_snapshot_cli/README.md),
[recorder](../sensor_snapshot_recorder/README.md) and application clients.
Depend on this package to write a client without depending on the component
implementation.

## Services

Snapshot services use the same fully qualified names as their input topics.

| Service definition | Payload field | Sensor message |
| --- | --- | --- |
| [GetImage](srv/GetImage.srv) | `image` | `sensor_msgs/msg/Image` |
| [GetPointCloud](srv/GetPointCloud.srv) | `cloud` | `sensor_msgs/msg/PointCloud2` |
| [GetImu](srv/GetImu.srv) | `imu` | `sensor_msgs/msg/Imu` |
| [GetLaserScan](srv/GetLaserScan.srv) | `scan` | `sensor_msgs/msg/LaserScan` |

Every response includes `status`, `sequence` and `age_seconds`. Image and cloud
responses also include the source dimensions and resolved region. Check for
`SnapshotStatus.OK` before consuming a payload.

## Messages

- [Region](msg/Region.msg): zero-based pixel/point-index rectangle with an exclusive
  far edge. Zero width or height extends to the corresponding source edge.
- [FieldFilter](msg/FieldFilter.msg): cloud field name, comparison, value and optional
  element index. Every filter must pass; non-finite tested values fail.
- [SnapshotStatus](msg/SnapshotStatus.msg): result code and explanatory message.
  Codes are `OK`, `NO_DATA`, `STALE`, `INVALID_REQUEST`, `INVALID_DATA`,
  `UNSUPPORTED`, `INTERNAL_ERROR` and `NOT_FOUND`.

## Python snippets

Create the clients once in your node's `__init__`. Keep them in a separate callback
group from the timer/subscription callback that makes the request.

```python
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_snapshot_interfaces.srv import GetImage, GetPointCloud, GetImu, GetLaserScan

self.snapshot_group = ReentrantCallbackGroup()
self.image_client = self.create_client(
    GetImage, '/camera/image_raw', callback_group=self.snapshot_group)
self.cloud_client = self.create_client(
    GetPointCloud, '/camera/points', callback_group=self.snapshot_group)
self.imu_client = self.create_client(
    GetImu, '/imu/data_raw', callback_group=self.snapshot_group)
self.scan_client = self.create_client(
    GetLaserScan, '/scan', callback_group=self.snapshot_group)
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

### Laser scan request

```python
client = self.scan_client
request = GetLaserScan.Request()
request.max_age_seconds = 0.5
```

The response's `scan` is an unchanged `sensor_msgs/LaserScan`. Angular and timing
metadata, range limits, ranges and intensities are preserved, including NaN/Inf
returns and empty intensity arrays. The component does not project or downsample
the scan. The recorder stores it as `sensor_msgs/msg/LaserScan` on its input topic.

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

Use `response.cloud`, `response.image`, `response.imu` or `response.scan` for the matching request.
Check `status` before using the payload; failed IMU/scan requests leave it at its default
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

IMU and laser scan requests support only `stamp` and `max_age_seconds`; region, sampling, field
selection, voxel and comparison options apply to image/cloud requests only.

An empty cache returns `NO_DATA`; an expired sample returns `STALE`. `age_seconds`
is measured since subscription receipt at selection time, separately from the sensor
header stamp. `sequence` identifies repeated snapshots. History retains whole frames;
service responses still incur copying and transport into Python. IMU samples and laser scans use
the same bounded history, timestamp-selection, sequence and freshness semantics.
