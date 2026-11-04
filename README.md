# sensor_snapshot

C++ components that retain image/cloud messages by shared pointer and return
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

Both components in an existing sensor container:

```bash
ros2 launch sensor_snapshot snapshot.launch.py \
  container:=/sensor_container \
  image_topic:=/camera/image_raw cloud_topic:=/camera/points
```

Each empty topic disables its component; both default to empty. Without `container`,
a multithreaded container is created when needed. For intra-process delivery, load
snapshots beside the sensor and enable intra-process communication on the sensor
as well. Publisher ownership and other subscribers can still introduce copies.

| Launch argument | Default | Meaning |
| --- | --- | --- |
| `image_topic` | Empty | Image input and `GetImage` service name |
| `cloud_topic` | Empty | Cloud input and `GetPointCloud` service name |
| `container` | Empty | Existing multithreaded container; empty creates one |
| `namespace` | Empty | Component namespace |
| `history_size` | `1` | Received frames retained per component, 1–10000 |
| `max_age_seconds` | `0.0` | Default reception-age limit; zero disables expiry |
| `reliability` | `best_effort` | Input reliability: `best_effort` or `reliable` |
| `service_qos_depth` | `1` | Service history depth, 1–100 |

## Python snippets

Create the clients once in your node's `__init__`. Keep them in a separate callback
group from the timer/subscription callback that makes the request.

```python
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_snapshot_interfaces.srv import GetImage, GetPointCloud

self.snapshot_group = ReentrantCallbackGroup()
self.image_client = self.create_client(
    GetImage, '/camera/image_raw', callback_group=self.snapshot_group)
self.cloud_client = self.create_client(
    GetPointCloud, '/camera/points', callback_group=self.snapshot_group)
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

### Send either request

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

Use `response.cloud` for a cloud request or `response.image` for an image request.

### Select a historical frame

Set the timestamp before sending either request:

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

An empty cache returns `NO_DATA`; an expired sample returns `STALE`. `age_seconds`
is measured since subscription receipt at selection time, separately from the sensor
header stamp. `sequence` identifies repeated snapshots. History retains whole frames;
service responses still incur copying and transport into Python.
