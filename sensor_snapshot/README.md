# sensor_snapshot

C++ components that retain image/cloud/IMU/laser scan messages by shared pointer and return
snapshots on demand. Python clients receive only the data they request.
The snapshot service uses the same name as its input topic.
Node names use `snapshot_<topic>` with slashes replaced by underscores:
`/camera/points` becomes `snapshot_camera_points`.
Standalone containers use `snapshot_<first-topic>_container`.

See the [CLI](../sensor_snapshot_cli/README.md) for automatic loading into running
sensor containers and the [interfaces](../sensor_snapshot_interfaces/README.md)
for request fields, processing behavior and Python client examples.

## Components

| Sensor message | Component plugin |
| --- | --- |
| `sensor_msgs/msg/Image` | `sensor_snapshot::ImageSnapshot` |
| `sensor_msgs/msg/PointCloud2` | `sensor_snapshot::PointCloudSnapshot` |
| `sensor_msgs/msg/Imu` | `sensor_snapshot::ImuSnapshot` |
| `sensor_msgs/msg/LaserScan` | `sensor_snapshot::LaserScanSnapshot` |

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

Laser scan only:

```bash
ros2 launch sensor_snapshot snapshot.launch.py scan_topic:=/scan
```

Image, cloud, IMU and laser scan topics can also be enabled together. Each empty topic disables
its component; all default to empty. Without `container`,
a multithreaded container is created when needed. For intra-process delivery, load
snapshots beside the sensor and enable intra-process communication on the sensor
as well. Publisher ownership and other subscribers can still introduce copies.

| Launch argument | Default | Meaning |
| --- | --- | --- |
| `image_topic` | Empty | Image input and `GetImage` service name |
| `cloud_topic` | Empty | Cloud input and `GetPointCloud` service name |
| `imu_topic` | Empty | IMU input and `GetImu` service name |
| `scan_topic` | Empty | LaserScan input and `GetLaserScan` service name |
| `container` | Empty | Existing multithreaded container; empty creates one |
| `namespace` | Empty | Component namespace |
| `history_size` | `1` | Received frames retained per component, 1–10000 |
| `max_age_seconds` | `0.0` | Default reception-age limit; zero disables expiry |
| `reliability` | `best_effort` | Input reliability: `best_effort` or `reliable` |
| `service_qos_depth` | `1` | Service history depth, 1–100 |

## Component parameters

These are the ROS parameter names used when loading components directly:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `input.topic` | `image`, `points`, `imu` or `scan` | Input topic and snapshot service name |
| `input.reliability` | `best_effort` | Input reliability: `best_effort` or `reliable` |
| `cache.history_size` | `1` | Whole messages retained by shared pointer, 1–10000 |
| `cache.max_age_seconds` | `0.0` | Default maximum reception age; zero disables expiry |
| `service.qos_depth` | `1` | Service request/response history depth, 1–100 |

Parameters are configured at startup. Subscription QoS keeps the latest message;
`cache.history_size` controls the separate retained snapshot history. History
retains whole frames, and service responses still incur copying and transport
into Python. See the [request API](../sensor_snapshot_interfaces/README.md#request-options)
for historical frame selection, age limits and image/cloud processing.
