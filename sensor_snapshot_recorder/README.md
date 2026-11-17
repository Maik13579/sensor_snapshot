# sensor_snapshot_recorder

C++ component for recording triggered batches of image, point-cloud, IMU and laser
scan snapshots into a rosbag. Sensor payloads remain in C++; the
[CLI](../sensor_snapshot_cli/README.md#triggered-bag-recording) provides keyboard
controls and configuration overrides. The
[snapshot components](../sensor_snapshot/README.md) must already be serving the
configured topics.

## Capture behavior

The recorder discovers services matching each topic and one of `GetImage`,
`GetPointCloud`, `GetImu` or `GetLaserScan`. Every trigger sends all requests asynchronously.
The returned sensor messages are stored on the original topic names, with
unchanged headers and acquisition timestamps. All messages from that trigger
share the recorder's ROS time at receipt of the trigger as their bag timestamp.

All configured topics are required. A missing service, unsuccessful response or
request timeout rejects that capture before writing its messages. A second
trigger while a capture is in progress is rejected. A storage write error stops
further captures; it can leave a partial capture in the bag.

The recorder can also run without the CLI:

```bash
ros2 launch sensor_snapshot_recorder recorder.launch.py params_file:=/path/to/recorder.yaml
ros2 service call /sensor_snapshot_recorder/trigger std_srvs/srv/Trigger '{}'
```

The component is `sensor_snapshot_recorder::SnapshotRecorder`; its private
`~/trigger` service uses `std_srvs/srv/Trigger`. This service remains available
when keyboard controls are disabled with `--disable-keyboard-controls` or when
standard input is not a terminal. Stop or unload the component to close its bag.

`~/stop` (`std_srvs/srv/Trigger`) finishes any active capture, closes the bag, and
rejects further triggers. Repeated stop requests after closure are harmless.
C++ owners can register `set_capture_callback()` before spinning to receive each
successfully recorded batch, including its bag timestamp and immutable serialized
payloads. The callback runs before the trigger response and should return quickly.

## Configuration

Start from [config/recorder.yaml](config/recorder.yaml). All parameters are fixed
at startup.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `input.topics` | Empty | Required list of unique snapshot service/topic names |
| `output.uri` | Empty | Bag directory; empty generates a name, existing paths are rejected |
| `output.storage_id` | `mcap` | rosbag2 storage plugin |
| `capture.timeout_seconds` | `5.0` | Finite positive deadline for each triggered capture |
| `discovery.period_seconds` | `1.0` | Finite positive interval for refreshing snapshot clients |
| `requests.<fully-qualified-topic>.*` | Request defaults | Typed snapshot request fields reused on each trigger |

For example:

```yaml
/**:
  ros__parameters:
    input.topics: [/velodyne/points, /imu/data]
    output.uri: keyframes
    output.storage_id: mcap
    capture.timeout_seconds: 5.0
    discovery.period_seconds: 1.0
    requests:
      /velodyne/points:
        voxel_size: 0.05
        fields: [x, y, z]
        max_age_seconds: 0.5
      /imu/data:
        max_age_seconds: 0.5
```

Image requests support `region`, `downsample_factor` and `max_age_seconds`.
Point-cloud requests also support `fields`, `voxel_size` and named `filters`
blocks; the example configuration includes a commented field-comparison filter.
IMU and laser scan requests support `max_age_seconds`. See the
[interfaces](../sensor_snapshot_interfaces/README.md#request-options) for request
semantics and the [CLI](../sensor_snapshot_cli/README.md#triggered-bag-recording)
for command-line overrides.
