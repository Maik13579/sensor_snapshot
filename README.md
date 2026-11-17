# sensor_snapshot

ROS 2 components and tools for taking snapshots of images, point clouds, IMU data
and laser scans. Components retain sensor messages by shared pointer and expose
on-demand services using the same names as their input topics. Clients can
request historical frames, age limits and image/cloud processing; a recorder
writes triggered batches into rosbag files.

| Package | Documentation |
| --- | --- |
| **sensor_snapshot** | [C++ components, launch files and cache parameters](sensor_snapshot/README.md) |
| **sensor_snapshot_cli** | [Automatic and interactive loading/unloading, snapshot commands and keyboard recording](sensor_snapshot_cli/README.md) |
| **sensor_snapshot_interfaces** | [Messages, service API, request options and Python clients](sensor_snapshot_interfaces/README.md) |
| **sensor_snapshot_recorder** | [Triggered bag recorder, configuration and control services](sensor_snapshot_recorder/README.md) |
