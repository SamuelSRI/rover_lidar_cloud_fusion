# Rover LiDAR Cloud Fusion

ROS2 package that fuses a 2D `LaserScan` and a `PointCloud2` into a single `PointCloud2`.

This package is designed for a rover using two complementary LiDAR sensors:

* a 360° 2D LiDAR publishing `/scan`;
* a front 3D / ToF LiDAR publishing a `PointCloud2`.

The output `/merged_cloud` can then be used as a single input for an elevation mapping or traversability mapping node.

## Purpose

The goal of this package is to keep the mapping pipeline simple by providing only one point cloud input to the elevation mapping node.

Instead of modifying the elevation mapping node to handle multiple sensor types, this package converts the 2D LiDAR scan into a point cloud, transforms both inputs into a common frame, merges them, and publishes a single cloud.

## Architecture

```text
/scan                  sensor_msgs/msg/LaserScan
/front_cloud           sensor_msgs/msg/PointCloud2
        │
        ▼
rover_lidar_cloud_fusion
        │
        ▼
/merged_cloud          sensor_msgs/msg/PointCloud2
        │
        ▼
elevation_mapping / traversability_mapping
```

## Subscribed Topics

| Topic          | Type                          | Description                                   |
| -------------- | ----------------------------- | --------------------------------------------- |
| `/scan`        | `sensor_msgs/msg/LaserScan`   | 2D LiDAR scan, typically from a 360° LiDAR    |
| `/front_cloud` | `sensor_msgs/msg/PointCloud2` | Point cloud from the front LiDAR / ToF sensor |

## Published Topics

| Topic           | Type                          | Description                                    |
| --------------- | ----------------------------- | ---------------------------------------------- |
| `/merged_cloud` | `sensor_msgs/msg/PointCloud2` | Fused point cloud in the selected target frame |

## Parameters

| Parameter      |         Default | Description                                          |
| -------------- | --------------: | ---------------------------------------------------- |
| `scan_topic`   |         `/scan` | Input `LaserScan` topic                              |
| `cloud_topic`  |  `/front_cloud` | Input `PointCloud2` topic                            |
| `output_topic` | `/merged_cloud` | Output fused cloud topic                             |
| `target_frame` |     `base_link` | Common frame used for the merged cloud               |
| `publish_rate` |          `10.0` | Output publication rate in Hz                        |
| `max_data_age` |           `0.5` | Maximum accepted age of each sensor input in seconds |

## Configuration

The default configuration is located in:

```text
config/lidar_cloud_fusion.yaml
```

Example:

```yaml
lidar_cloud_fusion_node:
  ros__parameters:
    scan_topic: "/scan"
    cloud_topic: "/front_cloud"
    output_topic: "/merged_cloud"
    target_frame: "base_link"
    publish_rate: 10.0
    max_data_age: 0.5
```

If the front LiDAR publishes on another topic, update `cloud_topic`.

Example:

```yaml
cloud_topic: "/as_dt1/points"
```

## Build

From the ROS2 workspace:

```bash
cd ~/ros2_ws

source /opt/ros/lyrical/setup.bash

colcon build --packages-select rover_lidar_cloud_fusion

source install/setup.bash
```

## Run

```bash
ros2 launch rover_lidar_cloud_fusion lidar_cloud_fusion.launch.py
```

## Check the Output

```bash
ros2 topic list | grep merged
ros2 topic hz /merged_cloud
ros2 topic echo /merged_cloud --once
```

## TF Requirements

Both input sensor frames must be connected to the selected `target_frame`.

For example, if `target_frame` is `base_link`, the following transforms must exist:

```text
base_link -> lidar_360_link
base_link -> front_lidar_link
```

Check the input frame names:

```bash
ros2 topic echo /scan --once | grep frame_id
ros2 topic echo /front_cloud --once | grep frame_id
```

Check TF:

```bash
ros2 run tf2_ros tf2_echo base_link lidar_360_link
ros2 run tf2_ros tf2_echo base_link front_lidar_link
```

## RViz Visualization

Set the RViz fixed frame to the configured `target_frame`.

For example:

```text
Fixed Frame: base_link
```

Then add:

```text
Add -> PointCloud2 -> /merged_cloud
```

## Intended Use

This package is mainly intended to feed an existing elevation mapping or traversability mapping node with a single cloud input.

Example pipeline:

```text
/scan
/front_cloud
   │
   ▼
/merged_cloud
   │
   ▼
elevation_mapping
   │
   ▼
/traversability_grid
   │
   ▼
Nav2 costmap
```

## Notes

The fused cloud currently keeps only the `x`, `y`, and `z` fields.

This is intentional to keep the output simple and compatible with mapping nodes that only require spatial point data.

Additional fields such as intensity, ring, or timestamp can be added later if required.

## License

MIT License.
