# ROS2 Companion Package

MapEverything publishes several custom message types under `reconstructor_msgs`.
The package lives in `ros2/reconstructor_msgs` and can be copied or symlinked
into a normal ROS2 colcon workspace on the recorder machine.

## Build the Messages

```bash
mkdir -p ~/mapeverything_ws/src
cp -R ros2/reconstructor_msgs ~/mapeverything_ws/src/
cd ~/mapeverything_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select reconstructor_msgs
source install/setup.bash
ros2 interface show reconstructor_msgs/msg/RadioObservation
```

If you prefer to keep the package linked to this repository, replace the `cp`
step with a symlink from `~/mapeverything_ws/src/reconstructor_msgs` to
`<repo>/ros2/reconstructor_msgs`.

## rosbridge Setup

Install and launch rosbridge after sourcing the workspace that contains
`reconstructor_msgs`:

```bash
sudo apt-get update
sudo apt-get install ros-$ROS_DISTRO-rosbridge-suite
source ~/mapeverything_ws/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

The MapEverything iOS app should then connect to
`ws://<recorder-host>:9090`. Source the same workspace in every terminal that
launches rosbridge, rosbag2, RViz, or topic inspection tools so the custom
interfaces are available.

## Recording With rosbag2

Record the lightweight default streams together:

```bash
source ~/mapeverything_ws/install/setup.bash
ros2 bag record \
  /reconstructor/pose \
  /reconstructor/gps/fix \
  /reconstructor/gps/metadata \
  /reconstructor/pointcloud \
  /reconstructor/satellite/image/compressed \
  /reconstructor/satellite/tile_info \
  /reconstructor/dem/tile
```

Replay with:

```bash
source ~/mapeverything_ws/install/setup.bash
ros2 bag play <bag_directory> --clock
```

Useful inspection commands:

```bash
ros2 topic echo /reconstructor/satellite/tile_info
ros2 topic echo /reconstructor/dem/tile
ros2 interface show reconstructor_msgs/msg/GeoTileInfo
ros2 interface show reconstructor_msgs/msg/GeoRasterTile
```

## RViz Sample

The sample RViz configuration is `ros2/rviz/mapeverything.rviz`:

```bash
source ~/mapeverything_ws/install/setup.bash
rviz2 -d <repo>/ros2/rviz/mapeverything.rviz
```

It opens native RViz displays for pose, GPS fix,
the `/reconstructor/pointcloud` Depth Anything fused point cloud, and satellite image.
The structured custom topics `/reconstructor/satellite/tile_info`,
`/reconstructor/dem/tile`, and `/reconstructor/gps/metadata` are intended
for rosbag2 recording and topic inspection. RViz does not render those custom
message payloads directly unless a local converter node or RViz plugin maps
them to standard `MarkerArray`, `Image`, `PointCloud2`, or `Map` topics.

`/reconstructor/pointcloud` intentionally stays on standard `sensor_msgs/PointCloud2`.
It carries `x`, `y`, `z`, and packed `rgb` fields from the Depth Anything + LiDAR
fused depth path so rosbag2 can record it without custom message generation.

The config includes disabled placeholder displays for common converter outputs:

- `/reconstructor/radio/markers`
- `/reconstructor/dem/image`
- `/reconstructor/dem/markers`

Enable those displays only after starting compatible converter nodes.

## Custom Message Notes

`RadioObservation.msg` and `MeshSnapshot.msg` mirror the schemas advertised by
the app in session metadata. Session, GPS, indoor localization, satellite tile,
and DEM messages keep high-value scalar fields typed and store nested metadata
objects as JSON strings. This keeps the recorder package compact while preserving
provider policies, georeference details, radio catalog metadata, bridge
transport details, local bag storage status, adaptive mapping mode decisions,
and diagnostics in rosbag2.
`GeoTileInfo` and `GeoRasterTile` also expose `device_pixel_x`,
`device_pixel_y`, `tile_width`, `tile_height`, `pixel_origin`, and `pixel_units`
as typed scalar fields so recorder-side code can place the phone inside each
downloaded satellite or DEM tile without parsing nested JSON.
