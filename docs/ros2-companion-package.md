# ROS2 Companion Package

MapEverything publishes several custom message types under `mapeverything_msgs`.
The package lives in `ros2/mapeverything_msgs` and can be copied or symlinked
into a normal ROS2 colcon workspace on the recorder machine.

## Build the Messages

```bash
mkdir -p ~/mapeverything_ws/src
cp -R ros2/mapeverything_msgs ~/mapeverything_ws/src/
cd ~/mapeverything_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select mapeverything_msgs
source install/setup.bash
ros2 interface show mapeverything_msgs/msg/RadioObservation
```

If you prefer to keep the package linked to this repository, replace the `cp`
step with a symlink from `~/mapeverything_ws/src/mapeverything_msgs` to
`<repo>/ros2/mapeverything_msgs`.

## rosbridge Setup

Install and launch rosbridge after sourcing the workspace that contains
`mapeverything_msgs`:

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
  /mapping/camera/image/compressed \
  /mapping/camera/camera_info \
  /mapping/pose \
  /mapping/gps/fix \
  /mapping/gps/metadata \
  /mapping/pointcloud/lidar \
  /mapping/pointcloud/depth_anything \
  /mapping/depth_anything/calibration \
  /mapping/satellite/image/compressed \
  /mapping/satellite/tile_info \
  /mapping/dem/tile
```

Replay with:

```bash
source ~/mapeverything_ws/install/setup.bash
ros2 bag play <bag_directory> --clock
```

Useful inspection commands:

```bash
ros2 topic echo /mapping/satellite/tile_info
ros2 topic echo /mapping/dem/tile
ros2 topic echo /mapping/depth_anything/calibration
ros2 interface show mapeverything_msgs/msg/GeoTileInfo
ros2 interface show mapeverything_msgs/msg/GeoRasterTile
ros2 interface show mapeverything_msgs/msg/DepthAnythingCalibration
```

## RViz Sample

The sample RViz configuration is `ros2/rviz/mapeverything.rviz`:

```bash
source ~/mapeverything_ws/install/setup.bash
rviz2 -d <repo>/ros2/rviz/mapeverything.rviz
```

It opens native RViz displays for pose, GPS fix,
the `/mapping/pointcloud/lidar` LiDAR point cloud,
the `/mapping/pointcloud/depth_anything` relative Depth Anything point cloud, and satellite image.
The structured custom topics `/mapping/satellite/tile_info`,
`/mapping/dem/tile`, `/mapping/depth_anything/calibration`, and `/mapping/gps/metadata` are intended
for rosbag2 recording and topic inspection. RViz does not render those custom
message payloads directly unless a local converter node or RViz plugin maps
them to standard `MarkerArray`, `Image`, `PointCloud2`, or `Map` topics.

Both point-cloud topics intentionally stay on standard `sensor_msgs/PointCloud2`.
They carry `x`, `y`, `z`, and packed `rgb` fields so rosbag2 can record them
without custom message generation. `/mapping/pointcloud/depth_anything` is a
relative-depth camera-frame cloud; pair it with
`/mapping/depth_anything/calibration` to rebuild the metric depth used by the
on-device overlay mesh.

The config includes disabled placeholder displays for common converter outputs:

- `/mapping/radio/markers`
- `/mapping/dem/image`
- `/mapping/dem/markers`

Enable those displays only after starting compatible converter nodes.

## Custom Message Notes

`RadioObservation.msg`, `DepthAnythingCalibration.msg`, and `MeshSnapshot.msg` mirror the schemas advertised by
the app in session metadata. Session, GPS, indoor localization, satellite tile,
and DEM messages keep high-value scalar fields typed and store nested metadata
objects as JSON strings. This keeps the recorder package compact while preserving
provider policies, georeference details, radio catalog metadata, bridge
transport details, local bag storage status, mapping-engine metadata,
and diagnostics in rosbag2.
`MeshSnapshot` schema version 2 stores triangle-list geometry as rosbridge
base64 `uint8[]` blobs: `vertex_data` is packed little-endian `float32 x,y,z`
with a 12-byte stride, and `index_data` is packed little-endian `uint32` with a
4-byte stride. This avoids the large JSON-object overhead of per-vertex
`geometry_msgs/Point` arrays while staying compatible with rosbridge JSON bags.
`GeoTileInfo` and `GeoRasterTile` also expose `device_pixel_x`,
`device_pixel_y`, `tile_width`, `tile_height`, `pixel_origin`, and `pixel_units`
as typed scalar fields so recorder-side code can place the phone inside each
downloaded satellite or DEM tile without parsing nested JSON.
