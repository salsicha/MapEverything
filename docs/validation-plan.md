# MapEverything Validation Plan

This plan separates simulator-safe validation from checks that require a LiDAR iOS device, a recorder workstation, or a second ROS2/RViz machine.

## Simulator-Safe Unit Coverage

Run the targeted validation tests from Xcode or the command line:

```bash
xcodebuild test \
  -project MapEverything/MapEverything.xcodeproj \
  -scheme MapEverything \
  -destination 'platform=iOS Simulator,name=iPhone 17,OS=26.5' \
  -only-testing:MapEverythingTests/MapEverythingTests
```

The validation suite covers:

- ROS2 topic registry serialization and advertised message metadata.
- GPS WGS84 fix conversion into local ENU meters and the ARKit `map` frame.
- Geo tile cache indexing, provider metadata, tile bounds, and pixel coordinates.
- Publish queue capacity drops, retry accounting, and final send order.

## Rosbridge Throughput Benchmark

Use the benchmark harness to publish representative default-profile camera, Depth Anything point-cloud, satellite, and DEM messages at target field rates. Mesh remains an opt-in stress case:

```bash
python3 tools/rosbridge-throughput-benchmark.py --dry-run --duration 5
python3 -m pip install websockets
python3 tools/rosbridge-throughput-benchmark.py --url ws://<RECORDER_IP>:9090 --duration 60
```

On the recorder workstation, monitor the same topics in parallel:

```bash
ros2 topic hz /mapping/pose
ros2 topic hz /mapping/gps/fix
ros2 topic hz /mapping/pointcloud
ros2 topic hz /mapping/satellite/image/compressed
ros2 topic hz /mapping/satellite/tile_info
ros2 topic hz /mapping/dem/tile

ros2 topic bw /mapping/pointcloud
ros2 topic bw /mapping/satellite/image/compressed
ros2 topic bw /mapping/dem/tile
```

Pass criteria:

- Observed rates remain within 10 percent of the target rate for each profile.
- The rosbridge process does not drop the WebSocket connection during a 60 second run.
- The rosbridge queue does not show sustained growth when optional diagnostics are enabled.
- Camera capture at 2 Hz and optional mesh stress profiles do not starve point-cloud, satellite, or DEM publications.

## Physical Device Test Matrix

Use a LiDAR-capable iPhone or iPad on the same network as the recorder workstation.

1. Start rosbridge:

   ```bash
   source ~/mapeverything_ws/install/setup.bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```

2. Start a bag recording:

   ```bash
ros2 bag record -o mapeverything_validation \
 /mapping/pose \
 /mapping/camera/image/compressed \
 /mapping/camera/camera_info \
 /mapping/gps/fix \
 /mapping/gps/metadata \
 /mapping/pointcloud \
    /mapping/satellite/image/compressed \
    /mapping/satellite/tile_info \
    /mapping/dem/tile
   ```

3. Validate each device capability:

| Area | Procedure | Pass Criteria |
| :--- | :--- | :--- |
| GPS | Start outdoors with precise location enabled, then walk at least 20 meters. | `/mapping/gps/fix` publishes finite lat/lon, covariance reflects accuracy, and `/mapping/gps/metadata` includes georeference JSON after an accurate fix. |
| LiDAR + Depth Anything | Record in LiDAR + Depth Anything mode while moving around varied geometry. | `/mapping/pointcloud` publishes a stable colored fused point cloud in `map`, while camera image and camera_info remain near the 2 Hz budget. |
| BLE | Configure one or more beacon filters and enable Bluetooth. | `/mapping/radio` includes BLE observations or `/mapping/status` explains permission/filter state. |
| Wi-Fi | Join the recorder network with Location permission and Wi-Fi info entitlement enabled. | Session metadata reports current Wi-Fi telemetry and avoids broad scan claims. |
| Satellite fetch | Record with a valid outdoor GPS fix and network access. | `/mapping/satellite/tile_info` publishes bounds, CRS, attribution, source policy, and `device_pixel_x`/`device_pixel_y`, with imagery on `/mapping/satellite/image/compressed`. |
| DEM fetch | Test once in USGS 3DEP coverage and once outside US coverage. | US locations prefer USGS 3DEP, global locations fall back to Mapzen Terrain Tiles, and `/mapping/dem/tile` carries raster data, source policy, and device pixel coordinates. |
| Rosbag recording | Stop recording after at least 5 minutes. | Bag contains all enabled default topics and no required custom message type is missing. |

## Rosbag Replay and RViz Validation

Replay the bag on a ROS2 machine that is separate from the recorder used during capture:

```bash
source ~/mapeverything_ws/install/setup.bash
ros2 bag info mapeverything_validation
ros2 bag play mapeverything_validation --clock
rviz2 -d ros2/rviz/mapeverything.rviz
```

Pass criteria:

- RViz fixed frame `map` resolves pose, GPS, point cloud, satellite imagery, and DEM metadata without missing message definitions.
- Replaying the bag preserves timing closely enough that point cloud, pose, GPS, and geotile context remain aligned.
- The replay machine can inspect `reconstructor_msgs` fields with `ros2 interface show` and `ros2 topic echo`.

## Long Session, Thermal, and Poor-Network Validation

Run at least one 45 minute capture on a physical device.

1. Begin on a strong Wi-Fi connection and record all default streams.
2. After 10 minutes, move through a lower signal area or use a router/network-conditioner profile with packet loss.
3. After 20 minutes, continue mapping while the device is under thermal load.
4. After 30 minutes, return to strong Wi-Fi and keep recording.
5. Stop at 45 minutes and save recorder logs, rosbag metadata, and app diagnostics.

Pass criteria:

- `/mapping/status` reports queue depth, dropped messages, retry count, recorder probe latency, thermal state, and last error throughout the run.
- The publish queue drops bounded old publish messages instead of growing without limit.
- The bridge reconnects or resumes publication after network recovery without requiring an app restart.
- The final rosbag remains replayable in RViz.
- No topic shows unbounded timestamp drift or non-finite numeric payloads.
