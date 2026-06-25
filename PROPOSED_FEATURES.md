# Proposed Robotics Mapping Features

This roadmap tracks MapEverything as a robotics mapping app. The app acts as an iPhone-based ROS2 sensor payload that captures pose, LiDAR geometry, camera context, GPS, radio signal observations, satellite imagery, and DEM/elevation data for recording on another device.

The detailed concept is in [docs/robotics-mapping-concept.md](docs/robotics-mapping-concept.md). Implementation tasks are tracked in [TODO.md](TODO.md).

## Product Direction

MapEverything is a robust field-mapping node for robotics teams. The primary output is a synchronized ROS2 data stream and rosbag recording, not a consumer floorplan/design asset.

The app has one operator mode: record. Start publishes the configured topic set, stop ends publication, and the recorder-side ROS2 system decides which topics to subscribe to, bag, replay, or discard. The iPhone should not hold local session recordings beyond transient buffers and caches required for reliable publication.

The production transport remains `rosbridge_suite` over WebSocket for this build. A native binary bridge is not enabled because no maintained iOS ROS2/DDS client or recorder-side binary receiver is integrated in the project. The first optimization path is compression, throttling, backpressure, and benchmarking; revisit a native binary bridge only if rosbridge cannot meet measured field rates.

Core positioning:

- Mobile robotics mapping payload.
- LiDAR and ARKit pose source.
- GPS and geospatial context recorder.
- Radio signal survey tool.
- Satellite imagery and DEM tile fetcher.
- ROS2 publisher for external recording, replay, and visualization.

## Differentiating Features

| Priority | Feature | ROS2 Output | Implementation Notes |
| --- | --- | --- | --- |
| P0 | Mapping session manager | `/mapping/session` | Owns session lifecycle, recorder URL, metadata, and transient recovery state. |
| P0 | Bridge transport profile | `/mapping/session`, `/mapping/status` | Reports active rosbridge transport, encoding, rationale, and native-binary upgrade path. |
| P0 | Pose, TF, and odometry publishing | `/tf`, `/mapping/pose`, `/mapping/odom` | Use ARKit pose and CoreMotion; add covariance and diagnostics where possible. |
| P0 | GPS publishing | `/mapping/gps/fix` | Use CoreLocation with accuracy metadata and stale-fix diagnostics. |
| P0 | Point-cloud publishing | `/mapping/pointcloud` | Keep voxel controls, rate limits, payload metrics, and backpressure. |
| P0 | Mesh snapshot publishing | `/mapping/mesh_markers`, optional custom mesh topic | Continue RViz MarkerArray support; add structured custom messages later. |
| P0 | Diagnostics and recorder dashboard | `/mapping/status` | Show connection, publish rates, queue depth, dropped messages, permissions, and last error without exposing per-stream app toggles. |
| P1 | Radio signal telemetry | `/mapping/radio` | Support current Wi-Fi quality, BLE RSSI, network path state, endpoint latency, and external adapters. |
| P1 | Georeferenced satellite imagery | `/mapping/satellite/image/compressed`, `/mapping/satellite/tile_info` | Provider abstraction required; check cache and recording rights before choosing a provider. |
| P1 | DEM/elevation tiles | `/mapping/dem/tile` | Provider abstraction required; preserve CRS, vertical datum, resolution, and attribution. |
| P1 | Publish queue and reconnect buffer | all topics | Prevent high-rate streams from overwhelming rosbridge or losing critical state during disconnects. |
| P1 | Companion ROS2 message package | `reconstructor_msgs/*` | Needed for radio observations, geotile metadata, DEM rasters, mesh snapshots, and session metadata. |
| P2 | RViz and rosbag tooling | sample launch/config files | Provide a reproducible recorder-side workflow. |

## Sensor Streams

### Required Streams

- ARKit pose and `/tf`.
- CoreMotion IMU.
- LiDAR or fused point cloud.
- AR mesh snapshots.
- Camera compressed image stream.
- CoreLocation GPS fix.
- Diagnostics and publisher stats.

### Planned Streams

- Current Wi-Fi signal quality where entitlement and permission allow it.
- BLE beacon/peripheral RSSI via CoreBluetooth.
- Network path state and recorder latency probes.
- Satellite imagery tiles.
- DEM/elevation raster tiles.
- Optional external radio adapter measurements.

### Explicit iOS Constraints

- Broad Wi-Fi scanning is not available through normal public iOS APIs.
- Reliable cellular RSSI/RSRP is not available through normal public iOS APIs.
- Satellite imagery and DEM providers must allow the app to cache and record fetched data into ROS bags.
- Some telemetry requires location, Bluetooth, local network, and Access WiFi Information permissions or entitlements.

## ROS2 Topic Roadmap

| Topic | Message Type | Status |
| --- | --- | --- |
| `/tf` | `tf2_msgs/msg/TFMessage` | Existing, expand frame tree. |
| `/mapping/pose` | `geometry_msgs/msg/PoseStamped` | Existing, add diagnostics and covariance companion data. |
| `/mapping/odom` | `nav_msgs/msg/Odometry` | Add. |
| `/mapping/imu` | `sensor_msgs/msg/Imu` | Existing, validate frame conventions. |
| `/mapping/gps/fix` | `sensor_msgs/msg/NavSatFix` | Add. |
| `/mapping/camera/image/compressed` | `sensor_msgs/msg/CompressedImage` | Existing, report publish status and metrics. |
| `/mapping/pointcloud` | `sensor_msgs/msg/PointCloud2` | Existing, add backpressure and metrics. |
| `/mapping/mesh_markers` | `visualization_msgs/msg/MarkerArray` | Existing map topic can evolve into this. |
| `/mapping/radio` | `reconstructor_msgs/msg/RadioObservation` | Add custom message. |
| `/mapping/satellite/image/compressed` | `sensor_msgs/msg/CompressedImage` | Add. |
| `/mapping/satellite/tile_info` | `reconstructor_msgs/msg/GeoTileInfo` | Add custom message. |
| `/mapping/dem/tile` | `reconstructor_msgs/msg/GeoRasterTile` | Add custom message. |
| `/mapping/status` | `diagnostic_msgs/msg/DiagnosticArray` | Add. |
| `/mapping/session` | `reconstructor_msgs/msg/MappingSession` | Add custom message. |

## Implementation Priorities

### Milestone 1: Robotics Core

- Mapping session manager.
- Topic registry.
- Publish queue with backpressure.
- Bridge transport metadata.
- Odometry and GPS topics.
- Diagnostics topic.
- Recorder dashboard.

### Milestone 2: Radio and Geospatial Sensors

- CoreLocation GPS stream and ENU georeferencing.
- Current Wi-Fi quality stream.
- BLE RSSI stream.
- Network path and latency probes.
- Radio observation message schema.

### Milestone 3: Terrain and Imagery Context

- Satellite imagery provider abstraction.
- DEM provider abstraction.
- Tile cache with attribution and license metadata.
- Georeferenced satellite and DEM ROS topics.

### Milestone 4: Recording Robustness

- Local buffering during disconnects.
- Mesh and point-cloud rate controls.
- Payload size reporting.
- Rosbridge throughput benchmark before any native binary bridge work.
- Recorder-side rosbag and RViz examples.
- Long-session field validation.

## Success Criteria

- A user can start a mapping session, connect to a ROS2 recorder, and record a rosbag containing pose, TF, IMU, GPS, point cloud, mesh, camera images, diagnostics, and session metadata.
- Radio observations are recorded when supported by public APIs or configured external adapters.
- Satellite and DEM tiles are fetched, cached with attribution, and published with georeferencing metadata.
- Rosbag replay in RViz reconstructs the session with visible trajectory, point cloud, mesh, GPS path, imagery, DEM context, and diagnostics.
- The app handles recorder disconnects, large payloads, permission failures, and long field sessions without silently losing critical state.
