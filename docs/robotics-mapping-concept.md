# Robotics Mapping Concept

## Summary

MapEverything should pivot from a general room-scanning and remodeling utility into a robotics mapping payload for iPhone and iPad Pro hardware. The app becomes a mobile sensor node that captures local geometry, geospatial context, radio observations, and device pose, then publishes synchronized ROS2 messages to a workstation or robot for rosbag recording, map fusion, and downstream autonomy workflows.

The app should prioritize reliable field mapping over consumer design features. A successful session produces a time-synchronized ROS2 bag containing camera-derived pose, LiDAR point clouds, reconstructed mesh, GPS fixes, radio signal observations, satellite imagery tiles, DEM/elevation tiles, device diagnostics, and session metadata.

MapEverything has one operator mode: record. Starting a session publishes the configured robotics topic set over ROS2; stopping a session stops publication. Stream selection, topic filtering, and data retention belong on the recorder-side ROS2/rosbag setup, not in per-stream app toggles. The iPhone should not be treated as the session recorder of record, aside from transient retry buffers and provider caches needed to publish reliably.

The production bridge remains `rosbridge_suite` over WebSocket for this build. A native binary bridge is not enabled because the project does not currently integrate a maintained iOS ROS2/DDS client or a recorder-side binary receiver. High-rate streams should first be controlled with compression, publish-rate limits, queue backpressure, and transient retry buffers. Revisit a native binary bridge only after measured rosbridge throughput fails field requirements and a maintained iOS client, Foxglove/ROS2 CDR path, or custom companion ROS2 receiver is selected.

## Target Use Cases

- Walk a site with an iPhone and record a ROS2 bag containing geometry, GPS, radio, and terrain context.
- Build a local map for robot navigation, inspection, or simulation from commodity mobile hardware.
- Survey Wi-Fi, BLE beacon, or link-quality coverage while collecting 3D geometry.
- Fetch georeferenced satellite imagery and DEM tiles for the area being mapped.
- Replay the session on another ROS2 system for SLAM, localization, coverage analysis, or RViz inspection.

## Product Positioning

MapEverything should be positioned as:

> A mobile robotics mapping payload that turns an iPhone into a ROS2 sensor node for LiDAR mesh capture, visual pose, GPS, radio signal telemetry, satellite imagery, and terrain elevation data.

This differs from room-scanning apps because the primary output is not a floorplan or a shareable 3D model. The primary output is a robotics dataset streamed into ROS2 and recorded on a separate machine.

## Sensor and Data Scope

### Pose and Motion

- Use ARKit camera transforms as the high-rate local pose source.
- Publish `geometry_msgs/PoseStamped` and `/tf` transforms.
- Add `nav_msgs/Odometry` for consumers that expect odometry-like pose, twist, and covariance.
- Continue publishing `sensor_msgs/Imu` from CoreMotion.
- Track ARKit quality, world-tracking state, and relocalization state as diagnostics.

### LiDAR, Point Cloud, and Mesh

- Continue publishing downsampled `sensor_msgs/PointCloud2`.
- Publish reconstructed AR mesh as either:
  - `visualization_msgs/MarkerArray` for RViz compatibility.
  - A custom `reconstructor_msgs/MeshSnapshot` for structured recording if a custom message package is installed.
- Add mesh snapshot cadence controls separate from point-cloud cadence.
- Persist mesh snapshots locally for retry if the ROS2 bridge disconnects.

### GPS and Geospatial Reference

- Use CoreLocation for latitude, longitude, altitude, horizontal accuracy, vertical accuracy, heading, course, and speed.
- Publish GPS as `sensor_msgs/NavSatFix`.
- Publish heading as `sensor_msgs/MagneticField` where appropriate, or diagnostics if confidence is insufficient.
- Add a georeferencing module that estimates an ENU transform from WGS84 GPS into the ARKit `map` frame.
- Publish frame relationships compatible with REP-105:
  - `earth`
  - `map`
  - `odom`
  - `base_link`
  - `iphone_camera`

### Radio Signal Observations

iOS public APIs restrict broad radio scanning. The implementation should use public APIs first and clearly report unavailable channels.

Supported public-data targets:

- Current Wi-Fi network signal quality via `NEHotspotNetwork` when the Access WiFi Information entitlement and location permission are available.
- BLE advertisement RSSI from configured beacon or peripheral scans via CoreBluetooth.
- Network path state, interface type, expensive/constrained state, and connection status via Network.framework.
- Optional throughput and latency probes to the ROS2 recorder endpoint.

Restricted or external-data targets:

- Full Wi-Fi access-point scans are not available through normal App Store public APIs.
- Cellular RSRP/RSRQ/RSSI is not reliably available through public iOS APIs.
- For cellular or spectrum-grade radio mapping, use an external sensor, router API, SDR, or companion ROS2 node and fuse by timestamp.

Recommended ROS output:

- Standard diagnostics on `/reconstructor/status`.
- Custom `reconstructor_msgs/RadioObservation` for Wi-Fi, BLE, network probes, and external radio adapters.
- Optional `sensor_msgs/PointCloud2` heatmap projection where each point stores signal fields.

### Satellite Imagery

The app should fetch map imagery through a provider abstraction instead of hard-coding one service.

Provider decision:

- Bake NASA GIBS WMTS as the default no-login satellite imagery source.
- Publish provider, layer, date, CRS, zoom, tile coordinate, attribution, and license metadata with every imagery tile.
- Keep commercial imagery providers out of the baked defaults unless the user supplies credentials and explicitly configures recording rights.

Provider requirements:

- Bounding-box or tile-coordinate fetch by current GPS region.
- Explicit licensing support for local cache and ROS bag recording.
- Offline cache metadata: provider, layer, zoom, tile coordinate, timestamp, attribution, and license notes.

Recommended output:

- `sensor_msgs/CompressedImage` for the raster tile.
- Custom `reconstructor_msgs/GeoTileInfo` with bounding box, CRS, zoom, resolution, provider, and attribution.
- Optional `nav_msgs/OccupancyGrid` only for derived costmap-like layers, not raw imagery.

### DEM and Elevation Maps

DEM fetching should also use a provider abstraction.

Provider decision:

- Keep Mapzen Terrain Tiles as the built-in global DEM convenience fallback because it is available through AWS Open Data without an account and works with the current tiled fetcher.
- Add USGS 3DEP as the preferred authoritative DEM source for US coverage.
- Treat OpenTopography, Copernicus DEM, USGS EROS/Earthdata downloads, and commercial terrain APIs as user-login or user-key providers.

Provider requirements:

- Fetch elevation raster for the session area.
- Support known vertical datum and coordinate reference system.
- Support local caching and recording rights.

Recommended output:

- Custom `reconstructor_msgs/GeoRasterTile` for DEM payloads and metadata.
- Optional `grid_map_msgs/GridMap` if the recorder stack uses `grid_map`.
- Derived `sensor_msgs/PointCloud2` terrain surface for RViz visualization.

## ROS2 Topic Plan

| Topic | Message Type | Rate | Purpose |
| --- | --- | --- | --- |
| `/tf` | `tf2_msgs/msg/TFMessage` | 10-30 Hz | Frame tree from geospatial and AR local frames. |
| `/reconstructor/pose` | `geometry_msgs/msg/PoseStamped` | 10-30 Hz | ARKit camera pose in local map frame. |
| `/reconstructor/odom` | `nav_msgs/msg/Odometry` | 10-30 Hz | Odometry-style pose for robotics consumers. |
| `/reconstructor/imu` | `sensor_msgs/msg/Imu` | 50-100 Hz | Device IMU. |
| `/reconstructor/gps/fix` | `sensor_msgs/msg/NavSatFix` | 1-10 Hz | GPS position and accuracy. |
| `/reconstructor/camera/image/compressed` | `sensor_msgs/msg/CompressedImage` | 1-10 Hz | Camera frames for context and replay. |
| `/reconstructor/pointcloud` | `sensor_msgs/msg/PointCloud2` | 1-10 Hz | Downsampled LiDAR or fused point cloud. |
| `/reconstructor/mesh_markers` | `visualization_msgs/msg/MarkerArray` | 0.2-2 Hz | RViz-friendly mesh and semantic objects. |
| `/reconstructor/radio` | `reconstructor_msgs/msg/RadioObservation` | 0.5-5 Hz | Wi-Fi, BLE, link, or external radio measurements. |
| `/reconstructor/satellite/image/compressed` | `sensor_msgs/msg/CompressedImage` | on fetch | Satellite imagery tile payloads. |
| `/reconstructor/satellite/tile_info` | `reconstructor_msgs/msg/GeoTileInfo` | on fetch | Satellite imagery georeference metadata. |
| `/reconstructor/dem/tile` | `reconstructor_msgs/msg/GeoRasterTile` | on fetch | DEM/elevation raster payloads. |
| `/reconstructor/status` | `diagnostic_msgs/msg/DiagnosticArray` | 1 Hz | App, sensor, permission, bridge, and recorder health. |
| `/reconstructor/session` | `reconstructor_msgs/msg/MappingSession` | on change | Session metadata and configuration. |

## Custom ROS Message Package

A companion `reconstructor_msgs` package should be created for data that has no clean standard ROS representation.

Initial messages:

- `MappingSession.msg`
- `RadioObservation.msg`
- `GeoTileInfo.msg`
- `GeoRasterTile.msg`
- `MeshSnapshot.msg`
- `PublisherStats.msg`

The recorder device must build this package before recording custom topics with `rosbag2`.

## iOS Architecture Plan

### Core Services

- `MappingSessionManager`: owns session lifecycle, permissions, recorder connection state, and topic configuration.
- `PosePublisher`: publishes ARKit pose, odometry, TF, and tracking diagnostics.
- `LocationPublisher`: wraps CoreLocation and publishes GPS, heading, accuracy, and georeference updates.
- `RadioTelemetryProvider`: collects Wi-Fi, BLE, network path, and active probe measurements.
- `MeshPublisher`: batches mesh anchor snapshots and point-cloud updates.
- `GeoTileProvider`: fetches, caches, and publishes satellite imagery and DEM tiles.
- `ROS2TopicRegistry`: defines advertised topics, message types, publish rates, and recorder-facing metadata.
- `ROS2BridgeTransportProfile`: publishes the active bridge kind, encoding, rationale, and upgrade path in session metadata and diagnostics.
- `PublishQueue`: applies backpressure, retry, compression, and drop policies.
- `SessionStore`: holds active session metadata, transient fetch history, and retryable samples without creating a local recording artifact.

### UI Changes

- Replace remodeling/landscape-first navigation with a single record-mode mapping view.
- Add prominent recorder connection status, publish rates, queue depth, and dropped-message counts.
- Keep stream selection and rosbag retention out of the app UI; configure subscriptions and recording policy on the recorder side.
- Keep map area defaults for satellite/DEM tile radius, zoom, provider, and cache behavior in build/config or recorder-side session setup.
- Add a permissions screen for camera, motion, location, Bluetooth, local network, and Wi-Fi information entitlement.
- Add diagnostics views for publisher health; do not add local session export as a primary workflow.

### Data Model Changes

- Add `MappingSessionModel` with start/end time, device info, coordinate frame config, provider config, and remote recorder URL.
- Add `SensorStreamModel` for publisher status, publish rate, message counts, and last error.
- Add `GeoTileModel` for satellite and DEM cache records.
- Add transient radio observation buffering if publishing is interrupted.

## Implementation Phases

### Phase 1: Robotics Core

- Rename the active product concept around robotics mapping.
- Add `MappingSessionManager`.
- Add topic registry and recorder-facing publisher metadata.
- Publish active bridge transport metadata and document the rosbridge/native-binary decision.
- Add `nav_msgs/Odometry`, `sensor_msgs/NavSatFix`, and diagnostics publishing.
- Add basic session metadata topic.
- Add a recorder connection dashboard.

### Phase 2: Radio and Geospatial Sensors

- Add CoreLocation-based GPS stream with covariance and accuracy.
- Add Wi-Fi current-network telemetry where entitlements allow it.
- Add BLE beacon RSSI capture for configured peripherals.
- Add network path and endpoint latency probes.
- Add `RadioObservation` custom message definition.

### Phase 3: Satellite and DEM Ingestion

- Add provider interfaces for satellite imagery and DEM tiles.
- Add source-policy metadata for provider attribution, credential requirements, cache policy, and recordable-by-default status.
- Add USGS 3DEP as the preferred US DEM provider while retaining Mapzen Terrain Tiles as the global fallback.
- Add tile cache with attribution and license metadata.
- Add geospatial tile publishing with compressed image and metadata topics.
- Add DEM publishing with georeferenced raster metadata.
- Add offline/failed-fetch behavior.

### Phase 4: Mesh and Map Recording Robustness

- Add mesh snapshot throttling and retry.
- Add local buffering for critical samples during bridge drops.
- Add publish queue backpressure and statistics.
- Add ROS bag validation checklist and sample launch files.
- Add RViz config for pose, GPS, mesh, radio, satellite, and DEM layers.

### Phase 5: Field Validation

- Validate indoor, outdoor, Wi-Fi, BLE, and GPS scenarios.
- Compare GPS/AR frame alignment against known survey points.
- Confirm rosbag replay reconstructs session state in RViz.
- Stress test long sessions, poor network, thermal pressure, and large mesh snapshots.

## Open Decisions

- Whether to add an offline packaged map product for field deployments, separate from the default online provider registry.
- Whether to support custom ROS messages only, or also standard-message fallbacks for every stream.
- Whether to keep RoomPlan as an optional semantic layer or remove it from the default robotics workflow.
- Whether to rename the app before implementing the robotics UI.
