# TODO

## P0 - Robotics Mapping Core

- [x] Create `MappingSessionManager` to own session lifecycle, active streams, remote recorder URL, and session metadata.
- [x] Add `ROS2TopicRegistry` for advertised topics, message types, publisher metadata, and publish rates.
- [x] Use maximum-likelihood LiDAR + Depth Anything fusion for mapping point clouds.
- [x] Add `PublishQueue` with backpressure, retry, drop policy, queue-depth metrics, and last-error reporting.
- [x] Add low-rate background satellite imagery and DEM tile fetch/publish during mapping sessions.
- [x] Publish `diagnostic_msgs/DiagnosticArray` on `/reconstructor/status`.
- [x] Publish session metadata on `/reconstructor/session`.
- [x] Access CoreLocation indoor localization services for indoor registration quality and global registration.
- [x] Publish `sensor_msgs/NavSatFix` on `/reconstructor/gps/fix`.
- [x] Add CoreLocation permission flow for precise location and heading.
- [x] Publish `nav_msgs/Odometry` on `/reconstructor/odom`.
- [x] Include horizontal accuracy, vertical accuracy, altitude, speed, course, and timestamp metadata where available.
- [x] Add ENU georeferencing from WGS84 GPS into the ARKit map frame.
- [x] Publish a REP-105-compatible frame tree: `earth`, `map`, `odom`, `base_link`, and `iphone_camera`.
- [x] Add GPS quality diagnostics for stale fixes, reduced accuracy, denied permission, and poor heading confidence.
- [x] Separate point-cloud publish rate from local scan/update rate.
- [x] Add mesh snapshot publish rate and payload-size controls.
- [x] Add local buffering for point-cloud and mesh samples during bridge disconnects.
- [x] Add a minimal recorder diagnostics panel showing connection state, queue depth, dropped messages, and last error.

## P1 - Radio Signal Telemetry

- [x] Define public-API-supported radio channels: current Wi-Fi network, BLE advertisements, network path state, latency probes, and optional external adapters.
- [x] Add Access WiFi Information entitlement notes and permission handling for current Wi-Fi signal quality.
- [x] Add Network.framework path diagnostics for interface type, constrained state, expensive state, and reachability.
- [x] Add endpoint latency and throughput probes against the ROS2 recorder.
- [x] Add CoreBluetooth scanning for configured BLE beacon RSSI.
- [x] Define `reconstructor_msgs/RadioObservation`.
- [ ] Publish radio observations on `/reconstructor/radio`.
- [ ] Document iOS restrictions: no broad Wi-Fi scans and no reliable public cellular RSSI/RSRP stream.

## P1 - Satellite Imagery and DEM Tiles

- [x] Add `GeoTileProvider` abstraction for satellite imagery and DEM sources.
- [x] Select initial satellite imagery provider after checking cache, recording, attribution, and redistribution terms.
- [x] Select initial DEM provider after checking datum, resolution, region support, cache, and recording terms.
- [ ] Add source-policy metadata to `GeoTileProvider` for recordable-by-default, transient-cache-only, attribution URL, and credential requirement.
- [ ] Add USGS 3DEP provider and prefer it for US DEM tiles, with Mapzen Terrain Tiles as the global fallback.
- [ ] Add user credential/configuration slots for optional Copernicus Data Space, OpenTopography, USGS EROS/Earthdata, and commercial provider integrations.
- [x] Add tile fetch by current GPS location, radius, zoom, and bounding box.
- [x] Add local tile cache with provider, layer, CRS, zoom, tile coordinate, timestamp, attribution, and license metadata.
- [x] Publish satellite imagery as `sensor_msgs/CompressedImage`.
- [x] Define and publish `reconstructor_msgs/GeoTileInfo` for satellite georeferencing metadata.
- [x] Define and publish `reconstructor_msgs/GeoRasterTile` for DEM payloads.
- [x] Publish device latitude, longitude, altitude, and tile pixel coordinates with satellite and DEM tiles.
- [x] Add offline behavior for unavailable network or provider errors.

## P1 - Mesh, Point Cloud, and Camera Publishing

- [ ] Add mesh publishing fallback through `visualization_msgs/MarkerArray`.
- [ ] Define `reconstructor_msgs/MeshSnapshot` for structured mesh recording.
- [ ] Add compression and payload-size metrics for camera and point-cloud streams.
- [ ] Add stress tests for large mesh snapshots and long-running point-cloud sessions.

## P2 - Companion ROS2 Package

- [ ] Create a `reconstructor_msgs` ROS2 package for custom message definitions.
- [ ] Add `MappingSession.msg`.
- [ ] Add `GPSMetadata.msg`.
- [ ] Add `RadioObservation.msg`.
- [ ] Add `GeoTileInfo.msg`.
- [ ] Add `GeoRasterTile.msg`.
- [ ] Add `IndoorLocalization.msg`.
- [ ] Add `MeshSnapshot.msg`.
- [ ] Add `PublisherStats.msg`.
- [ ] Add rosbridge and rosbag2 setup instructions for building and recording custom messages.
- [ ] Add sample RViz config for pose, GPS, point cloud, mesh, radio, satellite, and DEM layers.

## P2 - Data Model and Persistence

- [ ] Add `MappingSessionModel` for session metadata and recorder configuration.
- [ ] Add `SensorStreamModel` for publisher status, publish rate, message counts, and last error.
- [ ] Add `GeoTileModel` for satellite and DEM cache records.
- [ ] Add transient radio observation buffering for network loss.
- [ ] Add migration tests for existing `EnvironmentModel` data.
- [ ] Add cleanup logic for orphaned point-cloud, mesh, imagery, DEM, and session files.

## P2 - Validation

- [ ] Add simulator-safe unit tests for topic serialization and message schemas.
- [ ] Add unit tests for GPS-to-ENU conversion.
- [ ] Add unit tests for tile metadata and cache indexing.
- [ ] Add unit tests for publish queue backpressure and retry behavior.
- [ ] Benchmark rosbridge throughput for camera, point-cloud, mesh, satellite, and DEM topics at target field rates.
- [ ] Add physical-device test plan for GPS, LiDAR, BLE, Wi-Fi, satellite tile fetch, DEM fetch, and rosbag recording.
- [ ] Validate rosbag replay in RViz on a separate ROS2 machine.
- [ ] Validate long sessions under thermal pressure and poor network conditions.

## Completed Foundation

- [x] Reposition the app as a robotics mapping payload rather than a general room-scanning/remodeling tool.
- [x] Update README product language around ROS2 mapping, field data capture, GPS, radio telemetry, satellite imagery, and DEM support.
- [x] Keep RoomPlan as an optional semantic layer unless it conflicts with the robotics-first workflow.
- [x] Define MapEverything as a single record-mode ROS2 publisher with external recorder-side data retention.
- [x] Keep rosbridge WebSocket as the production bridge for now; no native ROS2/DDS iOS client or recorder-side binary receiver is integrated in this build.

## Open Product Decisions

- [x] Decide whether to rename the app before larger UI changes. Product name: MapEverything.
- [x] Decide whether stream-level UI toggles belong in the app or only in recorder-side configuration. Decision: keep the app to start/stop record mode; choose topic subscriptions and rosbag retention on the recorder side.
- [x] Decide whether rosbridge remains sufficient or whether high-rate streams need a native binary bridge. Decision: continue with rosbridge until throughput benchmarks prove it insufficient; revisit a native binary bridge only with a maintained iOS client or companion ROS2 binary receiver.
- [x] Choose satellite imagery and DEM providers after checking cache, recording, attribution, and redistribution terms. Decision: bake NASA GIBS imagery and Mapzen Terrain Tiles DEM fallback; add USGS 3DEP as preferred US DEM provider; keep login/API-key sources user-configured.
