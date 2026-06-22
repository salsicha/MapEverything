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
- [x] Publish radio observations on `/reconstructor/radio`.
- [x] Document iOS restrictions: no broad Wi-Fi scans and no reliable public cellular RSSI/RSRP stream.

## P1 - Satellite Imagery and DEM Tiles

- [x] Add `GeoTileProvider` abstraction for satellite imagery and DEM sources.
- [x] Select initial satellite imagery provider after checking cache, recording, attribution, and redistribution terms.
- [x] Select initial DEM provider after checking datum, resolution, region support, cache, and recording terms.
- [x] Add source-policy metadata to `GeoTileProvider` for recordable-by-default, transient-cache-only, attribution URL, and credential requirement.
- [x] Add USGS 3DEP provider and prefer it for US DEM tiles, with Mapzen Terrain Tiles as the global fallback.
- [x] Add user credential/configuration slots for optional Copernicus Data Space, OpenTopography, USGS EROS/Earthdata, and commercial provider integrations.
- [x] Add tile fetch by current GPS location, radius, zoom, and bounding box.
- [x] Add local tile cache with provider, layer, CRS, zoom, tile coordinate, timestamp, attribution, and license metadata.
- [x] Publish satellite imagery as `sensor_msgs/CompressedImage`.
- [x] Define and publish `reconstructor_msgs/GeoTileInfo` for satellite georeferencing metadata.
- [x] Define and publish `reconstructor_msgs/GeoRasterTile` for DEM payloads.
- [x] Publish device latitude, longitude, altitude, and tile pixel coordinates with satellite and DEM tiles.
- [x] Add offline behavior for unavailable network or provider errors.

## P1 - Mesh, Point Cloud, and Camera Publishing

- [x] Add mesh publishing fallback through `visualization_msgs/MarkerArray`.
- [x] Define `reconstructor_msgs/MeshSnapshot` for structured mesh recording.
- [x] Add compression and payload-size metrics for camera and point-cloud streams.
- [x] Add stress tests for large mesh snapshots and long-running point-cloud sessions.

## P2 - Companion ROS2 Package

- [x] Create a `reconstructor_msgs` ROS2 package for custom message definitions.
- [x] Add `MappingSession.msg`.
- [x] Add `GPSMetadata.msg`.
- [x] Add `RadioObservation.msg`.
- [x] Add `GeoTileInfo.msg`.
- [x] Add `GeoRasterTile.msg`.
- [x] Add `IndoorLocalization.msg`.
- [x] Add `MeshSnapshot.msg`.
- [x] Add `PublisherStats.msg`.
- [x] Add rosbridge and rosbag2 setup instructions for building and recording custom messages.
- [x] Add sample RViz config for pose, GPS, point cloud, mesh, radio, satellite, and DEM layers.

## P2 - Data Model and Persistence

- [x] Add `MappingSessionModel` for session metadata and recorder configuration.
- [x] Add `SensorStreamModel` for publisher status, publish rate, message counts, and last error.
- [x] Add `GeoTileModel` for satellite and DEM cache records.
- [x] Add transient radio observation buffering for network loss.
- [x] Add off-by-default local SQLite bag storage for chunked on-device ROS2 topic capture.
- [x] Add migration tests for existing `EnvironmentModel` data.
- [x] Add cleanup logic for orphaned point-cloud, mesh, imagery, DEM, and session files.

## P2 - Adaptive Indoor/Outdoor Mapping

- [x] Add an adaptive mapping-mode policy that scores RoomPlan suitability, outdoor GPS context, LiDAR depth confidence, Depth Anything availability, thermal pressure, and operator override state.
- [x] Prefer parametric RoomPlan capture for enclosed interiors and switch to LiDAR + Depth Anything outdoor mapping with GPS, satellite imagery, and DEM context when room semantics are not reliable.
- [x] Publish active mapping mode, confidence, reason codes, and override state in `/reconstructor/session` and `/reconstructor/status`.

## P2 - Validation

- [x] Add simulator-safe unit tests for topic serialization and message schemas.
- [x] Add unit tests for GPS-to-ENU conversion.
- [x] Add unit tests for tile metadata and cache indexing.
- [x] Add unit tests for publish queue backpressure and retry behavior.
- [x] Add rosbridge throughput benchmark harness for camera, point-cloud, mesh, satellite, and DEM topics at target field rates.
- [x] Add physical-device validation plan for GPS, LiDAR, BLE, Wi-Fi, satellite tile fetch, DEM fetch, and rosbag recording.
- [x] Add rosbag replay validation procedure for RViz on a separate ROS2 machine.
- [x] Add long-session validation procedure for thermal pressure and poor network conditions.

## P3 - Packaging and Distribution

- [ ] Decide supported distribution channels: local developer builds, signed ad hoc/internal beta builds, TestFlight, App Store, and GitHub release artifacts for ROS2 companion assets.
- [ ] Audit bundle identifier, signing team, provisioning profiles, capabilities, and entitlements for camera, precise location, Bluetooth, local network, and Wi-Fi info access.
- [ ] Add a release build/archive checklist for LiDAR-only device testing, simulator-safe tests, physical-device validation, rosbag replay, and rosbridge throughput benchmarking.
- [ ] Add signed iOS archive/export instructions, including `xcodebuild archive`, export options, version/build number handling, and TestFlight upload steps.
- [ ] Add App Store Connect metadata checklist for description, screenshots, privacy nutrition labels, export compliance, support URL, and TestFlight beta review notes.
- [ ] Add a privacy and compliance checklist for `Info.plist` usage strings, Privacy Manifest entries, data-retention language, radio/GPS disclaimers, and third-party geospatial attribution.
- [ ] Define release artifact layout for app builds, release notes, validation reports, sample RViz config, rosbridge benchmark results, and source-policy/attribution records.
- [ ] Package the ROS2 companion package as a versioned source archive with build instructions, supported ROS2 distros, message compatibility notes, and a colcon smoke-test command.
- [ ] Add GitHub release/tagging process with semantic versioning, changelog generation, migration notes, and rollback instructions for both iOS and ROS2 companion packages.
- [ ] Document beta operator setup: required hardware, recorder workstation setup, network assumptions, permission prompts, credential configuration, and known platform restrictions.
- [ ] Add a release-blocker checklist for missing provisioning, failed validation, missing attribution, credential leakage, non-replayable bags, queue instability, or unsupported sensor claims.

## Completed Foundation

- [x] Reposition the app as a robotics mapping payload rather than a general room-scanning/remodeling tool.
- [x] Update README product language around ROS2 mapping, field data capture, GPS, radio telemetry, satellite imagery, and DEM support.
- [x] Keep RoomPlan as an optional semantic layer unless it conflicts with the robotics-first workflow.
- [x] Define MapEverything as a single record-mode ROS2 publisher with external recorder-side data retention by default and optional local SQLite bag fallback.
- [x] Keep rosbridge WebSocket as the production bridge for now; no native ROS2/DDS iOS client or recorder-side binary receiver is integrated in this build.

## Open Product Decisions

- [x] Decide whether to rename the app before larger UI changes. Product name: MapEverything.
- [x] Decide whether stream-level UI toggles belong in the app or only in recorder-side configuration. Decision: keep the app to start/stop record mode; choose topic subscriptions and rosbag retention on the recorder side.
- [x] Decide whether rosbridge remains sufficient or whether high-rate streams need a native binary bridge. Decision: continue with rosbridge until throughput benchmarks prove it insufficient; revisit a native binary bridge only with a maintained iOS client or companion ROS2 binary receiver.
- [x] Choose satellite imagery and DEM providers after checking cache, recording, attribution, and redistribution terms. Decision: bake NASA GIBS imagery, prefer USGS 3DEP for US DEM tiles, keep Mapzen Terrain Tiles as the global DEM fallback, and keep login/API-key sources user-configured.
