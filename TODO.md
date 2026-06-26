# TODO

## P0 - Robotics Mapping Core

- [x] Create `MappingSessionManager` to own session lifecycle, active streams, remote recorder URL, and session metadata.
- [x] Add `ROS2TopicRegistry` for advertised topics, message types, publisher metadata, and publish rates.
- [x] Use maximum-likelihood LiDAR + Depth Anything fusion for mapping point clouds.
- [x] Add `PublishQueue` with backpressure, retry, drop policy, queue-depth metrics, and last-error reporting.
- [x] Add low-rate background satellite imagery and DEM tile fetch/publish during mapping sessions.
- [x] Publish `diagnostic_msgs/DiagnosticArray` on `/mapping/status`.
- [x] Publish session metadata on `/mapping/session`.
- [x] Access CoreLocation indoor localization services for indoor registration quality and global registration.
- [x] Publish `sensor_msgs/NavSatFix` on `/mapping/gps/fix`.
- [x] Add CoreLocation permission flow for precise location and heading.
- [x] Publish `nav_msgs/Odometry` on `/mapping/odom`.
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
- [x] Define `mapeverything_msgs/RadioObservation`.
- [x] Publish radio observations on `/mapping/radio`.
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
- [x] Define and publish `mapeverything_msgs/GeoTileInfo` for satellite georeferencing metadata.
- [x] Define and publish `mapeverything_msgs/GeoRasterTile` for DEM payloads.
- [x] Publish device latitude, longitude, altitude, and tile pixel coordinates with satellite and DEM tiles.
- [x] Add offline behavior for unavailable network or provider errors.

## P1 - Mesh, Point Cloud, and Camera Publishing

- [x] Add mesh publishing fallback through `visualization_msgs/MarkerArray`.
- [x] Define `mapeverything_msgs/MeshSnapshot` for structured mesh recording.
- [x] Add colored surfel fusion, PLY export, and ROS `PointCloud2` publication.
- [x] Add compression and payload-size metrics for camera and point-cloud streams.
- [x] Add stress tests for large mesh snapshots and long-running point-cloud sessions.

## P2 - Companion ROS2 Package

- [x] Create a `mapeverything_msgs` ROS2 package for custom message definitions.
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
- [x] Add local bag browser UI for listing, deleting, and sharing on-device bag files.
- [x] Add a local SQLite-to-native ROS2 bag conversion script.
- [x] Retire legacy `EnvironmentModel` saved-scan persistence after replacing local exports with session records and local ROS2 bag chunks.
- [x] Remove orphaned point-cloud, mesh, blueprint, video, preview, and legacy cleanup/export helpers from the record-mode app.

## P2 - Depth Anything Surface Mapping

- [x] Use a single Depth Anything surface mapping path for record mode.
- [x] Publish LiDAR and relative Depth Anything point clouds as separate topics.
- [x] Publish the Depth Anything calibration used by the live overlay mesh.
- [x] Remove the old indoor/outdoor mode router and semantic room capture path from the app.

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

- [x] Decide supported distribution channels: local developer builds, TestFlight, App Store, and GitHub release artifacts for ROS2 companion assets.
- [x] Add App Store publishing plan with release phases, account-side gates, TestFlight flow, metadata checklist, privacy/compliance checklist, and release blockers.
- [x] Add local App Store readiness checker for `Info.plist`, entitlements, assets, export options, docs, signing settings, and release-tool presence.
- [x] Add App Store Connect export options template for `xcodebuild -exportArchive` uploads.
- [x] Add a release build/archive checklist for LiDAR-only device testing, simulator-safe tests, physical-device validation, rosbag replay, and rosbridge throughput benchmarking.
- [x] Add signed iOS archive/export instructions, including `xcodebuild archive`, export options, version/build number handling, and TestFlight upload steps.
- [x] Add App Store Connect metadata checklist for description, screenshots, privacy nutrition labels, export compliance, support URL, and TestFlight beta review notes.
- [x] Add a privacy and compliance checklist for `Info.plist` usage strings, Privacy Manifest entries, data-retention language, radio/GPS disclaimers, and third-party geospatial attribution.
- [x] Define release artifact layout for app builds, release notes, validation reports, sample RViz config, rosbridge benchmark results, and source-policy/attribution records.
- [ ] Audit App Store Connect app record, bundle identifier ownership, signing team access, provisioning profiles, capabilities, and entitlement approval for camera, precise location, Bluetooth, local network, and Wi-Fi info access.
- [ ] Decide whether the current iOS deployment target intentionally limits App Store availability to the latest iOS devices.
- [ ] Create final App Store product metadata: app name, subtitle, category, description, keywords, support URL, privacy policy URL, screenshots, and review notes.
- [ ] Complete App Store privacy labels and export-compliance answers from the final data-flow audit.
- [ ] Run a TestFlight internal beta with a LiDAR device, ROS2 recorder workstation, `/mapping/...` rosbag capture, local SQLite bag sharing, and RViz replay.
- [ ] Run external TestFlight beta review with hardware/setup notes and collect operator feedback.
- [ ] Archive and upload the App Store candidate build using Xcode Organizer or `tools/app-store-export-options.plist`.
- [ ] Submit the first App Store version after privacy, compliance, screenshots, validation artifacts, and release blockers are clear.
- [ ] Package the ROS2 companion package as a versioned source archive with build instructions, supported ROS2 distros, message compatibility notes, and a colcon smoke-test command.
- [ ] Add GitHub release/tagging process with semantic versioning, changelog generation, migration notes, and rollback instructions for both iOS and ROS2 companion packages.
- [x] Document beta operator setup: required hardware, recorder workstation setup, network assumptions, permission prompts, credential configuration, and known platform restrictions.
- [x] Add a release-blocker checklist for missing provisioning, failed validation, missing attribution, credential leakage, non-replayable bags, queue instability, or unsupported sensor claims.

## Completed Foundation

- [x] Reposition the app as a robotics mapping payload rather than a general room-scanning/remodeling tool.
- [x] Update README product language around ROS2 mapping, field data capture, GPS, radio telemetry, satellite imagery, and DEM support.
- [x] Remove semantic room capture so the robotics workflow stays focused on Depth Anything surfaces, LiDAR, GPS, DEM, satellite imagery, and ROS2 recording.
- [x] Define MapEverything as a single record-mode ROS2 publisher with external recorder-side data retention by default and optional local SQLite bag fallback.
- [x] Keep rosbridge WebSocket as the production bridge for now; no native ROS2/DDS iOS client or recorder-side binary receiver is integrated in this build.
- [x] Remove unused measurement, remodeling, landscaping, saved-scan gallery, and legacy export trigger code from the single record-mode app.

## Open Product Decisions

- [x] Decide whether to rename the app before larger UI changes. Product name: MapEverything.
- [x] Decide whether stream-level UI toggles belong in the app or only in recorder-side configuration. Decision: keep the app to start/stop record mode; choose topic subscriptions and rosbag retention on the recorder side.
- [x] Decide whether rosbridge remains sufficient or whether high-rate streams need a native binary bridge. Decision: continue with rosbridge until throughput benchmarks prove it insufficient; revisit a native binary bridge only with a maintained iOS client or companion ROS2 binary receiver.
- [x] Choose satellite imagery and DEM providers after checking cache, recording, attribution, and redistribution terms. Decision: bake NASA GIBS imagery, prefer USGS 3DEP for US DEM tiles, keep Mapzen Terrain Tiles as the global DEM fallback, and keep login/API-key sources user-configured.
