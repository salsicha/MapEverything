# MapEverything: ROS2 Robotics Mapping Payload

<p align="center">
  <img src="MapEverything/MapEverything/Assets.xcassets/MapEverythingLogo.imageset/MapEverythingLogo.png" alt="MapEverything logo" width="180">
</p>

![iOS](https://img.shields.io/badge/iOS-17.0+-blue.svg?style=for-the-badge&logo=apple)
![Swift](https://img.shields.io/badge/Swift-5.9-orange.svg?style=for-the-badge&logo=swift)
![ARKit](https://img.shields.io/badge/ARKit-LiDAR-black.svg?style=for-the-badge&logo=arkit)
![ROS2](https://img.shields.io/badge/ROS2-Humble%2FIron-green.svg?style=for-the-badge&logo=ros)

**MapEverything** is a robotics-first mapping payload for iOS. It turns a LiDAR-equipped iPhone or iPad Pro into a lightweight ROS2 sensor node that publishes device pose, low-rate camera frames, GPS, LiDAR point clouds, relative Depth Anything point clouds with overlay calibration, satellite imagery, and DEM/elevation tiles for recording on another ROS2 device.

The default record profile publishes camera frames at a conservative 2 Hz and publishes downsampled LiDAR clouds, relative Depth Anything clouds, and Depth Anything calibration instead of surfel maps so the iPhone can spend its budget on AR tracking, dense depth inference, GPS/geotile context, and reliable WebSocket publishing. Mesh, IMU, radio, and diagnostic topics remain implemented for opt-in debug or field profiles.

MapEverything is intentionally a single record-mode publisher by default. The iPhone starts and stops ROS2 publication; topic selection, rosbag retention, replay, and discard policy are handled by the external recorder. An off-by-default local SQLite bag option can mirror published topics into chunked rosbag2-style files on device for field fallback or later conversion.

The mapping architecture now uses one surface reconstruction path: ARKit pose tracking plus Depth Anything dense depth, with LiDAR kept as a separate point-cloud stream and calibration source. GPS, satellite imagery, and DEM context run alongside the same pipeline indoors or outdoors.

See [docs/robotics-mapping-concept.md](docs/robotics-mapping-concept.md) for the implementation concept and [TODO.md](TODO.md) for the phased task list.

---

## 🌟 Architecture & Core Features

```
                     MAPEVERYTHING
                           |
             +-------------+-------------+
             |                           |
     Mapping Session Manager      SwiftData persistence
             |                           |
             v                           v
 Depth Anything Surface Pipeline  MappingSessionModel
             |                    SensorStreamModel
             v                    GeoTileModel
 ARKit pose + LiDAR calibration + camera color
             |
             v
 GPS / ENU georeference + satellite imagery + DEM tiles
             |
             v
 rosbridge WebSocket publishers + external rosbag recorder
             |
             v
 optional local SQLite bag chunks
```

### 1. Surface Mapping Pipeline
* **Depth Anything Surface Mapping:** Projects the camera's YCbCr frames onto metric 3D depth coordinates derived from Depth Anything dense relative depth with LiDAR used for scale calibration, not per-point fusion. The live solid mesh overlay is built from the calibrated Depth Anything depth grid, while ROS receives a LiDAR point cloud, a relative Depth Anything point cloud, and the calibration used by the overlay mesh as separate topics.
* **Colored Surfel Reconstruction:** Fuses repeated RGB-D samples into a bounded voxel-hashed surfel map with weighted color, position, view-facing normals, radius, confidence, and observation counts. This provides a fuller colored surface reconstruction on device without the training cost of Gaussian splatting.
* **Single Record Mode:** Recording always follows the same surface pipeline so ROS topics, local bags, and replay metadata stay predictable across indoor and outdoor sessions.
* **Geospatial Context:** GPS, heading, ENU frame registration, satellite imagery, and DEM/elevation tiles run alongside the surface pipeline so outdoor datasets carry terrain and map context rather than only local AR geometry.

### 2. ROS and Local Bag Recording
MapEverything treats the external ROS2 recorder as the authoritative dataset sink. The iPhone publishes synchronized pose, camera, point cloud, GPS, geotile, DEM, radio, and diagnostics topics through rosbridge, while an optional on-device fallback mirrors those same rosbridge JSON payloads into chunked rosbag2-style SQLite files.

Local bag storage is off by default and controlled from the main recording surface. The **Save Local** control enables or disables on-device chunking, and **Share Local Bags** opens the recorded-session browser for deleting old sessions or sharing `metadata.yaml` and `.db3` chunks through the iOS share sheet.

### 3. Record-Mode Operator Experience
* **Single Mapping Surface:** The first screen is the live mapping view with start/stop, local bag save, local bag share, ROS bridge IP entry, and published-topic chips.
* **Stopped Map Inspection:** After stopping a scan, the reconstructed surface mesh remains centered on screen for touch-driven pan, tilt, and pinch inspection.
* **Field Recording Fallback:** The local bag controls mirror outgoing ROS publish payloads into chunked SQLite files on device when enabled, then expose those chunks through the share sheet.

---

## 🛠️ Step-by-Step Operator Guide

### Core App Setup
1. Download, build, and run the app from **Xcode 15+** on a LiDAR-equipped iOS device (e.g., iPhone 12 Pro or newer, iPad Pro 2020 or newer).
2. Authorize **Camera Permissions** on launch.
3. Enter the recorder workstation address in the **ROS bridge IP** field. The app builds a `ws://<host>:9090` rosbridge URL unless you enter a full WebSocket URL.
4. Toggle **ROS** on when publishing to a remote rosbridge recorder. Leave it off when using only local SQLite bag capture.
5. Tap **Save Local** before recording if you want on-device chunked SQLite bag files for field fallback.

---

### How to Record a Mapping Session
1. Wait for the startup overlay to show **Ready** or **Ready Without Depth Model**.
2. Tap **Start Mapping**. ARKit begins tracking, Depth Anything inference starts, and enabled ROS/local bag streams begin publishing.
3. Walk slowly and steadily.
   * *Move Slower Warnings:* If tracking is degraded by quick sweeps, a warning hum will trigger via the haptic motor and display a caution banner.
   * *Thermal State Throttling:* MapEverything monitors CPU temperature. If the device starts heating up, the frame-processing interval is automatically scaled down to avoid thermal crashes.
4. Watch the bottom topic chips for the advertised `/mapping/...` topics and the recorder panel for queue depth, dropped publish count, local buffer count, and local bag message count.
5. Tap **Stop Mapping**. Publication stops, AR capture pauses, and the current mesh remains available for on-device inspection with pan, tilt, and pinch gestures.
6. Tap **Share Local Bags** to list recorded bag sessions and share individual `metadata.yaml` or `.db3` chunks.

---

### Depth Anything Surface Mapping
MapEverything uses a single mapping engine for record mode. ARKit provides camera pose, Depth Anything provides dense relative depth for the live surface mesh, LiDAR calibrates that depth for visualization and remains available as its own sparse point-cloud topic, and GPS/geotile context is published when available.

---

## 🛰️ ROS 2 Bridge Integration Guide

MapEverything acts as a robust WebSocket-based edge sensor node. It connects directly to standard `rosbridge_suite` networks and, by default, streams pose, low-rate camera imagery with intrinsics, GPS, LiDAR point clouds, relative Depth Anything point clouds with calibration, satellite tile payloads, satellite tile georeferencing, and DEM raster tiles into your ROS2 workspace.

The current ROS topic namespace is `/mapping`.

Transport decision: MapEverything continues to use `rosbridge_suite` over WebSocket in this build. A native binary bridge is not enabled until there is a maintained iOS ROS2/DDS client or companion ROS2 binary receiver and a throughput benchmark showing rosbridge is insufficient.

The companion ROS2 custom message package lives in [ros2/mapeverything_msgs](ros2/mapeverything_msgs). Build it in your recorder workspace before launching rosbridge or recording bags. Full setup notes are in [docs/ros2-companion-package.md](docs/ros2-companion-package.md), validation procedures are in [docs/validation-plan.md](docs/validation-plan.md), and a starter RViz config is available at [ros2/rviz/mapeverything.rviz](ros2/rviz/mapeverything.rviz).

Local SQLite bag storage is controlled by the **Save Local** button and is off by default. When enabled, MapEverything mirrors outgoing rosbridge publish payloads into a `ROS2Bags/<session>/metadata.yaml` directory with size-rotated `.db3` chunks using the rosbag2 SQLite table layout. The **Share Local Bags** button opens a browser for listing recorded bag sessions, deleting old sessions, and sharing individual `metadata.yaml` or `.db3` files through the iOS share sheet. The browser caches per-session preview metadata and camera thumbnails in hidden sidecar files so repeated scans of saved bags stay quick. These local chunks use `serialization_format: rosbridge_json`; native ROS2 replay requires conversion to CDR messages or a compatible bridge-side importer.

Use [tools/mapeverything-local-bag-to-ros2.py](tools/mapeverything-local-bag-to-ros2.py) to convert shared local chunks into a native ROS2 bag. Run it from a sourced ROS2 workspace that can import `rosbag2_py`, `rclpy`, and `mapeverything_msgs`:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select mapeverything_msgs
source install/setup.bash
python3 tools/mapeverything-local-bag-to-ros2.py ROS2Bags/<session> --output converted/<session>_native
ros2 bag info converted/<session>_native
ros2 bag play converted/<session>_native
```

The converter also accepts individual `.db3` chunks, supports `--dry-run` inspection without ROS2 imports, and can use `--skip-unknown` or `--type /topic=pkg/msg/Type` when a message package is unavailable or a topic type needs overriding.

### Radio Telemetry Notes

See [docs/ios-radio-restrictions.md](docs/ios-radio-restrictions.md) for operator-facing iOS radio API constraints and external-adapter guidance.

Current Wi-Fi signal quality uses Apple's public `NEHotspotNetwork.fetchCurrent` API. It only reports the network the device is already associated with, requires Location permission, and requires the app target's `com.apple.developer.networking.wifi-info` entitlement. MapEverything now includes that entitlement file and publishes the entitlement, permission, last fetch, and normalized signal-strength state through session metadata and `/mapping/status`; broad Wi-Fi scans are not available through normal iOS public APIs.

BLE beacon telemetry uses Apple's public `CoreBluetooth.CBCentralManager` API and only scans after service UUIDs, peripheral UUIDs, or local-name prefixes are configured for the deployment. It reports Bluetooth permission state, scan state, configured filters, recent beacon RSSI values, and summarized advertisement metadata through session metadata and `/mapping/status`.

Network path diagnostics use `NWPathMonitor` to report reachability, active and available interface types, expensive/constrained state, IPv4/IPv6/DNS support, and unsatisfied reasons through session metadata and `/mapping/status`. This describes the active network path, not raw RF signal power.

Recorder endpoint probes use a bounded disposable rosbridge WebSocket connection to measure ping/pong round-trip latency and a short upload write-rate probe on `/mapping/probe/throughput`. Results are published through session metadata and `/mapping/status`; the probe measures application-path recorder health rather than sustained bidirectional network bandwidth.

The app publishes `mapeverything_msgs/RadioObservation` messages on `/mapping/radio` for fresh radio samples and includes the schema in `/mapping/session` metadata. It covers current Wi-Fi, BLE advertisements, Network.framework path state, recorder endpoint probes, and optional external adapters; unset numeric fields use `0.0` for rosbridge JSON compatibility, unset strings are empty, unset arrays are empty, and channel-specific details go in `metadata_json`.

iOS does not expose broad Wi-Fi access-point scan results or a dependable public cellular RSSI/RSRP/RSRQ/SINR stream to normal apps. MapEverything publishes these platform restrictions in `/mapping/session` as `radio_platform_restrictions`; use external adapters, network equipment APIs, SDRs, or companion ROS2 nodes for those survey channels.

### ROS2 WebSocket Topic Directory

| Topic Name | ROS 2 Message Type | Update Rate | Description |
| :--- | :--- | :--- | :--- |
| `/tf` | `tf2_msgs/msg/TFMessage` | opt-in | Live spatial coordinate frames mapping the relative transform from the mobile `iphone_camera` frame to the world origin `map` frame. |
| `/mapping/pose` | `geometry_msgs/msg/PoseStamped` | ~10 Hz | Standard 6-DOF SLAM position and orientation tracking. |
| `/mapping/odom` | `nav_msgs/msg/Odometry` | opt-in | Odometry-style pose for robotics consumers. |
| `/mapping/imu` | `sensor_msgs/msg/Imu` | opt-in | High-fidelity IMU data containing orientation quaternions, angular velocities, and linear accelerations (including gravity). |
| `/mapping/gps/fix` | `sensor_msgs/msg/NavSatFix` | ~1 Hz | Standard GPS fix, status, and covariance metadata. |
| `/mapping/gps/metadata` | `mapeverything_msgs/msg/GPSMetadata` | ~1 Hz | Extended Core Location validity, source, and georeference metadata. |
| `/mapping/pointcloud/lidar` | `sensor_msgs/msg/PointCloud2` | ~5 Hz | ARKit LiDAR-only colored point-cloud payloads downsampled to a sparse 10cm grid. |
| `/mapping/pointcloud/depth_anything` | `sensor_msgs/msg/PointCloud2` | ~5 Hz | Relative Depth Anything colored point-cloud payloads in `iphone_camera`, downsampled to a sparse grid. Coordinates are not metric. |
| `/mapping/depth_anything/calibration` | `mapeverything_msgs/msg/DepthAnythingCalibration` | ~5 Hz | Scale/offset calibration used by the live overlay mesh: `metric_depth_m = scale * relative_depth + offset`. |
| `/mapping/camera/image/compressed` | `sensor_msgs/msg/CompressedImage` | 2 Hz | JPEG-compressed native ARKit camera image stream for visual loop closure and recorder context. |
| `/mapping/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 2 Hz | Same-timestamp camera intrinsics for the compressed image stream. |
| `/mapping/map` | `visualization_msgs/msg/MarkerArray` | ~0.5 Hz | Emits reconstructed triangular mesh markers (`TRIANGLE_LIST`) for instant Rviz2 display when mesh publishing is enabled. |
| `/mapping/mesh_snapshot` | `mapeverything_msgs/msg/MeshSnapshot` | ~0.5 Hz | Structured triangle-list mesh snapshot for rosbag recording, with base64 packed little-endian vertex/index bytes plus truncation and payload-size metadata. |
| `/mapping/radio` | `mapeverything_msgs/msg/RadioObservation` | up to 2 Hz | Publishes fresh Wi-Fi, BLE beacon, network path, and recorder endpoint probe observations. |
| `/mapping/indoor_localization` | `mapeverything_msgs/msg/IndoorLocalization` | ~1 Hz | Indoor-aware Core Location sample with floor, heading, and registration quality metadata. |
| `/mapping/satellite/image/compressed` | `sensor_msgs/msg/CompressedImage` | 1/min | Compressed satellite imagery tile payload. |
| `/mapping/satellite/tile_info` | `mapeverything_msgs/msg/GeoTileInfo` | 1/min | Satellite imagery provider, bounds, CRS, attribution, source policy, and the device's pixel coordinate inside the tile. |
| `/mapping/dem/tile` | `mapeverything_msgs/msg/GeoRasterTile` | 1/min | DEM raster payload with bounds, CRS, attribution, source policy, and the device's pixel coordinate inside the raster tile. |
| `/mapping/session` | `mapeverything_msgs/msg/MappingSession` | on change | Session lifecycle, enabled streams, advertised topics, schemas, recorder configuration, and local bag status. |
| `/mapping/status` | `diagnostic_msgs/msg/DiagnosticArray` | 1 Hz | App, bridge, queue, radio, GPS, geotile, and recorder health diagnostics. |

The default advertised topic set is `/mapping/pose`, `/mapping/camera/image/compressed`, `/mapping/camera/camera_info`, `/mapping/pointcloud/lidar`, `/mapping/pointcloud/depth_anything`, `/mapping/depth_anything/calibration`, `/mapping/gps/fix`, `/mapping/gps/metadata`, `/mapping/satellite/image/compressed`, `/mapping/satellite/tile_info`, and `/mapping/dem/tile`. Optional odometry, TF, mesh, IMU, radio, session, and diagnostics streams can still be re-enabled in custom profiles.

For loop-closure consumers such as ArrayDataEngine, MapEverything publishes intrinsic values on `/mapping/camera/camera_info`: image `width`/`height`, focal lengths `fx`/`fy`, principal point `cx`/`cy`, full `K` and `P` matrices, identity rectification `R`, `plumb_bob` distortion model, and zero distortion coefficients because ARKit frames are treated as rectified pinhole images. Camera JPEG encoding is capped at 2 Hz and skips overlapping encodes to protect AR tracking and Depth Anything inference.

### Validation and Throughput Checks

Simulator-safe validation covers ROS2 topic metadata serialization, message schema JSON compatibility, GPS-to-ENU georeferencing, geotile cache indexing, and publish queue backpressure/retry behavior. Field validation for physical sensors, rosbag replay, thermal pressure, and poor-network sessions is documented in [docs/validation-plan.md](docs/validation-plan.md).

Rosbridge throughput can be sized or exercised with the checked-in benchmark harness:

```bash
python3 tools/rosbridge-throughput-benchmark.py --dry-run --duration 5
python3 tools/rosbridge-throughput-benchmark.py --url ws://<RECORDER_IP>:9090 --duration 60
```

To run rosbridge and record chunked rosbag2 SQLite files from one terminal, use:

```bash
python3 tools/run-rosbridge-recorder.py \
  --setup ~/mapeverything_ws/install/setup.bash \
  --output bags/mapeverything_field_test \
  --chunk-size-mb 512
```

The helper records the default advertised MapEverything topics, rotates `.db3`
bag chunks with `--max-bag-size`, and accepts `--include-optional`,
`--topic /extra/topic`, or `--record-all` for broader capture profiles.

### App Store Publishing

The App Store release plan is tracked in [docs/app-store-publishing-plan.md](docs/app-store-publishing-plan.md). Before a release branch or archive, run:

```bash
python3 tools/app-store-release-check.py
```

The checker verifies local release inputs such as `Info.plist` usage strings, portrait iPhone orientation, Wi-Fi entitlement presence, app icon assets, App Store export options, validation docs, and recorder tooling. It also flags App Store Connect tasks that must be completed in the account, including privacy labels, privacy policy URL, screenshots, TestFlight notes, and export-compliance answers.

---

### Step-by-Step Remote ROS2 Workstation Configuration

#### Step 1: Build the Custom Message Package

Build [ros2/mapeverything_msgs](ros2/mapeverything_msgs) in a colcon workspace before launching rosbridge:

```bash
mkdir -p ~/mapeverything_ws/src
cp -R ros2/mapeverything_msgs ~/mapeverything_ws/src/
cd ~/mapeverything_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select mapeverything_msgs
source install/setup.bash
```

#### Step 2: Install the WebSocket Bridge Server
On your remote Linux workstation or robot computer running **ROS2 (Humble or Iron)**, install the `rosbridge-suite`:

```bash
sudo apt-get update
sudo apt-get install ros-$ROS_DISTRO-rosbridge-suite
```

#### Step 3: Launch the WebSocket Server Node
Launch the rosbridge WebSocket node on your workstation. By default, it binds to port `9090`:

```bash
source ~/mapeverything_ws/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

You should see log output confirming that the websocket server is running:
`[websocket_node-1] registered class rosbridge_library.capabilities.advertise.Advertise`

#### Step 4: Connect the iOS Device
1. Find your workstation's local IP address (e.g., by running `hostname -I` or `ifconfig` in the terminal).
2. Connect both the iOS device and your workstation to the **same Wi-Fi network**.
3. Open **MapEverything** on your iPhone or iPad.
4. Enter your workstation address in the **ROS bridge IP** field. You can enter either a host such as `192.168.1.150` or a full URL such as `ws://192.168.1.150:9090`.
5. Toggle **ROS** on. The recorder status in the HUD reports the connection state and turns green once rosbridge is connected.

---

### Visualizing Telemetry in RViz2
On your workstation, launch `rviz2` in a new terminal window:

```bash
rviz2
```

Or load the checked-in starter configuration:

```bash
rviz2 -d ros2/rviz/mapeverything.rviz
```

Configure your RViz2 workspace using the settings below:

1. **Global Options:**
   - Set **Fixed Frame** to `map`.
2. **Add the Pose Displays:**
   - Add `/mapping/pose` to observe the live device trajectory in the `map` frame.
3. **Add the Point Clouds:**
   - Click **Add**, select **By Topic**, and choose `/mapping/pointcloud/lidar` -> **PointCloud2**.
   - Add `/mapping/pointcloud/depth_anything` -> **PointCloud2** as a second display.
   - Change the **Style** to `Points` and set the **Size** to `0.02m`.
   - Set **Color Transformer** to `RGB8` to render both colored point clouds in realistic, full color.
4. **Add Geospatial Context:**
   - Record `/mapping/gps/fix`, `/mapping/gps/metadata`, `/mapping/satellite/image/compressed`, `/mapping/satellite/tile_info`, and `/mapping/dem/tile`.
   - `GeoTileInfo` and `GeoRasterTile` expose `device_pixel_x`, `device_pixel_y`, `tile_width`, `tile_height`, `pixel_origin`, and `pixel_units` so the recorder can place the phone inside each downloaded tile.

---

## ⚙️ Technical System Architecture

### Surface Mapping Architecture
`MappingSessionManager` owns the record-mode lifecycle, enabled streams, recorder URL, bridge transport, and session metadata. SwiftData persists the active mapping schema through `MapEverythingModelSchema`, including `MappingSessionModel`, `SensorStreamModel`, and `GeoTileModel`.

The mapping stack has one primary capture path. ARKit world tracking provides the camera pose, Depth Anything generates dense relative depth, LiDAR supplies an independent sparse point cloud and scale calibration for the overlay mesh, and GPS/ENU registration adds geospatial context when available. `/mapping/session` and `/mapping/status` report the fixed mapping engine, enabled streams, recorder configuration, local bag state, geotile state, and diagnostics so remote recorders can explain what was captured.

### Outlier Filtration Pipeline
The point cloud goes through three distinct stages of cleanup before publishing or recording:
1. **Camera Space Projector:** Standard pinhole-camera unprojection translates depth pixel coordinates into a 3D coordinate vector in camera-relative coordinates:
   
   $$X_c = \frac{(x - c_x) \cdot \text{Depth}}{f_x}$$
   $$Y_c = -\frac{(y - c_y) \cdot \text{Depth}}{f_y}$$
   $$Z_c = -\text{Depth}$$
   
2. **Voxel Grid Downsampling:** Points are quantized into 3D grid indexes (voxels) defined by `voxelSize`. The first point registering in a voxel is cached, and duplicate points inside the same index are discarded. This maintains uniform density and manages memory usage.
3. **Radius Outlier Filter:** Points with a distance to the origin greater than the configured `boundingBoxSize` are discarded. The remaining points are checked against a local spatial grid. Any point with fewer than 3 neighboring voxels in its surrounding 27-voxel neighborhood is flagged as noise and filtered out.

### Coordinate Synchronization & Timestamps
ARKit spatial anchors, room shapes, and camera transforms operate on Apple's system uptime clock (time since system boot). To ensure compatibility with standard ROS2 time-variant nodes (such as `robot_localization` or `cartographer`), `ROS2BridgeClient.swift` calculates the offset between system boot time and the UNIX epoch to stamp outgoing ROS2 headers:

```swift
let systemUptime = ProcessInfo.processInfo.systemUptime
let nowUnix = Date().timeIntervalSince1970
let hardwareUnix = nowUnix - systemUptime + timestamp
```
This aligns iOS data streams perfectly with standard sensor times on your remote ROS2 robot workstation.

Surfels, geotiles, and any opt-in high-bandwidth publishers report original payload bytes,
encoded payload bytes, maximum observed payload size, last encoding/compression
mode, and compression ratio when diagnostics/session streams are enabled.

---

## 🤝 Contributing & License
Contributions, bug reports, and features are welcome! Feel free to open a pull request if you'd like to implement new mesh generation pipelines, improve Depth Anything calibration, or support CBOR/binary WebSockets. This project is licensed under the MIT License.
