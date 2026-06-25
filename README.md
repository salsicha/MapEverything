# MapEverything: ROS2 Robotics Mapping Payload

![iOS](https://img.shields.io/badge/iOS-17.0+-blue.svg?style=for-the-badge&logo=apple)
![Swift](https://img.shields.io/badge/Swift-5.9-orange.svg?style=for-the-badge&logo=swift)
![ARKit](https://img.shields.io/badge/ARKit-LiDAR-black.svg?style=for-the-badge&logo=arkit)
![ROS2](https://img.shields.io/badge/ROS2-Humble%2FIron-green.svg?style=for-the-badge&logo=ros)

**MapEverything** is a robotics-first mapping payload for iOS. It turns a LiDAR-equipped iPhone or iPad Pro into a lightweight ROS2 sensor node that publishes device pose, GPS, Depth Anything fused point clouds, satellite imagery, and DEM/elevation tiles for recording on another ROS2 device.

The default record profile avoids high-bandwidth raw camera topics and publishes a downsampled Depth Anything fused point cloud instead of surfel maps so the iPhone can spend its budget on AR tracking, dense depth inference, GPS/geotile context, and reliable WebSocket publishing. Camera, mesh, IMU, radio, and diagnostic topics remain implemented for opt-in debug or field profiles.

MapEverything is intentionally a single record-mode publisher by default. The iPhone starts and stops ROS2 publication; topic selection, rosbag retention, replay, and discard policy are handled by the external recorder. An off-by-default local SQLite bag option can mirror published topics into chunked rosbag2-style files on device for field fallback or later conversion.

The mapping architecture separates capture engines from mode selection. An adaptive policy prefers RoomPlan for enclosed interiors and switches to outdoor LiDAR + Depth Anything mapping with GPS, satellite, and DEM context when room semantics are not reliable.

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
  Adaptive Mapping Policy         MappingSessionModel
                                  SensorStreamModel
             |                    GeoTileModel
    +--------+--------+
    |                 |
    v                 v
RoomPlan        LiDAR + Depth Anything
indoor          outdoor / open-area
parametrics     dense geometry
    |                 |
    +--------+--------+
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

### 1. Mapping Engines and Mode Routing
* **LiDAR + Depth Anything Mapping:** Projects the camera's YCbCr frames onto metric 3D depth coordinates derived from ARKit LiDAR and Depth Anything dense relative depth. Points are buffered in a custom `PointCloudManager` actor off the main thread, downsampled using **Voxel Grid filters**, and cleared of noise using a spatial **Radius Outlier Removal (ROR)** algorithm.
* **Colored Surfel Reconstruction:** Fuses repeated RGB-D samples into a bounded voxel-hashed surfel map with weighted color, position, view-facing normals, radius, confidence, and observation counts. This provides a fuller colored surface reconstruction on device without the training cost of Gaussian splatting.
* **Parametric RoomPlan Mapping:** Uses Apple's local Core ML scene understanding to isolate indoor surfaces (walls, windows, doors) and objects (tables, sofas, chairs), then publishes clean parametric bounding boxes alongside regular pose and sensor streams.
* **Adaptive Mapping Policy:** Scores RoomPlan suitability, outdoor GPS context, LiDAR confidence, Depth Anything availability, thermal pressure, and operator override state. It prefers RoomPlan inside enclosed rooms and switches to LiDAR + Depth Anything outdoors or in spaces where room semantics are weak.
* **Geospatial Context:** GPS, heading, ENU frame registration, satellite imagery, and DEM/elevation tiles run alongside either mapping engine so outdoor datasets carry terrain and map context rather than only local AR geometry.

### 2. Multi-Format Serialization & Export
A high-performance pipeline serializes inspection/export artifacts to the iOS local file system and syncs metadata over **SwiftData**. The authoritative robotics recording remains the external rosbag recorder unless local SQLite bag storage is explicitly enabled; local files support review, sharing, cache-backed publication, and field fallback. Exporting produces a standard iOS Share Sheet with files including:
* **`.ply` (Binary PLY):** A high-density point cloud or surfel file encoding `x, y, z` floating coordinates and `red, green, blue` color attributes. Surfel PLY exports also include normals, radius, confidence, and observation counts for surface-oriented rendering or offline meshing.
* **`.obj` (Wavefront 3D):** A standardized, polygonal mesh representing the tracked surfaces, textured with standard gray materials ready for importing into Blender, CAD, or Unity.
* **`.usdz` (Universal Scene Description):** Apple’s native AR file format. Meshes are baked with physically plausible PBR materials for realistic lighting in iOS QuickLook or iMessage.
* **`.pdf` (Dimensioned Blueprint):** A top-down 2D vector graphic blueprint featuring thick dark walls, red offset dimension ticks, measurement labels (in metric or imperial), and a graphic scale bar.
* **`.mp4` (Cinematic Flythrough):** A 10-second, high-definition 1080x1080 3D cinematic video orbit rendered on the GPU via SceneKit, ideal for instant progress reports.

### 3. Spatial Interaction & Planning Overlay
* **Linear & Closed-Polygon Measure Tool:** Drop interactive nodes directly on real-world planes. Calculated linear paths automatically update. Placing 3+ nodes triggers a mathematical polygon cross-product operation, yielding a closed surface area (square feet or square meters) in real-time.
* **Remodeling Mode:** Drop 3D volumetric boxes into the scene representing structural limits, appliances, or storage to map room volume restrictions.
* **Landscaping Mode:** Spawn volumetric cones and custom green foliage markers to lay out site bounds, plant coordinates, or site grading limits.

---

## 🛠️ Step-by-Step Operator Guide

### Core App Setup
1. Download, build, and run the app from **Xcode 15+** on a LiDAR-equipped iOS device (e.g., iPhone 12 Pro or newer, iPad Pro 2020 or newer).
2. Authorize **Camera Permissions** on launch. 
3. Tap the **Gear Icon** in the top right corner to customize settings:
   - **Voxel Size:** Adjust the spacing of the grid (e.g., `0.05m` for detailed indoor scans, `0.1m` for outdoor terrain).
   - **Max Point Limit:** Limit points (up to 2M) to avoid high memory spikes.
   - **Units:** Toggle between Metric (meters/sq m) and Imperial (feet/sq ft) values.
   - **Scan Mode:** Manually switch between LiDAR + Depth Anything point-cloud capture and ML-driven **RoomPlan** extraction. Adaptive indoor/outdoor switching is tracked in [TODO.md](TODO.md).

---

### How to Scan in Point Cloud & Mesh Mode
1. Set the scan selector to **LiDAR Mode**.
2. Point the camera at a surface and tap the central red **Record Icon** to initiate depth collection.
3. Walk slowly and steadily. 
   * *Move Slower Warnings:* If tracking is degraded by quick sweeps, a warning hum will trigger via the haptic motor and display a caution banner.
   * *Thermal State Throttling:* MapEverything monitors CPU temperature. If the device starts heating up, the frame-processing interval is automatically scaled from 10Hz down to 2Hz to prevent thermal crashes.
4. Toggle between the **Visualization modes** in real-time using the segmented controls:
   * `Point Cloud`: Displays live feature points.
   * `Wireframe`: RealityKit’s overlay showing active surface triangulation.
   * `Solid Mesh`: Renders a solid color bounding surface mapping the reconstructed terrain.
5. Tap the green **Checkmark Icon** to stop scanning. Enter a scan name and hit **Save**. A background task will serialize files to your local folder/iCloud.

---

### How to Use the Spatial Planning & Measurement Tools
Ensure scanning is paused or running in Point Cloud mode. Use the segmented mode bar to choose a mode:

```
┌────────────────────────────────────────────────────────────────────────┐
│ [ Scan Mode ]   [ Measure Mode ]   [ Remodel Mode ]   [ Landscape ]     │
└────────────────────────────────────────────────────────────────────────┘
```

#### 📏 Measuring Distances & Area
1. Tap **Measure**.
2. Tap anywhere on a tracked surface or plane in the camera viewport. A red spheres marker will drop.
3. Tap a second location. A yellow cylinder line will connect the two, and the HUD will display the linear distance (e.g., `Distance: 3.28 ft` / `1.00 m`).
4. Tap a third location. A closed triangular overlay is drawn. The HUD will immediately compute and display the closed surface area (e.g., `Area: 10.76 sq ft` / `1.00 sq m`).
5. Continue tapping to add edges to your custom room footprint.
6. **To delete nodes:** Long-press any dropped red marker sphere. The selected node and its relative lines will be removed.

#### 🔨 Remodeling & Landscaping Visualization
1. Tap **Remodel** or **Landscape**.
2. Tap the floor or any horizontal plane. 
   - In **Remodel Mode**, a volumetric, semi-transparent blue planning cube (0.5m dimension) spawns.
   - In **Landscape Mode**, an upright green planning cone (1.5m high) spawns to lay out foliage or grading pillars.
3. Tap and drag spawned shapes to position them.
4. Long-press on a placed shape to remove it from your AR scene.

---

### RoomPlan Modeling & Top-Down Blueprint Generation
1. Go to **Settings** and enable **Use RoomPlan Mode**.
2. Tap the red **Record Icon**. The interface will change to Apple's standardized, immersive RoomPlan UI.
3. Scan the room, aiming the camera at corners, floors, walls, and doorways. Bounding box meshes of chairs, tables, doors, and arches will float into view.
4. Tap the **Checkmark Icon** to save. This creates a parametric room model.
5. Go to the **Gallery View (Photos Icon)**:
   - Tap a scan card to select it.
   - Tap the **Floorplan Button (Ruler Icon)**. The app parses the 3D USDZ geometry, extracts all vertical walls, doors, and windows, and generates a vectorized PDF.
   - Tap the **Preview Button (Eye Icon)** to open the models in AR QuickLook. You can inspect, measure, or place the virtual room in your current physical space.

---

## 🛰️ ROS 2 Bridge Integration Guide

MapEverything acts as a robust WebSocket-based edge sensor node. It connects directly to standard `rosbridge_suite` networks and, by default, streams pose, GPS, Depth Anything fused point clouds, satellite tile payloads, satellite tile georeferencing, and DEM raster tiles into your ROS2 workspace.

The current ROS topic namespace remains `/reconstructor` for compatibility with existing recorder setups.

Transport decision: MapEverything continues to use `rosbridge_suite` over WebSocket in this build. A native binary bridge is not enabled until there is a maintained iOS ROS2/DDS client or companion ROS2 binary receiver and a throughput benchmark showing rosbridge is insufficient.

The companion ROS2 custom message package lives in [ros2/reconstructor_msgs](ros2/reconstructor_msgs). Build it in your recorder workspace before launching rosbridge or recording bags. Full setup notes are in [docs/ros2-companion-package.md](docs/ros2-companion-package.md), validation procedures are in [docs/validation-plan.md](docs/validation-plan.md), and a starter RViz config is available at [ros2/rviz/mapeverything.rviz](ros2/rviz/mapeverything.rviz).

Local SQLite bag storage is available in Settings and is off by default. When enabled, MapEverything mirrors outgoing rosbridge publish payloads into a `ROS2Bags/<session>/metadata.yaml` directory with size-rotated `.db3` chunks using the rosbag2 SQLite table layout. The Settings screen includes a local bag browser for listing recorded bag sessions, deleting old sessions, and sharing individual `metadata.yaml` or `.db3` files through the iOS share sheet. These local chunks use `serialization_format: rosbridge_json`; native ROS2 replay requires conversion to CDR messages or a compatible bridge-side importer.

Use [tools/mapeverything-local-bag-to-ros2.py](tools/mapeverything-local-bag-to-ros2.py) to convert shared local chunks into a native ROS2 bag. Run it from a sourced ROS2 workspace that can import `rosbag2_py`, `rclpy`, and `reconstructor_msgs`:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select reconstructor_msgs
source install/setup.bash
python3 tools/mapeverything-local-bag-to-ros2.py ROS2Bags/<session> --output converted/<session>_native
ros2 bag info converted/<session>_native
ros2 bag play converted/<session>_native
```

The converter also accepts individual `.db3` chunks, supports `--dry-run` inspection without ROS2 imports, and can use `--skip-unknown` or `--type /topic=pkg/msg/Type` when a message package is unavailable or a topic type needs overriding.

### Radio Telemetry Notes

See [docs/ios-radio-restrictions.md](docs/ios-radio-restrictions.md) for operator-facing iOS radio API constraints and external-adapter guidance.

Current Wi-Fi signal quality uses Apple's public `NEHotspotNetwork.fetchCurrent` API. It only reports the network the device is already associated with, requires Location permission, and requires the app target's `com.apple.developer.networking.wifi-info` entitlement. MapEverything now includes that entitlement file and publishes the entitlement, permission, last fetch, and normalized signal-strength state through session metadata and `/reconstructor/status`; broad Wi-Fi scans are not available through normal iOS public APIs.

BLE beacon telemetry uses Apple's public `CoreBluetooth.CBCentralManager` API and only scans after service UUIDs, peripheral UUIDs, or local-name prefixes are configured in Settings. It reports Bluetooth permission state, scan state, configured filters, recent beacon RSSI values, and summarized advertisement metadata through session metadata and `/reconstructor/status`.

Network path diagnostics use `NWPathMonitor` to report reachability, active and available interface types, expensive/constrained state, IPv4/IPv6/DNS support, and unsatisfied reasons through session metadata and `/reconstructor/status`. This describes the active network path, not raw RF signal power.

Recorder endpoint probes use a bounded disposable rosbridge WebSocket connection to measure ping/pong round-trip latency and a short upload write-rate probe on `/reconstructor/probe/throughput`. Results are published through session metadata and `/reconstructor/status`; the probe measures application-path recorder health rather than sustained bidirectional network bandwidth.

The app publishes `reconstructor_msgs/RadioObservation` messages on `/reconstructor/radio` for fresh radio samples and includes the schema in `/reconstructor/session` metadata. It covers current Wi-Fi, BLE advertisements, Network.framework path state, recorder endpoint probes, and optional external adapters; unset numeric fields use `0.0` for rosbridge JSON compatibility, unset strings are empty, unset arrays are empty, and channel-specific details go in `metadata_json`.

iOS does not expose broad Wi-Fi access-point scan results or a dependable public cellular RSSI/RSRP/RSRQ/SINR stream to normal apps. MapEverything publishes these platform restrictions in `/reconstructor/session` as `radio_platform_restrictions`; use external adapters, network equipment APIs, SDRs, or companion ROS2 nodes for those survey channels.

### ROS2 WebSocket Topic Directory

| Topic Name | ROS 2 Message Type | Update Rate | Description |
| :--- | :--- | :--- | :--- |
| `/tf` | `tf2_msgs/msg/TFMessage` | opt-in | Live spatial coordinate frames mapping the relative transform from the mobile `iphone_camera` frame to the world origin `map` frame. |
| `/reconstructor/pose` | `geometry_msgs/msg/PoseStamped` | ~10 Hz | Standard 6-DOF SLAM position and orientation tracking. |
| `/reconstructor/odom` | `nav_msgs/msg/Odometry` | opt-in | Odometry-style pose for robotics consumers. |
| `/reconstructor/imu` | `sensor_msgs/msg/Imu` | opt-in | High-fidelity IMU data containing orientation quaternions, angular velocities, and linear accelerations (including gravity). |
| `/reconstructor/gps/fix` | `sensor_msgs/msg/NavSatFix` | ~1 Hz | Standard GPS fix, status, and covariance metadata. |
| `/reconstructor/gps/metadata` | `reconstructor_msgs/msg/GPSMetadata` | ~1 Hz | Extended Core Location validity, source, and georeference metadata. |
| `/reconstructor/pointcloud` | `sensor_msgs/msg/PointCloud2` | ~2 Hz | Depth Anything + LiDAR fused point-cloud payloads downsampled to a sparse 10cm grid. |
| `/reconstructor/camera/image/compressed` | `sensor_msgs/msg/CompressedImage` | opt-in | JPEG-compressed native ARKit camera image stream for visual loop closure and recorder context. Disabled by default. |
| `/reconstructor/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | opt-in | Same-timestamp camera intrinsics for the compressed image stream. Disabled by default with the camera image stream. |
| `/reconstructor/map` | `visualization_msgs/msg/MarkerArray` | ~0.5 Hz | Emits active reconstructed LiDAR triangular meshes (`TRIANGLE_LIST`) and parametric RoomPlan bounding boxes (`CUBE`) for instant Rviz2 display. |
| `/reconstructor/mesh_snapshot` | `reconstructor_msgs/msg/MeshSnapshot` | ~0.5 Hz | Structured triangle-list mesh snapshot for rosbag recording, with truncation and payload-size metadata. |
| `/reconstructor/radio` | `reconstructor_msgs/msg/RadioObservation` | up to 2 Hz | Publishes fresh Wi-Fi, BLE beacon, network path, and recorder endpoint probe observations. |
| `/reconstructor/indoor_localization` | `reconstructor_msgs/msg/IndoorLocalization` | ~1 Hz | Indoor-aware Core Location sample with floor, heading, and registration quality metadata. |
| `/reconstructor/satellite/image/compressed` | `sensor_msgs/msg/CompressedImage` | on fetch | Compressed satellite imagery tile payload. |
| `/reconstructor/satellite/tile_info` | `reconstructor_msgs/msg/GeoTileInfo` | on fetch | Satellite imagery provider, bounds, CRS, attribution, source policy, and the device's pixel coordinate inside the tile. |
| `/reconstructor/dem/tile` | `reconstructor_msgs/msg/GeoRasterTile` | on fetch | DEM raster payload with bounds, CRS, attribution, source policy, and the device's pixel coordinate inside the raster tile. |
| `/reconstructor/session` | `reconstructor_msgs/msg/MappingSession` | on change | Session lifecycle, enabled streams, advertised topics, schemas, recorder configuration, and local bag status. |
| `/reconstructor/status` | `diagnostic_msgs/msg/DiagnosticArray` | 1 Hz | App, bridge, queue, radio, GPS, geotile, and recorder health diagnostics. |

The default advertised topic set is `/reconstructor/pose`, `/reconstructor/pointcloud`, `/reconstructor/gps/fix`, `/reconstructor/gps/metadata`, `/reconstructor/satellite/image/compressed`, `/reconstructor/satellite/tile_info`, and `/reconstructor/dem/tile`. Optional odometry, TF, camera, mesh, IMU, radio, session, and diagnostics streams can still be re-enabled in custom profiles.

If the optional camera stream is enabled for loop-closure consumers such as ArrayDataEngine, MapEverything publishes intrinsic values on `/reconstructor/camera/camera_info`: image `width`/`height`, focal lengths `fx`/`fy`, principal point `cx`/`cy`, full `K` and `P` matrices, identity rectification `R`, `plumb_bob` distortion model, and zero distortion coefficients because ARKit frames are treated as rectified pinhole images. The lightweight default profile relies on pose, GPS, Depth Anything point clouds, and geotile context instead of raw camera frames.

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

---

### Step-by-Step Remote ROS2 Workstation Configuration

#### Step 1: Build the Custom Message Package

Build [ros2/reconstructor_msgs](ros2/reconstructor_msgs) in a colcon workspace before launching rosbridge:

```bash
mkdir -p ~/mapeverything_ws/src
cp -R ros2/reconstructor_msgs ~/mapeverything_ws/src/
cd ~/mapeverything_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select reconstructor_msgs
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
4. Tap the **Gear Icon (Settings)**:
   - Toggle **Enable ROS2 Bridge** on.
   - Enter your workstation’s WebSocket URL: `ws://<YOUR_WORKSTATION_IP>:9090` (e.g., `ws://192.168.1.150:9090`).
5. Close Settings. The ROS2 status icon (antenna wave) in the top-left HUD will glow **green** when a connection is established.

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
   - Add `/reconstructor/pose` to observe the live device trajectory in the `map` frame.
3. **Add the Depth Anything Point Cloud:**
   - Click **Add**, select **By Topic**, and choose `/reconstructor/pointcloud` -> **PointCloud2**.
   - Change the **Style** to `Points` and set the **Size** to `0.02m`.
   - Set **Color Transformer** to `RGB8` to render the fused point cloud in realistic, full color.
4. **Add Geospatial Context:**
   - Record `/reconstructor/gps/fix`, `/reconstructor/gps/metadata`, `/reconstructor/satellite/image/compressed`, `/reconstructor/satellite/tile_info`, and `/reconstructor/dem/tile`.
   - `GeoTileInfo` and `GeoRasterTile` expose `device_pixel_x`, `device_pixel_y`, `tile_width`, `tile_height`, `pixel_origin`, and `pixel_units` so the recorder can place the phone inside each downloaded tile.

---

## ⚙️ Technical System Architecture

### Mapping Mode Architecture
`MappingSessionManager` owns the record-mode lifecycle, enabled streams, recorder URL, bridge transport, and session metadata. SwiftData persists the expanded mapping schema through `MapEverythingModelSchema`, including legacy `EnvironmentModel` records plus `MappingSessionModel`, `SensorStreamModel`, and `GeoTileModel`.

The adaptive mapping router belongs above the capture engines. It observes RoomPlan availability, AR tracking state, LiDAR depth confidence, Depth Anything health, GPS quality, Core Location indoor metadata, terrain tile coverage, thermal pressure, and operator override state. The router then selects one of two primary capture paths:

* **Indoor parametric path:** RoomPlan can publish semantic walls, openings, and object bounding boxes through optional mesh topics while the default recorder profile keeps pose, GPS, Depth Anything point clouds, satellite imagery, and DEM context active.
* **Outdoor/open-area path:** ARKit LiDAR, Depth Anything fusion, GPS/ENU registration, satellite imagery, and DEM tiles publish dense point-cloud geometry and geospatial context without depending on RoomPlan's indoor assumptions.

The selected mode, confidence, reason codes, and any operator override are reflected in `/reconstructor/session` and `/reconstructor/status` so the remote recorder can explain what it captured.

### Outlier Filtration Pipeline
The point cloud goes through three distinct stages of cleanup before saving:
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
Contributions, bug reports, and features are welcome! Feel free to open a pull request if you'd like to implement new mesh generation pipelines, support CBOR/binary WebSockets, or expand RoomPlan geometry parsing. This project is licensed under the MIT License.
