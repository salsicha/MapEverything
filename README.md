# MapEverything: ROS2 Robotics Mapping Payload

![iOS](https://img.shields.io/badge/iOS-17.0+-blue.svg?style=for-the-badge&logo=apple)
![Swift](https://img.shields.io/badge/Swift-5.9-orange.svg?style=for-the-badge&logo=swift)
![ARKit](https://img.shields.io/badge/ARKit-LiDAR-black.svg?style=for-the-badge&logo=arkit)
![ROS2](https://img.shields.io/badge/ROS2-Humble%2FIron-green.svg?style=for-the-badge&logo=ros)

**MapEverything** is a robotics-first mapping payload for iOS. It is designed to turn a LiDAR-equipped iPhone or iPad Pro into a mobile ROS2 sensor node that captures device pose, IMU, camera context, LiDAR point clouds, reconstructed mesh, GPS, radio signal observations, satellite imagery, DEM/elevation tiles, and diagnostics for recording on another ROS2 device.

The current app already streams raw color feeds, dense point clouds, 6-DOF pose frames (`/tf`), IMU data, and parametric RoomPlan cubes to a remote robotics workstation or simulator over a WebSocket ROS2 bridge. The roadmap now prioritizes robust field mapping, geospatial context, radio telemetry, and recorder-side rosbag workflows over consumer remodeling features.

## Robotics Mapping Pivot

The next product direction is to focus MapEverything fully on robotics mapping. The app should become an iPhone-based ROS2 sensor payload that records pose, IMU, GPS, LiDAR point clouds, reconstructed mesh, radio signal observations, satellite imagery, DEM/elevation tiles, and diagnostics for recording on another ROS2 device.

MapEverything is intentionally a single record-mode publisher. The iPhone starts and stops ROS2 publication; topic selection, rosbag retention, replay, and discard policy are handled by the external recorder. The app should not create local session recordings beyond transient buffers and caches needed to publish reliably.

See [docs/robotics-mapping-concept.md](docs/robotics-mapping-concept.md) for the implementation concept and [TODO.md](TODO.md) for the phased task list.

---

## 🌟 Architecture & Core Features

```
                   ┌────────────────────────────────────────────────────────┐
                   │                    MAPEVERYTHING                       │
                   └──────────────────────────┬─────────────────────────────┘
                                              │
                      ┌───────────────────────┴───────────────────────┐
                      ▼                                               ▼
          ┌───────────────────────┐                       ┌───────────────────────┐
          │   High-Density LiDAR  │                       │  Parametric RoomPlan  │
          │     (Raw Point Cloud) │                       │  (Machine Learning)   │
          └───────────┬───────────┘                       └───────────┬───────────┘
                      │                                               │
  ┌───────────────────┴───────────────────┐               ┌───────────┴───────────┐
  ▼                                       ▼               ▼                       ▼
Voxel Grid                            Radius Outlier    2D Blueprints         USDZ Bounding
Filtering                             Removal (ROR)     (Vector PDF)             Boxes
```

### 1. Dual Scanning Engines
* **High-Density LiDAR Mode:** Projects the camera's YCbCr frames directly onto spatial 3D depth coordinates derived from hardware `smoothedSceneDepth`. Points are buffered in a custom `PointCloudManager` actor off the main thread, downsampled using **Voxel Grid filters**, and cleared of noise using a high-efficiency spatial **Radius Outlier Removal (ROR)** algorithm.
* **Parametric RoomPlan Mode:** Harnesses Apple's local coreML-based scene understanding to automatically isolate surfaces (walls, windows, doors) and objects (tables, sofas, chairs) in real-time, mapping them to bounding boxes natively.

### 2. Multi-Format Serialization & Export
A high-performance pipeline serializes captures to the iOS local file system and syncs metadata over **SwiftData**. Exporting produces a standard iOS Share Sheet with files including:
* **`.ply` (Binary PLY):** A high-density point cloud file encoding `x, y, z` floating coordinates and `red, green, blue` color attributes per-point.
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
   - **Scan Mode:** Switch between Point Cloud LiDAR tracking and ML-driven **RoomPlan** extraction.

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

MapEverything acts as a robust WebSocket-based edge sensor node. It connects directly to standard `rosbridge_suite` networks, allowing you to stream camera feeds, high-density point clouds, IMU arrays, and RoomPlan boundaries directly into your ROS2 workspace.

The current ROS topic namespace remains `/reconstructor` for compatibility with existing recorder setups.

Transport decision: MapEverything continues to use `rosbridge_suite` over WebSocket in this build. A native binary bridge is not enabled until there is a maintained iOS ROS2/DDS client or companion ROS2 binary receiver and a throughput benchmark showing rosbridge is insufficient. The app publishes its active bridge profile on `/reconstructor/session` and `/reconstructor/status`.

### Radio Telemetry Notes

Current Wi-Fi signal quality uses Apple's public `NEHotspotNetwork.fetchCurrent` API. It only reports the network the device is already associated with, requires Location permission, and requires the app target's `com.apple.developer.networking.wifi-info` entitlement. MapEverything now includes that entitlement file and publishes the entitlement, permission, last fetch, and normalized signal-strength state through session metadata and `/reconstructor/status`; broad Wi-Fi scans are not available through normal iOS public APIs.

BLE beacon telemetry uses Apple's public `CoreBluetooth.CBCentralManager` API and only scans after service UUIDs, peripheral UUIDs, or local-name prefixes are configured in Settings. It reports Bluetooth permission state, scan state, configured filters, recent beacon RSSI values, and summarized advertisement metadata through session metadata and `/reconstructor/status`.

Network path diagnostics use `NWPathMonitor` to report reachability, active and available interface types, expensive/constrained state, IPv4/IPv6/DNS support, and unsatisfied reasons through session metadata and `/reconstructor/status`. This describes the active network path, not raw RF signal power.

Recorder endpoint probes use a bounded disposable rosbridge WebSocket connection to measure ping/pong round-trip latency and a short upload write-rate probe on `/reconstructor/probe/throughput`. Results are published through session metadata and `/reconstructor/status`; the probe measures application-path recorder health rather than sustained bidirectional network bandwidth.

### ROS2 WebSocket Topic Directory

| Topic Name | ROS 2 Message Type | Update Rate | Description |
| :--- | :--- | :--- | :--- |
| `/tf` | `tf2_msgs/msg/TFMessage` | ~10 Hz | Live spatial coordinate frames mapping the relative transform from the mobile `iphone_camera` frame to the world origin `map` frame. |
| `/reconstructor/pose` | `geometry_msgs/msg/PoseStamped` | ~10 Hz | Standard 6-DOF SLAM position and orientation tracking. |
| `/reconstructor/imu` | `sensor_msgs/msg/Imu` | 100 Hz | High-fidelity IMU data containing orientation quaternions, angular velocities, and linear accelerations (including gravity). |
| `/reconstructor/pointcloud` | `sensor_msgs/msg/PointCloud2` | ~10 Hz | Point cloud payloads downsampled to a sparse 10cm grid. Color values are packed into a single 32-bit integer (`rgb`). |
| `/reconstructor/camera/image/compressed` | `sensor_msgs/msg/CompressedImage` | ~10 Hz | JPEG-compressed image stream rotated to match current mobile screen orientation. |
| `/reconstructor/map` | `visualization_msgs/msg/MarkerArray` | ~0.5 Hz | Emits active reconstructed LiDAR triangular meshes (`TRIANGLE_LIST`) and parametric RoomPlan bounding boxes (`CUBE`) for instant Rviz2 display. |

---

### Step-by-Step Remote ROS2 Workstation Configuration

#### Step 1: Install the WebSocket Bridge Server
On your remote Linux workstation or robot computer running **ROS2 (Humble or Iron)**, install the `rosbridge-suite`:

```bash
sudo apt-get update
sudo apt-get install ros-$ROS_DISTRO-rosbridge-suite
```

#### Step 2: Launch the WebSocket Server Node
Launch the rosbridge WebSocket node on your workstation. By default, it binds to port `9090`:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

You should see log output confirming that the websocket server is running:
`[websocket_node-1] registered class rosbridge_library.capabilities.advertise.Advertise`

#### Step 3: Connect the iOS Device
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

Configure your RViz2 workspace using the settings below:

1. **Global Options:**
   - Set **Fixed Frame** to `map`.
2. **Add a TF Display:**
   - Add the **TF** display to observe the dynamic relationship between the static `map` frame and the mobile `iphone_camera` frame as you move.
3. **Add the Camera Stream:**
   - Click **Add**, select **By Topic**, and choose `/reconstructor/camera/image/compressed` -> **CompressedImage**. 
   - Set the image transport parameter to `compressed`.
4. **Add the Point Cloud:**
   - Click **Add**, select **By Topic**, and choose `/reconstructor/pointcloud` -> **PointCloud2**.
   - Change the **Style** to `Points` and set the **Size** to `0.02m`.
   - Set **Color Transformer** to `RGB8` to render the point cloud in realistic, full color.
5. **Add the Architectural Markers / Room Meshes:**
   - Click **Add**, select **By Topic**, and choose `/reconstructor/map` -> **MarkerArray**.
   - This displays parametric RoomPlan geometry boxes (walls, doors, tables) and reconstructed LiDAR meshes directly alongside the camera path.

---

## ⚙️ Technical System Architecture

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

---

## 🤝 Contributing & License
Contributions, bug reports, and features are welcome! Feel free to open a pull request if you'd like to implement new mesh generation pipelines, support CBOR/binary WebSockets, or expand RoomPlan geometry parsing. This project is licensed under the MIT License.
