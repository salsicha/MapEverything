# App Store Connect Metadata — Version 1.0

This file is the copy-ready source of truth for the first App Store submission.
It describes release candidate `1.0 (2)` with bundle identifier
`com.salsicha.MapEverything`.

## App Record

- Platforms: iOS
- Name: `MapEverything ROS2 Mapper`
- Primary language: English (U.S.)
- Bundle ID: `com.salsicha.MapEverything`
- SKU: `mapeverything-001`
- User access: Full Access

If the primary name is unavailable, use `MapEverything LiDAR Mapper`.

## Product Page

- Subtitle: `LiDAR mapping for ROS2`
- Promotional text: `Capture LiDAR, camera, depth, GPS, satellite, and elevation context for ROS2 recording and replay.`
- Keywords: `robotics,LiDAR,ROS2,rosbridge,rosbag,point cloud,ARKit,GPS,depth,elevation`
- Primary category: Developer Tools
- Secondary category: Utilities
- Support URL: https://github.com/salsicha/MapEverything/blob/main/SUPPORT.md
- Marketing URL: https://github.com/salsicha/MapEverything
- Privacy policy URL: https://github.com/salsicha/MapEverything/blob/main/PRIVACY.md
- Copyright: `2026 Alex Moran`

### Description

MapEverything turns a LiDAR-equipped iPhone or iPad into a ROS2 mapping sensor
for robotics data collection.

Capture device pose, camera images and intrinsics, LiDAR point clouds, relative
Depth Anything point clouds and calibration, GPS, satellite imagery, and
elevation tiles. Publish through rosbridge to a recorder on your local network,
or enable Local Bags to save shareable SQLite recording chunks on the device.

The app includes live mapping status, bounded publishing queues, recorder
health diagnostics, local bag browsing and sharing, and companion ROS2 message
definitions and RViz configuration.

A LiDAR-capable device is recommended for full functionality. MapEverything is
intended for robotics data collection. It is not a navigation app, does not
provide survey-grade measurements, and does not provide autonomous-navigation
capability.

## Pricing and Availability

- Distribution: Public
- Price: Free
- Tax category: App Store software
- Availability: All countries or regions
- Pre-order: Disabled
- Apple silicon Mac availability: Disabled
- Apple Vision Pro availability: Disabled
- Release option: Manually release this version

## Content Rights

- Contains, shows, or accesses third-party content: Yes
- Necessary rights are held: Yes

The third-party content is public geospatial data from NASA GIBS, USGS 3DEP,
and Mapzen terrain tiles hosted through AWS. Provider attribution and source
policy accompany published tiles.

## App Privacy Answers

- Developer or third-party collection: Yes
- Data type: Location -> Coarse Location
- Purpose: App Functionality
- Linked to identity: No
- Used for tracking: No
- Tracking: No

Camera, depth, LiDAR, Bluetooth, Wi-Fi, local bags, and diagnostics are
processed locally or sent only to infrastructure configured and controlled by
the user. The developer does not receive them.

## Export Compliance

- Uses encryption supplied by Apple's operating system: Yes
- Implements proprietary or non-standard encryption: No
- Implements encryption independently of Apple's operating system: No
- Uses custom cryptography: No

The app declares `ITSAppUsesNonExemptEncryption = false` and should require no
export documentation unless its cryptography changes.

## Age Rating

Answer No for every in-app control and capability, including parental
controls, age assurance, unrestricted web access, user-generated content,
social media, messaging, chat, and advertising. Answer None for every mature
theme, medical or wellness topic, sexuality or nudity category, violence
category, and chance-based activity. Do not select Made for Kids and do not
override the calculated rating. Expected rating: 4+.

## TestFlight Information

### Beta Description

MapEverything turns a LiDAR-equipped iPhone or iPad into a ROS2 mapping sensor.
This beta validates physical-device mapping, rosbridge publishing, local bag
storage and sharing, network recovery, and ROS2/RViz replay.

### What to Test

1. Launch the app on a LiDAR-equipped iPhone or iPad.
2. Enable Save Local.
3. Start a mapping session and move slowly around varied geometry.
4. Confirm the live mesh and mapping status continue updating.
5. Stop the session and open Share Local Bags.
6. Confirm the saved session can be listed and shared.
7. If a ROS2 recorder is available, connect to rosbridge on port 9090 and
   verify the default `/mapping` topics are recorded.
8. Report crashes, tracking failures, excessive thermal throttling, reconnect
   failures, missing topics, or local bags that cannot be shared.

## App Review Information

- Sign-in required: No
- Contact email: `salsicha@gmail.com`
- Demo attachment: `MapEverything-App-Review-Demo.mp4`

The release owner must enter a real contact phone number in App Store Connect.

### Review Notes

MapEverything does not require an account or external hardware for basic
review.

A LiDAR-equipped iPhone or iPad provides the complete mapping experience. The
app degrades gracefully when LiDAR is unavailable.

Standalone review procedure:

1. Launch the app and grant Camera and Location permissions.
2. Tap Save Local.
3. Tap Start Mapping and move slowly around nearby geometry.
4. Observe the live mapping mesh and status indicators.
5. Tap Stop Mapping.
6. Tap Share Local Bags to view and share the recorded local session.

The ROS switch is optional. When enabled, it connects only to a rosbridge
server configured by the user on the local network, which is why the Local
Network permission is requested.

The app does not perform broad Wi-Fi scanning, does not collect cellular RF
measurements, does not claim survey-grade accuracy, and does not provide
autonomous-navigation functionality.

A demonstration video is attached as `MapEverything-App-Review-Demo.mp4`.

## Screenshot Deliverables

Create the following five real-app captures for both a 6.9-inch iPhone and a
13-inch iPad:

1. Ready screen
2. Active colored mesh
3. ROS recorder and topic status
4. Local bag browser
5. Session history

Use real LiDAR/AR content. Target portrait PNG sizes are 1320 x 2868 for the
6.9-inch iPhone set and 2064 x 2752 for the 13-inch iPad set.
