# App Store Publishing Plan

This plan turns MapEverything from local developer builds into a reviewable App Store release while preserving the robotics workflow: a LiDAR iPhone or iPad publishes `/mapping/...` ROS2 topics to an external recorder, with optional local SQLite bag fallback.

## Current Release Target

- App Store display name: `Mapping`.
- Internal product and repository name: `MapEverything`.
- Bundle identifier: `com.salsicha.MapEverything`.
- Distribution channels: local developer builds first, then TestFlight, then App Store. GitHub releases carry source artifacts for the ROS2 companion package, RViz config, validation notes, and recorder scripts.
- Required hardware for meaningful validation: a LiDAR-equipped iPhone or iPad Pro, a ROS2 recorder workstation, and a shared local network for rosbridge.

## Apple-Side Gates

Apple's current App Store Connect workflow requires a processed build before TestFlight or App Review. Builds can be uploaded through Xcode, Transporter, altool, or the App Store Connect API, and App Store Connect associates each upload by bundle ID, version, and build string.

TestFlight should be used before App Review. Provide beta test information, upload a build, invite internal testers first, then invite external testers. External testing may require beta review, and the first build added to an external group is sent to App Review.

App Store privacy is an account-side gate. The iOS app needs a privacy policy URL, and App Store Connect privacy answers must accurately describe data collected by the app and by integrated third-party code.

Export compliance is also an account-side gate. Because MapEverything uses networking and may use Apple-provided or provider-side encrypted transport, the release owner must answer App Store Connect encryption questions before TestFlight or App Review. If no additional documentation is required, record the final answer in the release notes and optionally add the approved Info.plist export-compliance key.

Official references:
- App Store Connect upload builds: https://developer.apple.com/help/app-store-connect/manage-builds/upload-builds
- TestFlight overview: https://developer.apple.com/help/app-store-connect/test-a-beta-version/testflight-overview
- Manage app privacy: https://developer.apple.com/help/app-store-connect/manage-app-information/manage-app-privacy
- Export compliance overview: https://developer.apple.com/help/app-store-connect/manage-app-information/overview-of-export-compliance
- Submit an app: https://developer.apple.com/help/app-store-connect/manage-submissions-to-app-review/submit-an-app

## Local Release Readiness Tooling

Run the local readiness checker before cutting a release branch:

```bash
python3 tools/app-store-release-check.py
python3 tools/app-store-release-check.py --json
```

The checker verifies local plist metadata, usage strings, portrait iPhone orientation, Wi-Fi entitlement presence, app icon assets, archive export options, and required documentation. It also prints warnings for account-side work that cannot be checked from the repository, such as privacy labels, screenshots, support URL, age rating, and export compliance answers.

The App Store Connect export options template lives at:

```bash
tools/app-store-export-options.plist
```

It uses the current `xcodebuild` `app-store-connect` export method and `upload` destination. For a local IPA instead of direct upload, copy the file, change `destination` to `export`, and pass the copy to `xcodebuild -exportArchive`.

## Release Flow

1. Freeze the release branch.
   - Confirm `main` is clean except for intentional release changes.
   - Bump `MARKETING_VERSION` and `CURRENT_PROJECT_VERSION`.
   - Run `python3 tools/app-store-release-check.py`.

2. Run local validation.
   - `xcodebuild build -project MapEverything/MapEverything.xcodeproj -scheme MapEverything -configuration Release -destination generic/platform=iOS`
   - Simulator-safe tests when CoreSimulator is healthy.
   - `python3 tools/rosbridge-throughput-benchmark.py --dry-run --duration 5`
   - `python3 tools/run-rosbridge-recorder.py --dry-run --include-optional`

3. Run physical-device validation.
   - Install on a LiDAR device.
   - Confirm first launch, portrait lock, loading screen, camera permission, location permission, local network prompt, and Bluetooth permission.
   - Record at least one indoor and one outdoor session.
   - Verify `/mapping/pose`, `/mapping/camera/image/compressed`, `/mapping/camera/camera_info`, `/mapping/pointcloud/lidar`, `/mapping/pointcloud/depth_anything`, `/mapping/depth_anything/calibration`, `/mapping/gps/fix`, `/mapping/gps/metadata`, `/mapping/satellite/image/compressed`, `/mapping/satellite/tile_info`, and `/mapping/dem/tile` in rosbag2.
   - Replay the bag in RViz using `ros2/rviz/mapeverything.rviz`.

4. Archive.
   - Use Xcode Organizer for the first candidate, or run:

```bash
/Applications/Xcode.app/Contents/Developer/usr/bin/xcodebuild archive \
  -project MapEverything/MapEverything.xcodeproj \
  -scheme MapEverything \
  -configuration Release \
  -destination generic/platform=iOS \
  -archivePath build/AppStore/MapEverything.xcarchive \
  -allowProvisioningUpdates
```

5. Upload.
   - Upload through Xcode Organizer for the most visible signing diagnostics, or run:

```bash
/Applications/Xcode.app/Contents/Developer/usr/bin/xcodebuild -exportArchive \
  -archivePath build/AppStore/MapEverything.xcarchive \
  -exportOptionsPlist tools/app-store-export-options.plist \
  -exportPath build/AppStore
```

6. TestFlight.
   - Add beta app description, feedback email, and reviewer notes.
   - Start with internal testers.
   - Add external testers only after a successful internal capture and rosbag replay.
   - Include beta review notes that explain required LiDAR hardware, ROS bridge setup, local network behavior, and optional local SQLite bag fallback.

7. App Store Connect metadata.
   - App name: `Mapping`.
   - Subtitle: `ROS2 mapping payload`.
   - Category: choose the final primary category in App Store Connect after reviewing competing robotics, developer, and productivity tools.
   - Description should state that the app is for field mapping with LiDAR-capable devices and a ROS2 recorder, not a consumer navigation or surveying replacement.
   - Screenshots should show the record screen, ROS bridge panel, local bag browser/share flow, RViz replay, and App Store privacy-relevant permission prompts.
   - Support URL and privacy policy URL must be live before review.

8. Privacy and compliance.
   - Confirm whether camera frames, depth-derived point clouds, GPS fixes, BLE observations, current Wi-Fi metadata, local network endpoint information, geotile cache records, diagnostics, and optional provider configuration are collected, stored, shared, or only processed locally.
   - Confirm local SQLite bags remain off by default and are user-controlled.
   - Confirm optional credentials are not published in ROS topics or local bags.
   - Preserve NASA GIBS, USGS 3DEP, Mapzen, and optional-provider attribution/source-policy language.
   - Keep geospatial output framed as robotics mapping context, not certified surveying.

9. Submit for review.
   - Choose the processed build for the app version.
   - Add the version to a draft submission.
   - Attach review notes with hardware and ROS bridge setup.
   - Submit for review only after the release-blocker checklist is clear.

10. Release and monitor.
   - Prefer manual release for the first App Store version.
   - Watch App Store Connect crashes, TestFlight feedback, reviewer messages, and support inbox.
   - Tag the repo and publish GitHub release artifacts after App Review acceptance.

## Release Artifacts

Keep each candidate release directory or GitHub release organized with:

- App version, build number, commit SHA, and Xcode version.
- Archive/export logs.
- `tools/app-store-release-check.py --json` output.
- Physical-device validation report.
- Rosbridge throughput benchmark JSON.
- Rosbag replay notes and representative bag metadata.
- ROS2 companion package source archive from `ros2/reconstructor_msgs`.
- RViz config from `ros2/rviz/mapeverything.rviz`.
- Geospatial attribution/source-policy records.
- Known issues and rollback notes.

## Release Blockers

Do not submit to TestFlight external review or App Review if any of these are true:

- Signing team, bundle ID, App Store Connect app record, or provisioning profile is missing.
- Camera, location, Bluetooth, local network, or Wi-Fi entitlement behavior is unexplained or denied during validation.
- Privacy policy URL, privacy labels, export compliance answers, support URL, age rating, screenshots, or review notes are incomplete.
- Physical LiDAR capture cannot publish or save the default topic set.
- Rosbag2 replay fails on a separate ROS2 workstation.
- Local SQLite bag sharing exposes credentials or undocumented third-party provider payloads.
- Geospatial provider attribution or source policy is missing.
- The app claims survey-grade accuracy, broad Wi-Fi scanning, cellular RF survey support, or autonomous navigation capability without external validated sensors.
