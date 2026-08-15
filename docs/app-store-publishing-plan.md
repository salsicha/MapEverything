# App Store Publishing Plan

This plan turns MapEverything from local developer builds into a reviewable App Store release while preserving the robotics workflow: a LiDAR iPhone or iPad publishes `/mapping/...` ROS2 topics to an external recorder, with optional local SQLite bag fallback.

## Release Status — 2026-08-15

Repository-side preparation targets version `1.0` build `2`. The readiness
checker, Release build, full unit-test target, dry-run recorder/throughput
tools, and fresh archive all pass with Xcode 26.6. The release archive is at
`build/AppStore/MapEverything-1.0-2.xcarchive`; detailed evidence is recorded
in `docs/release-validation-1.0-2.md`. It has not been exported or uploaded.

### Completed in the repository

- Info.plist release metadata: display name, all privacy usage strings (camera, location when-in-use plus temporary precise-location purpose, Bluetooth, local network), portrait-only iPhone, launch screen, app icon.
- **Privacy manifest** (`MapEverything/MapEverything/PrivacyInfo.xcprivacy`): declares no tracking and no developer data collection, and the four required-reason API categories the app actually uses — UserDefaults (CA92.1), file timestamps of its own bag files (C617.1), system boot time for ROS header timestamps (35F9.1), and disk-space checks for the local bag store (E174.1).
- **Export compliance key**: `ITSAppUsesNonExemptEncryption = false`. The app uses only Apple's standard TLS (HTTPS tile fetches, optional `wss://`) and no custom cryptography, which is exempt. This suppresses the per-build encryption questionnaire. Revisit if custom cryptography is ever added.
- **Device requirements**: `UIRequiredDeviceCapabilities` now includes `arkit` so the App Store only offers the app to ARKit-capable devices. LiDAR itself has no capability key — state it in the description and review notes (the app already degrades gracefully without LiDAR).
- Signing configuration: automatic signing, team `4D5JPBFXSA`, bundle `com.salsicha.MapEverything`, version `1.0` build `2`.
- Public privacy policy, support copy, and App Store field values live in `PRIVACY.md`, `SUPPORT.md`, and `docs/app-store-metadata.md`.
- The historical build `1` archive must not be uploaded. The current local candidate is `build/AppStore/MapEverything-1.0-2.xcarchive`.
- Local release verification completed on 2026-08-15: 37 readiness checks passed with 0 failures, the Release build passed, the full `MapEverythingTests` target passed, and the build `2` archive passed inspection.
- Note: the iOS 26.4 deployment target is intentional — every LiDAR-equipped device runs iOS 26.

### Your remaining steps (in order)

Each step below requires your Apple ID, your App Store Connect account, a public GitHub push, or a real device — none should be performed as an unapproved local-only repository action.

1. **Apple Developer Program**: confirm team `4D5JPBFXSA` has an active membership and that the latest agreements are accepted (App Store Connect → Business). A free account cannot upload to App Store Connect.
2. **Sign into Xcode**: Xcode → Settings → Accounts → add your Apple ID. Your keychain currently has only an *Apple Development* certificate; the first Distribute/upload will create the *Apple Distribution* certificate automatically.
3. **Create the app record**: App Store Connect → My Apps → "+" → New App. Platform iOS, name `MapEverything ROS2 Mapper`, bundle ID `com.salsicha.MapEverything`, SKU `mapeverything-001`, primary language English (U.S.), and Full Access. If the primary name is unavailable, use `MapEverything LiDAR Mapper`. The on-device name stays "Mapping" regardless (CFBundleDisplayName).
4. **Publish and verify the two checked-in URLs** after pushing the release commit:
   - *Privacy policy URL*: `https://github.com/salsicha/MapEverything/blob/main/PRIVACY.md`
   - *Support URL*: `https://github.com/salsicha/MapEverything/blob/main/SUPPORT.md`
   Open both in a signed-out browser before entering them in App Store Connect.
5. **Upload the build**: open Xcode → Window → Organizer → select the archive (or re-archive with the command in "Release Flow" step 4) → Distribute App → App Store Connect → Upload. Command-line alternative:
   `xcodebuild -exportArchive -archivePath build/AppStore/MapEverything-1.0-2.xcarchive -exportOptionsPlist tools/app-store-export-options.plist -exportPath build/AppStore/1.0-2`
   Wait for App Store Connect to finish processing (email arrives when done).
6. **Privacy labels** (App Store Connect → App Privacy): recommended declaration — *Location* → "App Functionality", not linked to identity, not used for tracking (because tile requests reveal coarse location to public providers). Everything else (camera frames, depth, radio telemetry) never leaves the user's own devices/network, so it is not "collected" in Apple's sense. Answer "no" to tracking.
7. **Age rating**: complete the questionnaire (expect 4+).
8. **Screenshots**: device family is iPhone + iPad, so you need at least one set for 6.9" iPhone and 13" iPad (App Store Connect will list the exact required sizes). Capture on a real LiDAR device: scan view with mesh overlay, ROS panel with topics/streams, local bag browser, and session history. AR content must be real captures, not mockups.
9. **TestFlight** (recommended before review): add beta app description and feedback email, test with internal testers first, confirm a full capture → local bag → converter → RViz replay cycle on a physical device (see `docs/validation-plan.md`).
10. **Review notes + demo video** (critical for approval): reviewers will not have a LiDAR device paired with a ROS2 recorder. In App Review notes explain: (a) LiDAR iPhone/iPad recommended, (b) the app is fully usable standalone via "Local Bags" — enable local bag storage, scan, then browse/share the bag, (c) the local-network prompt exists for the optional ROS bridge. Attach or link a short demo video showing a scan session and the local bag workflow. Without this, expect a "minimum functionality" or "hardware required" rejection.
11. **Submit**: add the processed build to version 1.0, submit for review, choose **manual release**. After acceptance, tag the repo (`git tag v1.0 && git push --tags`) and publish the GitHub release artifacts listed under "Release Artifacts".

## Current Release Target

- App Store display name: `Mapping`.
- Internal product and repository name: `MapEverything`.
- Release candidate: version `1.0` build `2`.
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
  -archivePath build/AppStore/MapEverything-1.0-2.xcarchive \
  -allowProvisioningUpdates
```

5. Upload.
   - Upload through Xcode Organizer for the most visible signing diagnostics, or run:

```bash
/Applications/Xcode.app/Contents/Developer/usr/bin/xcodebuild -exportArchive \
  -archivePath build/AppStore/MapEverything-1.0-2.xcarchive \
  -exportOptionsPlist tools/app-store-export-options.plist \
  -exportPath build/AppStore/1.0-2
```

6. TestFlight.
   - Add beta app description, feedback email, and reviewer notes.
   - Start with internal testers.
   - Add external testers only after a successful internal capture and rosbag replay.
   - Include beta review notes that explain required LiDAR hardware, ROS bridge setup, local network behavior, and optional local SQLite bag fallback.

7. App Store Connect metadata.
   - App name: `MapEverything ROS2 Mapper` (fallback: `MapEverything LiDAR Mapper`).
   - Subtitle: `LiDAR mapping for ROS2`.
   - Primary category: Developer Tools. Secondary category: Utilities.
   - Description should state that the app is for field mapping with LiDAR-capable devices and a ROS2 recorder, not a consumer navigation or surveying replacement.
   - Screenshots should show the record screen, ROS bridge panel, local bag browser/share flow, RViz replay, and App Store privacy-relevant permission prompts.
   - Copy every product, privacy, age-rating, TestFlight, and review field from `docs/app-store-metadata.md`.
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
- ROS2 companion package source archive from `ros2/mapeverything_msgs`.
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
