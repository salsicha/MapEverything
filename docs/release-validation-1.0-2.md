# Release Validation — 1.0 (2)

Validation date: 2026-08-15

This report records the completed local checks for the MapEverything 1.0 (2)
App Store candidate. Account-side App Store Connect work and physical-device
field validation remain separate release gates.

## Candidate

- Xcode: 26.6 (Build 17F113)
- Scheme: `MapEverything`
- Configuration: `Release`
- Bundle identifier: `com.salsicha.MapEverything`
- Version: `1.0`
- Build: `2`
- Minimum OS: iOS 26.4
- Architecture: arm64
- Development team: `4D5JPBFXSA`
- Archive: `build/AppStore/MapEverything-1.0-2.xcarchive`
- Archive created: 2026-08-15 08:34:06 UTC
- Export/upload status: not attempted

The archive is signed with the configured Apple Development identity. Xcode's
App Store export step will create or use the appropriate distribution signing
assets and re-sign the exported build.

## Completed Checks

1. `python3 tools/app-store-release-check.py --json`
   - Result: 37 pass, 9 warn, 0 fail.
   - The nine warnings are App Store Connect, screenshot, TestFlight, or
     physical-device decisions; no repository failure remains.
2. `git diff --check`
   - Result: pass.
3. `python3 -m py_compile tools/*.py`
   - Result: pass.
4. `bash -n tools/*.sh`
   - Result: pass.
5. `python3 tools/run-rosbridge-recorder.py --dry-run --include-optional`
   - Result: pass; the expected default and optional topic set was produced.
6. `python3 tools/rosbridge-throughput-benchmark.py --dry-run --duration 5 --json`
   - Result: pass; the synthetic stream completed without an error.
7. Release build for `generic/platform=iOS`
   - Result: pass.
8. Full `MapEverythingTests` target on iPhone 17 / iOS 26.5 simulator
   - Result: pass; no test failures.
9. Release archive for `generic/platform=iOS`
   - Result: pass.
10. Archive metadata and code-signature inspection
    - Result: pass; bundle ID, version, build, team, arm64 architecture,
      minimum OS, and encryption declaration match the release plan.

## Non-blocking Build Output

- Xcode's generated `DepthAnythingV2SmallF16.swift` wrapper emits strict
  concurrency warnings. The file is generated in DerivedData rather than
  maintained project source, and the Release build, tests, and archive pass.
- Xcode logs connection errors for a separate passcode-locked device while
  building for a generic device or simulator. These do not affect the passing
  build, test, or archive result.

## Gates Still Requiring the Release Owner

- Push the release commit so the privacy and support URLs are publicly
  reachable, then verify both URLs while signed out.
- Confirm Apple Developer membership and agreements, and create or verify the
  App Store Connect app record.
- Run the physical LiDAR, permissions, rosbridge, local bag, converter, and
  RViz replay validation in `docs/validation-plan.md`.
- Capture the required real-device iPhone and iPad screenshots and App Review
  demo video.
- Export/upload this archive, complete App Privacy, content rights, age rating,
  pricing, availability, TestFlight, and review metadata, then submit and
  manually release the approved version.
