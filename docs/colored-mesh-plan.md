# Colored Depth Anything Mesh — Implementation Plan (Tier 1)

Publish the live Depth Anything overlay mesh with per-vertex RGB sampled from
the camera, end to end: mesh generation → ROS schema → publish path → local
artifacts and inspection scene. Written to be handoff-safe: each stage is
independently valuable, lands as its own commit prefixed `Colored mesh stage
N:`, and this document is the source of truth for scope and encoding.

## Status

- [x] Stage 0 — this plan (committed before implementation began)
- [x] Stage 1 — per-vertex colors in mesh generation
- [x] Stage 2 — MeshSnapshot schema v3 with colored vertex encoding
      (colored encoding named `float32_xyz_rgb8_le_base64` to match the
      existing `float32_xyz_le_base64` convention, not the `xyz_f32_rgb_u8`
      placeholder in the spec below)
- [ ] Stage 3 — publish colored mesh snapshot + RViz marker
- [ ] Stage 4 — colored local artifacts and inspection scene

If you are picking this up mid-way: `git log --oneline | grep 'Colored mesh
stage'` shows what landed; the unchecked stages above are the remaining work.

## Why this is cheap

The DA overlay mesh is built from the calibrated depth grid in
`MeshGenerator.createDepthAnythingMeshSnapshot` (MeshGenerator.swift:108).
Each accepted vertex comes from a depth-map pixel `(x, y)` — exactly the
coordinates `PointCloudProcessor.sampleColor` (PointCloudProcessor.swift:~360)
uses to colorize the published point cloud. The colors are already computed
for the same pixels in the same frame task; the mesh just doesn't carry them.

## Non-goals (explicitly out of scope for Tier 1)

- Coloring the live RealityKit overlay in the AR view — it stays teal for
  contrast against the real scene.
- Coloring the ARKit reconstruction mesh (that is Tier 2: world-space color
  accumulation across frames, see ColoredSurfelMap for the machinery).
- Texture atlasing / projective texturing.
- GPU mesh coloring — the overlay grid is decimated (configuration `step`),
  so per-vertex CPU sampling is thousands of pixels, not 268k; not worth a
  kernel change.

## Encoding specification

**MeshSnapshot schema_version: 2 → 3.**

- Uncolored vertex encoding (unchanged bytes): `vertex_encoding =
  "xyz_f32"`, `vertex_stride_bytes = 12` (3 × little-endian float32).
- New colored encoding: `vertex_encoding = "xyz_f32_rgb_u8"`,
  `vertex_stride_bytes = 15` — 12 bytes position followed by 3 bytes RGB,
  no padding, little-endian, base64 in `vertex_data` as before.
- `index_encoding` unchanged.
- Consumers must switch on `vertex_encoding`/`vertex_stride_bytes` (the
  fields have always been self-describing); the version bump is the signal
  that colored payloads now occur. Document both encodings and the version
  history in `ros2/mapeverything_msgs/msg/MeshSnapshot.msg`.
- RViz marker: the DA mesh publishes one `TRIANGLE_LIST` marker on the
  existing `/mapping/map` MarkerArray topic, `ns = "depth_mesh"`, stable
  `id = 0`, per-vertex `colors` (std_msgs/ColorRGBA, channel/255.0, a=1),
  `colors.count == points.count`. Republishing the same ns/id replaces the
  previous marker, so no DELETE bookkeeping is needed; publish an action=2
  DELETE for it when scanning stops (same pattern as publishMeshRemovals,
  ROS2BridgeClient.swift).

## Stage 1 — per-vertex colors in mesh generation

Files: `MeshGenerator.swift`, `PointCloudProcessor.swift`,
`ARViewContainer.swift`, new test file `ColoredMeshGenerationTests.swift`.

1. Extract the YCbCr→RGB sampling math into an internal reusable helper on
   `PointCloudProcessor`:
   `nonisolated static func sampleCameraColor(depthX:depthY:depthWidth:depthHeight:imageWidth:imageHeight:yPlane:yBytesPerRow:cbcrPlane:cbcrBytesPerRow:) -> SIMD3<UInt8>`
   using the exact expressions from `buildProjectionTable`
   (image index truncate-then-clamp) and `sampleColor` (BT.601 constants
   1.402 / 0.344136 / 0.714136 / 1.772, truncating UInt8 conversion).
   Keep the existing table-based `sampleColor` delegating to it so the
   point-cloud path is unchanged (and the Metal parity test keeps guarding
   the math).
2. `DepthAnythingMeshSnapshot` gains `let colors: [SIMD3<UInt8>]`
   (`colors.count == vertices.count`; empty when no camera buffer was
   provided — all call sites must tolerate empty).
3. `createDepthAnythingMeshSnapshot` gains an optional trailing parameter
   `cameraImage: CVPixelBuffer? = nil`. When present and biplanar YCbCr:
   lock base address read-only, and for every **accepted** vertex sample the
   color with the helper at the vertex's grid pixel `(x, y)`. Depth-map
   width/height map to image coordinates exactly as the point-cloud path
   does. When nil or non-planar: `colors = []` (existing behavior).
4. Call site ARViewContainer.swift:~802 (inside
   `processDepthAnythingMappingFrame`, which already holds `cameraImage`):
   pass the buffer.
5. Tests (new file; do NOT edit MapEverythingTests.swift):
   - Synthetic depth map + YCbCr buffer (reuse the pattern from
     MetalDepthParityTests): assert `colors.count == vertices.count`.
   - Cross-check: for a grid with step 1, every mesh vertex color equals the
     `processDepthAnythingPointCloudCPU` color of the same pixel (build a
     pixel→color dictionary from the point cloud by matching positions).
   - Nil camera buffer → `colors.isEmpty`, mesh otherwise identical.

Definition of done: `xcodebuild test` green; commit `Colored mesh stage 1:
per-vertex colors in DepthAnythingMeshSnapshot`; push; tick the Status box.

## Stage 2 — MeshSnapshot schema v3

Files: `MeshSnapshotMessageSchema.swift`,
`ros2/mapeverything_msgs/msg/MeshSnapshot.msg`, new test file
`ColoredMeshSchemaTests.swift` (existing schema tests in
MapEverythingTests.swift may need version-number updates only).

1. `MeshSnapshotMessageSchema.schemaVersion` 2 → 3; update the `.msg`
   comment block with the encoding table and version history (1: initial,
   2: inverse-depth era, 3: adds optional xyz_f32_rgb_u8 vertex encoding).
2. New builder `MeshSnapshotMessageBuilder.makeColoredGridMeshMessage(
   header:snapshotID:source:frameID:vertices:indices:colors:maxTriangles:
   maxPayloadBytes:)`:
   - Interleaves 15-byte vertex records (position little-endian float32 ×3,
     then RGB bytes); indices as today.
   - Payload fitting: iteratively halve the kept triangle count (drop
     trailing triangles, compact the referenced vertices, preserving order)
     until the encoded message fits `maxPayloadBytes`. Measure candidates
     with `publishedPayloadBytes: maxPayloadBytes` as the placeholder — the
     final rewrite must never exceed the measured size (see the fixed
     regression in `fittedTrianglePoints`, MeshSnapshotMessageSchema.swift).
   - Sets `is_truncated`, `original_*` counts, `compression =
     "mesh_snapshot_binary_base64"`, `vertex_encoding = "xyz_f32_rgb_u8"`,
     `vertex_stride_bytes = 15`. With empty `colors`, falls back to
     `"xyz_f32"` / 12 so the builder is usable for uncolored grids too.
3. Tests: byte-exact decode of a tiny colored mesh (parse base64, check
   position floats and RGB per vertex, stride math); fitting keeps the
   payload under budget at awkward sizes near the cap; empty-colors
   fallback produces v3 message with the legacy encoding strings; all keys
   match the `.msg` field list (rosbridge rejects unknown keys).

Definition of done: tests green; commit `Colored mesh stage 2: MeshSnapshot
schema v3 with colored vertex encoding`; push; tick Status.

## Stage 3 — publish path

Files: `ROS2BridgeClient.swift`, `ARViewContainer.swift`, `README.md`, new
test file `ColoredMeshPublishTests.swift`.

1. `nonisolated func publishDepthAnythingMesh(_ snapshot:
   MeshGenerator.DepthAnythingMeshSnapshot, timestamp: TimeInterval)` on
   ROS2BridgeClient:
   - Guard `.mesh` stream enabled and `hasPublishOrBufferTarget` (mesh is a
     buffered sample kind).
   - Publish the Stage 2 colored snapshot message on the `.meshSnapshot`
     topic, source `"calibrated_depthanything_grid"`, frame `map`, budgets
     from `meshSnapshotConfiguration`.
   - Publish one colored TRIANGLE_LIST marker on the `.meshMarkers` topic:
     ns `"depth_mesh"`, id 0, points expanded per triangle from the (fitted)
     mesh, parallel `colors` array (ColorRGBA, channel/255, a=1). Reuse the
     triangle budget: cap points at `maxTrianglePoints` and verify the
     encoded size with `encodedPublishPayloadByteCount`, halving the kept
     triangles until it fits (same loop shape as the snapshot fitting).
   - Both messages route through `publishOrBufferLocalSample(kind: .mesh, …)`
     so local bag recording and offline buffering behave like the ARKit
     mesh path.
2. On scan stop, publish an `action = 2` DELETE for ns `"depth_mesh"` id 0
   (hook beside `publishMeshRemovals`; call it from the same place the AR
   session teardown notifies the bridge — a `clearDepthMeshMarker()` invoked
   from ARViewContainer's stop flow).
3. ARViewContainer: where the mapping frame's `meshSnapshot` is handed to
   visualization (`updateLiveDepthMeshVisualization` call site inside the
   frame task), also call `publishDepthAnythingMesh` — the DA cadence is
   already ~2 Hz, no extra throttle needed. Capture the bridge reference
   before any detached hop (Swift 6: `ROS2BridgeClient.shared` is
   nonisolated; publish methods are nonisolated — no isolation friction).
4. README topic table: note that `/mapping/mesh_snapshot` and `/mapping/map`
   now carry the colored Depth Anything grid mesh alongside the ARKit mesh,
   and which `vertex_encoding` marks colored payloads.
5. Tests: builder-level assertions that the marker dictionary has
   `colors.count == points.count`, ColorRGBA values match vertex RGB /255,
   and the message passes `JSONSerialization.isValidJSONObject`. (The
   publish plumbing itself is exercised by the existing lifecycle tests.)

Definition of done: tests green; commit `Colored mesh stage 3: publish
colored mesh snapshot and RViz marker`; push; tick Status.

## Stage 4 — artifacts and inspection scene

Files: `LocalROS2BagRecorder.swift` (LocalOverlayMeshArtifact),
`ARViewContainer.swift`, new tests in `ColoredMeshArtifactTests.swift`.

1. `LocalOverlayMeshArtifact` gains `colors: [SIMD3<UInt8>]` (empty =
   uncolored). `objString()` emits the widely supported vertex-color OBJ
   extension — `v x y z r g b` with channels as 0–1 floats — only when
   colors are present and `colors.count == vertices.count`; metadata JSON
   gains `"has_vertex_colors": true/false`.
2. `makeFinalOverlayMeshArtifact` (ARViewContainer.swift:~617) passes the
   snapshot's colors through.
3. `makeInspectionScene` (ARViewContainer.swift:~659): add an
   `SCNGeometrySource(colors:)` (semantic `.color`) when colors are present
   so the stopped-scan inspection view — and the USDZ export built from it —
   shows vertex colors.
4. Tests: OBJ round-trip (parse emitted lines, check `v` lines have 6
   numeric components and match input colors within 1/255), uncolored
   artifact emits legacy 3-component `v` lines, metadata flag correct.
5. Run `python3 tools/app-store-release-check.py` (should stay 30/0) and the
   full test suite.

Definition of done: tests green; commit `Colored mesh stage 4: colored OBJ
artifact and inspection scene`; push; tick Status; update README if any
user-visible behavior notes remain.

## Risks and notes for whoever continues this

- **Payload budgets dominate visual quality on the wire.** The default
  `MeshSnapshotPublishConfiguration` byte budget aggressively decimates a
  268k-vertex grid; that is intended. Full fidelity lives in the local OBJ
  artifact and the local bag. Do not raise budgets as part of this work.
- **Keep the color math in one place.** The Stage 1 helper is the single
  source of truth; the Metal kernel mirrors it and MetalDepthParityTests
  guards the pair. If you touch the constants, update the kernel and the
  parity test together.
- **rosbridge rejects unknown JSON keys.** Any new message key must exist in
  the `.msg` definition (see the `metric_pointcloud_topic` incident in git
  history). MeshSnapshot.msg already has every field Stage 2 uses.
- **Swift 6:** MeshGenerator and the bridge publish path are `nonisolated`;
  `DepthAnythingMeshSnapshot` is `@unchecked Sendable` (it carries a
  MeshDescriptor). Adding `colors` keeps that valid. New test suites should
  follow the `@Suite(.serialized)` + `#require` patterns from
  ROS2BridgeLifecycleTests if they touch shared singletons; pure-builder
  tests need nothing special.
- Marker `colors` uses float RGBA; RViz interprets per-point colors on
  TRIANGLE_LIST only when `colors.count == points.count` — assert it in
  tests.
