# Mesh-propagated depth calibration

When the per-frame LiDAR calibration fails (`DepthAnythingProcessor.maximumLikelihoodCalibration`
returns nil), the app derives the Depth Anything calibration
`metric = 1/(scale * relative + offset)` by projecting the LiDAR-anchored part of the
accumulated scene mesh into the current camera and fitting the same constrained
inverse-depth family (`RobustDepthCalibration.fit`, reused verbatim) against those
pseudo-depth samples. Scans keep producing calibrated geometry when the camera points
beyond LiDAR range, at sunlit surfaces, or at LiDAR-hostile materials.

## Why per-frame fitting is still required

Depth Anything outputs affine-invariant relative inverse depth: each inference is
normalized to its own scale and offset, so coefficients are not transferable between
frames. What *is* transferable is metric geometry. The accumulated mesh is metric
world-space truth (built exclusively from LiDAR-calibrated frames), so projecting it
into the failing frame yields (relative, metric depth) pairs to fit that frame's own
coefficients — the same shape of problem the LiDAR path solves, with the map standing
in for the sensor.

## Drift safety: the reference-closure invariant

The single structural rule that bounds error:

> Every vertex with `lidarWeight > 0` has a position formed exclusively from
> LiDAR-calibrated observations. Calibration anchors are a subset of those vertices.

Enforced at write time in `AccumulatedDepthMesh.fuse`:

- `lidarWeight` accrues only on `.lidar` frames (incidence-only observation weight, no
  distance attenuation — a frontal LiDAR-calibrated sighting at 4 m is a valid anchor
  certificate; depth-dependent noise is handled by the fit sigma instead).
- `.meshPropagated` observations never update the position or weight of any vertex with
  `lidarWeight > 0` — color only. They may create new vertices (with `lidarWeight = 0`)
  and refine other fallback-born vertices, at a 4x observation-weight penalty.
- When a `.lidar` frame first touches a fallback-born vertex (`lidarWeight == 0`,
  `weight > 0`), the vertex's running average is reset to the LiDAR observation.
  LiDAR authority replaces propagated geometry on first contact.

Because fallback output can never re-enter the reference set, the provenance graph has
generation depth exactly 1: a fallback fit's error is (anchor error) + (ARKit pose
drift) + (fit residual), independent of how many fallback frames preceded it. There is
no compounding term. `propagatedCalibrationDoesNotCompoundScaleError` in
`AccumulatedDepthMeshTests` pins this.

## Anchor index

The actor maintains an incremental index instead of scanning 2M vertices per frame:

- `anchorVertexIndices: [UInt32]` + `anchorKeys: [SIMD3<Int>: UInt32]` — one vertex per
  0.2 m voxel (4x the weld voxel), inserted once when a vertex's `lidarWeight` crosses
  `anchorLidarWeight = 1.5` (proves >= 2 LiDAR-calibrated sightings). Hard cap 65,536
  (~2,600 m^2 of anchored surface); insertion stops at the cap, coverage remains valid.
- `calibrationAnchors()` returns cached positions; the cache is invalidated only when a
  `.lidar` integrate accepts a face, so it stays warm through an entire fallback streak.

## Per-frame fallback algorithm (`MeshDepthPropagation`)

Runs on the existing detached frame task, at most 2 Hz, only on frames whose LiDAR
calibration failed (~3–6 ms budget):

1. Project each anchor with the validated grid convention (`scaleX = depthW/imageW` etc.,
   `gridY = cy - fy*Pc.y/depth`); keep depths in 0.3–30 m.
2. Occlusion: z-buffer over a coarse cell grid (relative grid / 8). Each anchor splats
   its minimum depth over the cells covered by its 0.2 m physical footprint
   (radius clamped to 1–6 cells), so near surfaces suppress back-surface anchors even
   between sparse projected points.
3. Visibility: an anchor survives if its depth is within `max(0.10, 0.05*d)` of its
   cell's z-buffer minimum. One sample per cell (nearest visible anchor), sampled from
   the relative map at the anchor's exact rounded pixel.
4. Gates before/around the fit:
   - >= 200 samples (stricter than fit()'s >50 — pseudo-samples are individually weaker);
   - image coverage: 10–90th percentile spread of occupied cells >= 40% of the grid in
     both axes (no calibrating a whole frame from a sliver);
   - `RobustDepthCalibration.fit` verbatim (65% inliers, metric residuals, depth span);
   - sparse-LiDAR cross-check: if >= 10 individually valid LiDAR pixels exist (the fit
     failed as a *set*, not necessarily per pixel), the median metric residual of the
     propagated calibration against them must be <= max(0.12, 0.15*d). This is the only
     valid absolute-scale check; scale/offset themselves are not comparable across
     frames (independent normalizations), so there is deliberately no scale-jump gate.
5. Sample weights use the monocular sigma `max(0.15, 0.10 + 0.08*d)` in the LiDAR path's
   inverse-variance convention (`weight = 1 / (sigma/d^2)^2`) — honest for mesh-derived
   depth and ~100x weaker than LiDAR samples if the two were ever mixed.

## Integration behavior

- `DepthAnythingMappingFrame` carries `calibrationSource` (`.lidar` / `.meshPropagated`);
  `AccumulatedDepthMesh.integrate` applies the write rules above.
- ROS: `mapping/pointcloud/depth_anything_calibration` sets `calibration_source` to
  `arkit_lidar_maximum_likelihood` or `accumulated_mesh_propagated`, and
  `uses_lidar_for_scale_calibration` accordingly. Schema unchanged (the field existed).
- UX: fallback success is silent for 10 consecutive frames, then shows
  "Depth from scanned map — revisit nearby surfaces when possible." A fallback failure
  shows "Depth scale unavailable. Scan nearby surfaces first, then move outward." (no
  anchors) or "Depth scale unavailable. Keep previously scanned surfaces in view."
  (anchors exist but the fit was rejected). A rejected fallback drops the frame exactly
  like before — scale is never invented.

## What was deliberately left out

- **Temporal scale-jump gates** — invalid across independently normalized inferences.
- **Staleness cutoff** (e.g. 60 s / 10 m since last LiDAR fit) — it would cap the exact
  long-excursion scenario the feature exists for; ARKit's relocalization bounds pose
  drift, and the fit + cross-check gates reject geometrically inconsistent frames.
- **Mixing LiDAR and pseudo samples in one fit** — fallback runs only when the LiDAR fit
  already failed; mixing would let a degenerate LiDAR set skew a healthy mesh fit.
