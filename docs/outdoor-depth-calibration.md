# Outdoor depth distortion: recording diagnosis

## Current status after scan5

The outdoor reconstruction is **not yet metrically validated**. The fixes below
correct the inspector's initial viewpoint and color rendering. They do not
change depth calibration or remove distant geometry to disguise its errors.

Scan5 contains 25 calibrated depth frames. Replaying their grid triangles with
the production accumulator retains 540,076 vertices and 1,143,341 triangles,
without reaching either capacity limit. Reprojection into each frame's recorded
camera recovers the source pixel grid within 0.0002 pixels at the 99th percentile.
This checks the saved pose/cloud convention, not independent pose accuracy.

The original inspector placed an orthographic camera above the entire bounding
box. Distant predictions near the 100 m numerical limit dominated that framing.
Rendering the same accumulated geometry from a captured camera reveals the
street, cars, and houses. The screenshot's sparse fan shape is not evidence that
only LiDAR-range pixels were accumulated. A related scan4 camera-view check
measured 93.22% image coverage for frame 0's mesh and 96.48% for the accumulated
mesh; neither number establishes metric accuracy.

The inspector now retains the last camera pose/projection that contributed an
accepted mesh frame, including interface orientation. It starts at that viewpoint
and fits the captured field of view into the panel without aspect distortion.
An Overview button retains access to the full bounding-box view. Both modes show
the same full accumulated mesh. Starting another scan resets the capture camera;
ordinary updates preserve the user's navigation. Only matrices and aspect ratio
are retained, not camera images or ARFrames.

Camera RGB already contains lighting. Applying Blinn highlights and new scene
lights to it exaggerated the welded geometry's facets. Colored inspection
materials now use unlit vertex colors, and no longer compute unnecessary preview
normals. Uncolored geometry keeps its existing shaded material.

### Remaining reconstruction issue

Camera-aligned comparisons of scan5's individual and accumulated meshes show
additional overlapping layers and blurred colors after accumulation. The current
5 cm vertex weld only combines spatially coincident samples. It does not reconcile
conflicting depth estimates across views. A continuous inverse-depth calibration
does not itself create image holes: validity/triangle filters can remove coverage,
and differently calibrated frames can place the same surface at different depths.

Two offline experiments were deliberately not shipped: fitting the current
calibration to photometrically matched, reprojected previous depths did not
reliably improve the geometry; removing older foreground triangles contradicted
by newer predictions also erased useful surfaces. Neither is a verified fix.

Sparse feature triangulation with the recorded poses found usable distant
references in only 10 of 25 frames under the diagnostic's reprojection and
parallax checks. Some disagree substantially with the DA distances. These are
diagnostic estimates, not ground truth, and were not used to modify the scan.
Correcting the remaining layers needs validated cross-view depth reconciliation;
changing the preview does not accomplish that. Raw recordings and camera images
are not committed to the repository.

## Earlier calibration changes

The scan captured on 2026-09-11 at 22:07:23 UTC contains 17 calibrated Depth Anything point clouds, matching camera information and LiDAR point clouds, and 875 camera poses. It predates the triangle-spacing fix (`7ba3a9b`). The original recording is not committed to this repository.

Reprojecting the saved world-space points using the camera pose at each cloud's exact timestamp recovers their depth-image grid coordinates to within 0.0002 pixels at the 99th percentile. This rules out a timestamp mismatch in the recorded pose/cloud pairing; it does not independently establish tracking accuracy.

In frames 6–9, the fitted inverse-depth offsets are negative. Valid relative-depth pixels near the zero of `scale * relative + offset` become depths approaching the 100 m numerical cap. A negative offset alone is not an error; extrapolating near that zero without checking uncertainty is the problem.

Reapplying the triangle-spacing filter to the saved per-frame grids still retains 57,054 triangles touching points beyond 30 m across frames 6–10, before voxel accumulation. Locally smooth but incorrectly distant surfaces pass an edge-length check.

The first attempted fix attached an error envelope to each calibration and rejected distant pixels. The next scan (22:34 UTC) demonstrated why that was unsuitable: its effective maximum depth was only 3.6–6.0 m, and less than 1% of its retained points exceeded 5 m. That per-pixel support gate has been removed from both CPU and Metal projection.

The current fit uses same-frame LiDAR/Depth Anything pairs, with matching image-ray coordinates and unsmoothed LiDAR preferred. It solves `inverse_depth = a * relative + b` with `a > 0` and `b >= 0`. When the unconstrained solution has negative offset, it solves the weighted least-squares boundary `b = 0`; it does not simply clamp the offset while keeping the old slope. RANSAC includes both affine and boundary candidates. The selected fit must still explain the LiDAR overlap within the existing metric residual tolerances.

The nonnegative offset is a regularization choice, not a physical guarantee about every model output. It removes the positive-relative-depth reciprocal pole. If this family cannot explain the overlap, the frame is rejected with calibration feedback. It cannot guarantee metrically accurate distant geometry from an arbitrary monocular prediction.

Once accepted, the calibration is applied to the full Depth Anything image, including pixels with no LiDAR measurement. Only finite positive depths within the existing 0.1–100 m numerical bounds and the triangle continuity checks are required. LiDAR range and calibration sample spread no longer crop the image. Colors remain sampled from the exact inference input frame, and the resulting DA triangles feed the full-scene accumulator.

Tests replay sanitized LiDAR/DA pairs from six recorded frames (including four with the old reciprocal-pole failure), test image-ray alignment, and verify that calibration references entirely within 5 m produce accumulated colored meshes at 10, 30 and 50 m. The recording did not contain confidence maps, so replay assumes confidence 1 and is not an exact reconstruction of the original sensor fit. The raw user recording is not included in the repository. A new outdoor device scan is still needed to validate overall reconstruction accuracy.
