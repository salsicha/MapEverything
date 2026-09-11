# Outdoor depth distortion: recording diagnosis

The scan captured on 2026-09-11 at 22:07:23 UTC contains 17 calibrated Depth Anything point clouds, matching camera information and LiDAR point clouds, and 875 camera poses. It predates the triangle-spacing fix (`7ba3a9b`). The original recording is not committed to this repository.

Reprojecting the saved world-space points using the camera pose at each cloud's exact timestamp recovers their depth-image grid coordinates to within 0.0002 pixels at the 99th percentile. This rules out a timestamp mismatch in the recorded pose/cloud pairing; it does not independently establish tracking accuracy.

In frames 6–9, the fitted inverse-depth offsets are negative. Valid relative-depth pixels near the zero of `scale * relative + offset` become depths approaching the 100 m numerical cap. A negative offset alone is not an error; extrapolating near that zero without checking uncertainty is the problem.

Reapplying the triangle-spacing filter to the saved per-frame grids still retains 57,054 triangles touching points beyond 30 m across frames 6–10, before voxel accumulation. Locally smooth but incorrectly distant surfaces pass an edge-length check.

The first attempted fix attached an error envelope to each calibration and rejected distant pixels. The next scan (22:34 UTC) demonstrated why that was unsuitable: its effective maximum depth was only 3.6–6.0 m, and less than 1% of its retained points exceeded 5 m. That per-pixel support gate has been removed from both CPU and Metal projection.

The current fit uses same-frame LiDAR/Depth Anything pairs, with matching image-ray coordinates and unsmoothed LiDAR preferred. It solves `inverse_depth = a * relative + b` with `a > 0` and `b >= 0`. When the unconstrained solution has negative offset, it solves the weighted least-squares boundary `b = 0`; it does not simply clamp the offset while keeping the old slope. RANSAC includes both affine and boundary candidates. The selected fit must still explain the LiDAR overlap within the existing metric residual tolerances.

The nonnegative offset is a regularization choice, not a physical guarantee about every model output. It removes the positive-relative-depth reciprocal pole. If this family cannot explain the overlap, the frame is rejected with calibration feedback. It cannot guarantee metrically accurate distant geometry from an arbitrary monocular prediction.

Once accepted, the calibration is applied to the full Depth Anything image, including pixels with no LiDAR measurement. Only finite positive depths within the existing 0.1–100 m numerical bounds and the triangle continuity checks are required. LiDAR range and calibration sample spread no longer crop the image. Colors remain sampled from the exact inference input frame, and the resulting DA triangles feed the full-scene accumulator.

Tests replay sanitized LiDAR/DA pairs from six recorded frames (including four with the old reciprocal-pole failure), test image-ray alignment, and verify that calibration references entirely within 5 m produce accumulated colored meshes at 10, 30 and 50 m. The recording did not contain confidence maps, so replay assumes confidence 1 and is not an exact reconstruction of the original sensor fit. The raw user recording is not included in the repository. A new outdoor device scan is still needed to validate overall reconstruction accuracy.
