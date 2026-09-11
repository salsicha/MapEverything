# Outdoor depth distortion: recording diagnosis

The scan captured on 2026-09-11 at 22:07:23 UTC contains 17 calibrated Depth Anything point clouds, matching camera information and LiDAR point clouds, and 875 camera poses. It predates the triangle-spacing fix (`7ba3a9b`). The original recording is not committed to this repository.

Reprojecting the saved world-space points using the camera pose at each cloud's exact timestamp recovers their depth-image grid coordinates to within 0.0002 pixels at the 99th percentile. This rules out a timestamp mismatch in the recorded pose/cloud pairing; it does not independently establish tracking accuracy.

In frames 6–9, the fitted inverse-depth offsets are negative. Valid relative-depth pixels near the zero of `scale * relative + offset` become depths approaching the 100 m numerical cap. A negative offset alone is not an error; extrapolating near that zero without checking uncertainty is the problem.

Reapplying the triangle-spacing filter to the saved per-frame grids still retains 57,054 triangles touching points beyond 30 m across frames 6–10, before voxel accumulation. Locally smooth but incorrectly distant surfaces pass an edge-length check.

The fix attaches a support envelope to each live calibration. It uses the accepted samples' relative-depth spread and RMS inverse-depth residual, floored by the existing LiDAR noise model. Error grows outside that spread and does not shrink merely because correlated pixels are repeated. A point is omitted when a two-error envelope would change its reciprocal depth by more than 25%. This is a conservative quality heuristic, not a guarantee of physical accuracy or a statistical confidence interval. It can leave holes and reduce distant coverage.

The mesh, CPU point cloud, and Metal point cloud use the same gate. Calibration messages record the support parameters. No additional neural-network inference is needed.

Regression cases use summaries reconstructed from the recording's corresponding LiDAR/Depth Anything points. Original confidence images were not recorded, so these are not an exact replay of the live calibration fit. Tests also cover retained depth beyond 5 m, correlated samples, metadata, and CPU/Metal parity. Actual outdoor reconstruction still requires a new device scan to validate.
