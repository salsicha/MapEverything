# Consistent scene reconstruction plan

Goal: fused Depth Anything geometry that reads as one consistent, realistic scene
from any viewpoint — not layered, differently-stretched copies per frame.

Measured root cause (scan6, 20 frames, near-static camera): per-frame affine
inverse-depth fits are unconstrained beyond LiDAR's 0.2–5 m support. Consecutive
frames agree to 1–2% inside LiDAR range but diverge to ~22% of range at 10–25 m;
fits split into an offset=0 boundary family (far field 17–19 m) and interior fits
(4–7 m) with 20.5 m p90 same-ray displacement between families. TSDF alone cannot
fix >truncation disagreement (it duplicates zero crossings and preserves occluded
ghosts), so calibration consistency comes first, fusion second.

Stages (each committed separately, evaluated against the scan6 fixture):

1. **Map-anchored joint calibration, always on** — every frame fits against LiDAR
   samples (0.2–5 m) plus z-buffered map-anchor pseudo-samples (far field),
   pseudo-weights normalized to the LiDAR total. Validated by simulation on scan6:
   −58% median / −67% p90 far-field inter-frame spread in one causal pass.
2. **Calibration acceptance gating + support-capped integration** — a fit must
   agree with whichever independent references exist (sparse LiDAR, map anchors);
   each frame integrates only to ~1.75× its supporting samples' depth range, so
   unconstrained extrapolation never enters the persistent map.
3. **Free-space carving** — CHISEL-style projection-mapping carve on the vertex
   map: vertices observed well in front of a frame's measured surface decay and
   tombstone. The map gains its missing negative-evidence channel.
4. **Range cap + precision weighting** — accumulated-scene integration capped
   (~12 m) with 1/sigma^2 precision-weighted fusion (sigma = monocular model);
   streaming/overlay output unchanged.
5. **Sparse voxel-hash TSDF + incremental marching cubes** — 16^3 blocks, packed
   voxels, truncation tau(z), colored integration reusing the exposure/view
   weighting; per-dirty-block marching cubes for the stopped preview and exports.
   The vertex map remains for ROS streaming and calibration anchors.
6. **Prompt Depth Anything spike** — convert prompt-depth-anything-vits to
   CoreML (LiDAR prompt in-network, no affine fit at all); numerical validation
   against PyTorch; app-side scaffolding behind a flag, model side-loaded, not
   bundled.

Evaluation without a device: scan6 replayed through the real pipeline in unit
tests (fixture: downsampled relative maps inverted from the recorded clouds +
recorded calibrations/poses/intrinsics); metric = consecutive-frame far-field
along-ray spread, before vs after. Performance timed on Apple-silicon Mac with
documented ~3–6× CPU margin to A16.
