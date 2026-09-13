# Prompt Depth Anything spike (stage 6)

Prompt Depth Anything (CVPR 2025, arXiv:2412.14015) feeds ARKit LiDAR into the
Depth Anything decoder as a metric prompt, so the network outputs metric depth
directly — no per-frame affine calibration exists to disagree between frames.
It is the field's purpose-built answer to this app's exact sensor combination
and the long-term replacement for the fitting pipeline (which remains the
fusion layer under any depth source).

## Conversion result

`tools/promptda/convert_promptda_coreml.py` converts the ViT-S checkpoint
(`depth-anything/prompt-depth-anything-vits-hf`) to a CoreML mlpackage
(input: 1x3x392x518 image tensor + 1x1x192x256 LiDAR prompt; output: metric
depth; FP16, iOS 17 target). Measured on conversion day:

- Package size: 48 MB.
- CoreML vs PyTorch fidelity: mean abs 6.3 mm, p99 10.9 mm, max 20.5 mm at
  3–4 m depths (0.18% mean relative).
- The converter has no bicubic upsampling, so conversion substitutes
  bilinear: mean 2.6 mm / p99 10 mm / max 89 mm against the bicubic
  reference — noise-level for this use.
- Inference latency: 27.7 ms median on an Apple-silicon Mac; expect roughly
  60–120 ms on an A16 ANE — comfortable at the 2 fps enhanced-frame rate.

Conversion gotchas encoded in the script: the `-hf` checkpoint variant is the
transformers-format one; the torch.jit path fails on dynamic interpolate size
arithmetic, so conversion goes through `torch.export` +
`run_decompositions({})`; coremltools' BlobWriter requires Python <= 3.12.

## Integration plan (not yet wired)

1. Side-load: the mlpackage stays out of the app bundle (gitignored next to
   the converter). Compile with `MLModel.compileModel`, load from Documents.
2. `PromptDepthProcessor` behind a debug setting: input the captured image
   resized to 518x392 plus the LiDAR depth map resampled to 256x192 with
   invalid pixels zeroed; output is already metric — skip
   DepthCalibrationPipeline's fit and feed the depth image straight to the
   accumulator/TSDF with `.lidar` provenance and the standard 12 m cap.
3. Keep the map-anchored consistency stack active: PromptDA removes
   calibration variance, not pose drift or network shape error, so carving,
   support caps, and TSDF fusion still apply.
4. Evaluate on-device against the current pipeline with the scan6 replay
   metric before making it the default.
