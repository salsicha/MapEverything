#!/usr/bin/env python3
"""Convert Prompt Depth Anything (ViT-S) to a CoreML mlpackage.

Prompt Depth Anything (CVPR 2025, arXiv:2412.14015) fuses ARKit LiDAR into
the Depth Anything decoder as a metric prompt, producing metric depth
directly - no per-frame affine calibration at all. This converts the ViT-S
variant for on-device evaluation (stage 6 of docs/consistent-scene-plan.md).

The mlpackage is NOT bundled into the app; side-load it (see
docs/consistent-scene-plan.md) to keep the app binary small until the model
earns its place.

Usage:
    python3 convert_promptda_coreml.py [--height 392] [--width 518] [--out PromptDepthAnythingVitS.mlpackage]

Requires: torch, transformers>=4.48, coremltools>=8.
"""
import argparse
import sys

import numpy as np
import torch


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--height", type=int, default=392, help="input height (multiple of 14)")
    parser.add_argument("--width", type=int, default=518, help="input width (multiple of 14)")
    parser.add_argument("--prompt-height", type=int, default=192)
    parser.add_argument("--prompt-width", type=int, default=256)
    parser.add_argument("--out", default="PromptDepthAnythingVitS.mlpackage")
    args = parser.parse_args()

    if args.height % 14 or args.width % 14:
        print("height/width must be multiples of 14", file=sys.stderr)
        return 2

    from transformers import PromptDepthAnythingForDepthEstimation

    model = PromptDepthAnythingForDepthEstimation.from_pretrained(
        "depth-anything/prompt-depth-anything-vits-hf"
    ).eval()

    class Wrapper(torch.nn.Module):
        def __init__(self, inner):
            super().__init__()
            self.inner = inner

        def forward(self, pixel_values, prompt_depth):
            return self.inner(
                pixel_values=pixel_values, prompt_depth=prompt_depth
            ).predicted_depth

    wrapper = Wrapper(model)
    pixels = torch.rand(1, 3, args.height, args.width)
    prompt = torch.rand(1, 1, args.prompt_height, args.prompt_width) * 4 + 0.5
    with torch.no_grad():
        reference = wrapper(pixels, prompt)

    # CoreML's converter has no bicubic upsampling; substitute bilinear for
    # conversion and quantify the substitution against the bicubic reference.
    original_interpolate = torch.nn.functional.interpolate

    def bilinear_interpolate(x, *inner_args, **kwargs):
        if kwargs.get("mode") == "bicubic":
            kwargs["mode"] = "bilinear"
        return original_interpolate(x, *inner_args, **kwargs)

    torch.nn.functional.interpolate = bilinear_interpolate
    with torch.no_grad():
        bilinear_reference = wrapper(pixels, prompt)
    substitution = np.abs(bilinear_reference.numpy() - reference.numpy())
    print(f"bicubic->bilinear substitution: mean {substitution.mean():.5f} m, max {substitution.max():.5f} m")

    import coremltools as ct

    def convert(source):
        return ct.convert(
            source,
            inputs=[
                ct.TensorType(name="pixel_values", shape=pixels.shape, dtype=np.float32),
                ct.TensorType(name="prompt_depth", shape=prompt.shape, dtype=np.float32),
            ],
            outputs=[ct.TensorType(name="metric_depth", dtype=np.float32)],
            minimum_deployment_target=ct.target.iOS17,
            compute_precision=ct.precision.FLOAT16,
            compute_units=ct.ComputeUnit.ALL,
        )

    mlmodel = None
    with torch.no_grad():
        # Fixed-shape trace + freeze constant-folds the interpolate size
        # arithmetic that otherwise reaches CoreML as unsupported dynamic
        # aten::Int casts; torch.export is the fallback path.
        try:
            traced = torch.jit.trace(wrapper, (pixels, prompt))
            frozen = torch.jit.freeze(traced.eval())
            mlmodel = convert(frozen)
        except Exception as error:  # noqa: BLE001 - deliberate fallback chain
            print(f"trace+freeze path failed ({type(error).__name__}: {error}); trying torch.export", file=sys.stderr)
            exported = torch.export.export(wrapper, (pixels, prompt)).run_decompositions({})
            mlmodel = convert(exported)
    mlmodel.short_description = (
        "Prompt Depth Anything ViT-S: metric depth from camera image + LiDAR prompt"
    )
    mlmodel.save(args.out)

    # Numerical validation against PyTorch on the trace input.
    prediction = mlmodel.predict({
        "pixel_values": pixels.numpy().astype(np.float32),
        "prompt_depth": prompt.numpy().astype(np.float32),
    })["metric_depth"]
    torch.nn.functional.interpolate = original_interpolate
    reference_np = bilinear_reference.numpy()
    prediction = prediction.reshape(reference_np.shape)
    absolute = np.abs(prediction - reference_np)
    relative = absolute / np.maximum(np.abs(reference_np), 1e-3)
    print(f"reference depth range: {reference_np.min():.3f}..{reference_np.max():.3f} m")
    print(f"abs diff  mean {absolute.mean():.4f} m, p99 {np.quantile(absolute, 0.99):.4f} m, max {absolute.max():.4f} m")
    print(f"rel diff  mean {relative.mean()*100:.2f}%, p99 {np.quantile(relative, 0.99)*100:.2f}%")
    print(f"saved {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
