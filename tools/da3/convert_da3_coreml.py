#!/usr/bin/env python3
"""Export the official DA3-Small depth branch for live iOS inference.

Requires the official DA3 source on PYTHONPATH, its pinned model.safetensors,
torch, coremltools, and the DA3 inference dependencies. Example:
  PYTHONPATH=/path/to/Depth-Anything-3/src python convert_da3_coreml.py \
    --weights /path/to/model.safetensors --out DepthAnything3SmallF16.mlpackage \
    --frames /path/to/depth-comparison/data

Input: RGB, 518x392 in native landscape sensor coordinates. The graph rotates
clockwise to the upright 392x518 benchmark input, applies exact ImageNet
normalization, runs DA3, then returns 1/depth in native sensor coordinates.
The reciprocal is an interface adapter; LiDAR calibration MUST keep offset=0.
No camera-pose/point-cloud heads, multi-view fusion, or LiDAR inference runs here.
Validated source: 3d835ec1a5802d64a8b8b15f817a1ab54809bfe4.
Validated tools: Python 3.12, torch 2.14.1, coremltools 9.0 (numerical checks
are required because this torch version is newer than the converter's support matrix).
"""
import argparse
import json
from pathlib import Path
import time
import types

import numpy as np
from PIL import Image
import torch
from safetensors.torch import load_file

MODEL_REVISION = 'e08cab65ca0ec38e7826075418411ab90cab4da3'


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--weights', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--frames', type=Path, required=True)
    args = p.parse_args()
    torch.set_num_threads(4); torch.manual_seed(0)
    from depth_anything_3.cfg import create_object, load_config
    from depth_anything_3.registry import MODEL_REGISTRY
    model = create_object(load_config(MODEL_REGISTRY['da3-small'])).eval()
    weights = {k.removeprefix('model.'): v for k, v in load_file(args.weights).items()}
    # Safetensors omits duplicated names of shared auxiliary LayerNorm weights.
    # Restore only proven parameter aliases; require every actual weight.
    aliases = {id(param): weights[name] for name, param in model.named_parameters(remove_duplicate=False) if name in weights}
    for name, param in model.named_parameters(remove_duplicate=False):
        if name not in weights and id(param) in aliases:
            weights[name] = aliases[id(param)]
    model.load_state_dict(weights, strict=True)

    class Wrapper(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.backbone = model.backbone
            self.head = model.head
            self.export_camera_token = False
            self.register_buffer('mean', torch.tensor([.485, .456, .406]).view(1, 3, 1, 1))
            self.register_buffer('std', torch.tensor([.229, .224, .225]).view(1, 3, 1, 1))

        def forward(self, image):
            # ImageType supplies pixels / 255. Native sensor -> portrait.
            upright = image.transpose(-2, -1).flip(-1)
            x = ((upright-self.mean)/self.std).unsqueeze(1)
            # For S=1 the official unconditional camera token is exactly the
            # first learned token. Explicitly providing it avoids a zero-length
            # source-token concatenation that Core ML cannot lower correctly.
            token = self.backbone.pretrained.camera_token[:, :1] if self.export_camera_token else None
            features, _ = self.backbone(x, cam_token=token, export_feat_layers=[], ref_view_strategy='first')
            depth = self.head(features, 518, 392, patch_start_idx=0)['depth'][:, 0]
            # Keep the existing inverse-depth CPU/Metal/ROS contract, with b=0.
            return (1.0/depth).flip(-1).transpose(-2, -1)

    wrapper = Wrapper().eval()
    frame_paths = sorted(args.frames.glob('frame-*.png'))
    image = Image.open(frame_paths[0]).convert('RGB').transpose(Image.Transpose.ROTATE_90)
    pixels = torch.from_numpy(np.array(image).copy()).permute(2, 0, 1)[None].float()/255
    with torch.no_grad():
        reference = wrapper(pixels)
        wrapper.export_camera_token = True
        # Position interpolation depends only on the fixed input shape. Bake
        # its exact bicubic result; do not substitute bilinear interpolation.
        vit = wrapper.backbone.pretrained
        position = vit.interpolate_pos_encoding(torch.empty(1, 37*28+1, vit.embed_dim), 518, 392)
        vit.register_buffer('fixed_position', position)
        def fixed_position(self, x, w, h):
            return self.fixed_position.to(x.dtype)
        vit.interpolate_pos_encoding = types.MethodType(fixed_position, vit)
        vit.register_buffer('fixed_grid', vit.position_getter(1, 37, 28, torch.device('cpu')))
        vit.position_getter = lambda batch_size, height, width, device: vit.fixed_grid.expand(batch_size, -1, -1)
        original_uv = wrapper.head._add_pos_embed
        def fixed_uv(self, x, width, height, ratio=.1):
            name = 'fixed_uv_' + '_'.join(str(int(d)) for d in x.shape[1:])
            if not hasattr(self, name):
                assert not torch.jit.is_tracing(), 'Unexpected feature shape during export'
                self.register_buffer(name, original_uv(torch.zeros_like(x), width, height, ratio))
            return x + getattr(self, name)
        wrapper.head._add_pos_embed = types.MethodType(fixed_uv, wrapper.head)
        assert torch.allclose(wrapper(pixels), reference, atol=1e-6, rtol=1e-5)
        traced = torch.jit.trace(wrapper, pixels, check_trace=False)
        frozen = torch.jit.freeze(traced.eval())
    import coremltools as ct
    mlmodel = ct.convert(
        frozen,
        inputs=[ct.ImageType(name='image', shape=pixels.shape, scale=1/255., color_layout=ct.colorlayout.RGB)],
        outputs=[ct.TensorType(name='inverse_depth', dtype=np.float32)],
        minimum_deployment_target=ct.target.iOS17,
        compute_precision=ct.precision.FLOAT16,
        compute_units=ct.ComputeUnit.ALL,
    )
    mlmodel.short_description = 'DA3-Small single-frame depth, LiDAR scale calibration required'
    mlmodel.author = 'ByteDance Seed; Core ML adaptation by MapEverything'
    mlmodel.license = 'Apache-2.0'
    mlmodel.user_defined_metadata.update({
        'source': 'https://huggingface.co/depth-anything/DA3-SMALL',
        'revision': MODEL_REVISION,
        'output_semantics': 'inverse_relative_depth',
        'calibration': 'metric_depth=1/(scale*inverse_depth);offset=0',
        'image_orientation': 'native_sensor_landscape;upright_rotation_inside_graph',
    })
    # Real recorded frames, including different views from both recordings.
    checks = []
    for path in frame_paths[::max(1, len(frame_paths)//8)]:
        image = Image.open(path).convert('RGB').transpose(Image.Transpose.ROTATE_90)
        pixels = torch.from_numpy(np.array(image).copy()).permute(2, 0, 1)[None].float()/255
        with torch.no_grad(): expected = wrapper(pixels).numpy()
        start = time.perf_counter(); actual = mlmodel.predict({'image': image})['inverse_depth']
        elapsed = time.perf_counter()-start
        assert actual.shape == expected.shape == (1, 392, 518)
        assert np.isfinite(actual).all() and (actual > 0).all()
        relative = np.abs(actual-expected)/expected
        check = {'frame': path.name, 'mean_relative_error': float(relative.mean()),
                 'p99_relative_error': float(np.quantile(relative, .99)), 'seconds': elapsed}
        checks.append(check); print(json.dumps(check), flush=True)
        assert relative.mean() < .02 and np.quantile(relative, .99) < .05, check
    mlmodel.save(str(args.out))
    args.out.with_suffix('.validation.json').write_text(json.dumps(checks, indent=2))
    print('Saved and validated', args.out, flush=True)


if __name__ == '__main__': main()
