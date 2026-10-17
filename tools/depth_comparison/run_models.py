"""Run pinned official small models locally; preserve raw, unfiltered depth.

PYTHONPATH must contain the DA3 src directory and UniDepth checkout root.
DA3's optional export import must be made lazy (see runtime patch artifact).
"""
import argparse
import json
import platform
import time
from pathlib import Path

import numpy as np
import torch
from PIL import Image


def sync(device):
    if device == 'mps':
        torch.mps.synchronize()
    elif device == 'cuda':
        torch.cuda.synchronize()


def run(args):
    torch.manual_seed(0)
    torch.set_num_threads(4)
    manifest = json.loads((args.data / 'manifest.json').read_text())
    frames = manifest['frames'][:args.limit or None]
    dest = args.output / args.mode
    dest.mkdir(parents=True, exist_ok=True)
    if args.mode == 'unidepth':
        from unidepth.models import UniDepthV2
        model = UniDepthV2.from_pretrained(str(args.weights / 'unidepth-v2-small')).to(args.device).eval()
        model.resolution_level = 0
        model_id = 'unidepth-v2-small'
    else:
        from depth_anything_3.api import DepthAnything3

        class Float32DA3(DepthAnything3):
            # Official API enables FP16 automatically. Use FP32 on both models
            # to avoid conflating model quality with Apple mixed-precision effects.
            @torch.inference_mode()
            def forward(self, image, extrinsics=None, intrinsics=None,
                        export_feat_layers=None, infer_gs=False, use_ray_pose=False,
                        ref_view_strategy='middle'):
                return self.model(image, extrinsics, intrinsics, export_feat_layers,
                                  infer_gs, use_ray_pose, ref_view_strategy)

        model = Float32DA3.from_pretrained(str(args.weights / 'da3-small')).to(args.device).eval()
        model_id = 'da3-small'

    def predict(batch):
        images = [np.array(Image.open(args.data / f['image']).convert('RGB')) for f in batch]
        meta = [np.load(args.data / f['data']) for f in batch]
        if args.mode == 'unidepth':
            with torch.inference_mode():
                out = model.infer(torch.from_numpy(images[0]).permute(2, 0, 1),
                                  torch.tensor(meta[0]['K'], dtype=torch.float32)[None])
            return out['depth'][:, 0].cpu().numpy(), out['confidence'][:, 0].cpu().numpy()
        kwargs = {}
        if args.mode in ('da3-multi', 'da3-scene'):
            kwargs = {'intrinsics': np.stack([m['K'] for m in meta]),
                      'extrinsics': np.stack([m['E'] for m in meta]),
                      'align_to_input_ext_scale': True}
        out = model.inference(images, process_res=518, ref_view_strategy='middle', **kwargs)
        return out.depth, out.conf

    batches = []
    if args.mode in ('da3-multi', 'da3-scene'):
        for session in sorted({f['session'] for f in frames}):
            sf = [f for f in frames if f['session'] == session]
            if len(sf) < 3:
                raise ValueError('Pose-conditioned comparison requires at least three frames')
            if args.mode == 'da3-scene':
                batches.append(sf)
                continue
            for start in range(0, len(sf), 5):
                batches.append(sf[min(start, max(0, len(sf) - 5)):min(start + 5, len(sf))])
    else:
        batches = [[f] for f in frames]
    sync(args.device)
    start = time.perf_counter()
    predict(batches[0])
    sync(args.device)
    warmup = time.perf_counter() - start
    timings = []
    for batch_id, batch in enumerate(batches):
        pending = [f for f in batch if not (dest / f"frame-{f['index']:03d}.npz").exists()]
        if not pending:
            continue
        sync(args.device)
        start = time.perf_counter()
        depth, confidence = predict(batch)
        sync(args.device)
        elapsed = time.perf_counter() - start
        assert depth.shape == (len(batch), 518, 392), depth.shape
        for j, f in enumerate(batch):
            if f not in pending:
                continue
            np.savez_compressed(dest / f"frame-{f['index']:03d}.npz",
                                depth=depth[j].astype(np.float32),
                                confidence=confidence[j].astype(np.float32),
                                window=batch_id, input_indices=[b['index'] for b in batch])
        timings.append({'batch': batch_id, 'input_frames': [f['index'] for f in batch],
                        'saved_frames': [f['index'] for f in pending], 'seconds': elapsed})
        (dest / 'timings.json').write_text(json.dumps(timings, indent=2))
        print(json.dumps({'mode': args.mode, 'batch': batch_id, 'seconds': round(elapsed, 3),
                          'frames': [f['index'] for f in pending],
                          'finite_fraction': float(np.isfinite(depth).mean()),
                          'depth_percentiles': np.nanpercentile(depth, [1, 50, 99]).tolist()}), flush=True)
    provenance = {'model': json.loads((args.weights / model_id / 'provenance.json').read_text()),
                  'mode': args.mode, 'dtype': 'float32', 'device': args.device,
                  'torch': torch.__version__, 'platform': platform.platform(),
                  'parameters': sum(p.numel() for p in model.parameters()),
                  'warmup_seconds': warmup, 'input_size': [392, 518],
                  'notes': 'No confidence threshold or depth clipping. DA3 multi uses five-frame windows; DA3 scene uses a complete recording. Both use recorded metric poses.'}
    (dest / 'provenance.json').write_text(json.dumps(provenance, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--data', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--weights', type=Path, required=True)
    p.add_argument('--mode', choices=['unidepth', 'da3-single', 'da3-multi', 'da3-scene'], required=True)
    p.add_argument('--device', default='mps')
    p.add_argument('--limit', type=int)
    run(p.parse_args())
