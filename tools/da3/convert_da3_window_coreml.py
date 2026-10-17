#!/usr/bin/env python3
"""Export a causal, pose-conditioned three-view DA3 depth branch.

Uses the same pinned source/weights/environment as convert_da3_coreml.py.
Inputs: three native 518x392 RGB images, oldest first; normalized camera-to-world
pose encodings [translation, quaternion XYZW (w>=0), vertical FOV, horizontal FOV]
in the upright first-camera coordinate system. See DA3WindowGeometry.swift.
Only the newest view runs through the depth decoder. No predicted-pose alignment:
metric scale is supplied by same-frame LiDAR, as in the single-frame app.
"""
import argparse
import json
import time
import types
from pathlib import Path

import numpy as np
from PIL import Image
import torch
from safetensors.torch import load_file
from convert_da3_coreml import MODEL_REVISION


def load_wrapper(weights_path):
    from depth_anything_3.cfg import create_object, load_config
    from depth_anything_3.registry import MODEL_REGISTRY
    model = create_object(load_config(MODEL_REGISTRY['da3-small'])).eval()
    weights = {k.removeprefix('model.'): v for k, v in load_file(weights_path).items()}
    aliases = {id(p): weights[n] for n, p in model.named_parameters(remove_duplicate=False) if n in weights}
    for n, p in model.named_parameters(remove_duplicate=False):
        if n not in weights and id(p) in aliases: weights[n] = aliases[id(p)]
    model.load_state_dict(weights, strict=True)

    class Window(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.backbone, self.head, self.cam_enc = model.backbone, model.head, model.cam_enc
            self.register_buffer('mean', torch.tensor([.485, .456, .406]).view(1, 1, 3, 1, 1))
            self.register_buffer('std', torch.tensor([.229, .224, .225]).view(1, 1, 3, 1, 1))

        def forward(self, image0, image1, image2, poses):
            native = torch.stack([image0, image1, image2], dim=1)
            x = (native.transpose(-2, -1).flip(-1) - self.mean) / self.std
            enc = self.cam_enc
            token = enc.trunk_norm(enc.trunk(enc.token_norm(enc.pose_branch(poses))))
            features, _ = self.backbone(x, cam_token=token, export_feat_layers=[], ref_view_strategy='first')
            # Cross-view attention already ran. DPT is independent per view.
            newest = [(f[0][:, -1:], f[1][:, -1:]) for f in features]
            depth = self.head(newest, 518, 392, patch_start_idx=0)['depth'][:, 0]
            return (1 / depth).flip(-1).transpose(-2, -1)

    return model, Window().eval()


def window_inputs(data, batch):
    from depth_anything_3.model.utils.transform import extri_intri_to_pose_encoding
    from depth_anything_3.utils.geometry import affine_inverse
    images = [Image.open(data / f['image']).convert('RGB').transpose(Image.Transpose.ROTATE_90) for f in batch]
    pixels = [torch.from_numpy(np.array(im).copy()).permute(2, 0, 1)[None].float()/255 for im in images]
    meta = [np.load(data / f['data']) for f in batch]
    ext = torch.tensor(np.stack([m['E'] for m in meta]), dtype=torch.float32)[None]
    intr = torch.tensor(np.stack([m['K'] for m in meta]), dtype=torch.float32)[None]
    # Exactly the official API normalization, including the 10 cm scale floor.
    ext = ext @ affine_inverse(ext[:, :1])
    distances = affine_inverse(ext)[..., :3, 3].norm(dim=-1)
    ext[..., :3, 3] /= distances.median().clamp(min=.1)
    poses = extri_intri_to_pose_encoding(affine_inverse(ext), intr, (518, 392))
    return images, pixels, poses, ext, intr



def write_synthetic_fixture(model, output):
    """Independent full-network reference used by the app's RGB/pose test."""
    from scipy.spatial.transform import Rotation
    from depth_anything_3.model.utils.transform import extri_intri_to_pose_encoding
    from depth_anything_3.utils.geometry import affine_inverse
    q=np.array([[0,1,0],[1,0,0],[0,0,-1.]])
    transforms=[]; extrinsics=[]; pixels=[]
    y,x=np.mgrid[:392,:518]
    for i in range(3):
        t=np.eye(4)
        t[:3,:3]=Rotation.from_rotvec(np.array([.02,.05,-.01])*i).as_matrix()
        t[:3,3]=[.1*i,.02*i,-.03*i]; transforms.append(t)
        e=np.eye(4); e[:3,:3]=q@t[:3,:3].T; e[:3,3]=-e[:3,:3]@t[:3,3]
        extrinsics.append(e)
        rgb=np.stack([(x*255//517+i*11)%256,(y*255//391+i*7)%256,(x+y+i*13)%256],-1).astype('uint8')
        pixels.append(torch.tensor(rgb).permute(2,0,1)[None].float()/255)
    ext=torch.tensor(np.stack(extrinsics),dtype=torch.float32)[None]
    ext=ext@affine_inverse(ext[:,:1])
    ext[...,:3,3]/=affine_inverse(ext)[...,:3,3].norm(dim=-1).median().clamp(min=.1)
    intr=torch.tensor([[500.,0,195.],[0,500.,259.],[0,0,1.]])[None,None].expand(1,3,3,3)
    poses=extri_intri_to_pose_encoding(affine_inverse(ext),intr,(518,392))
    rgb=torch.stack(pixels,1).transpose(-2,-1).flip(-1)
    mean=torch.tensor([.485,.456,.406]).view(1,1,3,1,1)
    std=torch.tensor([.229,.224,.225]).view(1,1,3,1,1)
    with torch.no_grad():
        depth=model((rgb-mean)/std,extrinsics=ext,intrinsics=intr)['depth'][:,-1]
        inv=(1/depth).flip(-1).transpose(-2,-1).numpy()[0]
    points=[{'x':x,'y':y,'inverseDepth':float(inv[y,x])} for y in [40,195,350] for x in [40,258,475]]
    output.write_text(json.dumps({'transforms':[t.tolist() for t in transforms],
        'poseEncoding':poses.numpy().flatten().tolist(),'pixels':points},indent=2)+'\n')


def bake_static(wrapper, inputs):
    vit = wrapper.backbone.pretrained
    vit.register_buffer('fixed_position', vit.interpolate_pos_encoding(torch.empty(1, 37*28+1, vit.embed_dim), 518, 392))
    vit.interpolate_pos_encoding = types.MethodType(lambda self, x, w, h: self.fixed_position.to(x.dtype), vit)
    vit.register_buffer('fixed_grid', vit.position_getter(1, 37, 28, torch.device('cpu')))
    vit.position_getter = lambda batch_size, height, width, device: vit.fixed_grid.expand(batch_size, -1, -1)
    original_uv = wrapper.head._add_pos_embed
    def fixed_uv(self, x, width, height, ratio=.1):
        name = 'fixed_uv_' + '_'.join(str(int(d)) for d in x.shape[1:])
        if not hasattr(self, name):
            assert not torch.jit.is_tracing()
            self.register_buffer(name, original_uv(torch.zeros_like(x), width, height, ratio))
        return x + getattr(self, name)
    wrapper.head._add_pos_embed = types.MethodType(fixed_uv, wrapper.head)
    wrapper(*inputs)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--weights', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--frames', type=Path, required=True)
    p.add_argument('--synthetic-fixture', type=Path, help='Optional independent Swift test reference JSON')
    args = p.parse_args()
    torch.set_num_threads(4); torch.manual_seed(0)
    model, wrapper = load_wrapper(args.weights)
    if args.synthetic_fixture: write_synthetic_fixture(model, args.synthetic_fixture)
    frames = json.loads((args.frames/'manifest.json').read_text())['frames']
    images, pixels, poses, ext, intr = window_inputs(args.frames, frames[:3])
    with torch.no_grad():
        reference = wrapper(*pixels, poses)
        x = (torch.stack(pixels, 1).transpose(-2, -1).flip(-1)-wrapper.mean)/wrapper.std
        official = model(x, extrinsics=ext, intrinsics=intr)['depth'][:, -1]
        official = (1/official).flip(-1).transpose(-2, -1)
        assert torch.allclose(reference, official, atol=1e-5, rtol=1e-4), (reference-official).abs().max()
        bake_static(wrapper, (*pixels, poses))
        assert torch.allclose(wrapper(*pixels, poses), reference, atol=1e-6, rtol=1e-5)
        traced = torch.jit.freeze(torch.jit.trace(wrapper, (*pixels, poses), check_trace=False).eval())
    import coremltools as ct
    mlmodel = ct.convert(traced,
        inputs=[ct.ImageType(name=f'image{i}', shape=p.shape, scale=1/255., color_layout=ct.colorlayout.RGB) for i,p in enumerate(pixels)] +
               [ct.TensorType(name='poses', shape=(1,3,9), dtype=np.float32)],
        outputs=[ct.TensorType(name='inverse_depth', dtype=np.float32)],
        minimum_deployment_target=ct.target.iOS17, compute_precision=ct.precision.FLOAT16)
    mlmodel.short_description = 'DA3-Small three-view causal window; newest depth; LiDAR scale required'
    mlmodel.author = 'ByteDance Seed; Core ML adaptation by MapEverything'
    mlmodel.license = 'Apache-2.0'
    mlmodel.user_defined_metadata.update({'revision': MODEL_REVISION, 'window_size': '3',
        'output_view': 'newest', 'pose_encoding': 'normalized_upright_c2w_t_xyz_q_xyzw_fov_hw',
        'calibration': 'metric_depth=1/(scale*inverse_depth);offset=0'})
    checks=[]
    for start in range(0,len(frames)-2,5):
        batch=frames[start:start+3]
        if len({f['session'] for f in batch})!=1: continue
        images,pixels,poses,_,_=window_inputs(args.frames,batch)
        with torch.no_grad(): expected=wrapper(*pixels,poses).numpy()
        inputs={f'image{i}':im for i,im in enumerate(images)}; inputs['poses']=poses.numpy()
        t=time.perf_counter(); actual=mlmodel.predict(inputs)['inverse_depth']; elapsed=time.perf_counter()-t
        assert actual.shape==expected.shape==(1,392,518)
        assert np.isfinite(actual).all() and (actual>0).all()
        rel=np.abs(actual-expected)/expected
        check={'frames':[f['index'] for f in batch], 'mean_relative_error':float(rel.mean()),
               'p99_relative_error':float(np.quantile(rel,.99)), 'seconds':elapsed}
        print(json.dumps(check),flush=True); checks.append(check)
        assert rel.mean()<.02 and np.quantile(rel,.99)<.05,check
    mlmodel.save(str(args.out))
    args.out.with_suffix('.validation.json').write_text(json.dumps(checks,indent=2)+'\n')
    print('Saved and validated',args.out,flush=True)

if __name__=='__main__': main()
