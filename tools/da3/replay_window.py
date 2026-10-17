#!/usr/bin/env python3
"""Replay causal three-view Core ML on bags extracted by depth_comparison/prepare.py.

No future frames, cross-session context, or repeated integration. Mirrors the
app's keyframe gates and 1 Hz opportunity cap. Saves newest outputs only. Bag
images and predictions stay in the local ignored artifact directory.
"""
import argparse
import json
import time
from pathlib import Path
import sys

import coremltools as ct
import numpy as np
from PIL import Image
import torch
from convert_da3_window_coreml import window_inputs


def relative_motion(a, b, data):
    ea = np.load(data/a['data'])['E']; eb = np.load(data/b['data'])['E']
    ca, cb = np.linalg.inv(ea), np.linalg.inv(eb)
    translation = np.linalg.norm(ca[:3,3]-cb[:3,3])
    angle = np.arccos(np.clip((np.trace(ca[:3,:3].T@cb[:3,:3])-1)/2,-1,1))
    return translation, angle


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--data',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--single-model',type=Path,required=True)
    p.add_argument('--window-model',type=Path,required=True)
    args=p.parse_args(); torch.set_num_threads(4)
    manifest=json.loads((args.data/'manifest.json').read_text())
    models={'da3-single':ct.models.MLModel(str(args.single_model)),
            'da3-window':ct.models.MLModel(str(args.window_model))}
    for mode in models: (args.output/'predictions'/mode).mkdir(parents=True,exist_ok=True)
    selected=[]; history=[]; session=None; previous=-np.inf; timings={m:[] for m in models}; windows=[]; validation=[]
    for frame in manifest['frames']:
        if frame['session']!=session:
            session=frame['session']; history=[]; previous=-np.inf
        now=frame['stamp_ns']/1e9
        if now-previous<1: continue
        previous=now; selected.append(frame)
        history=[f for f in history if 0<now-f['stamp_ns']/1e9<=5 and
                 relative_motion(f,frame,args.data)[0]<=1.5 and relative_motion(f,frame,args.data)[1]<=np.pi/4]
        keyframe=not history or (lambda d,a: d>=.04 or a>=np.deg2rad(3))(*relative_motion(history[-1],frame,args.data))
        batch=history+[frame] if keyframe else []
        if keyframe: history=batch[-2:]
        batch=batch if len(batch)==3 else [frame]
        native=Image.open(args.data/frame['image']).convert('RGB').transpose(Image.Transpose.ROTATE_90)
        single_input={'image':native}
        start=time.perf_counter(); single=models['da3-single'].predict(single_input)['inverse_depth']; single_time=time.perf_counter()-start
        if len(batch)==3:
            images,_,poses,_,_=window_inputs(args.data,batch)
            inputs={f'image{i}':im for i,im in enumerate(images)}; inputs['poses']=poses.numpy()
            start=time.perf_counter(); window=models['da3-window'].predict(inputs)['inverse_depth']; window_time=time.perf_counter()-start
        else: window=single; window_time=single_time
        for mode,inv,elapsed in [('da3-single',single,single_time),('da3-window',window,window_time)]:
            # Model output native sensor -> upright evaluation coordinates.
            depth=np.rot90(1/inv[0],k=-1).copy()
            assert depth.shape==(518,392) and np.isfinite(depth).all() and (depth>0).all()
            used=batch if mode=='da3-window' else [frame]
            indices=[b['index'] for b in used]
            assert indices[-1]==frame['index'] and all(b['stamp_ns']<=frame['stamp_ns'] for b in used)
            np.savez_compressed(args.output/'predictions'/mode/f"frame-{frame['index']:03d}.npz",depth=depth,input_indices=indices)
            timings[mode].append({'seconds':elapsed,'input_frames':indices,'saved_frames':[frame['index']]})
        meta=np.load(args.data/frame['data'])
        # Native resized camera coordinates for direct production-Swift validation.
        q=np.eye(4); q[:3,:3]=[[0,1,0],[1,0,0],[0,0,-1]]
        transform=np.linalg.inv(meta['E'])@q
        k=np.array([[meta['K'][1,1],0,meta['K'][1,2]],
                    [0,meta['K'][0,0],391-meta['K'][0,2]],[0,0,1.]])
        import cv2
        inv_upright=np.rot90(window[0],k=-1).copy()
        uv=meta['lidar_uv'].astype(np.float32)
        values=cv2.remap(inv_upright,uv[:,0:1],uv[:,1:2],cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_REPLICATE).ravel() if len(uv) else []
        validation.append({'index':frame['index'], 'session':frame['session'], 'timestamp':now,
            'transform':transform.tolist(), 'intrinsics':k.tolist(),
            'inputIndices':[b['index'] for b in batch],
            'poseEncoding':poses.numpy().flatten().tolist() if len(batch)==3 else None,
            'samples':[[float(v),float(z)] for v,z in zip(values,meta['lidar_depth'])]})
        windows.append({'output':frame['index'],'input_frames':[b['index'] for b in batch]})
        print(json.dumps(windows[-1]),flush=True)
    (args.output/'swift-validation.json').write_text(json.dumps(validation))
    outdata=args.output/'data';outdata.mkdir(parents=True,exist_ok=True)
    for f in selected:
        for key in ['data','image']:
            dst=outdata/f[key]
            if not dst.exists():dst.symlink_to((args.data/f[key]).resolve())
    manifest['frames']=selected
    (outdata/'manifest.json').write_text(json.dumps(manifest,indent=2))
    (args.output/'windows.json').write_text(json.dumps(windows,indent=2))
    for mode in models:
        (args.output/'predictions'/mode/'timings.json').write_text(json.dumps(timings[mode],indent=2))
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'depth_comparison'))
    import evaluate
    evaluate.MODES=['da3-single','da3-window']
    evaluate.evaluate(args.output)
    # Neither export claims native metric depth. Avoid reporting arbitrary-scale
    # 'native' errors as metres. Latency is per newest output, not /3 input views.
    path=args.output/'metrics.json'
    result=json.loads(path.read_text())
    result['summary']=[s for s in result['summary'] if s['variant']=='scaled']
    for s in result['summary']:
        times=timings[s['mode']]
        s['median_seconds_per_newest_output']=float(np.median([t['seconds'] for t in times]))
        del s['median_seconds_per_input_frame']
    result['causal_outputs']=len(windows)
    result['three_view_outputs']=sum(len(w['input_frames'])==3 for w in windows)
    result['single_frame_fallbacks']=len(windows)-result['three_view_outputs']
    path.write_text(json.dumps(result,indent=2)+'\n')

if __name__=='__main__':main()
