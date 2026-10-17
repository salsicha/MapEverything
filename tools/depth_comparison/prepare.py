"""Extract read-only rosbridge_json bags without publishing private recordings."""
import argparse, base64, hashlib, io, json, sqlite3
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation

TOPICS = {'/mapping/pose', '/mapping/camera/image/compressed', '/mapping/camera/camera_info',
          '/mapping/pointcloud/lidar', '/mapping/pointcloud/depth_anything', '/mapping/depth_anything/calibration'}

def stamp(m):
    s=m['header']['stamp']; return int(s['sec'])*10**9+int(s['nanosec'])

def cloud(m):
    assert not m['is_bigendian']
    b=base64.b64decode(m['data']); fields={f['name']:f['offset'] for f in m['fields']}
    n=int(m['width'])*int(m['height']); step=m['point_step']
    return np.stack([np.ndarray((n,),'<f4',b,offset=fields[k],strides=(step,)) for k in ['x','y','z']],axis=1)

def prepare(source,output):
    output.mkdir(parents=True,exist_ok=True); topics={t:{} for t in TOPICS}; provenance=[];duplicates=0
    for p in sorted(source.glob('*.db3')):
        provenance.append({'file':p.name,'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'bytes':p.stat().st_size})
        c=sqlite3.connect(f'file:{p.resolve()}?mode=ro',uri=True)
        for name,data in c.execute('select t.name,m.data from messages m join topics t on t.id=m.topic_id'):
            if name not in TOPICS:continue
            m=json.loads(data)['msg'];t=stamp(m)
            if t in topics[name]:duplicates+=1
            topics[name][t]=m
        c.close()
    frames=[];previous=None;session=-1
    for t,m in sorted(topics['/mapping/camera/image/compressed'].items()):
        if previous is None or t-previous>10_000_000_000:session+=1
        previous=t
        if t not in topics['/mapping/camera/camera_info'] or t not in topics['/mapping/pose']:continue
        p=topics['/mapping/pose'][t]['pose'];q=p['orientation'];T=np.eye(4)
        T[:3,:3]=Rotation.from_quat([q[k] for k in ['x','y','z','w']]).as_matrix()
        T[:3,3]=[p['position'][k] for k in ['x','y','z']]
        info=topics['/mapping/camera/camera_info'][t];K=np.array(info['k']).reshape(3,3)
        original=Image.open(io.BytesIO(base64.b64decode(m['data']))).convert('RGB');W,H=original.size
        assert (W,H)==(info['width'],info['height'])
        # Native sensor image is sideways. Rotate both pixels and camera axes.
        Q=np.array([[0,-1,0],[1,0,0],[0,0,1.]])
        E=np.eye(4);E[:3,:3]=Q@np.diag([1,-1,-1])@T[:3,:3].T;E[:3,3]=-E[:3,:3]@T[:3,3]
        Knew=np.array([[K[1,1],0,H-1-K[1,2]],[0,K[0,0],K[0,2]],[0,0,1.]])
        # Pixel-centre aware PIL resize; same 203,056 input pixels for both models.
        target=(392,518);scale=np.array(target)/[H,W]
        Knew[:2]*=scale[:,None];Knew[0,2]+=(scale[0]-1)/2;Knew[1,2]+=(scale[1]-1)/2
        image=original.transpose(Image.Transpose.ROTATE_270).resize(target,Image.Resampling.LANCZOS)
        i=len(frames);name=f'frame-{i:03d}';image.save(output/f'{name}.png')
        lidar=cloud(topics['/mapping/pointcloud/lidar'][t]) if t in topics['/mapping/pointcloud/lidar'] else np.empty((0,3))
        lidar_camera=lidar@E[:3,:3].T+E[:3,3]
        uvz=lidar_camera@Knew.T;uv=uvz[:,:2]/uvz[:,2:]
        ok=np.isfinite(lidar_camera).all(axis=1)&(lidar_camera[:,2]>.2)&(lidar_camera[:,2]<5)&(uv[:,0]>=0)&(uv[:,0]<392)&(uv[:,1]>=0)&(uv[:,1]<518)
        np.savez_compressed(output/f'{name}.npz',K=Knew,E=E,lidar_xyz=lidar[ok],lidar_uv=uv[ok],lidar_depth=lidar_camera[ok,2])
        frames.append({'index':i,'session':session,'stamp_ns':t,'image':f'{name}.png','data':f'{name}.npz','lidar_samples':int(ok.sum())})
    manifest={'bags':provenance,'frames':frames,'sessions':session+1,'duplicate_topic_stamps':duplicates,'image_size':[392,518],
              'orientation':'90 degrees clockwise from native sensor; K and world-to-camera extrinsics transformed consistently',
              'reference':'recorded voxel-filtered LiDAR points, 0.2–5m; confidence maps absent'}
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2))
    print(json.dumps({'frames':len(frames),'sessions':session+1,'lidar_frames':sum(f['lidar_samples']>0 for f in frames),'duplicates':duplicates}))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('output',type=Path);a=p.parse_args();prepare(a.source,a.output)
