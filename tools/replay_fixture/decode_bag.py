"""Decode /Users/alexmoran/Downloads/mapeverything_0.db3 (failing street scan)
into decoded_new.npz, following the scan6 decode.py conventions."""
import sys
import sqlite3, json, base64, os
import numpy as np

BAG = sys.argv[1] if len(sys.argv) > 1 else '/Users/alexmoran/Downloads/mapeverything_0.db3'
OUT = os.path.dirname(os.path.abspath(__file__))

def stamp(h): return h['stamp']['sec'] + h['stamp']['nanosec']*1e-9

poses, clouds, calibs, lidar = [], [], [], []

con = sqlite3.connect(BAG)
tid = {name: i for i, name in con.execute('SELECT id,name FROM topics')}
for (d,) in con.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp', (tid['/mapping/pose'],)):
    m = json.loads(d)['msg']
    p, q = m['pose']['position'], m['pose']['orientation']
    poses.append((stamp(m['header']), [p['x'],p['y'],p['z']], [q['x'],q['y'],q['z'],q['w']]))
for topic, store in (('/mapping/pointcloud/depth_anything', clouds), ('/mapping/pointcloud/lidar', lidar)):
    for (d,) in con.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp', (tid[topic],)):
        m = json.loads(d)['msg']
        raw = base64.b64decode(m['data'])
        step = m['point_step']
        n = m['width']*m['height']
        arr = np.frombuffer(raw, dtype=np.uint8).reshape(n, step)
        xyz = arr[:, :12].copy().view(np.float32).reshape(n, 3)
        # keep ALL rows (clouds are compacted already; preserve publish order)
        store.append(dict(t=stamp(m['header']), pts=xyz))
for (d,) in con.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp', (tid['/mapping/depth_anything/calibration'],)):
    m = json.loads(d)['msg']
    calibs.append(dict(t=stamp(m['header']), scale=m['scale'], offset=m['offset']))
row = con.execute('SELECT data FROM messages WHERE topic_id=? LIMIT 1', (tid['/mapping/camera/camera_info'],)).fetchone()
m = json.loads(row[0])['msg']
K = np.array(m['k']).reshape(3, 3)
res = (m['width'], m['height'])
con.close()

poses.sort(key=lambda x: x[0])
pt = np.array([p[0] for p in poses])
pp = np.array([p[1] for p in poses])
pq = np.array([p[2] for p in poses])
clouds.sort(key=lambda c: c['t']); calibs.sort(key=lambda c: c['t']); lidar.sort(key=lambda c: c['t'])

print('poses', len(poses), 'clouds', len(clouds), 'calibs', len(calibs), 'lidar', len(lidar))
print('pose t range', pt[0], pt[-1])
for i, c in enumerate(clouds):
    cal = min(calibs, key=lambda k: abs(k['t']-c['t']))
    j = np.argmin(np.abs(pt - c['t']))
    c['scale'], c['offset'] = cal['scale'], cal['offset']
    c['calib_dt'] = cal['t']-c['t']
    c['cam'] = pp[j]; c['quat'] = pq[j]; c['pose_dt'] = pt[j]-c['t']
    print(f"cloud {i}: t={c['t']:.3f} n={len(c['pts'])} scale={c['scale']:.4f} offset={c['offset']:.5f} "
          f"calib_dt={c['calib_dt']*1000:.1f}ms pose_dt={c['pose_dt']*1000:.1f}ms cam={np.round(c['cam'],2)}")

np.savez_compressed(os.path.join(OUT, 'decoded_new.npz'),
    n=len(clouds),
    **{f'pts{i}': c['pts'] for i, c in enumerate(clouds)},
    **{f'lpts{i}': c['pts'] for i, c in enumerate(lidar)},
    nlidar=len(lidar),
    lidar_t=np.array([c['t'] for c in lidar]),
    t=np.array([c['t'] for c in clouds]),
    scale=np.array([c['scale'] for c in clouds]),
    offset=np.array([c['offset'] for c in clouds]),
    cam=np.array([c['cam'] for c in clouds]),
    quat=np.array([c['quat'] for c in clouds]),
    K=K, res=np.array(res))
print('saved decoded_new.npz')
