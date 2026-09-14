"""Build StreetScanReplayFrames.json from decoded_new.npz with the EXACT schema
the Scan6ReplayTests.swift loader parses. Verifies pixel identity per frame by
projection (clouds are compacted: no NaN rows, counts < 518*392), rebuilds the
relative map by inverting the recorded calibration, and reports scene-extent
ground truth (depth bands under the recorded on-device calibration)."""
import base64, json, os, sys
import numpy as np

SCRATCH = os.path.dirname(os.path.abspath(__file__))
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    os.path.dirname(__file__),
    '../../MapEverything/MapEverythingTests/Fixtures/StreetScanReplayFrames.json')

d = np.load(os.path.join(SCRATCH, 'decoded_new.npz'), allow_pickle=True)
n = int(d['n'])
W, H = 518, 392
K = d['K']; res = tuple(int(v) for v in d['res'])
print('K=', K.tolist(), 'res=', res)

def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])

sx, sy = W/res[0], H/res[1]
fx, fy, cx, cy = K[0,0]*sx, K[1,1]*sy, K[0,2]*sx, K[1,2]*sy

BANDS = [(0, 8), (8, 12), (12, 18), (18, 30), (30, np.inf)]
frames = []
used_lidar = set()
print(f"{'i':>2} {'pts':>7} {'inb':>7} {'dup':>5} {'mono%':>6} {'resid p50':>9} {'p99':>7} {'max':>7} | bands (0,8](8,12](12,18](18,30] >30 | p99z maxz valid%")
for i in range(n):
    pts = d[f'pts{i}'].astype(np.float64)
    cam = d['cam'][i]
    R = quat_to_R(d['quat'][i])            # camera-to-world
    scale, offset = float(d['scale'][i]), float(d['offset'][i])
    t = float(d['t'][i])

    local = (pts - cam) @ R                # world -> camera
    z = -local[:, 2]                       # ARKit forward = -Z
    finite = np.isfinite(local).all(axis=1) & (z > 0.05)
    with np.errstate(invalid='ignore', divide='ignore'):
        pxf = cx + fx*local[:, 0]/z
        pyf = cy - fy*local[:, 1]/z
        rvals = (1.0/np.maximum(z, 1e-6) - offset) / scale
    px = np.rint(pxf).astype(np.int64)
    py = np.rint(pyf).astype(np.int64)
    keep = finite & (px >= 0) & (px < W) & (py >= 0) & (py < H)

    # --- raster/pixel-identity verification ---
    resid = np.hypot(pxf[keep]-px[keep], pyf[keep]-py[keep])
    r50, r99, rmax = (np.percentile(resid, [50, 99]).tolist() + [float(resid.max())]) if resid.size else (np.nan,)*3
    lin = py[keep]*W + px[keep]
    dup = int(len(lin) - len(np.unique(lin)))
    mono = float(np.mean(np.diff(lin) > 0))*100 if len(lin) > 1 else np.nan  # % strictly raster-increasing
    inb = int(keep.sum())

    # --- rebuild relative map on full grid ---
    r = np.full((H, W), np.nan, dtype=np.float32)
    r[py[keep], px[keep]] = rvals[keep]

    # --- scene-extent ground truth under the RECORDED calibration ---
    valid = np.isfinite(r)
    with np.errstate(invalid='ignore', divide='ignore'):
        metric = 1.0 / (scale*r + offset)
    mv = metric[valid & (metric > 0)]
    fracs = [float(((mv > a) & (mv <= b)).mean()) for a, b in BANDS]
    p99z = float(np.percentile(mv, 99)); maxz = float(mv.max())
    validfrac = float(valid.mean())

    # --- downsample 2x, float16, like scan6 ---
    r_ds = r[::2, ::2].astype(np.float16)   # 196x259

    # --- nearest real LiDAR cloud -> sparse (px, py, z) on 518x392 grid ---
    lidx = int(np.argmin(np.abs(d['lidar_t'] - t)))
    used_lidar.add(lidx)
    lid_dt = abs(float(d['lidar_t'][lidx]) - t)
    lp = d[f'lpts{lidx}'].astype(np.float64)
    ll = (lp - cam) @ R
    lz = -ll[:, 2]
    ok = lz > 0.05
    lpx = cx + fx*ll[ok, 0]/lz[ok]
    lpy = cy - fy*ll[ok, 1]/lz[ok]
    linb = (lpx >= 0) & (lpx < W) & (lpy >= 0) & (lpy < H)
    samples = np.stack([lpx[linb], lpy[linb], lz[ok][linb]], axis=1).astype(np.float32)

    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = cam
    frames.append({
        't': t,
        'scale': scale, 'offset': offset,
        'transform': [float(v) for v in T.T.flatten()],   # column-major for simd_float4x4
        'relativeWidth': int(r_ds.shape[1]), 'relativeHeight': int(r_ds.shape[0]),
        'relativeFloat16': base64.b64encode(r_ds.tobytes()).decode(),
        'lidarSamples': base64.b64encode(samples.tobytes()).decode(),
        'lidarSampleCount': int(samples.shape[0]),
    })
    print(f"{i:>2} {len(pts):>7} {inb:>7} {dup:>5} {mono:>6.2f} {r50:>9.4f} {r99:>7.4f} {rmax:>7.4f} | "
          + ' '.join(f'{f*100:5.1f}' for f in fracs)
          + f" | {p99z:5.2f} {maxz:5.2f} {validfrac*100:5.1f}%  lidar[{lidx}] n={samples.shape[0]} dt={lid_dt*1000:.0f}ms")

dropped = sorted(set(range(int(d['nlidar']))) - used_lidar)
print('lidar clouds never paired (dropped):', dropped,
      'at t-t0 =', [round(float(d['lidar_t'][j]-d['t'][0]), 2) for j in dropped])

out = {
    'imageWidth': res[0], 'imageHeight': res[1],
    'fx': float(K[0, 0]), 'fy': float(K[1, 1]), 'cx': float(K[0, 2]), 'cy': float(K[1, 2]),
    'relativeGridNote': 'relative maps downsampled 2x from 518x392; float16 row-major',
    'lidarNote': 'sparse (px_518grid, py_392grid, metric_z) float32 triplets from nearest real LiDAR cloud',
    'frames': frames,
}
os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, 'w') as f:
    json.dump(out, f)
print('wrote', OUT, os.path.getsize(OUT)//1024, 'KB')
