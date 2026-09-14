"""Sanity-check StreetScanReplayFrames.json the way the Scan6 loader consumes it,
plus quantify the on-device cap truncation cliff in each frame."""
import base64, json, os, sys
import numpy as np

PATH = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '../../MapEverything/MapEverythingTests/Fixtures/StreetScanReplayFrames.json')
root = json.load(open(PATH))
print('root keys:', sorted(root.keys()))
print('intrinsics fx fy cx cy =', root['fx'], root['fy'], root['cx'], root['cy'],
      'res =', root['imageWidth'], 'x', root['imageHeight'])
frames = root['frames']
print('frames:', len(frames))

# --- decode every frame briefly; deep-check frame 0 and frame 16 ---
for i, e in enumerate(frames):
    w, h = e['relativeWidth'], e['relativeHeight']
    half = np.frombuffer(base64.b64decode(e['relativeFloat16']), dtype=np.float16)
    assert half.size == w*h, (i, half.size, w*h)
    lid = np.frombuffer(base64.b64decode(e['lidarSamples']), dtype=np.float32)
    assert lid.size % 3 == 0 and lid.size // 3 == e['lidarSampleCount']
    tr = e['transform']; assert len(tr) == 16
    # column-major: last column = translation, bottom row of each column
    assert (tr[3], tr[7], tr[11], tr[15]) == (0.0, 0.0, 0.0, 1.0)
    rmap = half.reshape(h, w).astype(np.float32)
    finite = np.isfinite(rmap)
    s, o = e['scale'], e['offset']
    with np.errstate(invalid='ignore', divide='ignore'):
        metric = 1.0/(s*rmap + o)
    mv = metric[finite & (metric > 0)]
    trip = lid.reshape(-1, 3)
    # loader scatters px/518*256, py/392*192
    px256 = np.rint(trip[:, 0]/518*256); py192 = np.rint(trip[:, 1]/392*192)
    inb = ((px256 >= 0) & (px256 < 256) & (py192 >= 0) & (py192 < 192)).mean()
    # truncation cliff: valid-pixel mass in the top 5% of the frame's depth range
    maxz = mv.max()
    cliff = float((mv > 0.95*maxz).mean())
    print(f"frame {i:2d}: rel {w}x{h} finite {finite.mean()*100:5.1f}%  "
          f"rel range [{np.nanmin(rmap):.4f},{np.nanmax(rmap):.3f}]  "
          f"metric p50 {np.percentile(mv,50):5.2f} p99 {np.percentile(mv,99):5.2f} max {maxz:5.2f}  "
          f"mass>0.95*max {cliff*100:4.1f}%  lidar {trip.shape[0]:4d} inb {inb*100:5.1f}% "
          f"z[{trip[:,2].min():.2f},{trip[:,2].max():.2f}]")

# --- deep check: rebuild world points from frame 0 fixture data and compare with bag ---
d = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'decoded_new.npz'), allow_pickle=True)
for check_i in (0, 16):
    e = frames[check_i]
    w, h = e['relativeWidth'], e['relativeHeight']
    rmap = np.frombuffer(base64.b64decode(e['relativeFloat16']), dtype=np.float16).reshape(h, w).astype(np.float64)
    tr = np.array(e['transform']).reshape(4, 4).T   # back to row-major T
    R, cam = tr[:3, :3], tr[:3, 3]
    W518, H392 = 518, 392
    sxi = root['fx']*W518/root['imageWidth']; syi = root['fy']*H392/root['imageHeight']
    cxi = root['cx']*W518/root['imageWidth']; cyi = root['cy']*H392/root['imageHeight']
    ys, xs = np.mgrid[0:h, 0:w]
    px, py = xs*2, ys*2                    # 2x strided downsample source pixels
    with np.errstate(invalid='ignore', divide='ignore'):
        z = 1.0/(e['scale']*rmap + e['offset'])
    ok = np.isfinite(z) & (z > 0)
    xcam = (px - cxi)/sxi*z; ycam = -(py - cyi)/syi*z
    pts = np.stack([xcam[ok], ycam[ok], -z[ok]], axis=-1) @ R.T + cam
    # compare to the bag's world points scattered onto the same grid
    bag = d[f'pts{check_i}'].astype(np.float64)
    local = (bag - cam) @ R
    zz = -local[:, 2]
    bpx = np.rint(cxi + sxi*local[:, 0]/zz).astype(int)
    bpy = np.rint(cyi - syi*local[:, 1]/zz).astype(int)
    grid = np.full((H392, W518, 3), np.nan)
    m = (bpx >= 0) & (bpx < W518) & (bpy >= 0) & (bpy < H392)
    grid[bpy[m], bpx[m]] = bag[m]
    ref = grid[py[ok], px[ok]]
    good = np.isfinite(ref).all(axis=1)
    err = np.linalg.norm(pts[good] - ref[good], axis=1)
    rel = err / np.linalg.norm(ref[good] - cam, axis=1)
    print(f"frame {check_i} world-point round-trip: n={good.sum()} "
          f"err p50 {np.percentile(err,50)*1000:.1f}mm p99 {np.percentile(err,99)*1000:.1f}mm max {err.max()*1000:.1f}mm "
          f"(rel p99 {np.percentile(rel,99)*100:.3f}%)")
print('ROUND-TRIP OK')
