"""Create a local HTML report, depth comparisons, and colored PLY point clouds."""
import argparse
import html
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm
import numpy as np
from PIL import Image

from evaluate import MODES, unproject

LABELS = {'da3-single': 'DA3 single image', 'da3-multi': 'DA3 five views',
          'da3-scene': 'DA3 entire recording', 'unidepth': 'UniDepth v2'}


def ply(path, xyz, rgb):
    records = np.empty(len(xyz), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                                       ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')])
    for j, k in enumerate(['x', 'y', 'z']): records[k] = xyz[:, j]
    for j, k in enumerate(['red', 'green', 'blue']): records[k] = rgb[:, j]
    with path.open('wb') as f:
        f.write(('ply\nformat binary_little_endian 1.0\nelement vertex '+str(len(xyz))+
                 '\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n').encode())
        f.write(records.tobytes())


def report(root):
    result = json.loads((root / 'metrics.json').read_text())
    manifest = json.loads((root / 'data/manifest.json').read_text())
    frames = manifest['frames']; scales = json.loads((root / 'scales.json').read_text())
    figures = root / 'figures'; figures.mkdir(exist_ok=True)
    clouds = root / 'pointclouds'; clouds.mkdir(exist_ok=True)
    norm = LogNorm(.5, 30); cmap = plt.get_cmap('turbo')
    selected = [8, 18, 34, 41]
    overview, axs = plt.subplots(4, 5, figsize=(13, 14), layout='constrained')
    available = []
    for f in frames:
        i = f['index']
        if any(scales[mode][i] is None for mode in MODES): continue
        rgb = np.array(Image.open(root / 'data' / f['image']))
        imgs = [rgb]
        for mode in MODES:
            z = np.load(root / 'predictions' / mode / f'frame-{i:03d}.npz')['depth'] * scales[mode][i]
            imgs.append((cmap(norm(z))[..., :3]*255).astype(np.uint8))
        canvas = Image.new('RGB', (392*5, 518))
        for j, im in enumerate(imgs): canvas.paste(Image.fromarray(im), (j*392, 0))
        canvas.save(figures / f'frame-{i:03d}.jpg', quality=90)
        available.append(i)
        if i in selected:
            row = selected.index(i)
            for j, im in enumerate(imgs):
                axs[row, j].imshow(im); axs[row, j].axis('off')
                if row == 0: axs[row, j].set_title((['Recorded RGB']+[LABELS[m] for m in MODES])[j], fontsize=10)
                if j == 0: axs[row, j].set_title(f'Recording {f["session"]+1}, frame {i}', fontsize=10)
    bar = overview.colorbar(plt.cm.ScalarMappable(norm=norm, cmap=cmap), ax=axs, location='bottom', shrink=.65, pad=.02)
    bar.set_ticks([.5, 1, 2, 5, 10, 20, 30]); bar.set_ticklabels(['0.5', '1', '2', '5', '10', '20', '30'])
    bar.set_label('LiDAR-calibrated depth in metres; shared logarithmic color scale, saturated outside 0.5–30m')
    overview.savefig(figures / 'depth-comparison.png', dpi=140); plt.close(overview)

    fig, axes = plt.subplots(2, 3, figsize=(16, 10), subplot_kw={'projection': '3d'}, layout='constrained')
    cloud_modes = ['da3-multi', 'da3-scene', 'unidepth']
    for session in [0, 1]:
        session_frames = [f for f in frames if f['session'] == session]
        centers = np.array([np.linalg.inv(np.load(root / 'data' / f['data'])['E'])[:3, 3] for f in session_frames])
        center = centers.mean(axis=0)
        for col, mode in enumerate(cloud_modes):
            all_xyz, all_rgb = [], []
            for f in session_frames:
                i = f['index']; scale = scales[mode][i]
                if scale is None: continue
                z = np.load(root / 'predictions' / mode / f'frame-{i:03d}.npz')['depth'] * scale
                meta = np.load(root / 'data' / f['data']); rgb = np.array(Image.open(root / 'data' / f['image']))
                y, x = np.mgrid[0:518:6, 0:392:6]; depth = z[y, x].ravel()
                valid = np.isfinite(depth) & (depth > .2) & (depth < 30)
                xyz = unproject(np.c_[x.ravel(), y.ravel()][valid], depth[valid], meta)
                all_xyz.append(xyz); all_rgb.append(rgb[y, x].reshape(-1, 3)[valid])
            xyz = np.concatenate(all_xyz); rgb = np.concatenate(all_rgb)
            ply(clouds / f'recording-{session+1}-{mode}.ply', xyz, rgb)
            ax = axes[session, col]
            # Fixed physical bounds and same view for all models in each recording.
            # This is direct accumulation, without fusion, confidence culling, or optimization.
            show = (np.abs(xyz-center) < np.array([12, 8, 12])).all(axis=1)
            ax.scatter(xyz[show, 0], xyz[show, 2], xyz[show, 1], c=rgb[show]/255., s=.5, depthshade=False, rasterized=True)
            ax.set(xlim=(center[0]-12, center[0]+12), ylim=(center[2]-12, center[2]+12), zlim=(center[1]-8, center[1]+8))
            ax.view_init(elev=25, azim=-65); ax.set_box_aspect((24, 24, 16))
            ax.set_title(f'Recording {session+1} · {LABELS[mode]}', fontsize=11)
            ax.set_xlabel('World X (m)'); ax.set_ylabel('World Z (m)'); ax.set_zlabel('World Y (m)')
    fig.savefig(figures / 'accumulated-clouds.png', dpi=160, bbox_inches='tight', pad_inches=.3); plt.close(fig)

    summary = result['summary']; calibrated = [r for r in summary if r['variant'] == 'scaled']
    table = ''.join(f'<tr><td>{LABELS[r["mode"]]}</td><td>{100*r["absrel"]:.1f}%</td><td>{100*r["mae_m"]:.1f} cm</td><td>{100*r["pair_median_relative_distance"]:.1f}%</td><td>{1000*r["median_seconds_per_input_frame"]:.0f} ms</td></tr>' for r in calibrated)
    native = ''.join(f'<tr><td>{LABELS[r["mode"]]}</td><td>{100*r["absrel"]:.1f}%</td><td>{100*r["sessions"]["0"]["absrel"]:.1f}%</td><td>{100*r["sessions"]["1"]["absrel"]:.1f}%</td></tr>' for r in summary if r['variant'] == 'native')
    session_table = ''.join(f'<tr><td>{LABELS[r["mode"]]}</td><td>{100*r["sessions"]["0"]["absrel"]:.1f}%</td><td>{100*r["sessions"]["1"]["absrel"]:.1f}%</td></tr>' for r in calibrated)
    prov = {m: json.loads((root / 'predictions' / m / 'provenance.json').read_text()) for m in MODES}
    provenance = html.escape(json.dumps({'models': prov, 'source_revisions': json.loads((root / 'source-revisions.json').read_text()), 'bags': manifest['bags']}, indent=2))
    photo = result['photogrammetry']
    options = ''.join(f'<option value="{i:03d}">Recording {frames[i]["session"]+1}, frame {i}</option>' for i in available)
    links = ''.join(f'<li><a href="pointclouds/{p.name}">{p.stem}</a></li>' for p in sorted(clouds.glob('*.ply')))
    body = f'''<!doctype html><html lang="en"><meta charset="utf-8"><title>DA3 vs UniDepth v2 · Recorded scans</title>
<style>body{{max-width:1150px;margin:40px auto;padding:0 24px;font:16px/1.55 system-ui;color:#172333;background:#fafbfc}}h1{{font-size:34px}}h2{{margin-top:36px}}table{{width:100%;border-collapse:collapse;background:white}}th,td{{text-align:left;padding:11px;border-bottom:1px solid #d8dfe6}}th{{background:#e9eff5}}img{{width:100%;height:auto}}.note{{background:#e9eff5;padding:18px;border-radius:10px}}pre{{white-space:pre-wrap;font-size:12px}}select{{padding:10px;font:inherit}}.labels{{display:grid;grid-template-columns:repeat(5,1fr);font-size:13px;text-align:center}}a{{color:#1764ac}}</style>
<h1>DA3 vs UniDepth v2 on your recorded scans</h1><p>Local benchmark · 16 October 2026 · Apple M5, 32 GB · 5 bag files, 2 recordings, 49 camera frames.</p>
<p class="note"><b>Finding:</b> DA3 is the stronger overall candidate in these recordings after LiDAR calibration. UniDepth is competitive on the first recording, but less reliable on the second. Neither model's native scale is accurate enough here to replace calibration. These results do not establish distant-surface accuracy or iPhone performance.</p>
<h2>LiDAR-calibrated comparison</h2><p>Lower is better in every column. 47 frames have usable LiDAR; 22,154 withheld reference points are evaluated. Errors are averaged equally across frames. Every model receives the same 392 × 518 RGB images; model parameters are approximately 34.3 million for DA3-Small and 34.2 million for UniDepth v2 Small.</p>
<table><tr><th>Model</th><th>Relative depth error</th><th>Absolute depth error</th><th>Cross-view disagreement</th><th>Time / input frame</th></tr>{table}</table>
<p>Cross-view disagreement compares the 3D locations of matching image features across 81 frame pairs; it measures consistency, not ground-truth accuracy. Times include input handling and output transfer, exclude model loading and one warm-up run, and use FP32 on the Mac GPU. DA3 five-view windows take about a third of a second per window; per-frame numbers are amortized, not streaming latency. Entire-recording DA3 uses all 27 or 22 frames jointly. No iPhone speed or thermal measurements were made.</p>
<h2>Results differ between recordings</h2><table><tr><th>Calibrated model</th><th>Recording 1 relative error</th><th>Recording 2 relative error</th></tr>{session_table}</table>
<h2>Before LiDAR calibration</h2><p>DA3 multi-view uses the recorded metric camera poses. UniDepth uses the recorded camera intrinsics. Single-image DA3-Small predicts arbitrary-scale depth, so a native metric error would be meaningless.</p><table><tr><th>Model</th><th>Overall relative error</th><th>Recording 1</th><th>Recording 2</th></tr>{native}</table>
<h2>Depth maps</h2><p>All panels use the same 0.5–30 m logarithmic color scale after calibration. Colors outside that display range saturate; numerical predictions remain unfiltered. Red distant regions indicate predicted distance, not verified distance. The examples below are fixed sample frames from both recordings.</p><img src="figures/depth-comparison.png" alt="Recorded images and four model depth predictions">
<h2>Inspect every evaluated frame</h2><select id="frame">{options}</select><div class="labels"><span>Recorded RGB</span>{''.join('<span>'+LABELS[m]+'</span>' for m in MODES)}</div><img id="gallery" src="figures/frame-{available[0]:03d}.jpg" alt="Selected frame comparison"><script>document.getElementById('frame').onchange=e=>document.getElementById('gallery').src='figures/frame-'+e.target.value+'.jpg';</script>
<h2>Accumulated colored point clouds</h2><p>Same recorded poses and RGB colors, with no fusion or surface optimization. Points are sampled every six pixels and retained at 0.2–30 m camera depth for these artifacts. Plots share fixed physical bounds and viewing angles; the PLY files contain all retained points. Frames without calibration are omitted. These are point clouds, not a replay of the app's mesh-fusion pipeline.</p><img src="figures/accumulated-clouds.png" alt="Accumulated colored point clouds in the same coordinate system"><ul>{links}</ul>
<h2>Can photogrammetry provide distant ground truth?</h2><p>Potentially, with translated overlapping views, independently measured scale, and separate check measurements. These two recordings span about 1.04 m and 0.35 m in their widest camera-position axes. An image-feature triangulation check using recorded ARKit poses found {photo['far_match_observations']} matched observations beyond 5 m across {photo['far_pairs']} pairs with at least eight such observations. Observations may repeat the same scene feature. This is sparse evidence, not a dense reference reconstruction.</p>
<p>The feasibility check requires a baseline ≥ 0.1 m, triangulation angle ≥ 1°, positive depth below 50 m, and reprojection error &lt; 1 pixel. It does not optimize camera poses and is not a COLMAP reconstruction. ARKit pose errors, modest baselines, and image-matching uncertainty remain. For a defensible benchmark, capture a separate translated photo survey and validate it against independent measurements. See <a href="https://colmap.github.io/tutorial.html">COLMAP capture guidance</a> and <a href="https://pubs.usgs.gov/publication/ofr20231033/full">USGS control and validation guidance</a>.</p>
<h2>Method and limits</h2><ul><li>Original camera JPEGs are rotated upright, with matching transformations of intrinsics and camera poses, then resized identically for both models.</li><li>Recorded voxel-filtered LiDAR points at 0.2–5 m provide a noisy reference. The bags do not contain LiDAR confidence images. Agreement with this reference is not surveyed absolute accuracy.</li><li>Calibration fits one positive multiplicative depth scale per frame using alternating 32-pixel spatial blocks. Only the other blocks are scored. No offset, confidence threshold, clipping, or learned-depth selection is used for scoring.</li><li>Camera intrinsics are supplied to UniDepth; intrinsics and poses are supplied to multi-view DA3. DA3-Small single-image inference uses its standard relative-depth API.</li><li>Mutual SIFT matches pass a 0.7 descriptor-ratio test in both directions and a &lt;1 pixel known-pose epipolar check. All models are evaluated on the same matches.</li><li>Only two recordings, both outdoor and from the same device, were tested. No statistical claim about general model superiority is justified.</li><li>Official inference architecture and weights are used. DA3's unused exporter import is made lazy, and its automatic half-precision wrapper is overridden so both models run in FP32. CUDA-only training/evaluation extensions are not used.</li><li>Raw images, predictions, reports, and point clouds stay local. The app has not been changed by this benchmark.</li></ul>
<p>Machine-readable results: <a href="metrics.json">summary</a> · <a href="per-frame.csv">per-frame metrics</a> · <a href="per-pair.csv">cross-view metrics</a> · <a href="photogrammetry-feasibility.json">triangulation feasibility</a> · <a href="requirements-lock.txt">dependency versions</a> · <a href="da3-runtime.patch">DA3 compatibility patch</a>.</p>
<details><summary>Exact model, source, and bag provenance</summary><pre>{provenance}</pre></details></html>'''
    (root / 'report.html').write_text(body)
    print(root / 'report.html')


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('root', type=Path); report(p.parse_args().root)
