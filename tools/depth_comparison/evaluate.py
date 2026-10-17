"""Evaluate saved predictions against held-out LiDAR and independent image matches."""
import argparse
import csv
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

MODES = ['da3-single', 'da3-multi', 'da3-scene', 'unidepth']


def sample(depth, uv):
    if not len(uv):
        return np.empty(0, dtype=np.float32)
    return cv2.remap(depth.astype(np.float32), uv[:, 0:1].astype(np.float32),
                     uv[:, 1:2].astype(np.float32), cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_REPLICATE).ravel()


def unproject(uv, depth, meta):
    rays = np.c_[uv, np.ones(len(uv))] @ np.linalg.inv(meta['K']).T
    cam = rays * depth[:, None]
    return (cam - meta['E'][:3, 3]) @ meta['E'][:3, :3]


def statistics(predicted, reference):
    valid = np.isfinite(predicted) & (predicted > 0)
    p, r = predicted[valid], reference[valid]
    return {'samples': len(r), 'coverage': float(valid.mean()),
            'absrel': float(np.mean(np.abs(p-r)/r)),
            'mae_m': float(np.mean(np.abs(p-r))),
            'rmse_m': float(np.sqrt(np.mean((p-r)**2))),
            'delta125': float(np.mean(np.maximum(p/r, r/p) < 1.25))}


def evaluate(root):
    data = root / 'data'; manifest = json.loads((data / 'manifest.json').read_text())
    frames = manifest['frames']; metadata = [dict(np.load(data / f['data'])) for f in frames]
    predictions = {mode: [np.load(root / 'predictions' / mode / f"frame-{f['index']:03d}.npz")['depth']
                          for f in frames] for mode in MODES}
    rows = []; scales = {mode: [] for mode in MODES}
    for i, (frame, meta) in enumerate(zip(frames, metadata)):
        uv, reference = meta['lidar_uv'], meta['lidar_depth']
        # Fixed spatial blocks shared by every model, never selected by prediction error.
        train = ((uv[:, 0] // 32 + uv[:, 1] // 32).astype(int) % 2) == 0
        for mode in MODES:
            values = sample(predictions[mode][i], uv)
            fit = train & np.isfinite(values) & (values > 0)
            scale = float(np.median(reference[fit] / values[fit])) if fit.sum() >= 20 else None
            scales[mode].append(scale)
            if scale is None or (~train).sum() < 20:
                continue
            for variant, factor in [('scaled', scale), ('native', 1.)]:
                if variant == 'native' and mode == 'da3-single':
                    continue  # Single-frame DA3-Small has arbitrary scale, not metres.
                rows.append({'frame': i, 'session': frame['session'], 'mode': mode,
                             'variant': variant, 'scale': scale, 'fit_samples': int(fit.sum()),
                             **statistics(values[~train]*factor, reference[~train])})
    with (root / 'per-frame.csv').open('w') as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys()); writer.writeheader(); writer.writerows(rows)
    (root / 'scales.json').write_text(json.dumps(scales, indent=2))

    # Mutual SIFT matches, ratio < 0.7 in both directions. Image-only selection,
    # plus known-pose epipolar distance < 1 pixel; no model influences selection.
    cv2.setRNGSeed(0); sift = cv2.SIFT_create(nfeatures=2500)
    features = [sift.detectAndCompute(cv2.imread(str(data / f['image']), 0), None) for f in frames]
    matcher = cv2.BFMatcher(); pairs = []; triangulated = []; pair_rows = []
    for i in range(len(frames)):
        for gap in [1, 2, 4]:
            j = i + gap
            if j >= len(frames) or frames[i]['session'] != frames[j]['session']:
                continue
            ka, da = features[i]; kb, db = features[j]
            if da is None or db is None:
                continue
            a = {(m.queryIdx, m.trainIdx) for m, n in matcher.knnMatch(da, db, k=2) if m.distance < .7*n.distance}
            b = {(m.trainIdx, m.queryIdx) for m, n in matcher.knnMatch(db, da, k=2) if m.distance < .7*n.distance}
            matches = sorted(a & b)
            if not matches:
                continue
            ua = np.array([ka[k].pt for k, _ in matches]); ub = np.array([kb[k].pt for _, k in matches])
            ma, mb = metadata[i], metadata[j]; rel = mb['E'] @ np.linalg.inv(ma['E'])
            t = rel[:3, 3]; skew = np.array([[0, -t[2], t[1]], [t[2], 0, -t[0]], [-t[1], t[0], 0]])
            F = np.linalg.inv(mb['K']).T @ skew @ rel[:3, :3] @ np.linalg.inv(ma['K'])
            ah, bh = np.c_[ua, np.ones(len(ua))], np.c_[ub, np.ones(len(ub))]
            la, lb = bh @ F, ah @ F.T
            residual = np.abs(np.sum(lb*bh, axis=1))
            epipolar = np.maximum(residual/np.maximum(np.linalg.norm(la[:, :2], axis=1), 1e-12),
                                  residual/np.maximum(np.linalg.norm(lb[:, :2], axis=1), 1e-12))
            keep = epipolar < 1.; ua, ub = ua[keep], ub[keep]
            if len(ua) < 8:
                continue
            pair = {'i': i, 'j': j, 'session': frames[i]['session'], 'matches': len(ua),
                    'baseline_m': float(np.linalg.norm(t))}
            pairs.append(pair)
            for mode in MODES:
                for variant in ['scaled', 'native']:
                    if variant == 'native' and mode == 'da3-single':
                        continue
                    si, sj = (scales[mode][i], scales[mode][j]) if variant == 'scaled' else (1., 1.)
                    if si is None or sj is None:
                        continue
                    za = sample(predictions[mode][i], ua)*si; zb = sample(predictions[mode][j], ub)*sj
                    valid = np.isfinite(za) & np.isfinite(zb) & (za > 0) & (zb > 0)
                    pa = unproject(ua[valid], za[valid], ma); pb = unproject(ub[valid], zb[valid], mb)
                    error = np.linalg.norm(pa-pb, axis=1)
                    relative = error / ((za[valid]+zb[valid])/2)
                    pair_rows.append({**pair, 'mode': mode, 'variant': variant,
                                      'median_distance_m': float(np.median(error)),
                                      'median_relative_distance': float(np.median(relative)),
                                      'coverage': float(valid.mean())})
            # Photogrammetry feasibility, not certified ground truth: known ARKit poses.
            if np.linalg.norm(t) < .1:
                continue
            xyzw = cv2.triangulatePoints(ma['K'] @ ma['E'][:3], mb['K'] @ mb['E'][:3], ua.T, ub.T)
            xyz = (xyzw[:3] / xyzw[3]).T
            ca = (xyz @ ma['E'][:3, :3].T) + ma['E'][:3, 3]
            cb = (xyz @ mb['E'][:3, :3].T) + mb['E'][:3, 3]
            center_a = np.linalg.inv(ma['E'])[:3, 3]; center_b = np.linalg.inv(mb['E'])[:3, 3]
            ra = xyz-center_a; rb = xyz-center_b
            angle = np.degrees(np.arccos(np.clip(np.sum(ra*rb, axis=1)/(np.linalg.norm(ra, axis=1)*np.linalg.norm(rb, axis=1)), -1, 1)))
            qa = ca @ ma['K'].T; qb = cb @ mb['K'].T
            reproj = np.maximum(np.linalg.norm(qa[:, :2]/qa[:, 2:]-ua, axis=1), np.linalg.norm(qb[:, :2]/qb[:, 2:]-ub, axis=1))
            good = (ca[:, 2] > .2) & (cb[:, 2] > .2) & (ca[:, 2] < 50) & (angle >= 1) & (reproj < 1)
            triangulated.append({**pair, 'usable_matches': int(good.sum()),
                                 'far_matches_over5m': int((good & (ca[:, 2] > 5)).sum()),
                                 'median_angle_deg': float(np.median(angle[good])) if good.any() else None})
    with (root / 'per-pair.csv').open('w') as f:
        writer = csv.DictWriter(f, fieldnames=pair_rows[0].keys()); writer.writeheader(); writer.writerows(pair_rows)
    (root / 'photogrammetry-feasibility.json').write_text(json.dumps(triangulated, indent=2))
    summary = []
    for mode in MODES:
        timing = json.loads((root / 'predictions' / mode / 'timings.json').read_text())
        for variant in ['scaled', 'native']:
            rr = [r for r in rows if r['mode'] == mode and r['variant'] == variant]
            pp = [r for r in pair_rows if r['mode'] == mode and r['variant'] == variant]
            if not rr:
                continue
            def aggregate(rs):
                return {k: float(np.mean([r[k] for r in rs])) for k in ['absrel', 'mae_m', 'rmse_m', 'delta125', 'coverage']}
            summary.append({'mode': mode, 'variant': variant, 'frames': len(rr), 'test_samples': sum(r['samples'] for r in rr),
                            **aggregate(rr), 'sessions': {str(s): aggregate([r for r in rr if r['session'] == s]) for s in [0, 1]},
                            'pair_count': len(pp),
                            'pair_median_relative_distance': float(np.median([r['median_relative_distance'] for r in pp])),
                            'pair_median_distance_m': float(np.median([r['median_distance_m'] for r in pp])),
                            'median_seconds_per_input_frame': float(np.median([t['seconds']/len(t['input_frames']) for t in timing])),
                            'total_seconds': sum(t['seconds'] for t in timing)})
    result = {'summary': summary, 'photogrammetry': {'pairs': len(triangulated),
              'usable_match_observations': sum(p['usable_matches'] for p in triangulated),
              'far_match_observations': sum(p['far_matches_over5m'] for p in triangulated),
              'far_pairs': sum(p['far_matches_over5m'] >= 8 for p in triangulated)},
              'method': 'Equal-weight per-frame held-out LiDAR metrics; 32px checkerboard split, train-only median multiplicative scale. Pair metric: median across pairs of median 3D disagreement / mean camera-z depth on mutual SIFT matches with <1px known-pose epipolar error. No confidence filtering or depth clipping.'}
    (root / 'metrics.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('root', type=Path); evaluate(p.parse_args().root)
