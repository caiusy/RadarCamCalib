"""Operational planar calibration from independent radar XYZ and camera boxes.

No annotation, CATS output, or evaluation ground truth is read here. Native
XYZ projections provide candidate correspondences, not surveyed ground truth.
"""
from __future__ import annotations

import configparser
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment


def load_geometry(config_path):
    cfg = configparser.ConfigParser()
    if not cfg.read(config_path):
        raise FileNotFoundError(config_path)
    K = np.asarray(json.loads(cfg['cameraIntrinsic']['cameramatrix']), float)
    D = np.asarray(json.loads(cfg['cameraIntrinsic']['distcoeffs']), float)
    rt = np.asarray(json.loads(cfg['radar2camera']['rt']), float)
    return K, D, rt[:, :3], rt[:, 3]


def native_pairs(radar_frames, camera_frames, geometry, step=3, lag=0):
    """Unambiguous native XYZ/box seeds, excluding clipped boxes.

    Lag convention: radar frame i corresponds to camera frame i + lag.
    The box anchor at 75% height is only used for seed scoring; the final
    homography maps radar (x,z) to the bbox bottom centre.
    """
    K, D, R, T = geometry
    pairs = []
    for i in range(0, len(radar_frames), step):
        j = i + lag
        if j < 0 or j >= len(camera_frames):
            continue
        radars, cameras = radar_frames[i].get('data', []), camera_frames[j]
        boxes = np.asarray(cameras.get('boxes', []), float).reshape(-1, 4)
        if not radars or not len(boxes):
            continue
        xyz = np.asarray([[r['x'], r['y'], r['z']] for r in radars], float)
        cam = (xyz + T) @ R.T
        positive = cam[:, 2] > 0
        uv = np.full((len(xyz), 2), np.nan)
        if positive.any():
            uv[positive] = cv2.projectPoints(cam[positive], np.zeros(3), np.zeros(3), K, D)[0].reshape(-1, 2)
        wh = boxes[:, 2:] - boxes[:, :2]
        good_boxes = (wh > 0).all(axis=1) & (boxes[:, 3] < 715) & (boxes[:, 1] > 2)
        anchor = (boxes[:, :2] + boxes[:, 2:]) / 2
        anchor[:, 1] = boxes[:, 1] + .75 * wh[:, 1]
        cost = np.sum(((uv[:, None, :] - anchor[None]) / np.maximum(wh[None], 1)) ** 2, axis=2)
        inside = ((uv[:, None] >= boxes[None, :, :2]) & (uv[:, None] <= boxes[None, :, 2:])).all(axis=2)
        cost[~inside | ~np.isfinite(cost) | ~good_boxes[None]] = 1e6
        ri, ci = linear_sum_assignment(cost)
        for r, c in zip(ri, ci):
            if cost[r, c] > .12:
                continue
            # Reject ambiguous candidates even if Hungarian selected one.
            column, row = np.sort(cost[:, c]), np.sort(cost[r])
            if ((len(column) > 1 and column[1] - column[0] < .05)
                    or (len(row) > 1 and row[1] - row[0] < .05)):
                continue
            pairs.append({'frame': i, 'camera_frame': j,
                          'radar_id': int(radars[r]['id']), 'camera_id': int(cameras['ids'][c]),
                          'radar': xyz[r, [0, 2]].tolist(),
                          'image': [float((boxes[c, 0] + boxes[c, 2]) / 2), float(boxes[c, 3])],
                          'seed_cost': float(cost[r, c])})
    return pairs


def error_stats(values):
    values = np.asarray(values, float)
    if not len(values):
        return {'count': 0, 'median': None, 'mean': None, 'p95': None, 'max': None}
    return {'count': len(values), 'median': float(np.median(values)), 'mean': float(np.mean(values)),
            'p95': float(np.percentile(values, 95)), 'max': float(np.max(values))}


def fit_native_pairs(pairs, train_end_frame, ransac_threshold=12.):
    """Fit on earlier frames; validate on later frames AND unseen camera ids."""
    radar = np.asarray([p['radar'] for p in pairs], float)
    image = np.asarray([p['image'] for p in pairs], float)
    frames = np.asarray([p['frame'] for p in pairs])
    train = frames < train_end_frame
    ids = {p['camera_id'] for p in pairs if p['frame'] < train_end_frame}
    heldout = np.asarray([p['frame'] >= train_end_frame and p['camera_id'] not in ids for p in pairs])
    if train.sum() < 8 or heldout.sum() < 8:
        raise ValueError('at least 8 train and 8 temporally held-out unseen-id pairs required')
    cv2.setRNGSeed(42)
    H, mask = cv2.findHomography(radar[train], image[train], cv2.RANSAC, ransac_threshold)
    if H is None or np.linalg.matrix_rank(H) < 3:
        raise ValueError('degenerate native pair fit')
    # Refine only using the training inliers, never holdout points.
    inliers = mask.ravel().astype(bool)
    H, _ = cv2.findHomography(radar[train][inliers], image[train][inliers], 0)
    H_inv = np.linalg.inv(H)
    uv = cv2.perspectiveTransform(radar[None], H)[0]
    xy = cv2.perspectiveTransform(image[None], H_inv)[0]
    pixel_error = np.linalg.norm(uv - image, axis=1)
    metric_error = np.linalg.norm(xy - radar, axis=1)
    report = {'method': 'native XYZ containment seeds + RANSAC planar effective mapping',
              'inputs': ['radarTrack.npy', 'config', 'output-keypoints-ByteTrack.npy'],
              'annotation_used': False, 'cats_used': False,
              'train_end_frame_exclusive': train_end_frame,
              'train_camera_ids': len(ids), 'train_inliers': int(inliers.sum()),
              'train_inlier_pixels': error_stats(pixel_error[train][inliers]),
              'holdout_pixels': error_stats(pixel_error[heldout]),
              'holdout_metres': error_stats(metric_error[heldout]),
              'holdout_camera_ids': len({p['camera_id'] for p, h in zip(pairs, heldout) if h}),
              'validation_scope': 'automatic native-projection correspondences; not independent manual ground truth',
              'anchor': 'radar track centre vs camera bbox bottom centre; includes vehicle extent bias'}
    return {'version': '2.0', 'calibration_type': 'esmeot_native_pair_fit',
            'homography': H.tolist(), 'inverse_homography': H_inv.tolist(),
            'pixel_space': 'raw', 'ground_plane_axes': ['x', 'z'],
            'validation_status': 'operational_fit', 'fit': report}


def fit_dataset(dataset_root, camera_tracks, output, train_fraction=.5, step=3):
    root, output = Path(dataset_root), Path(output)
    radars = np.load(root / 'radarTrack.npy', allow_pickle=True)
    cameras = np.load(camera_tracks, allow_pickle=True)
    geometry = load_geometry(root / 'config')
    pairs = native_pairs(radars, cameras, geometry, step=step)
    result = fit_native_pairs(pairs, int(min(len(radars), len(cameras)) * train_fraction))
    result['source_config'] = str(root / 'config')
    result['source_camera_tracks'] = str(camera_tracks)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2), encoding='utf-8')
    output.with_name(output.stem + '_pairs.json').write_text(json.dumps(pairs, indent=2), encoding='utf-8')
    return result
