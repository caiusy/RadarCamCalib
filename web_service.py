"""UI-independent analysis services for the browser workbench."""
import numpy as np

from association.associator import associate_radar_vision
from calibration.projector import project_image_to_radar
from evaluation.metrics import compute_association_metrics
from evaluation.effective_range import compute_effective_detection_area
from range_estimation.range_estimator import estimate_radar_range


def association(radar, vision, session):
    if session.H is None:
        return None
    result = associate_radar_vision(radar, vision, session.H, session.H_inv)
    return {"association": result.to_dict(),
            "metrics": compute_association_metrics(result, radar, vision, session.H_inv).to_dict(),
            "vision_bev": project_image_to_radar(
                np.asarray([v.bottom_center for v in vision]).reshape(-1, 2), session.H_inv).tolist()}


def analyze(dataset, session, start=0, end=None, step=1):
    """Explicit, bounded frame window for trajectory/coverage analysis."""
    start, end, step = int(start), dataset.num_frames if end is None else int(end), int(step)
    if start < 0 or end > dataset.num_frames or end <= start or step < 1:
        raise ValueError("Invalid analysis window")
    if len(range(start, end, step)) > 2000:
        raise ValueError("Analyze at most 2000 frames per request; increase frame step")
    radar_tracks, camera_tracks, points = {}, {}, []
    all_matches, all_vision, all_radar = [], [], []
    totals = {"radar": 0, "vision": 0, "matched": 0}
    errors = []
    for i in range(start, end, step):
        radar, vision = dataset.objects(i)
        rp = np.asarray([[r.x, r.y] for r in radar], dtype=float).reshape(-1, 2)
        points.extend(rp.tolist())
        for obj in radar:
            radar_tracks.setdefault(str(obj.track_id), []).append([i, obj.x, obj.y])
        totals["radar"] += len(radar)
        totals["vision"] += len(vision)
        if session.H is not None:
            result = associate_radar_vision(radar, vision, session.H, session.H_inv)
            totals["matched"] += result.num_matches
            errors.extend(m.bev_distance for m in result.matches)
            all_matches.append(result.matches)
            all_vision.append(vision)
            all_radar.append(rp)
            cp = project_image_to_radar(np.asarray([v.bottom_center for v in vision]).reshape(-1, 2), session.H_inv)
            for obj, xy in zip(vision, cp):
                camera_tracks.setdefault(str(obj.track_id), []).append([i, float(xy[0]), float(xy[1])])
    coverage = estimate_radar_range(np.asarray(points, dtype=float).reshape(-1, 2)) if points else None
    effective = compute_effective_detection_area(all_matches, all_vision, all_radar, session.H_inv) if session.H is not None else None
    return {"window": {"start": start, "end": end, "step": step},
            "radar_tracks": radar_tracks, "camera_tracks": camera_tracks, "coverage": coverage,
            "effective_area": effective, "metrics": {
                **totals, "radar_recall": totals["matched"] / totals["vision"] if totals["vision"] and session.H is not None else None,
                "radar_fp_rate": (totals["radar"] - totals["matched"]) / totals["radar"] if totals["radar"] and session.H is not None else None,
                "mean_bev_error": float(np.mean(errors)) if errors else None,
            }, "metric_scope": "association consistency; camera detections are a reference, not independent ground truth"}
