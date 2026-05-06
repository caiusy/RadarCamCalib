"""
json_exporter.py - Export calibration and range estimation results to JSON.

Delegates to calibration_io for actual serialization, but provides
a cleaner high-level interface for the UI and CLI layers.
"""

import json
from pathlib import Path
from typing import Optional
from datetime import datetime

from data.data_models import CalibrationResult
from calibration.calibration_io import (
    save_calibration_json,
    save_range_json,
    save_matched_points_json,
)


def export_calibration_json(
    result: CalibrationResult,
    output_path: Path,
    camera_sensor_id: int = 1,
    radar_sensor_id: int = 2,
) -> Path:
    """Export calibration result to JSON file.

    Args:
        result: CalibrationResult to export.
        output_path: Output file path. If directory, auto-generates filename.
        camera_sensor_id: Camera sensor ID for metadata.
        radar_sensor_id: Radar sensor ID for metadata.

    Returns:
        Path to the saved file.
    """
    output_path = Path(output_path)

    if output_path.is_dir():
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = output_path / f"calibration_{ts}.json"

    save_calibration_json(output_path, result, camera_sensor_id, radar_sensor_id)
    return output_path


def export_range_json(
    range_data: dict,
    output_path: Path,
    radar_sensor_id: int = 2,
) -> Path:
    """Export radar range estimation result to JSON file.

    Args:
        range_data: Range estimation result dictionary.
        output_path: Output file path. If directory, auto-generates filename.
        radar_sensor_id: Radar sensor ID for metadata.

    Returns:
        Path to the saved file.
    """
    output_path = Path(output_path)

    if output_path.is_dir():
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = output_path / f"radar_range_{ts}.json"

    # Add radar_sensor_id if not present
    range_data.setdefault("radar_sensor_id", radar_sensor_id)

    save_range_json(output_path, range_data)
    return output_path


def export_report(
    result: CalibrationResult,
    range_data: Optional[dict],
    output_dir: Path,
) -> Path:
    """Export a combined text report summarizing calibration and range results.

    Args:
        result: CalibrationResult.
        range_data: Optional range estimation result.
        output_dir: Directory to save the report.

    Returns:
        Path to the report file.
    """
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_path = output_dir / f"report_{ts}.txt"

    lines = [
        "=" * 60,
        "Radar-Camera Calibration Report",
        f"Generated: {datetime.now().isoformat()}",
        "=" * 60,
        "",
        "--- Homography Matrix ---",
    ]

    for row in result.H.tolist():
        lines.append(f"  [{', '.join(f'{v:12.6f}' for v in row)}]")

    lines.extend([
        "",
        f"Mean reprojection error: {result.mean_error:.4f} px",
        f"Max reprojection error:  {result.max_error:.4f} px",
        f"Number of matched points: {len(result.matched_points)}",
        "",
        "--- Matched Points ---",
    ])

    for i, mp in enumerate(result.matched_points):
        inlier_str = "INLIER" if mp.inlier else "OUTLIER"
        err_str = f"{mp.reprojection_error:.2f}px" if mp.reprojection_error else "N/A"
        lines.append(
            f"  P{i+1}: radar=({mp.radar_x:.2f}, {mp.radar_y:.2f}) "
            f"image=({mp.image_u:.1f}, {mp.image_v:.1f}) "
            f"error={err_str} [{inlier_str}]"
        )

    if range_data:
        polys = range_data.get("range_polygons", [])
        lines.extend([
            "",
            "--- Range Estimation ---",
            f"Number of polygons: {len(polys)}",
        ])
        for j, poly in enumerate(polys):
            lines.append(f"  Polygon {j+1}: area={poly.get('area_m2', 0):.1f} m²")

    lines.append("")
    lines.append("=" * 60)

    report_path.write_text("\n".join(lines), encoding="utf-8")
    print(f"[Report] Saved to {report_path}")
    return report_path
