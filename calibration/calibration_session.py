"""
calibration_session.py - Manage the calibration workflow.

Holds matched points, computes homography, stores results.
This is a pure logic class with no UI dependency.
"""

from typing import List, Optional
import numpy as np

from data.data_models import MatchedPoint, CalibrationResult
from .homography_solver import compute_homography, compute_reprojection_errors, invert_homography


class CalibrationSession:
    """Manages matched point pairs and homography computation.

    This class holds the state of an active calibration session:
    matched points, computed homography, and reprojection statistics.

    Usage:
        session = CalibrationSession()
        session.add_point(radar_x, radar_y, image_u, image_v)
        session.add_point(...)
        result = session.compute()
    """

    def __init__(self, ransac_threshold: float = 5.0):
        """Initialize calibration session.

        Args:
            ransac_threshold: RANSAC reprojection threshold in pixels.
        """
        self.matched_points: List[MatchedPoint] = []
        self.result: Optional[CalibrationResult] = None
        self.ransac_threshold = ransac_threshold

    @property
    def num_points(self) -> int:
        """Number of matched point pairs."""
        return len(self.matched_points)

    @property
    def has_homography(self) -> bool:
        """Whether a valid homography has been computed."""
        return self.result is not None

    @property
    def H(self) -> Optional[np.ndarray]:
        """Current homography matrix, or None."""
        return self.result.H if self.result else None

    @property
    def H_inv(self) -> Optional[np.ndarray]:
        """Current inverse homography matrix, or None."""
        return self.result.H_inv if self.result else None

    def add_point(self, radar_x: float, radar_y: float,
                  image_u: float, image_v: float):
        """Add a matched point pair.

        Args:
            radar_x: Radar x coordinate (lateral, meters).
            radar_y: Radar y coordinate (forward, meters).
            image_u: Image u coordinate (horizontal, pixels).
            image_v: Image v coordinate (vertical, pixels).
        """
        point = MatchedPoint(
            radar_x=radar_x, radar_y=radar_y,
            image_u=image_u, image_v=image_v,
        )
        self.matched_points.append(point)

    def remove_point(self, index: int):
        """Remove a matched point pair by index.

        Args:
            index: Index of the point to remove.
        """
        if 0 <= index < len(self.matched_points):
            self.matched_points.pop(index)

    def clear_points(self):
        """Remove all matched point pairs and reset result."""
        self.matched_points.clear()
        self.result = None

    def set_points(self, points: List[MatchedPoint]):
        """Replace all matched points with a new list.

        Args:
            points: New list of matched points.
        """
        self.matched_points = list(points)

    def _get_point_arrays(self):
        """Extract radar and image point arrays from matched points.

        Returns:
            Tuple of (radar_points, image_points), both Nx2 numpy arrays.
        """
        radar_pts = np.array(
            [[p.radar_x, p.radar_y] for p in self.matched_points],
            dtype=np.float64
        )
        image_pts = np.array(
            [[p.image_u, p.image_v] for p in self.matched_points],
            dtype=np.float64
        )
        return radar_pts, image_pts

    def compute(self) -> CalibrationResult:
        """Compute homography from current matched points.

        Updates all matched points with reprojection errors and inlier status.

        Returns:
            CalibrationResult with homography and statistics.

        Raises:
            ValueError: If fewer than 4 matched points exist.
        """
        if self.num_points < 4:
            raise ValueError(
                f"At least 4 matched points required, have {self.num_points}"
            )

        radar_pts, image_pts = self._get_point_arrays()

        # Compute homography with RANSAC
        H, mask = compute_homography(radar_pts, image_pts, self.ransac_threshold)
        if H is None:
            raise ValueError("Homography computation failed")

        H_inv = invert_homography(H)

        # Compute per-point reprojection errors
        errors = compute_reprojection_errors(radar_pts, image_pts, H)

        # Update matched points with error and inlier info
        inlier_mask = mask.ravel().astype(bool) if mask is not None else np.ones(len(errors), dtype=bool)
        for i, point in enumerate(self.matched_points):
            point.reprojection_error = float(errors[i])
            point.inlier = bool(inlier_mask[i])

        # Compute statistics (over inlier points)
        inlier_errors = errors[inlier_mask] if inlier_mask.any() else errors
        mean_error = float(np.mean(inlier_errors)) if len(inlier_errors) > 0 else 0.0
        max_error = float(np.max(errors)) if len(errors) > 0 else 0.0

        self.result = CalibrationResult(
            H=H,
            H_inv=H_inv,
            matched_points=self.matched_points,
            mean_error=mean_error,
            max_error=max_error,
            inlier_mask=inlier_mask,
        )

        return self.result

    def load_from_calibration_data(self, calib_data: dict):
        """Load calibration result from a parsed calibration JSON dict.

        Args:
            calib_data: Dictionary with 'homography', 'inverse_homography',
                        'matched_points', etc.
        """
        H = np.array(calib_data["homography"], dtype=np.float64)
        H_inv = np.array(calib_data.get("inverse_homography",
                                        np.linalg.inv(H)), dtype=np.float64)

        points = []
        for mp in calib_data.get("matched_points", []):
            points.append(MatchedPoint.from_dict(mp))

        self.matched_points = points
        self.result = CalibrationResult(
            H=H,
            H_inv=H_inv,
            matched_points=points,
            mean_error=calib_data.get("mean_reprojection_error", 0.0),
            max_error=calib_data.get("max_reprojection_error", 0.0),
        )
