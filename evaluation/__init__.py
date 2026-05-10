"""
evaluation - Evaluation metrics and effective detection area analysis.

This package provides:
- Association quality metrics (recall, FP rate, BEV errors)
- Distance-binned, lane-based, and grid-level metrics
- Effective radar detection area computation
- Blind zone identification
"""

from .eval_models import EvaluationResult, DistanceBinMetrics, LaneMetrics, GridMetrics
from .metrics import compute_association_metrics
from .effective_range import compute_effective_detection_area
