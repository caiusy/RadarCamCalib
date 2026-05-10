"""
association - Radar-vision target association using Hungarian matching.

This package provides:
- Gating functions (image, BEV, class compatibility)
- Cost matrix computation (normalized BEV + pixel distance)
- Hungarian matching via scipy.optimize.linear_sum_assignment
- Full association pipeline
"""

from .association_models import Match, AssociationResult
from .associator import associate_radar_vision
