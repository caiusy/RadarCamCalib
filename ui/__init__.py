"""
ui - PyQt6 user interface for radar-camera calibration system.

Preserves the original interactive workflow:
- Point pair selection (click radar → click image)
- Lane drawing for vanishing point pitch estimation
- Camera/Radar parameter tuning
- Trajectory visualization mode
- Calibration optimization

All algorithm logic is delegated to the data/, calibration/, trajectory/,
and range_estimation/ modules. The UI only handles display and interaction.
"""
