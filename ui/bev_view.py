"""
bev_view.py - BEV (Bird's Eye View) viewport widget (PyQt6).

Preserves the original interactive BEV display from viewports.py:
- Radar point markers (with coordinate transform)
- Pair markers for calibration
- Trajectory visualization mode
- Grid overlay

Refactored from the original viewports.py BEVViewport class.
"""

from typing import Optional, List, Tuple, Dict
from PyQt6.QtWidgets import (
    QGraphicsView, QGraphicsScene, QGraphicsPixmapItem,
    QGraphicsEllipseItem, QGraphicsLineItem, QGraphicsTextItem,
    QGraphicsRectItem
)
from PyQt6.QtCore import Qt, QPointF, pyqtSignal
from PyQt6.QtGui import (
    QPixmap, QPen, QBrush, QColor, QFont, QWheelEvent, QMouseEvent, QPainter
)
import math

from visualization.color_map import PAIR_COLORS, UI_COLORS


class BEVViewport(QGraphicsView):
    """Bird's Eye View viewport for radar data visualization.

    Supports:
    - Radar point display with configurable coordinate ranges
    - Point pair markers with pair-index coloring
    - Trajectory visualization (radar + camera)
    - Playback trail drawing
    - Range polygon overlays
    """

    pointClicked = pyqtSignal(int, int)  # (target_id, frame_id)

    # --- BEV coordinate settings ---
    X_RANGE = (-30, 30)   # meters
    Y_RANGE = (0, 200)    # meters
    BEV_WIDTH = 400
    BEV_HEIGHT = 600

    def __init__(self, parent=None):
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self.setRenderHint(QPainter.RenderHint.Antialiasing)

        self._zoom = 1.0
        self._panning = False
        self._pan_start = None

        # Radar markers
        self._radar_markers: List[Tuple[QGraphicsEllipseItem, dict]] = []
        self._radar_labels: List = []

        # Pair markers
        self._pair_markers: List = []
        self._comparison_pairs: List = []

        # Trajectory items
        self._trajectory_items: List = []
        self._camera_trajectory_items: List = []
        self._trail_items: List = []
        self._trajectory_head: Optional[QGraphicsEllipseItem] = None
        self._trajectory_mode = False

        # Highlighted items
        self._highlighted_items: List = []

        # Draw initial grid
        self._drawGrid()

    def _bev_to_pixel(self, x: float, y: float) -> Tuple[float, float]:
        """Convert BEV meters to scene pixel coordinates."""
        x_min, x_max = self.X_RANGE
        y_min, y_max = self.Y_RANGE
        px = (x - x_min) / (x_max - x_min) * self.BEV_WIDTH
        py = (1.0 - (y - y_min) / (y_max - y_min)) * self.BEV_HEIGHT
        return px, py

    def _drawGrid(self):
        """Draw reference grid on BEV."""
        bg_color = QColor(30, 30, 30)
        self._scene.setBackgroundBrush(QBrush(bg_color))

        grid_pen = QPen(QColor(50, 50, 50), 1)
        axis_pen = QPen(QColor(80, 80, 80), 1)
        label_color = QColor(100, 100, 100)
        font = QFont("Arial", 7)

        x_min, x_max = self.X_RANGE
        y_min, y_max = self.Y_RANGE
        x_step = 10
        y_step = 20

        for x in range(int(x_min), int(x_max) + 1, x_step):
            px, _ = self._bev_to_pixel(x, y_min)
            _, py0 = self._bev_to_pixel(x, y_min)
            _, py1 = self._bev_to_pixel(x, y_max)
            pen = axis_pen if x == 0 else grid_pen
            self._scene.addLine(px, py0, px, py1, pen)
            label = QGraphicsTextItem(f"{x}m")
            label.setDefaultTextColor(label_color)
            label.setFont(font)
            label.setPos(px + 2, py0 - 12)
            self._scene.addItem(label)

        for y in range(int(y_min), int(y_max) + 1, y_step):
            _, py = self._bev_to_pixel(x_min, y)
            px0, _ = self._bev_to_pixel(x_min, y)
            px1, _ = self._bev_to_pixel(x_max, y)
            pen = axis_pen if y == 0 else grid_pen
            self._scene.addLine(px0, py, px1, py, pen)
            label = QGraphicsTextItem(f"{y}m")
            label.setDefaultTextColor(label_color)
            label.setFont(font)
            label.setPos(px0 + 2, py - 14)
            self._scene.addItem(label)

        # Ego marker
        ego_px, ego_py = self._bev_to_pixel(0, 0)
        ego = QGraphicsRectItem(ego_px - 4, ego_py - 8, 8, 8)
        ego.setPen(QPen(QColor(0, 120, 200), 2))
        ego.setBrush(QBrush(QColor(0, 120, 200)))
        self._scene.addItem(ego)

        self._scene.setSceneRect(0, 0, self.BEV_WIDTH, self.BEV_HEIGHT)

    # --- Radar Point Display ---
    def clearRadarMarkers(self):
        for item, _ in self._radar_markers:
            if item.scene():
                self._scene.removeItem(item)
        for item in self._radar_labels:
            if item.scene():
                self._scene.removeItem(item)
        self._radar_markers.clear()
        self._radar_labels.clear()

    def addRadarPoint(self, x: float, y: float, target: dict, color: str = None):
        """Add a radar point marker."""
        px, py = self._bev_to_pixel(x, y)
        size = 8
        c = QColor(color or UI_COLORS['radar'])
        marker = QGraphicsEllipseItem(px - size / 2, py - size / 2, size, size)
        marker.setPen(QPen(c, 2))
        marker.setBrush(QBrush(c))
        marker.setZValue(10)
        self._scene.addItem(marker)
        self._radar_markers.append((marker, target))

        tid = target.get('id', target.get('track_id', '?'))
        label = QGraphicsTextItem(f"R{tid}")
        label.setDefaultTextColor(c)
        label.setFont(QFont("Arial", 8))
        label.setPos(px + 6, py - 6)
        label.setZValue(10)
        self._scene.addItem(label)
        self._radar_labels.append(label)

    def highlightRadarMarker(self, target: dict):
        """Highlight a radar marker as pending selection."""
        tid = target.get('id', target.get('track_id'))
        for marker, t in self._radar_markers:
            if t.get('id', t.get('track_id')) == tid:
                marker.setPen(QPen(QColor(UI_COLORS['radar_pending']), 3))
                break

    def clearPendingRadar(self):
        for marker, _ in self._radar_markers:
            marker.setPen(QPen(QColor(UI_COLORS['radar']), 2))

    # --- Pair Markers ---
    def addPairMarker(self, x: float, y: float, pair_index: int):
        """Add a completed calibration pair marker."""
        px, py = self._bev_to_pixel(x, y)
        color = QColor(PAIR_COLORS[pair_index % len(PAIR_COLORS)])
        size = 10
        marker = QGraphicsEllipseItem(px - size / 2, py - size / 2, size, size)
        marker.setPen(QPen(color, 2))
        marker.setBrush(QBrush(color))
        marker.setZValue(20)
        self._scene.addItem(marker)
        self._pair_markers.append(marker)

        label = QGraphicsTextItem(str(pair_index + 1))
        label.setDefaultTextColor(color)
        label.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        label.setPos(px + 6, py - 8)
        label.setZValue(20)
        self._scene.addItem(label)
        self._pair_markers.append(label)

    def addComparisonPair(self, radar_bev, image_bev, pair_index: int):
        """Draw radar vs image BEV projection comparison."""
        color = QColor(PAIR_COLORS[pair_index % len(PAIR_COLORS)])

        r_px, r_py = self._bev_to_pixel(radar_bev[0], radar_bev[1])
        i_px, i_py = self._bev_to_pixel(image_bev[0], image_bev[1])

        # Image BEV projection (square marker)
        size = 8
        sq = QGraphicsRectItem(i_px - size / 2, i_py - size / 2, size, size)
        sq.setPen(QPen(color, 2))
        sq.setBrush(QBrush(Qt.BrushStyle.NoBrush))
        sq.setZValue(21)
        self._scene.addItem(sq)
        self._comparison_pairs.append(sq)

        # Connecting line
        line = QGraphicsLineItem(r_px, r_py, i_px, i_py)
        line.setPen(QPen(color, 1, Qt.PenStyle.DashLine))
        line.setZValue(19)
        self._scene.addItem(line)
        self._comparison_pairs.append(line)

    def clearPairs(self):
        for item in self._pair_markers + self._comparison_pairs:
            if item.scene():
                self._scene.removeItem(item)
        self._pair_markers.clear()
        self._comparison_pairs.clear()

    # --- Trajectory Mode ---
    def setTrajectoryMode(self, enabled: bool):
        self._trajectory_mode = enabled

    def loadTrajectories(self, trajectories: dict):
        """Load radar trajectories as polylines.

        Args:
            trajectories: Dict of target_id → list of (frame_id, x, y)
        """
        for i, (tid, points) in enumerate(trajectories.items()):
            if not points:
                continue

            color = QColor(PAIR_COLORS[i % len(PAIR_COLORS)])

            prev_px, prev_py = None, None
            for pt in points:
                if isinstance(pt, tuple) and len(pt) >= 3:
                    _, x, y = pt[0], pt[1], pt[2]
                elif isinstance(pt, tuple) and len(pt) == 2:
                    x, y = pt
                else:
                    continue

                px, py = self._bev_to_pixel(x, y)

                # Draw point
                size = 4
                dot = QGraphicsEllipseItem(px - size / 2, py - size / 2, size, size)
                dot.setPen(QPen(color, 1))
                dot.setBrush(QBrush(color))
                dot.setZValue(10)
                self._scene.addItem(dot)
                self._trajectory_items.append(dot)

                # Connect to previous point
                if prev_px is not None:
                    line = QGraphicsLineItem(prev_px, prev_py, px, py)
                    line.setPen(QPen(color, 1))
                    line.setZValue(9)
                    self._scene.addItem(line)
                    self._trajectory_items.append(line)

                prev_px, prev_py = px, py

            # Label at first point
            if points:
                first_pt = points[0]
                if isinstance(first_pt, tuple):
                    fx, fy = (first_pt[1], first_pt[2]) if len(first_pt) >= 3 else first_pt
                    fpx, fpy = self._bev_to_pixel(fx, fy)
                    label = QGraphicsTextItem(f"T{tid}")
                    label.setDefaultTextColor(color)
                    label.setFont(QFont("Arial", 8, QFont.Weight.Bold))
                    label.setPos(fpx + 5, fpy - 10)
                    label.setZValue(11)
                    self._scene.addItem(label)
                    self._trajectory_items.append(label)

    def loadCameraTrajectories(self, trajectories: dict):
        """Load camera trajectories (dashed lines, square markers)."""
        for i, (tid, points) in enumerate(trajectories.items()):
            if not points:
                continue

            color = QColor(PAIR_COLORS[i % len(PAIR_COLORS)])
            color.setAlpha(180)

            prev_px, prev_py = None, None
            for pt in points:
                if isinstance(pt, tuple) and len(pt) >= 4:
                    # (frame_id, u, v, x_bev, y_bev) format
                    x, y = pt[3], pt[4]
                elif isinstance(pt, tuple) and len(pt) == 2:
                    x, y = pt
                else:
                    continue

                px, py = self._bev_to_pixel(x, y)

                # Square marker
                size = 5
                sq = QGraphicsRectItem(px - size / 2, py - size / 2, size, size)
                sq.setPen(QPen(color, 1))
                sq.setBrush(QBrush(Qt.BrushStyle.NoBrush))
                sq.setZValue(8)
                self._scene.addItem(sq)
                self._camera_trajectory_items.append(sq)

                # Dashed line
                if prev_px is not None:
                    line = QGraphicsLineItem(prev_px, prev_py, px, py)
                    line.setPen(QPen(color, 1, Qt.PenStyle.DashLine))
                    line.setZValue(7)
                    self._scene.addItem(line)
                    self._camera_trajectory_items.append(line)

                prev_px, prev_py = px, py

    def clearTrajectories(self):
        for item in self._trajectory_items + self._camera_trajectory_items:
            if item.scene():
                self._scene.removeItem(item)
        self._trajectory_items.clear()
        self._camera_trajectory_items.clear()

    # --- Trail (Playback mode) ---
    def drawTrajectoryTrail(self, trail_points, color: QColor = None):
        """Draw a trail of points for playback."""
        self.clearTrails()
        c = color or QColor(0, 255, 255)

        prev_px, prev_py = None, None
        for pt in trail_points:
            if isinstance(pt, tuple) and len(pt) >= 2:
                x, y = pt[0], pt[1]
            else:
                continue

            px, py = self._bev_to_pixel(x, y)

            # Small dot
            size = 3
            dot = QGraphicsEllipseItem(px - size / 2, py - size / 2, size, size)
            dot.setPen(QPen(c, 1))
            dot.setBrush(QBrush(c))
            dot.setZValue(12)
            self._scene.addItem(dot)
            self._trail_items.append(dot)

            if prev_px is not None:
                line = QGraphicsLineItem(prev_px, prev_py, px, py)
                line.setPen(QPen(c, 1, Qt.PenStyle.DashLine))
                line.setZValue(11)
                self._scene.addItem(line)
                self._trail_items.append(line)

            prev_px, prev_py = px, py

    def drawTrajectoryHead(self, x: float, y: float, color: QColor = None):
        """Draw a highlighted head marker at current position."""
        if self._trajectory_head and self._trajectory_head.scene():
            self._scene.removeItem(self._trajectory_head)

        c = color or QColor(0, 255, 255)
        px, py = self._bev_to_pixel(x, y)
        size = 12
        self._trajectory_head = QGraphicsEllipseItem(
            px - size / 2, py - size / 2, size, size
        )
        self._trajectory_head.setPen(QPen(c, 3))
        self._trajectory_head.setBrush(QBrush(QColor(c.red(), c.green(), c.blue(), 120)))
        self._trajectory_head.setZValue(15)
        self._scene.addItem(self._trajectory_head)

    def clearTrails(self):
        for item in self._trail_items:
            if item.scene():
                self._scene.removeItem(item)
        self._trail_items.clear()
        if self._trajectory_head and self._trajectory_head.scene():
            self._scene.removeItem(self._trajectory_head)
            self._trajectory_head = None

    def highlightTrajectoryPoint(self, target_id: int, frame_id: int):
        """Highlight a specific trajectory point."""
        self.clearHighlights()
        # Find the point in trajectory items (visual search)
        # Emit signal for main window to handle
        self.pointClicked.emit(target_id, frame_id)

    def clearHighlights(self):
        for item in self._highlighted_items:
            if item.scene():
                self._scene.removeItem(item)
        self._highlighted_items.clear()

    def clearAll(self):
        """Clear everything except the grid."""
        self.clearRadarMarkers()
        self.clearPairs()
        self.clearTrajectories()
        self.clearTrails()
        self.clearHighlights()

    # --- Zoom/Pan ---
    def wheelEvent(self, event: QWheelEvent):
        factor = 1.2 if event.angleDelta().y() > 0 else 1 / 1.2
        new_zoom = self._zoom * factor
        if 0.1 <= new_zoom <= 20.0:
            self._zoom = new_zoom
            self.scale(factor, factor)
        event.accept()

    def mousePressEvent(self, event: QMouseEvent):
        if event.button() == Qt.MouseButton.RightButton:
            self._panning = True
            self._pan_start = event.position()
            self.setCursor(Qt.CursorShape.ClosedHandCursor)
            event.accept()
        else:
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._panning and self._pan_start is not None:
            delta = event.position() - self._pan_start
            self._pan_start = event.position()
            self.horizontalScrollBar().setValue(
                self.horizontalScrollBar().value() - int(delta.x())
            )
            self.verticalScrollBar().setValue(
                self.verticalScrollBar().value() - int(delta.y())
            )
            event.accept()
        else:
            super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QMouseEvent):
        if event.button() == Qt.MouseButton.RightButton and self._panning:
            self._panning = False
            self._pan_start = None
            self.setCursor(Qt.CursorShape.ArrowCursor)
            event.accept()
        else:
            super().mouseReleaseEvent(event)
