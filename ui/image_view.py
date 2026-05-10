"""
image_view.py - Image viewport widget (PyQt6).

Preserves the original interactive point selection workflow:
- Radar point click → highlight → image point click → pair created
- Lane start/end drawing
- Mouse tracking preview
- Radar projection display

Refactored from the original viewports.py ImageViewport class.
"""

from typing import Optional, List, Tuple
from PyQt6.QtWidgets import (
    QGraphicsView, QGraphicsScene, QGraphicsPixmapItem,
    QGraphicsEllipseItem, QGraphicsLineItem, QGraphicsTextItem
)
from PyQt6.QtCore import Qt, QPointF, pyqtSignal
from PyQt6.QtGui import (
    QPixmap, QPen, QBrush, QColor, QFont, QWheelEvent, QMouseEvent, QPainter
)
import os

from visualization.color_map import UI_COLORS, PAIR_COLORS


# Marker sizes (pixels)
MARKER_SIZE_RADAR = 20
MARKER_SIZE_IMAGE = 16
MARKER_SIZE_PREVIEW = 24
LANE_POINT_SIZE = 10


class ZoomPanView(QGraphicsView):
    """Base graphics view with zoom/pan support.

    - Scroll wheel: zoom in/out
    - Right-click + drag: pan canvas
    - Left-click: point selection (when click mode enabled)
    """

    clicked = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.ViewportAnchor.AnchorViewCenter)
        self.setDragMode(QGraphicsView.DragMode.NoDrag)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self._zoom = 1.0
        self._click_mode = False
        self._panning = False
        self._pan_start = None

    def setClickMode(self, enabled: bool):
        self._click_mode = enabled
        self.setCursor(
            Qt.CursorShape.CrossCursor if enabled else Qt.CursorShape.ArrowCursor
        )

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
        elif self._click_mode and event.button() == Qt.MouseButton.LeftButton:
            pos = self.mapToScene(event.position().toPoint())
            self.clicked.emit(pos.x(), pos.y())
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
            self.setCursor(
                Qt.CursorShape.CrossCursor if self._click_mode
                else Qt.CursorShape.ArrowCursor
            )
            event.accept()
        else:
            super().mouseReleaseEvent(event)

    def fitContent(self):
        self._zoom = 1.0
        self.resetTransform()
        if self.scene():
            rect = self.scene().itemsBoundingRect()
            if rect.isValid():
                self.fitInView(rect, Qt.AspectRatioMode.KeepAspectRatio)


class ImageViewport(ZoomPanView):
    """Viewport for displaying camera image with interactive overlays.

    Supports:
    - Projected radar point markers (clickable for point pair selection)
    - Point pair markers with numbered colors
    - Lane line drawing
    - Mouse-tracking preview circle
    - Trajectory projection markers

    Preserves the complete interactive workflow from the original codebase.
    """

    radarClicked = pyqtSignal(dict)
    imageClicked = pyqtSignal(float, float)
    mouseMoved = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self.setMouseTracking(True)

        self._pixmap_item: Optional[QGraphicsPixmapItem] = None

        # Radar projection markers: [(ellipse_item, target_dict), ...]
        self._radar_markers: List[Tuple[QGraphicsEllipseItem, dict]] = []
        self._radar_labels: List = []

        # Completed pair markers
        self._pair_markers: List = []

        # Pending selection markers
        self._pending_radar_marker: Optional[QGraphicsEllipseItem] = None
        self._preview_marker: Optional[QGraphicsEllipseItem] = None

        # Lane markers
        self._lane_markers: List = []
        self._pending_lane_start: Optional[QPointF] = None
        self._pending_lane_line: Optional[QGraphicsLineItem] = None

        # Trajectory projection
        self._trajectory_projection: List = []

        self._mode = 'normal'

    def setMode(self, mode: str):
        self._mode = mode
        self.setClickMode(mode != 'normal')
        if self._preview_marker and self._preview_marker.scene():
            self._scene.removeItem(self._preview_marker)
            self._preview_marker = None

    def loadImage(self, path: str) -> bool:
        if not os.path.exists(path):
            return False
        pix = QPixmap(path)
        if pix.isNull():
            return False
        if self._pixmap_item:
            self._scene.removeItem(self._pixmap_item)
        self._pixmap_item = QGraphicsPixmapItem(pix)
        self._scene.addItem(self._pixmap_item)
        self._scene.setSceneRect(self._pixmap_item.boundingRect())
        self.fitContent()
        return True

    def setOverlayImage(self, cv_image):
        """Display a pre-rendered OpenCV BGR image as the viewport content.

        Used by association mode to show bboxes, matching lines, etc.

        Args:
            cv_image: OpenCV BGR image (numpy array).
        """
        import cv2
        import numpy as np
        from PyQt6.QtGui import QImage

        if cv_image is None:
            return

        h, w = cv_image.shape[:2]
        rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        qimg = QImage(rgb.data, w, h, w * 3, QImage.Format.Format_RGB888)
        pix = QPixmap.fromImage(qimg)

        if self._pixmap_item:
            self._scene.removeItem(self._pixmap_item)
        self._pixmap_item = QGraphicsPixmapItem(pix)
        self._scene.addItem(self._pixmap_item)
        self._scene.setSceneRect(self._pixmap_item.boundingRect())

    # --- Radar Projections ---
    def clearRadarMarkers(self):
        for item, _ in self._radar_markers:
            if item.scene():
                self._scene.removeItem(item)
        for item in self._radar_labels:
            if item.scene():
                self._scene.removeItem(item)
        self._radar_markers.clear()
        self._radar_labels.clear()

    def addRadarProjection(self, u: float, v: float, target: dict):
        """Add a projected radar point marker."""
        size = MARKER_SIZE_RADAR
        marker = QGraphicsEllipseItem(u - size / 2, v - size / 2, size, size)
        marker.setPen(QPen(QColor(UI_COLORS['radar']), 3))
        marker.setBrush(QBrush(Qt.BrushStyle.NoBrush))
        marker.setZValue(10)
        self._scene.addItem(marker)
        self._radar_markers.append((marker, target))

        tid = target.get('id', target.get('track_id', '?'))
        label = QGraphicsTextItem(f"R{tid}")
        label.setDefaultTextColor(QColor(UI_COLORS['radar']))
        label.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        label.setPos(u + size / 2 + 4, v - size / 2)
        label.setZValue(10)
        self._scene.addItem(label)
        self._radar_labels.append(label)

    def highlightPendingRadar(self, target: dict):
        """Highlight a radar point as pending selection (yellow)."""
        tid = target.get('id', target.get('track_id'))
        for marker, t in self._radar_markers:
            if t.get('id', t.get('track_id')) == tid:
                marker.setPen(QPen(QColor(UI_COLORS['radar_pending']), 4))
                self._pending_radar_marker = marker
                break

    def clearPendingRadar(self):
        if self._pending_radar_marker:
            self._pending_radar_marker.setPen(QPen(QColor(UI_COLORS['radar']), 3))
            self._pending_radar_marker = None

    # --- Preview Marker ---
    def updatePreview(self, x: float, y: float):
        if self._mode not in ['select_image', 'lane_end']:
            return
        size = MARKER_SIZE_PREVIEW
        if not self._preview_marker:
            self._preview_marker = QGraphicsEllipseItem(0, 0, size, size)
            self._preview_marker.setPen(
                QPen(QColor(UI_COLORS['image_preview']), 2, Qt.PenStyle.DashLine)
            )
            self._preview_marker.setBrush(QBrush(Qt.BrushStyle.NoBrush))
            self._preview_marker.setZValue(50)
            self._scene.addItem(self._preview_marker)
        self._preview_marker.setPos(x - size / 2, y - size / 2)

        if self._mode == 'lane_end' and self._pending_lane_start:
            if not self._pending_lane_line:
                self._pending_lane_line = QGraphicsLineItem()
                self._pending_lane_line.setPen(
                    QPen(QColor(UI_COLORS['lane']), 2, Qt.PenStyle.DashLine)
                )
                self._pending_lane_line.setZValue(14)
                self._scene.addItem(self._pending_lane_line)
            self._pending_lane_line.setLine(
                self._pending_lane_start.x(), self._pending_lane_start.y(), x, y
            )

    def clearPreview(self):
        if self._preview_marker and self._preview_marker.scene():
            self._scene.removeItem(self._preview_marker)
            self._preview_marker = None
        if self._pending_lane_line and self._pending_lane_line.scene():
            self._scene.removeItem(self._pending_lane_line)
            self._pending_lane_line = None

    def mouseMoveEvent(self, event):
        if self._mode in ['select_image', 'lane_end']:
            pos = self.mapToScene(event.position().toPoint())
            self.updatePreview(pos.x(), pos.y())
        super().mouseMoveEvent(event)

    # --- Point Pair Markers ---
    def clearPairMarkers(self):
        for item in self._pair_markers:
            if item.scene():
                self._scene.removeItem(item)
        self._pair_markers.clear()

    def addPairMarker(self, u: float, v: float, pair_index: int, is_radar: bool):
        """Add a completed pair marker with color matching pair index."""
        color = QColor(PAIR_COLORS[pair_index % len(PAIR_COLORS)])

        if is_radar:
            size = MARKER_SIZE_RADAR
            marker = QGraphicsEllipseItem(u - size / 2, v - size / 2, size, size)
            marker.setPen(QPen(color, 3))
            marker.setBrush(QBrush(Qt.BrushStyle.NoBrush))
            marker.setZValue(20)
            self._scene.addItem(marker)
            self._pair_markers.append(marker)
        else:
            size = MARKER_SIZE_IMAGE
            marker = QGraphicsEllipseItem(u - size / 2, v - size / 2, size, size)
            marker.setPen(QPen(color, 2))
            marker.setBrush(QBrush(Qt.BrushStyle.NoBrush))
            marker.setZValue(20)
            self._scene.addItem(marker)
            self._pair_markers.append(marker)

            # Crosshair
            cross_size = size / 2 - 2
            h_line = QGraphicsLineItem(u - cross_size, v, u + cross_size, v)
            h_line.setPen(QPen(color, 2))
            h_line.setZValue(21)
            self._scene.addItem(h_line)
            self._pair_markers.append(h_line)

            v_line = QGraphicsLineItem(u, v - cross_size, u, v + cross_size)
            v_line.setPen(QPen(color, 2))
            v_line.setZValue(21)
            self._scene.addItem(v_line)
            self._pair_markers.append(v_line)

        # Number label
        size_for_label = MARKER_SIZE_RADAR if is_radar else MARKER_SIZE_IMAGE
        label = QGraphicsTextItem(str(pair_index + 1))
        label.setDefaultTextColor(color)
        label.setFont(QFont("Arial", 11, QFont.Weight.Bold))
        label.setPos(u + size_for_label / 2 + 2, v - size_for_label / 2 - 4)
        label.setZValue(20)
        self._scene.addItem(label)
        self._pair_markers.append(label)

    # --- Lane Markers ---
    def clearLaneMarkers(self):
        for item in self._lane_markers:
            if item.scene():
                self._scene.removeItem(item)
        self._lane_markers.clear()
        self._pending_lane_start = None
        self.clearPreview()

    def setLaneStart(self, x: float, y: float):
        self._pending_lane_start = QPointF(x, y)
        size = LANE_POINT_SIZE
        dot = QGraphicsEllipseItem(x - size / 2, y - size / 2, size, size)
        dot.setPen(QPen(QColor(UI_COLORS['lane']), 2))
        dot.setBrush(QBrush(QColor(UI_COLORS['lane'])))
        dot.setZValue(15)
        self._scene.addItem(dot)
        self._lane_markers.append(dot)

    def completeLane(self, end_x: float, end_y: float):
        if not self._pending_lane_start:
            return None
        start = self._pending_lane_start

        size = LANE_POINT_SIZE
        dot = QGraphicsEllipseItem(end_x - size / 2, end_y - size / 2, size, size)
        dot.setPen(QPen(QColor(UI_COLORS['lane']), 2))
        dot.setBrush(QBrush(QColor(UI_COLORS['lane'])))
        dot.setZValue(15)
        self._scene.addItem(dot)
        self._lane_markers.append(dot)

        line = QGraphicsLineItem(start.x(), start.y(), end_x, end_y)
        line.setPen(QPen(QColor(UI_COLORS['lane']), 3))
        line.setZValue(14)
        self._scene.addItem(line)
        self._lane_markers.append(line)

        self._pending_lane_start = None
        self.clearPreview()
        return ((start.x(), start.y()), (end_x, end_y))

    def undoLastLanePoint(self):
        if self._pending_lane_start:
            if self._lane_markers:
                item = self._lane_markers.pop()
                if item.scene():
                    self._scene.removeItem(item)
            self._pending_lane_start = None
            self.clearPreview()
            return True
        return False

    # --- Click Handling ---
    def mousePressEvent(self, event: QMouseEvent):
        if not self._click_mode or event.button() != Qt.MouseButton.LeftButton:
            super().mousePressEvent(event)
            return

        pos = self.mapToScene(event.position().toPoint())
        x, y = pos.x(), pos.y()

        if self._mode == 'select_radar':
            min_dist = float('inf')
            nearest = None
            for marker, target in self._radar_markers:
                rect = marker.rect()
                cx = rect.x() + rect.width() / 2
                cy = rect.y() + rect.height() / 2
                dist = ((cx - x) ** 2 + (cy - y) ** 2) ** 0.5
                if dist < min_dist and dist < 50:
                    min_dist = dist
                    nearest = target
            if nearest:
                self.radarClicked.emit(nearest)

        elif self._mode == 'select_image':
            self.imageClicked.emit(x, y)

        elif self._mode in ('lane_start', 'lane_end'):
            self.clicked.emit(x, y)

    # --- Trajectory Projection ---
    def showTrajectoryProjection(self, u: float, v: float, target_id: int):
        self.clearTrajectoryProjection()
        size = 20
        marker = QGraphicsEllipseItem(u - size / 2, v - size / 2, size, size)
        marker.setPen(QPen(QColor('#FFFF00'), 4))
        marker.setBrush(QBrush(QColor(255, 255, 0, 100)))
        self._scene.addItem(marker)
        self._trajectory_projection.append(marker)

        label = QGraphicsTextItem(f"T{target_id}")
        label.setDefaultTextColor(QColor('#FFFFFF'))
        label.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        label.setPos(u + 12, v - 12)
        self._scene.addItem(label)
        self._trajectory_projection.append(label)

    def clearTrajectoryProjection(self):
        for item in self._trajectory_projection:
            if item.scene():
                self._scene.removeItem(item)
        self._trajectory_projection.clear()

    def showCameraDetection(self, u: float, v: float, target_id: int):
        """Show camera detection position (ground truth) as a square marker."""
        size = 16
        from PyQt6.QtWidgets import QGraphicsRectItem
        marker = QGraphicsRectItem(u - size / 2, v - size / 2, size, size)
        marker.setPen(QPen(QColor('#00FFFF'), 3))
        marker.setBrush(QBrush(QColor(0, 255, 255, 80)))
        self._scene.addItem(marker)
        self._trajectory_projection.append(marker)

        label = QGraphicsTextItem(f"C{target_id}")
        label.setDefaultTextColor(QColor('#00FFFF'))
        label.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        label.setPos(u + 12, v + 8)
        self._scene.addItem(label)
        self._trajectory_projection.append(label)

    def drawConnectionLine(self, u1, v1, u2, v2, color=None):
        line = QGraphicsLineItem(u1, v1, u2, v2)
        c = color if color else QColor('#FFFFFF')
        pen = QPen(c, 2, Qt.PenStyle.DashLine)
        line.setPen(pen)
        self._scene.addItem(line)
        self._trajectory_projection.append(line)
