"""
viewports.py - Custom QGraphicsView widgets for image and BEV display
"""

from typing import Optional, List, Tuple, Dict
from PyQt6.QtWidgets import (
    QGraphicsView, QGraphicsScene, QGraphicsPixmapItem,
    QGraphicsEllipseItem, QGraphicsLineItem, QGraphicsTextItem
)
from PyQt6.QtCore import Qt, QPointF, pyqtSignal
from PyQt6.QtGui import (
    QPixmap, QPen, QBrush, QColor, QFont, QWheelEvent, QMouseEvent, QPainter
)
import os

from config import COLORS, PAIR_COLORS, MARKER_SIZE_RADAR, MARKER_SIZE_IMAGE, MARKER_SIZE_PREVIEW, LANE_POINT_SIZE


class ZoomPanView(QGraphicsView):
    """
    Base graphics view with:
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
        self.setDragMode(QGraphicsView.DragMode.NoDrag)  # We handle drag manually
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self._zoom = 1.0
        self._click_mode = False
        self._panning = False
        self._pan_start = None
    
    def setClickMode(self, enabled: bool):
        """Enable/disable left-click point selection mode."""
        self._click_mode = enabled
        if enabled:
            self.setCursor(Qt.CursorShape.CrossCursor)
        else:
            self.setCursor(Qt.CursorShape.ArrowCursor)
    
    def wheelEvent(self, event: QWheelEvent):
        """Zoom in/out with scroll wheel."""
        factor = 1.2 if event.angleDelta().y() > 0 else 1/1.2
        new_zoom = self._zoom * factor
        if 0.1 <= new_zoom <= 20.0:
            self._zoom = new_zoom
            self.scale(factor, factor)
        event.accept()

    
    def mousePressEvent(self, event: QMouseEvent):
        """Handle mouse press: left-click for selection, right-click for pan."""
        if event.button() == Qt.MouseButton.RightButton:
            # Start panning
            self._panning = True
            self._pan_start = event.position()
            self.setCursor(Qt.CursorShape.ClosedHandCursor)
            event.accept()
        elif self._click_mode and event.button() == Qt.MouseButton.LeftButton:
            # Point selection
            pos = self.mapToScene(event.position().toPoint())
            self.clicked.emit(pos.x(), pos.y())
        else:
            super().mousePressEvent(event)
    
    def mouseMoveEvent(self, event):
        """Handle mouse move for panning."""
        if self._panning and self._pan_start is not None:
            delta = event.position() - self._pan_start
            self._pan_start = event.position()
            # Scroll the view
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
        """Handle mouse release."""
        if event.button() == Qt.MouseButton.RightButton and self._panning:
            self._panning = False
            self._pan_start = None
            # Restore cursor
            if self._click_mode:
                self.setCursor(Qt.CursorShape.CrossCursor)
            else:
                self.setCursor(Qt.CursorShape.ArrowCursor)
            event.accept()
        else:
            super().mouseReleaseEvent(event)
    
    def fitContent(self):
        """Fit scene content to view."""
        self._zoom = 1.0
        self.resetTransform()
        if self.scene():
            rect = self.scene().itemsBoundingRect()
            if rect.isValid():
                self.fitInView(rect, Qt.AspectRatioMode.KeepAspectRatio)



class ImageViewport(ZoomPanView):
    """
    Viewport for displaying camera image with:
    - Projected radar points (clickable)
    - Point pair markers
    - Lane lines
    - Mouse tracking for preview
    """
    
    radarClicked = pyqtSignal(dict)      # Clicked on a radar marker
    imageClicked = pyqtSignal(float, float)  # Clicked to select image point
    mouseMoved = pyqtSignal(float, float)    # Mouse moved (for preview)
    
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
        
        self._mode = 'normal'  # 'normal', 'select_radar', 'select_image', 'lane_start', 'lane_end'
    
    def setMode(self, mode: str):
        self._mode = mode
        self.setClickMode(mode != 'normal')
        
        # Clear preview when changing mode
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
    
    # -------------------------------------------------------------------------
    # Radar Projections
    # -------------------------------------------------------------------------
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
        marker = QGraphicsEllipseItem(u - size/2, v - size/2, size, size)
        marker.setPen(QPen(QColor(COLORS['radar']), 3))
        marker.setBrush(QBrush(Qt.BrushStyle.NoBrush))
        marker.setZValue(10)
        self._scene.addItem(marker)
        self._radar_markers.append((marker, target))
        
        # Label
        tid = target.get('id', '?')
        label = QGraphicsTextItem(f"R{tid}")
        label.setDefaultTextColor(QColor(COLORS['radar']))
        label.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        label.setPos(u + size/2 + 4, v - size/2)
        label.setZValue(10)
        self._scene.addItem(label)
        self._radar_labels.append(label)
    
    def highlightPendingRadar(self, target: dict):
        """Highlight a radar point as pending selection (yellow)."""
        # Find the marker for this target
        for marker, t in self._radar_markers:
            if t.get('id') == target.get('id'):
                marker.setPen(QPen(QColor(COLORS['radar_pending']), 4))
                self._pending_radar_marker = marker
                break
    
    def clearPendingRadar(self):
        """Reset pending radar highlight."""
        if self._pending_radar_marker:
            self._pending_radar_marker.setPen(QPen(QColor(COLORS['radar']), 3))
            self._pending_radar_marker = None
    
    # -------------------------------------------------------------------------
    # Preview Marker (follows mouse)
    # -------------------------------------------------------------------------
    def updatePreview(self, x: float, y: float):
        """Update preview circle at mouse position."""
        if self._mode not in ['select_image', 'lane_end']:
            return
        
        size = MARKER_SIZE_PREVIEW
        if not self._preview_marker:
            self._preview_marker = QGraphicsEllipseItem(0, 0, size, size)
            self._preview_marker.setPen(QPen(QColor(COLORS['image_preview']), 2, Qt.PenStyle.DashLine))
            self._preview_marker.setBrush(QBrush(Qt.BrushStyle.NoBrush))
            self._preview_marker.setZValue(50)
            self._scene.addItem(self._preview_marker)
        
        self._preview_marker.setPos(x - size/2, y - size/2)
        
        # Update pending lane line preview
        if self._mode == 'lane_end' and self._pending_lane_start:
            if not self._pending_lane_line:
                self._pending_lane_line = QGraphicsLineItem()
                self._pending_lane_line.setPen(QPen(QColor(COLORS['lane']), 2, Qt.PenStyle.DashLine))
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
    
    # -------------------------------------------------------------------------
    # Point Pair Markers
    # -------------------------------------------------------------------------
    def clearPairMarkers(self):
        for item in self._pair_markers:
            if item.scene():
                self._scene.removeItem(item)
        self._pair_markers.clear()
    
    def addPairMarker(self, u: float, v: float, pair_index: int, is_radar: bool):
        """Add a completed pair marker with color matching pair index."""
        color = QColor(PAIR_COLORS[pair_index % len(PAIR_COLORS)])
        
        if is_radar:
            # Radar point: larger circle, no fill
            size = MARKER_SIZE_RADAR
            marker = QGraphicsEllipseItem(u - size/2, v - size/2, size, size)
            marker.setPen(QPen(color, 3))
            marker.setBrush(QBrush(Qt.BrushStyle.NoBrush))
            marker.setZValue(20)
            self._scene.addItem(marker)
            self._pair_markers.append(marker)
        else:
            # Image/pixel point: smaller circle with crosshair
            size = MARKER_SIZE_IMAGE  # 16px, smaller than radar's 20px
            
            # Circle
            marker = QGraphicsEllipseItem(u - size/2, v - size/2, size, size)
            marker.setPen(QPen(color, 2))
            marker.setBrush(QBrush(Qt.BrushStyle.NoBrush))
            marker.setZValue(20)
            self._scene.addItem(marker)
            self._pair_markers.append(marker)
            
            # Crosshair in center
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
        label.setPos(u + size_for_label/2 + 2, v - size_for_label/2 - 4)
        label.setZValue(20)
        self._scene.addItem(label)
        self._pair_markers.append(label)

    
    # -------------------------------------------------------------------------
    # Lane Markers
    # -------------------------------------------------------------------------
    def clearLaneMarkers(self):
        for item in self._lane_markers:
            if item.scene():
                self._scene.removeItem(item)
        self._lane_markers.clear()
        self._pending_lane_start = None
        self.clearPreview()
    
    def setLaneStart(self, x: float, y: float):
        """Set the start point of a new lane."""
        self._pending_lane_start = QPointF(x, y)
        
        # Draw start point
        size = LANE_POINT_SIZE
        dot = QGraphicsEllipseItem(x - size/2, y - size/2, size, size)
        dot.setPen(QPen(QColor(COLORS['lane']), 2))
        dot.setBrush(QBrush(QColor(COLORS['lane'])))
        dot.setZValue(15)
        self._scene.addItem(dot)
        self._lane_markers.append(dot)
    
    def completeLane(self, end_x: float, end_y: float) -> Optional[Tuple[Tuple[float, float], Tuple[float, float]]]:
        """Complete lane with end point. Returns ((start_x, start_y), (end_x, end_y))."""
        if not self._pending_lane_start:
            return None
        
        start = self._pending_lane_start
        
        # Draw end point
        size = LANE_POINT_SIZE
        dot = QGraphicsEllipseItem(end_x - size/2, end_y - size/2, size, size)
        dot.setPen(QPen(QColor(COLORS['lane']), 2))
        dot.setBrush(QBrush(QColor(COLORS['lane'])))
        dot.setZValue(15)
        self._scene.addItem(dot)
        self._lane_markers.append(dot)
        
        # Draw final line
        line = QGraphicsLineItem(start.x(), start.y(), end_x, end_y)
        line.setPen(QPen(QColor(COLORS['lane']), 3))
        line.setZValue(14)
        self._scene.addItem(line)
        self._lane_markers.append(line)
        
        # Clear pending
        self._pending_lane_start = None
        self.clearPreview()
        
        return ((start.x(), start.y()), (end_x, end_y))
    
    def undoLastLanePoint(self):
        """Undo the last lane start point if pending."""
        if self._pending_lane_start:
            # Remove the start dot
            if self._lane_markers:
                item = self._lane_markers.pop()
                if item.scene():
                    self._scene.removeItem(item)
            self._pending_lane_start = None
            self.clearPreview()
            return True
        return False
    
    # -------------------------------------------------------------------------
    # Click Handling
    # -------------------------------------------------------------------------
    def mousePressEvent(self, event: QMouseEvent):
        if not self._click_mode or event.button() != Qt.MouseButton.LeftButton:
            super().mousePressEvent(event)
            return
        
        pos = self.mapToScene(event.position().toPoint())
        x, y = pos.x(), pos.y()
        
        if self._mode == 'select_radar':
            # Find clicked radar point
            min_dist = float('inf')
            nearest = None
            for marker, target in self._radar_markers:
                rect = marker.rect()
                cx = rect.x() + rect.width() / 2
                cy = rect.y() + rect.height() / 2
                dist = ((cx - x)**2 + (cy - y)**2)**0.5
                if dist < min_dist and dist < 50:
                    min_dist = dist
                    nearest = target
            if nearest:
                self.radarClicked.emit(nearest)
        
        elif self._mode == 'select_image':
            self.imageClicked.emit(x, y)
        
        elif self._mode == 'lane_start':
            self.clicked.emit(x, y)
        
        elif self._mode == 'lane_end':
            self.clicked.emit(x, y)


class BEVViewport(ZoomPanView):
    """Viewport for Bird's Eye View radar visualization."""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        
        self.x_range = (0, 60)    # Forward (m)
        self.y_range = (-15, 15)  # Lateral (m)
        self._scale_factor = 10           # px/m
        
        self._grid_items: List = []
        self._radar_items: List = []
        self._pair_items: List = []
        
        self._drawGrid()
    
    def _drawGrid(self):
        for item in self._grid_items:
            if item.scene():
                self._scene.removeItem(item)
        self._grid_items.clear()
        
        x0, x1 = self.x_range
        y0, y1 = self.y_range
        
        # Grid lines (x is forward, y is lateral)
        # Scene: x_scene = y_radar * scale, y_scene = -x_radar * scale
        for x in range(x0, x1 + 1, 10):
            sx0, sy = self._toScene(x, y0)
            sx1, _ = self._toScene(x, y1)
            color = '#444' if x % 20 == 0 else '#2a2a2a'
            line = QGraphicsLineItem(sx0, sy, sx1, sy)
            line.setPen(QPen(QColor(color), 1))
            self._scene.addItem(line)
            self._grid_items.append(line)
            
            if x > 0:
                label = QGraphicsTextItem(f"{x}m")
                label.setDefaultTextColor(QColor('#555'))
                label.setFont(QFont("Arial", 8))
                label.setPos(sx1 + 5, sy - 8)
                self._scene.addItem(label)
                self._grid_items.append(label)
        
        for y in range(-10, 11, 10):
            sx, sy0 = self._toScene(x0, y)
            _, sy1 = self._toScene(x1, y)
            color = '#666' if y == 0 else '#2a2a2a'
            width = 2 if y == 0 else 1
            line = QGraphicsLineItem(sx, sy0, sx, sy1)
            line.setPen(QPen(QColor(color), width))
            self._scene.addItem(line)
            self._grid_items.append(line)
        
        # Ego vehicle
        from PyQt6.QtWidgets import QGraphicsRectItem
        ego = QGraphicsRectItem(-8, -4, 16, 8)
        ego.setPen(QPen(QColor(COLORS['accent']), 2))
        ego.setBrush(QBrush(QColor(COLORS['accent'])))
        self._scene.addItem(ego)
        self._grid_items.append(ego)
        
        # Scene rect
        margin = 30
        w = (y1 - y0) * self._scale_factor + 2 * margin
        h = (x1 - x0) * self._scale_factor + 2 * margin
        self._scene.setSceneRect(y0 * self._scale_factor - margin, -x1 * self._scale_factor - margin, w, h)
    
    def _toScene(self, rx: float, ry: float) -> Tuple[float, float]:
        return (ry * self._scale_factor, -rx * self._scale_factor)
    
    def clearRadar(self):
        for item in self._radar_items:
            if item.scene():
                self._scene.removeItem(item)
        self._radar_items.clear()
    
    def clearPairs(self):
        for item in self._pair_items:
            if item.scene():
                self._scene.removeItem(item)
        self._pair_items.clear()
    
    def loadRadarData(self, data: dict):
        self.clearRadar()
        for t in data.get('targets', []):
            rx, ry = t.get('x', 0), t.get('y', 0)
            sx, sy = self._toScene(rx, ry)
            
            size = 10
            dot = QGraphicsEllipseItem(sx - size/2, sy - size/2, size, size)
            dot.setPen(QPen(QColor(COLORS['radar']), 1))
            dot.setBrush(QBrush(QColor(COLORS['radar'])))
            self._scene.addItem(dot)
            self._radar_items.append(dot)
            
            label = QGraphicsTextItem(f"#{t.get('id', '?')}")
            label.setDefaultTextColor(QColor(COLORS['radar']))
            label.setFont(QFont("Arial", 8))
            label.setPos(sx + 8, sy - 8)
            self._scene.addItem(label)
            self._radar_items.append(label)
    
    def addPairMarker(self, rx: float, ry: float, pair_index: int):
        sx, sy = self._toScene(rx, ry)
        color = QColor(PAIR_COLORS[pair_index % len(PAIR_COLORS)])
        
        size = 18
        ring = QGraphicsEllipseItem(sx - size/2, sy - size/2, size, size)
        ring.setPen(QPen(color, 3))
        self._scene.addItem(ring)
        self._pair_items.append(ring)
        
        label = QGraphicsTextItem(str(pair_index + 1))
        label.setDefaultTextColor(color)
        label.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        label.setPos(sx + 10, sy - 12)
        self._scene.addItem(label)
        self._pair_items.append(label)
    
    def addRadarBEVPoint(self, x_bev: float, y_bev: float, label: str, color: str = '#ff00ff'):
        """Add a radar point in BEV coordinates (magenta circle)."""
        try:
            sx, sy = self._toScene(x_bev, y_bev)
            size = 12
            dot = QGraphicsEllipseItem(sx - size/2, sy - size/2, size, size)
            dot.setPen(QPen(QColor(color), 2))
            dot.setBrush(QBrush(QColor(color)))
            self._scene.addItem(dot)
            self._radar_items.append(dot)
            text = QGraphicsTextItem(label)
            text.setDefaultTextColor(QColor(color))
            text.setFont(QFont("Arial", 10, QFont.Weight.Bold))
            text.setPos(sx + 10, sy - 10)
            self._scene.addItem(text)
            self._radar_items.append(text)
        except Exception as e:
            print(f"addRadarBEVPoint error: {e}")
    
    def addImageBEVPoint(self, x_bev: float, y_bev: float, label: str, color: str = '#ffff00'):
        """Add an image point in BEV (yellow cross)."""
        try:
            sx, sy = self._toScene(x_bev, y_bev)
            size = 10
            line1 = QGraphicsLineItem(sx - size, sy, sx + size, sy)
            line1.setPen(QPen(QColor(color), 3))
            self._scene.addItem(line1)
            self._pair_items.append(line1)
            line2 = QGraphicsLineItem(sx, sy - size, sx, sy + size)
            line2.setPen(QPen(QColor(color), 3))
            self._scene.addItem(line2)
            self._pair_items.append(line2)
            text = QGraphicsTextItem(label)
            text.setDefaultTextColor(QColor(color))
            text.setFont(QFont("Arial", 10))
            text.setPos(sx + 10, sy + 10)
            self._scene.addItem(text)
            self._pair_items.append(text)
        except Exception as e:
            print(f"addImageBEVPoint error: {e}")
    
    def addComparisonPair(self, radar_bev: tuple, image_bev: tuple, pair_index: int):
        """Add comparison pair in BEV."""
        try:
            # Radar point overlap fix: Don't redraw radar point (it's already drawn in step 1/2)
            # self.addRadarBEVPoint(radar_bev[0], radar_bev[1], f"R{pair_index+1}", '#ff00ff')
            
            self.addImageBEVPoint(image_bev[0], image_bev[1], f"I{pair_index+1}", '#ffff00')
            sx1, sy1 = self._toScene(radar_bev[0], radar_bev[1])
            sx2, sy2 = self._toScene(image_bev[0], image_bev[1])
            line = QGraphicsLineItem(sx1, sy1, sx2, sy2)
            pen = QPen(QColor('#ffffff'), 2, Qt.PenStyle.DashLine)
            line.setPen(pen)
            self._scene.addItem(line)
            self._pair_items.append(line)
        except Exception as e:
            print(f"addComparisonPair error: {e}")
    
    def clearAll(self):
        """Clear all BEV items."""
        self.clearRadar()
        self.clearPairs()
