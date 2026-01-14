"""
viewports.py - Custom QGraphicsView widgets for image and BEV display
viewports.py - 用于图像和BEV显示的自定义QGraphicsView组件
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
    - Scroll wheel: zoom in/out (滚轮缩放)
    - Right-click + drag: pan canvas (右键拖拽平移)
    - Left-click: point selection (when click mode enabled) (左键点击选点)
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
    用于显示相机图像的视口，包含:
    - Projected radar points (clickable) (投影的雷达点 - 可点击)
    - Point pair markers (点对标记)
    - Lane lines (车道线)
    - Mouse tracking for preview (用于预览的鼠标跟踪)
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
    """Viewport for Bird's Eye View radar visualization. (雷达鸟瞰图可视化视口)"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        
        # New Range Definition:
        # X: Lateral (Right is positive)
        # Y: Forward (Forward is positive)
        self.x_range = (-20, 20)  # Lateral (m)
        self.y_range = (0, 160)   # Forward (m)
        self._scale_factor = 6    # px/m (Adjusted for larger range)
        
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
        
        # Grid lines
        # X lines (Forward lines, constant X)
        for x in range(x0, x1 + 1, 10):
            sx, sy0 = self._toScene(x, y0)
            sx_end, sy1 = self._toScene(x, y1)
            
            color = '#444' if x == 0 else '#2a2a2a'
            width = 2 if x == 0 else 1
            
            line = QGraphicsLineItem(sx, sy0, sx_end, sy1)
            line.setPen(QPen(QColor(color), width))
            self._scene.addItem(line)
            self._grid_items.append(line)
            
            if x != 0:
                label = QGraphicsTextItem(f"{x}m")
                label.setDefaultTextColor(QColor('#555'))
                label.setFont(QFont("Arial", 8))
                label.setPos(sx + 2, sy0 - 20) # Pos at bottom
                self._scene.addItem(label)
                self._grid_items.append(label)

        # Y lines (Lateral lines, constant Y)
        for y in range(y0, y1 + 1, 20):
            sx0, sy = self._toScene(x0, y)
            sx1, sy_end = self._toScene(x1, y)
            
            color = '#444' if y % 40 == 0 else '#2a2a2a'
            line = QGraphicsLineItem(sx0, sy, sx1, sy)
            line.setPen(QPen(QColor(color), 1))
            self._scene.addItem(line)
            self._grid_items.append(line)
            
            label = QGraphicsTextItem(f"{y}m")
            label.setDefaultTextColor(QColor('#555'))
            label.setFont(QFont("Arial", 8))
            label.setPos(sx0 - 25, sy - 10)
            self._scene.addItem(label)
            self._grid_items.append(label)
        
        # Ego vehicle (Center bottom)
        from PyQt6.QtWidgets import QGraphicsRectItem
        # Vehicle is at (0, 0). Width=2m, Length=4m?
        # Scene coords: (0,0) is center-bottom.
        ego_w_px = 2.0 * self._scale_factor
        ego_l_px = 4.0 * self._scale_factor
        ego = QGraphicsRectItem(-ego_w_px/2, -ego_l_px, ego_w_px, ego_l_px) 
        ego.setPen(QPen(QColor(COLORS['accent']), 2))
        ego.setBrush(QBrush(QColor(COLORS['accent'])))
        self._scene.addItem(ego)
        self._grid_items.append(ego)
        
        # Scene rect updates
        # Scene X: x0*s to x1*s. (-120 to 120)
        # Scene Y: -y1*s to -y0*s. (-960 to 0)
        margin = 50
        w = (x1 - x0) * self._scale_factor + 2 * margin
        h = (y1 - y0) * self._scale_factor + 2 * margin
        # Top-Left of Rect
        rect_x = x0 * self._scale_factor - margin
        rect_y = -y1 * self._scale_factor - margin
        self._scene.setSceneRect(rect_x, rect_y, w, h)
    
    def _toScene(self, x_bev: float, y_bev: float) -> Tuple[float, float]:
        """
        Convert BEV coordinates (X=Right, Y=Forward) to Scene coordinates.
        Scene: X=Right, Y=Down.
        """
        return (x_bev * self._scale_factor, -y_bev * self._scale_factor)
    
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
        """
        Load raw radar data.
        Raw Data assumed: X=Forward, Y=Left.
        Target BEV View: X=Right, Y=Forward.
        """
        self.clearRadar()
        for t in data.get('targets', []):
            rx_raw, ry_raw = t.get('x', 0), t.get('y', 0)
            
            # Convert raw (Forward, Left) to BEV (Right, Forward)
            x_bev = -ry_raw
            y_bev = rx_raw
            
            sx, sy = self._toScene(x_bev, y_bev)
            
            size = 8
            dot = QGraphicsEllipseItem(sx - size/2, sy - size/2, size, size)
            dot.setPen(QPen(QColor(COLORS['radar']), 1))
            dot.setBrush(QBrush(QColor(COLORS['radar'])))
            # Store ID in data
            dot.setData(0, t.get('id')) 
            
            self._scene.addItem(dot)
            self._radar_items.append(dot)
            
            label = QGraphicsTextItem(f"#{t.get('id', '?')}")
            label.setDefaultTextColor(QColor(COLORS['radar']))
            label.setFont(QFont("Arial", 8))
            label.setPos(sx + 6, sy - 6)
            self._scene.addItem(label)
            self._radar_items.append(label)

    def highlightRadarMarker(self, target: dict):
        """Highlight a radar point by ID."""
        tid = target.get('id')
        for item in self._radar_items:
            if isinstance(item, QGraphicsEllipseItem):
                if item.data(0) == tid:
                    item.setPen(QPen(QColor(COLORS['radar_pending']), 3))
                    item.setBrush(QBrush(QColor(COLORS['radar_pending'])))

    def clearPendingRadar(self):
        """Reset radar highlights."""
        for item in self._radar_items:
            if isinstance(item, QGraphicsEllipseItem):
                # Reset to normal
                item.setPen(QPen(QColor(COLORS['radar']), 1))
                item.setBrush(QBrush(QColor(COLORS['radar'])))

    
    def addPairMarker(self, radar_x_bev: float, radar_y_bev: float, pair_index: int):
        """
        Add marker for a pair. 
        Args assumed to be already in BEV frame (X=Right, Y=Forward).
        """
        sx, sy = self._toScene(radar_x_bev, radar_y_bev)
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
        """Add a radar point in BEV coordinates (X=Right, Y=Forward)."""
        try:
            sx, sy = self._toScene(x_bev, y_bev)
            size = 10
            dot = QGraphicsEllipseItem(sx - size/2, sy - size/2, size, size)
            dot.setPen(QPen(QColor(color), 2))
            dot.setBrush(QBrush(QColor(color)))
            self._scene.addItem(dot)
            self._radar_items.append(dot)
            
            if label:
                text = QGraphicsTextItem(label)
                text.setDefaultTextColor(QColor(color))
                text.setFont(QFont("Arial", 8))
                text.setPos(sx + 6, sy - 6)
                self._scene.addItem(text)
                self._radar_items.append(text)
        except Exception as e:
            print(f"addRadarBEVPoint error: {e}")
    
    def addImageBEVPoint(self, x_bev: float, y_bev: float, label: str, color: str = '#ffff00'):
        """Add an image point in BEV (yellow cross)."""
        try:
            sx, sy = self._toScene(x_bev, y_bev)
            size = 8
            line1 = QGraphicsLineItem(sx - size, sy, sx + size, sy)
            line1.setPen(QPen(QColor(color), 2))
            self._scene.addItem(line1)
            self._pair_items.append(line1)
            line2 = QGraphicsLineItem(sx, sy - size, sx, sy + size)
            line2.setPen(QPen(QColor(color), 2))
            self._scene.addItem(line2)
            self._pair_items.append(line2)
            
            if label:
                text = QGraphicsTextItem(label)
                text.setDefaultTextColor(QColor(color))
                text.setFont(QFont("Arial", 8))
                text.setPos(sx + 8, sy + 8)
                self._scene.addItem(text)
                self._pair_items.append(text)
        except Exception as e:
            print(f"addImageBEVPoint error: {e}")
    
    def addComparisonPair(self, radar_bev: tuple, image_bev: tuple, pair_index: int):
        """Add comparison pair in BEV."""
        try:
            self.addRadarBEVPoint(radar_bev[0], radar_bev[1], "", PAIR_COLORS[pair_index % len(PAIR_COLORS)])
            self.addImageBEVPoint(image_bev[0], image_bev[1], "", "#FFFFFF")
            
            # Draw line
            sx1, sy1 = self._toScene(radar_bev[0], radar_bev[1])
            sx2, sy2 = self._toScene(image_bev[0], image_bev[1])
            line = QGraphicsLineItem(sx1, sy1, sx2, sy2)
            line.setPen(QPen(QColor('#FFFFFF'), 1, Qt.PenStyle.DashLine))
            self._scene.addItem(line)
            self._pair_items.append(line)
        except Exception as e:
            print(f"addComparisonPair error: {e}")
    
    def clearAll(self):
        self.clearPairs()
        self.clearRadar()
