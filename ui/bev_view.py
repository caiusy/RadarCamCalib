"""
bev_view.py - BEV (Bird's Eye View) viewport widget (PyQt6).

BEV 鸟瞰图视口:
- 栅格坐标范围 X: [-50, 50]m, Y: [0, 300]m
- 右键拖拽平移，滚轮缩放
- 雷达点/标定点对/轨迹/范围多边形 显示
- 拖拽时网格刻度随可见区域更新

Author: caiusy
"""

from typing import Optional, List, Tuple, Dict
from PyQt6.QtWidgets import (
    QGraphicsView, QGraphicsScene, QGraphicsEllipseItem,
    QGraphicsLineItem, QGraphicsTextItem, QGraphicsRectItem,
    QGraphicsPolygonItem
)
from PyQt6.QtCore import Qt, QPointF, QRectF, pyqtSignal
from PyQt6.QtGui import (
    QPixmap, QPen, QBrush, QColor, QFont, QWheelEvent,
    QMouseEvent, QPainter, QPolygonF
)
import math

from visualization.color_map import PAIR_COLORS, UI_COLORS


class BEVViewport(QGraphicsView):
    """Bird's Eye View viewport for radar data visualization.

    坐标系: X=横向(右正), Y=纵向(前方正)
    场景像素: _bev_to_pixel() 将米转为场景像素

    Supports:
    - 雷达点显示 (addRadarPoint)
    - 标定点对标记 (addPairMarker)
    - 轨迹折线 (loadTrajectories)
    - 范围多边形 (drawRangePolygons)
    - 拖拽/缩放
    """

    pointClicked = pyqtSignal(int, int)  # (target_id, frame_id)

    # --- 默认 BEV 坐标范围 (米), 可通过构造函数覆盖 ---
    DEFAULT_X_RANGE = (-50, 50)
    DEFAULT_Y_RANGE = (0, 300)
    PIXELS_PER_METER = 3  # 场景像素/米, 控制分辨率

    def __init__(self, parent=None, x_range=None, y_range=None):
        """初始化 BEV 视口.

        Args:
            parent: 父 widget
            x_range: 横向范围 (x_min, x_max) 米, 默认 (-50, 50)
            y_range: 纵向范围 (y_min, y_max) 米, 默认 (0, 300)
        """
        super().__init__(parent)

        # 可配置的坐标范围
        self.X_RANGE = x_range or self.DEFAULT_X_RANGE
        self.Y_RANGE = y_range or self.DEFAULT_Y_RANGE

        # 根据范围自动计算场景像素大小
        self.BEV_WIDTH = int((self.X_RANGE[1] - self.X_RANGE[0]) * self.PIXELS_PER_METER)
        self.BEV_HEIGHT = int((self.Y_RANGE[1] - self.Y_RANGE[0]) * self.PIXELS_PER_METER)

        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self.setRenderHint(QPainter.RenderHint.Antialiasing)

        self._zoom = 1.0
        self._panning = False
        self._pan_start = None

        # --- 图形元素列表 ---
        self._grid_items: List = []          # 栅格线+标签
        self._radar_markers: List[Tuple[QGraphicsEllipseItem, dict]] = []
        self._radar_labels: List = []
        self._pair_markers: List = []
        self._comparison_pairs: List = []
        self._trajectory_items: List = []
        self._camera_trajectory_items: List = []
        self._trail_items: List = []
        self._trajectory_head: Optional[QGraphicsEllipseItem] = None
        self._trajectory_mode = False
        self._highlighted_items: List = []
        self._range_items: List = []         # 范围多边形

        self._drawGrid()

    # =========================================================================
    # 坐标转换
    # =========================================================================
    def _bev_to_pixel(self, x: float, y: float) -> Tuple[float, float]:
        """BEV 米坐标 → 场景像素坐标.

        X轴: 左到右 (x_min → x_max)
        Y轴: 下到上 (y_min 在底部, y_max 在顶部)
        """
        x_min, x_max = self.X_RANGE
        y_min, y_max = self.Y_RANGE
        px = (x - x_min) / (x_max - x_min) * self.BEV_WIDTH
        py = (1.0 - (y - y_min) / (y_max - y_min)) * self.BEV_HEIGHT
        return px, py

    def _pixel_to_bev(self, px: float, py: float) -> Tuple[float, float]:
        """场景像素坐标 → BEV 米坐标."""
        x_min, x_max = self.X_RANGE
        y_min, y_max = self.Y_RANGE
        x = px / self.BEV_WIDTH * (x_max - x_min) + x_min
        y = (1.0 - py / self.BEV_HEIGHT) * (y_max - y_min) + y_min
        return x, y

    # =========================================================================
    # 栅格绘制 (固定场景坐标, 覆盖完整范围)
    # =========================================================================
    def _drawGrid(self):
        """绘制 BEV 栅格, 覆盖完整 X_RANGE × Y_RANGE."""
        bg_color = QColor(30, 30, 30)
        self._scene.setBackgroundBrush(QBrush(bg_color))

        grid_pen = QPen(QColor(50, 50, 50), 1)
        axis_pen = QPen(QColor(80, 80, 80), 1)
        label_color = QColor(120, 120, 120)
        font = QFont("Arial", 8)

        x_min, x_max = self.X_RANGE
        y_min, y_max = self.Y_RANGE

        # X 轴栅格 (每 10m)
        for x in range(int(x_min), int(x_max) + 1, 10):
            px, _ = self._bev_to_pixel(x, y_min)
            _, py0 = self._bev_to_pixel(x, y_min)
            _, py1 = self._bev_to_pixel(x, y_max)
            pen = axis_pen if x == 0 else grid_pen
            item = self._scene.addLine(px, py0, px, py1, pen)
            self._grid_items.append(item)
            # 标签在底部
            label = QGraphicsTextItem(f"{x}m")
            label.setDefaultTextColor(label_color)
            label.setFont(font)
            label.setPos(px + 2, py0 - 14)
            self._scene.addItem(label)
            self._grid_items.append(label)

        # Y 轴栅格 (每 20m, 从0到300)
        for y in range(int(y_min), int(y_max) + 1, 20):
            _, py = self._bev_to_pixel(x_min, y)
            px0, _ = self._bev_to_pixel(x_min, y)
            px1, _ = self._bev_to_pixel(x_max, y)
            pen = axis_pen if y == 0 else grid_pen
            item = self._scene.addLine(px0, py, px1, py, pen)
            self._grid_items.append(item)
            # 标签在左侧
            label = QGraphicsTextItem(f"{y}m")
            label.setDefaultTextColor(label_color)
            label.setFont(font)
            label.setPos(px0 + 2, py - 16)
            self._scene.addItem(label)
            self._grid_items.append(label)

        # 自车标记 (蓝色方块在原点)
        ego_px, ego_py = self._bev_to_pixel(0, 0)
        ego = QGraphicsRectItem(ego_px - 5, ego_py - 10, 10, 10)
        ego.setPen(QPen(QColor(0, 120, 200), 2))
        ego.setBrush(QBrush(QColor(0, 120, 200)))
        self._scene.addItem(ego)
        self._grid_items.append(ego)

        # 场景范围覆盖完整栅格
        self._scene.setSceneRect(0, 0, self.BEV_WIDTH, self.BEV_HEIGHT)

    # =========================================================================
    # 雷达点显示
    # =========================================================================
    def clearRadarMarkers(self):
        """清除所有雷达点标记."""
        for item, _ in self._radar_markers:
            if item.scene():
                self._scene.removeItem(item)
        for item in self._radar_labels:
            if item.scene():
                self._scene.removeItem(item)
        self._radar_markers.clear()
        self._radar_labels.clear()

    def addRadarPoint(self, x: float, y: float, target: dict, color: str = None):
        """添加一个雷达目标点.

        Args:
            x, y: BEV 坐标 (米)
            target: 目标信息 dict (含 id, class 等)
            color: 可选颜色 hex
        """
        px, py = self._bev_to_pixel(x, y)
        size = 8
        c = QColor(color or UI_COLORS['radar'])
        marker = QGraphicsEllipseItem(px - size/2, py - size/2, size, size)
        marker.setPen(QPen(c, 2))
        marker.setBrush(QBrush(c))
        marker.setZValue(10)
        self._scene.addItem(marker)
        self._radar_markers.append((marker, target))

        # ID 标签
        tid = target.get('id', target.get('track_id', '?'))
        label = QGraphicsTextItem(f"R{tid}")
        label.setDefaultTextColor(c)
        label.setFont(QFont("Arial", 8))
        label.setPos(px + 6, py - 6)
        label.setZValue(10)
        self._scene.addItem(label)
        self._radar_labels.append(label)

    def highlightRadarMarker(self, target: dict):
        """高亮选中的雷达点 (黄色边框)."""
        tid = target.get('id', target.get('track_id'))
        for marker, t in self._radar_markers:
            if t.get('id', t.get('track_id')) == tid:
                marker.setPen(QPen(QColor(UI_COLORS['radar_pending']), 3))
                break

    def clearPendingRadar(self):
        """取消高亮, 恢复默认颜色."""
        for marker, _ in self._radar_markers:
            marker.setPen(QPen(QColor(UI_COLORS['radar']), 2))

    # =========================================================================
    # 标定点对标记
    # =========================================================================
    def addPairMarker(self, x: float, y: float, pair_index: int):
        """添加标定点对标记 (彩色圆点+编号).

        Args:
            x, y: 雷达 BEV 坐标 (米)
            pair_index: 点对编号 (0-based)
        """
        px, py = self._bev_to_pixel(x, y)
        color = QColor(PAIR_COLORS[pair_index % len(PAIR_COLORS)])
        size = 10
        marker = QGraphicsEllipseItem(px - size/2, py - size/2, size, size)
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
        """绘制雷达点 vs 图像反投影点的对比连线."""
        color = QColor(PAIR_COLORS[pair_index % len(PAIR_COLORS)])
        r_px, r_py = self._bev_to_pixel(radar_bev[0], radar_bev[1])
        i_px, i_py = self._bev_to_pixel(image_bev[0], image_bev[1])

        size = 8
        sq = QGraphicsRectItem(i_px - size/2, i_py - size/2, size, size)
        sq.setPen(QPen(color, 2))
        sq.setBrush(QBrush(Qt.BrushStyle.NoBrush))
        sq.setZValue(21)
        self._scene.addItem(sq)
        self._comparison_pairs.append(sq)

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

    # =========================================================================
    # 轨迹显示
    # =========================================================================
    def setTrajectoryMode(self, enabled: bool):
        """设置是否处于轨迹模式."""
        self._trajectory_mode = enabled

    def loadTrajectories(self, trajectories: dict):
        """加载雷达轨迹并绘制为折线.

        Args:
            trajectories: {target_id: [(frame_id, x, y), ...]}
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

                # 轨迹点 (小圆)
                size = 4
                dot = QGraphicsEllipseItem(px - size/2, py - size/2, size, size)
                dot.setPen(QPen(color, 1))
                dot.setBrush(QBrush(color))
                dot.setZValue(10)
                self._scene.addItem(dot)
                self._trajectory_items.append(dot)

                # 连线
                if prev_px is not None:
                    line = QGraphicsLineItem(prev_px, prev_py, px, py)
                    line.setPen(QPen(color, 1))
                    line.setZValue(9)
                    self._scene.addItem(line)
                    self._trajectory_items.append(line)

                prev_px, prev_py = px, py

            # 首点标签
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
        """加载相机轨迹 (方块标记 + 虚线)."""
        for i, (tid, points) in enumerate(trajectories.items()):
            if not points:
                continue
            color = QColor(PAIR_COLORS[i % len(PAIR_COLORS)])
            color.setAlpha(180)

            prev_px, prev_py = None, None
            for pt in points:
                if isinstance(pt, tuple) and len(pt) >= 4:
                    x, y = pt[3], pt[4]
                elif isinstance(pt, tuple) and len(pt) == 2:
                    x, y = pt
                else:
                    continue

                px, py = self._bev_to_pixel(x, y)
                size = 5
                sq = QGraphicsRectItem(px - size/2, py - size/2, size, size)
                sq.setPen(QPen(color, 1))
                sq.setBrush(QBrush(Qt.BrushStyle.NoBrush))
                sq.setZValue(8)
                self._scene.addItem(sq)
                self._camera_trajectory_items.append(sq)

                if prev_px is not None:
                    line = QGraphicsLineItem(prev_px, prev_py, px, py)
                    line.setPen(QPen(color, 1, Qt.PenStyle.DashLine))
                    line.setZValue(7)
                    self._scene.addItem(line)
                    self._camera_trajectory_items.append(line)

                prev_px, prev_py = px, py

    def clearTrajectories(self):
        """清除所有轨迹."""
        for item in self._trajectory_items + self._camera_trajectory_items:
            if item.scene():
                self._scene.removeItem(item)
        self._trajectory_items.clear()
        self._camera_trajectory_items.clear()

    # =========================================================================
    # 轨迹回放 (Trail)
    # =========================================================================
    def drawTrajectoryTrail(self, trail_points, color: QColor = None):
        """绘制回放轨迹尾迹.

        Args:
            trail_points: [(x, y), ...] BEV 米坐标
            color: 尾迹颜色
        """
        self.clearTrails()
        c = color or QColor(0, 255, 255)

        prev_px, prev_py = None, None
        for pt in trail_points:
            if isinstance(pt, tuple) and len(pt) >= 2:
                x, y = pt[0], pt[1]
            else:
                continue
            px, py = self._bev_to_pixel(x, y)

            size = 3
            dot = QGraphicsEllipseItem(px - size/2, py - size/2, size, size)
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
        """绘制回放当前位置大圆标记."""
        if self._trajectory_head and self._trajectory_head.scene():
            self._scene.removeItem(self._trajectory_head)

        c = color or QColor(0, 255, 255)
        px, py = self._bev_to_pixel(x, y)
        size = 12
        self._trajectory_head = QGraphicsEllipseItem(
            px - size/2, py - size/2, size, size
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

    # =========================================================================
    # 范围多边形 (感知区域可视化)
    # =========================================================================
    def drawRangePolygons(self, polygons: list):
        """绘制雷达感知范围多边形.

        Args:
            polygons: [{"vertices": [[x,y], ...], "area_m2": float}, ...]
        """
        self.clearRangePolygons()

        for poly_data in polygons:
            vertices = poly_data.get("vertices", [])
            if len(vertices) < 3:
                continue

            # 构建 QPolygonF
            qpoly = QPolygonF()
            for vx, vy in vertices:
                px, py = self._bev_to_pixel(vx, vy)
                qpoly.append(QPointF(px, py))

            # 半透明绿色填充
            poly_item = QGraphicsPolygonItem(qpoly)
            fill_color = QColor(0, 200, 100, 40)
            border_color = QColor(0, 200, 100, 180)
            poly_item.setBrush(QBrush(fill_color))
            poly_item.setPen(QPen(border_color, 2))
            poly_item.setZValue(5)  # 在轨迹之下
            self._scene.addItem(poly_item)
            self._range_items.append(poly_item)

            # 面积标签
            area = poly_data.get("area_m2", 0)
            if area > 0:
                # 标签放在多边形质心
                cx = sum(v[0] for v in vertices) / len(vertices)
                cy = sum(v[1] for v in vertices) / len(vertices)
                cpx, cpy = self._bev_to_pixel(cx, cy)
                label = QGraphicsTextItem(f"{area:.0f}m²")
                label.setDefaultTextColor(QColor(0, 200, 100))
                label.setFont(QFont("Arial", 9, QFont.Weight.Bold))
                label.setPos(cpx - 15, cpy - 10)
                label.setZValue(6)
                self._scene.addItem(label)
                self._range_items.append(label)

    def clearRangePolygons(self):
        """清除范围多边形."""
        for item in self._range_items:
            if item.scene():
                self._scene.removeItem(item)
        self._range_items.clear()

    # =========================================================================
    # 清除
    # =========================================================================
    def highlightTrajectoryPoint(self, target_id: int, frame_id: int):
        self.clearHighlights()
        self.pointClicked.emit(target_id, frame_id)

    def clearHighlights(self):
        for item in self._highlighted_items:
            if item.scene():
                self._scene.removeItem(item)
        self._highlighted_items.clear()

    def clearAll(self):
        """清除所有标记 (保留栅格)."""
        self.clearRadarMarkers()
        self.clearPairs()
        self.clearTrajectories()
        self.clearTrails()
        self.clearHighlights()
        self.clearRangePolygons()

    # =========================================================================
    # 缩放 + 平移
    # =========================================================================
    def wheelEvent(self, event: QWheelEvent):
        """滚轮缩放 (栅格刻度不变, 只改变视图比例)."""
        factor = 1.2 if event.angleDelta().y() > 0 else 1 / 1.2
        new_zoom = self._zoom * factor
        if 0.1 <= new_zoom <= 20.0:
            self._zoom = new_zoom
            self.scale(factor, factor)
        event.accept()

    def mousePressEvent(self, event: QMouseEvent):
        """右键开始拖拽平移."""
        if event.button() == Qt.MouseButton.RightButton:
            self._panning = True
            self._pan_start = event.position()
            self.setCursor(Qt.CursorShape.ClosedHandCursor)
            event.accept()
        else:
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        """拖拽平移 — 移动滚动条, 栅格随内容移动."""
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
