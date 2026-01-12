#!/usr/bin/env python3
"""
Radar-Camera Fusion Calibration System

Main application window - integrates all modules.

Modules:
- config.py: Colors, styles, constants
- backend.py: Data loading, calibration, export
- viewports.py: Image and BEV viewport widgets
- operations.py: Point pair and lane logic with undo

Author: AI Assistant
Date: 2026-01-13
"""

import sys
import os

os.environ["QT_ENABLE_HIGHDPI_SCALING"] = "1"

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QFrame, QPushButton, QSlider, QLabel, QSplitter, QFileDialog,
    QStatusBar, QDoubleSpinBox, QGroupBox, QMessageBox, QRadioButton,
    QSizePolicy
)
from PyQt6.QtCore import Qt

from config import COLORS, QSS, MAX_POINT_PAIRS
from backend import DataManager, Calibration, DataExporter
from viewports import ImageViewport, BEVViewport
from operations import OperationsController, AppMode


class MainWindow(QMainWindow):
    """Main application window."""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Radar-Camera Fusion Calibration System")
        self.setMinimumSize(1280, 720)
        
        # Backend
        self.data_mgr = DataManager()
        self.calibration = Calibration()
        
        # Operations
        self.ops = OperationsController()
        
        self._setupUI()
        self._connectSignals()
        self._updateModeUI()
    
    def _setupUI(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        
        self._createTopBar(layout)
        self._createMainArea(layout)
        
        self.statusbar = QStatusBar()
        self.setStatusBar(self.statusbar)
        self.statusbar.showMessage("Load Sync JSON to start")
    
    def _createTopBar(self, parent):
        bar = QFrame()
        bar.setObjectName("TopBar")
        bar.setMinimumHeight(110)
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(10, 8, 10, 8)
        lay.setSpacing(12)
        
        # === Data Group ===
        g1 = QGroupBox("Data")
        l1 = QHBoxLayout(g1)
        self.btn_sync = QPushButton("📂 Sync JSON")
        l1.addWidget(self.btn_sync)
        self.btn_calib = QPushButton("📐 Coarse TXT")
        l1.addWidget(self.btn_calib)
        self.calib_light = QLabel("●")
        self.calib_light.setStyleSheet("color: #555; font-size: 20px;")
        self.calib_light.setToolTip("Not loaded")
        l1.addWidget(self.calib_light)
        lay.addWidget(g1)
        
        # === Batch Navigation ===
        g2 = QGroupBox("Batch")
        l2 = QHBoxLayout(g2)
        self.btn_prev = QPushButton("◀")
        self.btn_prev.setFixedWidth(40)
        self.btn_prev.setEnabled(False)
        l2.addWidget(self.btn_prev)
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(0)
        self.slider.setEnabled(False)
        self.slider.setMinimumWidth(180)
        self.slider.setTracking(True)
        l2.addWidget(self.slider)
        self.btn_next = QPushButton("▶")
        self.btn_next.setFixedWidth(40)
        self.btn_next.setEnabled(False)
        l2.addWidget(self.btn_next)
        self.lbl_batch = QLabel("0/0")
        self.lbl_batch.setMinimumWidth(50)
        l2.addWidget(self.lbl_batch)
        lay.addWidget(g2)
        
        # === Mode ===
        g3 = QGroupBox("Mode")
        l3 = QHBoxLayout(g3)
        self.radio_pair = QRadioButton("Point Pair")
        self.radio_pair.setChecked(True)
        self.radio_lane = QRadioButton("Lane (2pts)")
        l3.addWidget(self.radio_pair)
        l3.addWidget(self.radio_lane)
        lay.addWidget(g3)
        
        # === Camera ===
        g4 = QGroupBox("Camera")
        l4 = QHBoxLayout(g4)
        l4.addWidget(QLabel("H:"))
        self.spin_h = QDoubleSpinBox()
        self.spin_h.setRange(0.5, 5.0)
        self.spin_h.setValue(1.5)
        self.spin_h.setSuffix("m")
        l4.addWidget(self.spin_h)
        l4.addWidget(QLabel("fx:"))
        self.spin_fx = QDoubleSpinBox()
        self.spin_fx.setRange(100, 5000)
        self.spin_fx.setValue(1000)
        l4.addWidget(self.spin_fx)
        lay.addWidget(g4)
        
        # === Actions ===
        g5 = QGroupBox("Actions")
        l5 = QHBoxLayout(g5)
        self.btn_undo = QPushButton("↩ Undo")
        self.btn_undo.setObjectName("UndoBtn")
        l5.addWidget(self.btn_undo)
        self.btn_clear = QPushButton("🗑 Clear")
        self.btn_clear.setObjectName("ClearBtn")
        l5.addWidget(self.btn_clear)
        self.btn_save = QPushButton("💾 Save")
        l5.addWidget(self.btn_save)
        lay.addWidget(g5)
        
        # Stats
        self.lbl_pairs = QLabel("Pairs: 0/10")
        lay.addWidget(self.lbl_pairs)
        self.lbl_lanes = QLabel("Lanes: 0")
        lay.addWidget(self.lbl_lanes)
        
        lay.addStretch()
        
        # Mode status
        self.lbl_mode = QLabel("Ready")
        self.lbl_mode.setStyleSheet("padding: 6px 12px; background: #333; border-radius: 4px; font-weight: bold;")
        lay.addWidget(self.lbl_mode)
        
        parent.addWidget(bar)
    
    def _createMainArea(self, parent):
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setHandleWidth(4)
        
        self.image_vp = ImageViewport()
        self.image_vp.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        splitter.addWidget(self.image_vp)
        
        self.bev_vp = BEVViewport()
        self.bev_vp.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        splitter.addWidget(self.bev_vp)
        
        splitter.setSizes([700, 300])
        splitter.setStretchFactor(0, 7)
        splitter.setStretchFactor(1, 3)
        parent.addWidget(splitter, 1)
    
    def _connectSignals(self):
        # Data
        self.btn_sync.clicked.connect(self._onLoadSync)
        self.btn_calib.clicked.connect(self._onLoadCalib)
        
        # Navigation
        self.btn_prev.clicked.connect(lambda: self._goBatch(-1))
        self.btn_next.clicked.connect(lambda: self._goBatch(1))
        self.slider.valueChanged.connect(self._onSlider)
        self.slider.sliderMoved.connect(self._onSlider)
        
        # Mode
        self.radio_pair.toggled.connect(self._onModeSwitch)
        
        # Actions
        self.btn_undo.clicked.connect(self._onUndo)
        self.btn_clear.clicked.connect(self._onClear)
        self.btn_save.clicked.connect(self._onSave)
        
        # Viewport interactions
        self.image_vp.radarClicked.connect(self._onRadarClicked)
        self.image_vp.imageClicked.connect(self._onImageClicked)
        self.image_vp.clicked.connect(self._onViewportClicked)
    
    # =========================================================================
    # Data Loading
    # =========================================================================
    def _onLoadSync(self):
        path, _ = QFileDialog.getOpenFileName(self, "Open Sync JSON", "", "JSON (*.json)")
        if not path:
            return
        try:
            n = self.data_mgr.load_sync_json(path)
            self.slider.setMaximum(n - 1)
            self.slider.setEnabled(True)
            self.slider.setValue(0)
            self.btn_prev.setEnabled(True)
            self.btn_next.setEnabled(True)
            self._loadBatch(0)
            self.statusbar.showMessage(f"Loaded {n} batches")
        except Exception as e:
            QMessageBox.critical(self, "Error", str(e))
    
    def _onLoadCalib(self):
        if self.calibration.loaded:
            QMessageBox.information(self, "Info", "Calibration already loaded!")
            return
        path, _ = QFileDialog.getOpenFileName(self, "Open Calibration TXT", "", "TXT (*.txt)")
        if not path:
            return
        try:
            n = self.calibration.load_from_file(path)
            self.calib_light.setStyleSheet(f"color: {COLORS['success']}; font-size: 20px;")
            self.calib_light.setToolTip(f"Loaded {n} points")
            self.statusbar.showMessage(f"Calibration loaded: {n} points")
            # Re-project current frame
            self._projectRadar()
        except Exception as e:
            self.calib_light.setStyleSheet(f"color: {COLORS['error']}; font-size: 20px;")
            QMessageBox.critical(self, "Error", str(e))
    
    # =========================================================================
    # Batch Navigation
    # =========================================================================
    def _goBatch(self, delta: int):
        new_idx = self.ops.current_batch + delta
        if 0 <= new_idx < self.data_mgr.num_batches:
            self.slider.setValue(new_idx)
    
    def _onSlider(self, val: int):
        if val != self.ops.current_batch:
            self._loadBatch(val)
    
    def _loadBatch(self, idx: int):
        self.ops.current_batch = idx
        self.ops.cancel()  # Cancel any pending operation
        
        # Load data
        img_path, radar_data = self.data_mgr.get_batch(idx)
        
        # Clear views
        self.image_vp.clearRadarMarkers()
        self.image_vp.clearPairMarkers()
        self.image_vp.clearLaneMarkers()
        self.image_vp.clearPreview()
        self.image_vp.clearPendingRadar()
        self.bev_vp.clearRadar()
        self.bev_vp.clearPairs()
        
        # Load image
        if img_path:
            self.image_vp.loadImage(img_path)
        
        # Load radar
        if radar_data:
            self.bev_vp.loadRadarData(radar_data)
        
        # Project radar to image
        self._projectRadar()
        
        # Redraw existing pairs/lanes for this batch
        self._redrawPairs()
        self._redrawLanes()
        
        # Update UI
        self.lbl_batch.setText(f"{idx + 1}/{self.data_mgr.num_batches}")
        
        # Re-enter the correct mode based on radio selection
        if self.radio_pair.isChecked():
            self.ops.start_pair_selection()
        else:
            self.ops.start_lane_drawing()
        
        self._updateModeUI()
    
    def _projectRadar(self):
        """Project radar points onto image."""
        if not self.calibration.loaded:
            return
        
        self.image_vp.clearRadarMarkers()
        radar_data = self.data_mgr.current_radar_data
        
        for t in radar_data.get('targets', []):
            rx, ry = t['x'], t['y']
            u, v = self.calibration.project_radar_to_image(rx, ry)
            self.image_vp.addRadarProjection(u, v, t)
    
    def _redrawPairs(self):
        """Redraw pair markers for current batch."""
        self.image_vp.clearPairMarkers()
        self.bev_vp.clearPairs()
        
        for i, pair in self.ops.get_pairs_for_batch(self.ops.current_batch):
            # Radar position marker
            self.image_vp.addPairMarker(pair.radar_u, pair.radar_v, i, is_radar=True)
            # Image position marker
            self.image_vp.addPairMarker(pair.pixel_u, pair.pixel_v, i, is_radar=False)
            # BEV marker
            self.bev_vp.addPairMarker(pair.radar_x, pair.radar_y, i)
    
    def _redrawLanes(self):
        """Redraw lane markers for current batch."""
        self.image_vp.clearLaneMarkers()
        
        for i, lane in self.ops.get_lanes_for_batch(self.ops.current_batch):
            self.image_vp.setLaneStart(lane.start[0], lane.start[1])
            self.image_vp.completeLane(lane.end[0], lane.end[1])
    
    # =========================================================================
    # Mode Management
    # =========================================================================
    def _onModeSwitch(self):
        self.ops.cancel()
        self.image_vp.clearPreview()
        self.image_vp.clearPendingRadar()
        
        if self.radio_pair.isChecked():
            self.ops.start_pair_selection()
        else:
            self.ops.start_lane_drawing()
        
        self._updateModeUI()
    
    def _updateModeUI(self):
        """Update UI based on current mode."""
        mode = self.ops.mode
        
        if mode == AppMode.NORMAL:
            self.lbl_mode.setText("Ready")
            self.lbl_mode.setStyleSheet("padding: 6px 12px; background: #333; border-radius: 4px;")
            self.image_vp.setMode('normal')
        
        elif mode == AppMode.SELECT_RADAR:
            self.lbl_mode.setText("🎯 Click Radar Point")
            self.lbl_mode.setStyleSheet(f"padding: 6px 12px; background: {COLORS['radar']}; color: white; border-radius: 4px;")
            self.image_vp.setMode('select_radar')
        
        elif mode == AppMode.SELECT_IMAGE:
            self.lbl_mode.setText("📍 Click Image Point")
            self.lbl_mode.setStyleSheet(f"padding: 6px 12px; background: {COLORS['radar_pending']}; color: black; border-radius: 4px;")
            self.image_vp.setMode('select_image')
        
        elif mode == AppMode.LANE_START:
            self.lbl_mode.setText("🚗 Click Lane Start")
            self.lbl_mode.setStyleSheet(f"padding: 6px 12px; background: {COLORS['lane']}; color: white; border-radius: 4px;")
            self.image_vp.setMode('lane_start')
        
        elif mode == AppMode.LANE_END:
            self.lbl_mode.setText("🚗 Click Lane End")
            self.lbl_mode.setStyleSheet(f"padding: 6px 12px; background: {COLORS['lane']}; color: white; border-radius: 4px;")
            self.image_vp.setMode('lane_end')
        
        # Update counts
        self.lbl_pairs.setText(f"Pairs: {self.ops.num_pairs}/{MAX_POINT_PAIRS}")
        self.lbl_lanes.setText(f"Lanes: {self.ops.num_lanes}")
    
    # =========================================================================
    # Point Pair Selection
    # =========================================================================
    def _onRadarClicked(self, target: dict):
        """Radar point clicked on image."""
        if self.ops.mode != AppMode.SELECT_RADAR:
            return
        
        if not self.ops.can_add_pair:
            self.statusbar.showMessage("Max 10 pairs reached!")
            return
        
        # Get projected position
        rx, ry = target['x'], target['y']
        proj_u, proj_v = self.calibration.project_radar_to_image(rx, ry)
        
        # Record selection
        if self.ops.select_radar_point(target, proj_u, proj_v):
            # Highlight radar point as pending (yellow)
            self.image_vp.highlightPendingRadar(target)
            self.statusbar.showMessage(f"Selected R{target.get('id')}. Now click corresponding image point.")
            self._updateModeUI()
    
    def _onImageClicked(self, u: float, v: float):
        """Image point clicked."""
        if self.ops.mode != AppMode.SELECT_IMAGE:
            return
        
        pair = self.ops.select_image_point(u, v)
        if pair:
            idx = self.ops.num_pairs - 1
            
            # Clear pending highlight
            self.image_vp.clearPendingRadar()
            self.image_vp.clearPreview()
            
            # Draw completed pair markers
            self.image_vp.addPairMarker(pair.radar_u, pair.radar_v, idx, is_radar=True)
            self.image_vp.addPairMarker(pair.pixel_u, pair.pixel_v, idx, is_radar=False)
            self.bev_vp.addPairMarker(pair.radar_x, pair.radar_y, idx)
            
            self.statusbar.showMessage(f"Pair {idx+1} created: R{pair.radar_id} ↔ ({u:.0f}, {v:.0f})")
            self._updateModeUI()
    
    # =========================================================================
    # Lane Drawing
    # =========================================================================
    def _onViewportClicked(self, x: float, y: float):
        """Generic click handler for lane drawing."""
        if self.ops.mode == AppMode.LANE_START:
            self.ops.set_lane_start(x, y)
            self.image_vp.setLaneStart(x, y)
            self.statusbar.showMessage(f"Lane start at ({x:.0f}, {y:.0f}). Click end point.")
            self._updateModeUI()
        
        elif self.ops.mode == AppMode.LANE_END:
            lane = self.ops.set_lane_end(x, y)
            if lane:
                self.image_vp.completeLane(x, y)
                self.statusbar.showMessage(f"Lane {self.ops.num_lanes} completed")
                self._updateModeUI()
    
    # =========================================================================
    # Undo / Clear / Save
    # =========================================================================
    def _onUndo(self):
        # Try undo pending first
        if self.ops.undo_pending():
            self.image_vp.clearPendingRadar()
            self.image_vp.clearPreview()
            self.image_vp.undoLastLanePoint()
            self.statusbar.showMessage("Pending selection cancelled")
            self._updateModeUI()
            return
        
        # Undo based on mode
        if self.radio_pair.isChecked():
            pair = self.ops.undo_last_pair()
            if pair:
                self._redrawPairs()
                self.statusbar.showMessage(f"Undid pair with R{pair.radar_id}")
                self._updateModeUI()
            else:
                self.statusbar.showMessage("Nothing to undo")
        else:
            lane = self.ops.undo_last_lane()
            if lane:
                self._redrawLanes()
                self.statusbar.showMessage("Undid last lane")
                self._updateModeUI()
            else:
                self.statusbar.showMessage("Nothing to undo")
    
    def _onClear(self):
        reply = QMessageBox.question(
            self, "Confirm Clear",
            "Clear all point pairs and lanes?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        if reply == QMessageBox.StandardButton.Yes:
            self.ops.clear_all()
            self.image_vp.clearPairMarkers()
            self.image_vp.clearLaneMarkers()
            self.image_vp.clearPendingRadar()
            self.image_vp.clearPreview()
            self.bev_vp.clearPairs()
            self._updateModeUI()
            self.statusbar.showMessage("Cleared all")
    
    def _onSave(self):
        if self.ops.num_pairs == 0 and self.ops.num_lanes == 0:
            QMessageBox.information(self, "Info", "Nothing to save.")
            return
        
        save_dir = QFileDialog.getExistingDirectory(self, "Select Save Directory")
        if not save_dir:
            return
        
        saved = []
        
        if self.ops.pairs:
            path = DataExporter.save_point_pairs(
                [p.to_dict() for p in self.ops.pairs], save_dir
            )
            saved.append(f"{self.ops.num_pairs} pairs")
        
        if self.ops.lanes:
            path = DataExporter.save_all_lanes(
                [l.to_list() for l in self.ops.lanes], save_dir
            )
            saved.append(f"{self.ops.num_lanes} lanes")
        
        self.statusbar.showMessage(f"Saved: {', '.join(saved)}")


def main():
    try:
        from PyQt6.QtCore import Qt
        QApplication.setHighDpiScaleFactorRoundingPolicy(
            Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
        )
    except:
        pass
    
    app = QApplication(sys.argv)
    app.setStyleSheet(QSS)
    win = MainWindow()
    win.showMaximized()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
