#!/usr/bin/env python3
"""
Radar-Camera Fusion Calibration System
雷达-相机融合标定系统

Main application window - integrates all modules.
主应用程序窗口 - 集成所有模块。

Modules / 模块:
- config.py: Colors, styles, constants (配置：颜色、样式、常量)
- backend.py: Data loading, calibration, export (后台：数据加载、标定计算、导出)
- viewports.py: Image and BEV viewport widgets (视图：图像和鸟瞰图窗口)
- operations.py: Point pair and lane logic with undo (操作：点对和车道线逻辑，含撤销功能)

Author: AI Assistant
Date: 2026-01-13
"""

import sys
import os

from datetime import datetime

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

try:
    from calib_manager import CalibrationManager
except ImportError:
    CalibrationManager = None

try:
    from calib_manager import CalibrationManager
except ImportError:
    CalibrationManager = None


class MainWindow(QMainWindow):
    """Main application window."""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Radar-Camera Fusion Calibration System")
        self.setMinimumSize(1280, 720)
        
        # Backend
        self.data_mgr = DataManager()
        self.data_exporter = DataExporter()
        self.calibration = Calibration()
        
        # Operations
        self.ops = OperationsController()
        
        # Calibration manager
        self.calib_mgr = CalibrationManager() if CalibrationManager else None
        if self.calib_mgr:
            # Sync default params
            self.calib_mgr.update_camera_params(height=1.5, fx=1000, fy=1000, cx=640, cy=480)
            self.calib_mgr.update_radar_params(yaw=0, x_offset=3.5, y_offset=0)
        
        print(f"[DEBUG] calib_mgr initialized: {self.calib_mgr is not None}")
        
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
        l4 = QVBoxLayout(g4)
        l4.setSpacing(2)
        # Row 1: H, Pitch(readonly)
        r1 = QHBoxLayout()
        r1.addWidget(QLabel("H:"))
        self.spin_h = QDoubleSpinBox()
        self.spin_h.setRange(0.5, 5.0)
        self.spin_h.setValue(1.5)
        self.spin_h.setSuffix("m")
        self.spin_h.setFixedWidth(70)
        r1.addWidget(self.spin_h)
        r1.addWidget(QLabel("Pitch:"))
        self.lbl_pitch = QLabel("0.0000")
        self.lbl_pitch.setStyleSheet("color: #888; font-family: monospace;")
        self.lbl_pitch.setFixedWidth(60)
        r1.addWidget(self.lbl_pitch)
        l4.addLayout(r1)
        # Row 2: fx, fy
        r2 = QHBoxLayout()
        r2.addWidget(QLabel("fx:"))
        self.spin_fx = QDoubleSpinBox()
        self.spin_fx.setRange(100, 3000)
        self.spin_fx.setValue(1000)
        self.spin_fx.setDecimals(0)
        self.spin_fx.setFixedWidth(70)
        r2.addWidget(self.spin_fx)
        r2.addWidget(QLabel("fy:"))
        self.spin_fy = QDoubleSpinBox()
        self.spin_fy.setRange(100, 3000)
        self.spin_fy.setValue(1000)
        self.spin_fy.setDecimals(0)
        self.spin_fy.setFixedWidth(70)
        r2.addWidget(self.spin_fy)
        l4.addLayout(r2)
        # Row 3: cx, cy
        r3 = QHBoxLayout()
        r3.addWidget(QLabel("cx:"))
        self.spin_cx = QDoubleSpinBox()
        self.spin_cx.setRange(0, 2000)
        self.spin_cx.setValue(640)
        self.spin_cx.setDecimals(0)
        self.spin_cx.setFixedWidth(70)
        r3.addWidget(self.spin_cx)
        r3.addWidget(QLabel("cy:"))
        self.spin_cy = QDoubleSpinBox()
        self.spin_cy.setRange(0, 2000)
        self.spin_cy.setValue(480)
        self.spin_cy.setDecimals(0)
        self.spin_cy.setFixedWidth(70)
        r3.addWidget(self.spin_cy)
        l4.addLayout(r3)
        lay.addWidget(g4)
        
        # === Radar ===
        g_radar = QGroupBox("Radar")
        l_radar = QVBoxLayout(g_radar)
        l_radar.setSpacing(2)
        r4 = QHBoxLayout()
        r4.addWidget(QLabel("Yaw:"))
        self.spin_yaw = QDoubleSpinBox()
        self.spin_yaw.setRange(-1, 1)
        self.spin_yaw.setValue(0)
        self.spin_yaw.setSuffix(" rad")
        self.spin_yaw.setDecimals(4)
        self.spin_yaw.setFixedWidth(90)
        r4.addWidget(self.spin_yaw)
        l_radar.addLayout(r4)
        r5 = QHBoxLayout()
        r5.addWidget(QLabel("X:"))
        self.spin_rx = QDoubleSpinBox()
        self.spin_rx.setRange(-5, 5)
        self.spin_rx.setValue(0.0)
        self.spin_rx.setSuffix(" m")
        self.spin_rx.setDecimals(2)
        self.spin_rx.setFixedWidth(90)
        r5.addWidget(self.spin_rx)
        l_radar.addLayout(r5)
        r6 = QHBoxLayout()
        r6.addWidget(QLabel("Y:"))
        self.spin_ry = QDoubleSpinBox()
        self.spin_ry.setRange(0, 10)
        self.spin_ry.setValue(3.5)
        self.spin_ry.setSuffix(" m")
        self.spin_ry.setDecimals(2)
        self.spin_ry.setFixedWidth(90)
        r6.addWidget(self.spin_ry)
        l_radar.addLayout(r6)
        lay.addWidget(g_radar)
        
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
        self.btn_pitch = QPushButton("🔧 Pitch")
        self.btn_pitch.setToolTip("Compute pitch from lanes")
        l5.addWidget(self.btn_save)
        l5.addWidget(self.btn_pitch)
        
        self.btn_opt = QPushButton("✨ Optimize")
        self.btn_opt.setToolTip("Optimize pitch using all collected point pairs")
        l5.addWidget(self.btn_opt)
        
        self.btn_json = QPushButton("📄 JSON")
        self.btn_json.setToolTip("Save Camera Params to JSON")
        l5.addWidget(self.btn_json)
        
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
        self.btn_pitch.clicked.connect(self._onComputePitch)
        self.btn_opt.clicked.connect(self._onOptimizePitch)
        self.btn_json.clicked.connect(self._onSaveParams)
        
        # Parameter changes trigger BEV refresh
        self.spin_h.valueChanged.connect(self._onParamChanged)
        self.spin_fx.valueChanged.connect(self._onParamChanged)
        self.spin_fy.valueChanged.connect(self._onParamChanged)
        self.spin_cx.valueChanged.connect(self._onParamChanged)
        self.spin_cy.valueChanged.connect(self._onParamChanged)
        
        self.spin_yaw.valueChanged.connect(self._onParamChanged)
        self.spin_rx.valueChanged.connect(self._onParamChanged)
        self.spin_ry.valueChanged.connect(self._onParamChanged)
        
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
        
        # Load radar (using refresh for correct coordinates)
        if radar_data:
            self._refreshBEV()
        
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
    
    def _refreshBEV(self):
        """Refresh BEV display."""
        # print("[DEBUG] _refreshBEV called")
        if not self.calib_mgr:
            return
        
        # Clear and rebuild
        self.bev_vp.clearAll()
        
        # 1. Re-add radar points
        if self.data_mgr.current_batch >= 0:
            _, radar_data = self.data_mgr.get_batch(self.data_mgr.current_batch)
            if radar_data:
                for target in radar_data.get('targets', []):
                    x_bev, y_bev = self.calib_mgr.radar_to_bev(target['x'], target['y'])
                    self.bev_vp.addRadarBEVPoint(x_bev, y_bev, f"R{target.get('id')}")
        
        # 2. Re-add pair markers (Rings) - CRITICAL for user feedback
        for i, pair in self.ops.get_pairs_for_batch(self.ops.current_batch):
            # Recalculate BEV position based on potentially new parameters
            radar_bev = self.calib_mgr.radar_to_bev(pair.radar_x, pair.radar_y)
            self.bev_vp.addPairMarker(radar_bev[0], radar_bev[1], i)
        
        # 3. Re-add point pairs (comparison)
        if self.calib_mgr.camera.pitch != 0:
            for i, pair in self.ops.get_pairs_for_batch(self.ops.current_batch):
                try:
                    r_bev = self.calib_mgr.radar_to_bev(pair.radar_x, pair.radar_y)
                    i_bev = self.calib_mgr.image_to_bev(pair.pixel_u, pair.pixel_v)
                    if r_bev and i_bev:
                        self.bev_vp.addComparisonPair(r_bev, i_bev, i)
                except Exception as e:
                    print(f"Error refreshing pair {i}: {e}")
    
    def _projectRadar(self):
        """
        Project radar points onto image.
        Requires calibration to be loaded first.
        """
        if not self.calibration.loaded:
            return
            
        if not self.calib_mgr:
            return
            
        self.image_vp.clearRadarMarkers()
        
        if self.data_mgr.current_batch < 0:
            return
            
        _, radar_data = self.data_mgr.get_batch(self.data_mgr.current_batch)
        if not radar_data:
            return
        
        for t in radar_data.get('targets', []):
            rx, ry = t['x'], t['y']
            
            # Use Geometric Projection since we have corrected it
            # This allows slider tuning.
            # Ideally, self.calibration.loaded implies we have a base to work from.
            x_bev, y_bev = self.calib_mgr.radar_to_bev(rx, ry)
            result = self.calib_mgr.bev_to_image(x_bev, y_bev)
            
            if result:
                u, v = result
                if -2000 < u < 4000 and -2000 < v < 4000:
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
            if self.calib_mgr:
                radar_bev = self.calib_mgr.radar_to_bev(pair.radar_x, pair.radar_y)
                self.bev_vp.addPairMarker(radar_bev[0], radar_bev[1], i)
            else:
                # Fallback if no calib manager (shouldn't happen)
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
    def _onComputePitch(self):
        """Compute pitch from lane lines."""
        print("[DEBUG] _onComputePitch called")
        if not self.calib_mgr:
            self.statusbar.showMessage("No calibration manager")
            return
        
        if self.ops.num_lanes < 2:
            self.statusbar.showMessage("Need at least 2 lane lines")
            return
        
        try:
            pitch = self.calib_mgr.compute_pitch_from_lanes(self.ops.lanes)
            if pitch:
                import numpy as np
                self.lbl_pitch.setText(f"{pitch:.4f}")
                self.lbl_pitch.setStyleSheet("color: #0f0; font-family: monospace; font-weight: bold;")
                self.statusbar.showMessage(f"✅ Pitch: {pitch:.4f} rad ({np.degrees(pitch):.2f}°)")
                print(f"[DEBUG] Pitch computed: {pitch}")
                self._refreshBEV()
            else:
                self.statusbar.showMessage("❌ Failed to compute pitch")
        except Exception as e:
            print(f"[DEBUG] Pitch error: {e}")
            self.statusbar.showMessage(f"Error: {e}")

    def _onOptimizePitch(self):
        """Optimize pitch using all collected point pairs."""
        if not self.calib_mgr:
            return
            
        # Load all pairs
        try:
            pairs = self.data_mgr.load_all_point_pairs(self.data_mgr.data_root)
            if not pairs:
                QMessageBox.warning(self, "Warning", "No point pairs found in dataset folder!")
                return
                
            n = len(pairs)
            self.statusbar.showMessage(f"Optimizing pitch using {n} pairs...")
            QApplication.processEvents() # Update UI
            
            new_pitch = self.calib_mgr.optimize_pitch(pairs, search_range=50)
            
            self.lbl_pitch.setText(f"{new_pitch:.4f}")
            self.lbl_pitch.setStyleSheet("color: #0ff; font-family: monospace; font-weight: bold;")
            self.statusbar.showMessage(f"✨ Optimized Pitch: {new_pitch:.4f} rad (from {n} pairs)")
            
            self._refreshBEV()
            self._projectRadar()  # Refresh green dots on image
            QMessageBox.information(self, "Success", f"Pitch optimized using {n} pairs.\nNew Pitch: {new_pitch:.5f}")
            
        except Exception as e:
            QMessageBox.critical(self, "Error", str(e))

    def _onSaveParams(self):
        """Save camera parameters to JSON."""
        if not self.calib_mgr:
            return
            
        try:
            # Construct params dict
            cam = self.calib_mgr.camera
            radar = self.calib_mgr.radar
            
            params = {
                "camera": {
                    "height": cam.height,
                    "pitch": cam.pitch,
                    "fx": cam.fx,
                    "fy": cam.fy,
                    "cx": cam.cx,
                    "cy": cam.cy
                },
                "radar": {
                    "yaw": radar.yaw,
                    "x_offset": radar.x_offset, # Transformed (Lateral)
                    "y_offset": radar.y_offset  # Transformed (Forward)
                },
                "homography": {
                    "radar_to_bev": self.calib_mgr.get_radar_bev_homography(),
                    "camera_to_bev": self.calib_mgr.get_camera_bev_homography()
                },
                "timestamp": datetime.now().isoformat()
            }
            
            path = self.data_exporter.save_camera_params(params, self.data_mgr.data_root)
            QMessageBox.information(self, "Saved", f"Parameters saved to:\n{os.path.basename(path)}")
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save params: {e}")
    
    def _onParamChanged(self):
        """Handle parameter changes - update and refresh BEV."""
        # print("[PARAM] _onParamChanged called")
        if not self.calib_mgr:
            return
        
        # Update camera params (preserve pitch!)
        self.calib_mgr.update_camera_params(
            height=self.spin_h.value(),
            fx=self.spin_fx.value(),
            fy=self.spin_fy.value(),
            cx=self.spin_cx.value(),
            cy=self.spin_cy.value()
        )
        
        # Update radar params
        self.calib_mgr.update_radar_params(
            yaw=self.spin_yaw.value(),
            x_offset=self.spin_rx.value(),
            y_offset=self.spin_ry.value()
        )
        
        # Refresh Image Projection (green dots)
        # User requested to disable image projection update during param change
        # self._projectRadar()
        
        # Restore pending highlight if exists (needs to find new projected pos)
        if self.ops.pending_radar:
             # Find the marker for this target and re-highlight
             # _projectRadar clears markers, so we grab the new one
             self.image_vp.highlightPendingRadar(self.ops.pending_radar.target)

        # Refresh BEV
        self._refreshBEV()
        
        # Restore BEV highlight (since refresh clears it)
        if self.ops.pending_radar:
            self.bev_vp.highlightRadarMarker(self.ops.pending_radar.target)

    
    def _onModeSwitch(self):
        self.ops.cancel()
        self.image_vp.clearPreview()
        self.image_vp.clearPendingRadar()
        self.bev_vp.clearPendingRadar()
        
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
        
        # Use Geometric Projection if available
        if self.calib_mgr:
            x_bev, y_bev = self.calib_mgr.radar_to_bev(rx, ry)
            res = self.calib_mgr.bev_to_image(x_bev, y_bev)
            if res:
                proj_u, proj_v = res
            else:
                proj_u, proj_v = 0, 0 # Should not happen if visible
        else:
            proj_u, proj_v = 0, 0
        
        # Record selection
        if self.ops.select_radar_point(target, proj_u, proj_v):
            # Highlight radar point as pending (yellow)
            self.image_vp.highlightPendingRadar(target)
            self.bev_vp.highlightRadarMarker(target)
            self.statusbar.showMessage(f"Selected R{target.get('id')}. Now click corresponding image point.")
            self._updateModeUI()
    
    def _onImageClicked(self, u: float, v: float):
        """Image point clicked."""
        if self.ops.mode != AppMode.SELECT_IMAGE:
            return

        # Boundary Check
        pix_width = 0
        pix_height = 0
        if self.image_vp._pixmap_item:
            pix = self.image_vp._pixmap_item.pixmap()
            if pix:
                pix_width = pix.width()
                pix_height = pix.height()
        
        # Default to 1280x720 if not loaded, or skip check? 
        # Safer to skip check if pixmap not loaded, but typically it is.
        if pix_width > 0 and pix_height > 0:
            if not (0 <= u < pix_width and 0 <= v < pix_height):
                self.statusbar.showMessage("❌ Clicked outside image bounds")
                return
        
        pair = self.ops.select_image_point(u, v)
        if pair:
            idx = self.ops.num_pairs - 1
            
            # Clear pending highlight
            self.image_vp.clearPendingRadar()
            self.bev_vp.clearPendingRadar()
            self.image_vp.clearPreview()
            
            # Draw completed pair markers
            self.image_vp.addPairMarker(pair.radar_u, pair.radar_v, idx, is_radar=True)
            self.image_vp.addPairMarker(pair.pixel_u, pair.pixel_v, idx, is_radar=False)
            
            # Fix BEV projection (use transformed coordinates)
            if self.calib_mgr:
                r_bev_pt = self.calib_mgr.radar_to_bev(pair.radar_x, pair.radar_y)
                self.bev_vp.addPairMarker(r_bev_pt[0], r_bev_pt[1], idx)
            else:
                self.bev_vp.addPairMarker(pair.radar_x, pair.radar_y, idx)
            
            # BEV Comparison Projection
            if self.calib_mgr and self.calib_mgr.camera.pitch != 0:
                try:
                    r_bev = self.calib_mgr.radar_to_bev(pair.radar_x, pair.radar_y)
                    i_bev = self.calib_mgr.image_to_bev(pair.pixel_u, pair.pixel_v)
                    if r_bev and i_bev:
                        self.bev_vp.addComparisonPair(r_bev, i_bev, idx)
                except Exception as e:
                    print(f"Error adding BEV pair: {e}")
            
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
                
                # Auto-compute pitch when 2+ lanes
                if self.calib_mgr and self.ops.num_lanes >= 2:
                    try:
                        pitch = self.calib_mgr.compute_pitch_from_lanes(self.ops.lanes)
                        if pitch:
                            import numpy as np
                            self.lbl_pitch.setText(f"{pitch:.4f}")
                            self.lbl_pitch.setStyleSheet("color: #0f0; font-family: monospace; font-weight: bold;")
                            self.statusbar.showMessage(f"✅ Pitch: {pitch:.4f} rad ({np.degrees(pitch):.2f}°)")
                    except Exception as e:
                        print(f"Auto-pitch error: {e}")
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
