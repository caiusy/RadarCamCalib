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
import json

from datetime import datetime

os.environ["QT_ENABLE_HIGHDPI_SCALING"] = "1"

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QFrame, QPushButton, QSlider, QLabel, QSplitter, QFileDialog,
    QStatusBar, QDoubleSpinBox, QGroupBox, QMessageBox, QRadioButton,
    QSizePolicy, QListWidget, QSpinBox
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QColor, QFont

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

try:
    from trajectory_db import TrajectoryDB
except ImportError:
    TrajectoryDB = None

try:
    from trajectory_dialog import TrajectoryMatchDialog
except ImportError:
    TrajectoryMatchDialog = None


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
        
        # Trajectory database
        self.trajectory_db = TrajectoryDB() if TrajectoryDB else None
        self._trajectory_mode_active = False
        self._match_dialog = None
        
        self._setupUI()
        self._connectSignals()
        self._updateModeUI()
        
        # Auto-load saved calibration state
        self._autoLoadState()
    
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
        self.radio_trajectory = QRadioButton("🔍 Trajectory")
        l3.addWidget(self.radio_pair)
        l3.addWidget(self.radio_lane)
        l3.addWidget(self.radio_trajectory)
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
        
        # Trajectory ID selector (hidden by default)
        self.trajectory_group = QGroupBox("Trajectory")
        self.trajectory_group.setVisible(False)
        traj_lay = QVBoxLayout(self.trajectory_group)
        
        # Quick select list
        self.trajectory_list = QListWidget()
        self.trajectory_list.setMaximumHeight(120)
        self.trajectory_list.setStyleSheet("background: #2a2a2a; color: #fff; font-size: 11px;")
        self.trajectory_list.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.trajectory_list.customContextMenuRequested.connect(self._showTrajectoryContextMenu)
        traj_lay.addWidget(self.trajectory_list)
        
        # Button to open matching dialog
        self.btn_match_dialog = QPushButton("🔗 Match Radar↔Camera")
        self.btn_match_dialog.setToolTip("Open dialog to create radar-camera trajectory pairs")
        self.btn_match_dialog.setStyleSheet("padding: 6px; font-weight: bold;")
        traj_lay.addWidget(self.btn_match_dialog)
        
        
        # Playback control
        self.btn_play = QPushButton("▶ Play")
        self.btn_play.setToolTip("Play trajectory visualization")
        self.btn_play.clicked.connect(self._togglePlayback)
        traj_lay.addWidget(self.btn_play) 
        
        self.spin_fps = QSpinBox()
        self.spin_fps.setRange(1, 60)
        self.spin_fps.setValue(10)
        self.spin_fps.setSuffix(" FPS")
        self.spin_fps.setToolTip("Playback Speed (Frames Per Second)")
        self.spin_fps.setFixedWidth(80)
        traj_lay.addWidget(self.spin_fps)
        
        lay.addWidget(self.trajectory_group)
        
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
        
        # Trajectory mode
        self.radio_trajectory.toggled.connect(self._onTrajectoryModeToggle)
        self.bev_vp.trajectoryPointClicked.connect(self._onTrajectoryPointClicked)
        self.trajectory_list.itemClicked.connect(self._onTrajectoryIdSelected)
        self.btn_match_dialog.clicked.connect(self._onOpenMatchDialog)
    
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
            self.btn_prev.setEnabled(True)
            self.btn_next.setEnabled(True)
            self._loadBatch(0)
            
            # Playback
            self._play_timer = QTimer(self)
            self._play_timer.timeout.connect(self._play_step)
            self._matched_pairs_cache = {} # {radar_id: camera_id}
            
            # Initial UI Update DB with dataset path (radar_data.db in same dir as radar folder usually, or data_root)
            # data_mgr.data_root is set by load_sync_json
            db_path = os.path.join(self.data_mgr.data_root, 'radar_data.db')
            print(f"[DB] Initializing database at {db_path}")
            
            # Re-init DB with file path
            if hasattr(self, 'trajectory_db'):
                self.trajectory_db.close()
            self.trajectory_db = TrajectoryDB(db_path)
            
            # Load radar files if new DB
            if not self.trajectory_db.loaded:
                print("[DB] Loading radar files into database...")
                self.statusbar.showMessage("⏳ Parsing radar files into database...")
                QApplication.processEvents()
                count = self.trajectory_db.load_all_radar_files(self.data_mgr.data_root)
                if count > 0:
                     self.trajectory_db.loaded = True
                     self.statusbar.showMessage(f"✅ Loaded {count} points into DB")
            
            # Restore previous session state (calibration, points)
            self._autoLoadState()
            
            self.statusbar.showMessage(f"Loaded {n} batches and database")
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
    def _showTrajectoryContextMenu(self, pos):
        """Context menu for trajectory list (Unbind)."""
        item = self.trajectory_list.itemAt(pos)
        if not item:
            return
            
        import re
        match = re.match(r'🔗 R(\d+)↔C(\d+)', item.text())
        if match:
            rid = int(match.group(1))
            cid = int(match.group(2))
            
            from PyQt6.QtWidgets import QMenu
            menu = QMenu(self)
            action_unbind = menu.addAction("❌ Unbind Pair")
            action = menu.exec(self.trajectory_list.mapToGlobal(pos))
            
            if action == action_unbind:
                self._unbindPair(rid, cid)

    def _unbindPair(self, rid, cid):
        """Unbind a pair."""
        # Delete from DB
        try:
            self.trajectory_db.cursor.execute(
                "DELETE FROM matched_pairs WHERE radar_id=? AND camera_id=?", 
                (rid, cid)
            )
            self.trajectory_db.conn.commit()
            print(f"[TrajectoryDB] Unbound pair R{rid}↔C{cid}")
            
            # Update Cache
            if rid in self._matched_pairs_cache:
                del self._matched_pairs_cache[rid]
                
            # Update List UI
            # We just re-enter mode to refresh list easily
            self.trajectory_list.clear()
            self._enterTrajectoryMode()
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to unbind: {e}")

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
             # If Playing, hide background to show only selected target
             if self.btn_play.text() == "⏸ Pause":
                 self.bev_vp.clearAll()
             else:
                 self._refreshBEV()
        
        # Project radar to image
        self._projectRadar()
        
        # Redraw existing pairs/lanes for this batch
        self._redrawPairs()
        self._redrawLanes()
        
        # Visualize Trajectory Playback (Frame-by-Frame)
        if self.radio_trajectory.isChecked() and self._current_trajectory_id is not None:
             self._visualizeTrajectoryAtFrame(idx, radar_data)
             
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
                self._autoSaveState()  # Auto-save after pitch calculation
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
            self._autoSaveState()  # Auto-save after optimization
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
    
    def _autoSaveState(self):
        """Auto-save calibration state and points to DB (persistent)."""
        if not self.calib_mgr: 
            return
            
        cam_params = {
            'height': self.spin_h.value(),
            'fx': self.spin_fx.value(),
            'fy': self.spin_fy.value(),
            'cx': self.spin_cx.value(),
            'cy': self.spin_cy.value(),
            'pitch': self.calib_mgr.camera.pitch
        }
        
        radar_params = {
            'yaw': self.spin_yaw.value(),
            'x_offset': self.spin_rx.value(),
            'y_offset': self.spin_ry.value()
        }
        
        # Save to DB
        if self.trajectory_db:
             self.trajectory_db.save_calibration_state(cam_params, radar_params)
             
             # Save points (convert PointPair objects to dicts)
             if hasattr(self, 'ops'):
                 pair_dicts = [p.to_dict() for p in self.ops.pairs]
                 self.trajectory_db.save_calibration_points(pair_dicts)
                 
             print(f"[AUTO-SAVE] Saved state to DB")
    
    def _autoLoadState(self):
        """Auto-load calibration state and points from DB (persistent)."""
        if not hasattr(self, 'trajectory_db') or not self.trajectory_db:
             return
             
        try:
            cam, radar = self.trajectory_db.load_calibration_state()
            
            if self.calib_mgr:
                if cam:
                    self.calib_mgr.update_camera_params(**cam)
                    # Update UI
                    self.spin_h.setValue(cam.get('height', 1.5))
                    self.spin_fx.setValue(int(cam.get('fx', 1000)))
                    self.spin_fy.setValue(int(cam.get('fy', 1000)))
                    self.spin_cx.setValue(int(cam.get('cx', 640)))
                    self.spin_cy.setValue(int(cam.get('cy', 480)))
                    # Pitch is special
                    if 'pitch' in cam:
                        self.calib_mgr.camera_pitch = cam['pitch']
                        self.lbl_pitch.setText(f"{cam['pitch']:.4f}")
                        
                if radar:
                    self.calib_mgr.update_radar_params(**radar)
                    # Update UI
                    self.spin_yaw.setValue(radar.get('yaw', 0))
                    self.spin_rx.setValue(radar.get('x_offset', 0))
                    self.spin_ry.setValue(radar.get('y_offset', 3.5))
            
            # Load points
            points_data = self.trajectory_db.load_calibration_points()
            if points_data and hasattr(self, 'ops'):
                self.ops.restore_points(points_data)
                self._redrawPairs() # To show them on screen
                
            self.statusbar.showMessage("📂 Loaded calibration state from DB")
            print(f"[AUTO-LOAD] State loaded from DB. {len(points_data)} points restored.")
            
            # Show popup after a short delay
            from PyQt6.QtCore import QTimer
            QTimer.singleShot(1000, lambda: self.statusbar.showMessage("✅ Calibration state & points auto-loaded", 5000))
            
        except Exception as e:
            print(f"[AUTO-LOAD] Error: {e}")
    
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
            
        # Auto-save immediately on any parameter change
        self._autoSaveState()

    
    def _togglePlayback(self):
        """Toggle playback of trajectory."""
        if self._play_timer.isActive():
            self._play_timer.stop()
            self.btn_play.setText("▶ Play")
            
            # Restore Static View
            self.bev_vp.clearTrails() # Clear trails
            if self._current_trajectory_id is not None:
                self._onTrajectoryIdSelected(self._current_trajectory_id)
                
        else:
            # Clear Static View to avoid overlap
            self.bev_vp.clearTrajectories()
            
            # Jump to start frame of current trajectory if starting fresh
            rid = self._current_trajectory_id
            if rid is not None:
                # Query start frame
                try:
                    self.trajectory_db.cursor.execute(
                        "SELECT MIN(frame_id), MAX(frame_id) FROM radar_trajectories WHERE target_id=?", 
                        (rid,)
                    )
                    row = self.trajectory_db.cursor.fetchone()
                    if row and row[0] is not None:
                        start_frame = row[0]
                        # If current frame is far past end or before start, jump
                        curr = self.slider.value()
                        if curr < start_frame or curr > row[1]:
                             self.slider.setValue(row[0])
                except Exception as e:
                    print(f"Error checking start frame: {e}")
            
            # Use FPS
            fps = self.spin_fps.value()
            interval = 1000 // fps
            self._play_timer.start(interval)
            self.btn_play.setText("⏸ Pause")

    def _play_step(self):
        """Advance frame for preview."""
        curr = self.slider.value()
        if curr < self.slider.maximum():
            self.slider.setValue(curr + 1)
        else:
            # Stop at end (No Loop)
            self._togglePlayback()
            
    def _visualizeTrajectoryAtFrame(self, frame_id, radar_data):
        """Visualize specific trajectory targets at current frame."""
        try:
            rid = self._current_trajectory_id
            if rid is None:
                return
                

            # 1. Trail Logic (Query history for BEV trail)
            try:
                # Get points up to current frame for trail
                # Table: radar_trajectories (id, frame_id, target_id, x, y, ...)
                self.trajectory_db.cursor.execute(
                    "SELECT x, y FROM radar_trajectories WHERE target_id=? AND frame_id <= ? ORDER BY frame_id ASC",
                    (rid, frame_id)
                )
                trail_points = self.trajectory_db.cursor.fetchall() # list of (x, y)
                if trail_points:
                     # Transform to BEV if calibration available
                     if self.calib_mgr:
                         trail_points_bev = [self.calib_mgr.radar_to_bev(p[0], p[1]) for p in trail_points]
                         self.bev_vp.drawTrajectoryTrail(trail_points_bev, QColor(0, 255, 255))
                     else:
                         # Fallback raw (pass as is, viewports might interpret)
                         self.bev_vp.drawTrajectoryTrail(trail_points, QColor(0, 255, 255))
            except Exception as e:
                print(f"Error drawing trail: {e}")

            # 2. Find Radar Point in current batch (Keep existing)
            r_pt = None
            if radar_data and isinstance(radar_data, dict):
                # Safe access
                for p in radar_data.get('targets', []): # Access 'targets' key
                    if p.get('id') == rid:
                        r_pt = p
                        break
        except Exception as e:
            print(f"[ERROR] _visualizeTrajectoryAtFrame crash: {e}")
            return
        
        # 2. Find Camera Point (query DB)
        cid = self._matched_pairs_cache.get(rid)
        c_pt = None
        if cid is not None:
            # Query DB for camera point at this frame
            # We need a helper in TrajectoryDB, or just direct query
            try:
                self.trajectory_db.cursor.execute(
                    "SELECT u, v, w, h FROM camera_trajectories WHERE target_id=? AND batch_id=?", 
                    (cid, frame_id)
                )
                row = self.trajectory_db.cursor.fetchone()
                if row:
                    c_pt = {'u': row[0], 'v': row[1], 'w': row[2], 'h': row[3]}
            except Exception as e:
                print(f"Error querying camera pt: {e}")

        # 3. Draw
        # Highlight Radar in BEV
        if r_pt:
            # Highlight in BEV (Flash color)
            # Use custom head drawing because standard markers are hidden
            rx_val = r_pt.get('x')
            ry_val = r_pt.get('y')
            if rx_val is not None and ry_val is not None:
                if self.calib_mgr:
                    tx, ty = self.calib_mgr.radar_to_bev(rx_val, ry_val)
                    self.bev_vp.drawTrajectoryHead(tx, ty, QColor(0, 255, 255)) # Cyan Head
                else:
                    self.bev_vp.drawTrajectoryHead(rx_val, ry_val, QColor(0, 255, 255)) # Cyan Head
            
            # Highlight in Image (Project it)
            # _projectRadar already drew green dots. We overwrite/add special marker.
            # We need world coords.
            rx = r_pt.get('x', 0)
            ry = r_pt.get('y', 0)
            # Use calib_mgr to project
            if self.calib_mgr and self.calib_mgr.is_calibrated: # Check calib_mgr exists
                # Chain: Radar -> BEV -> Image
                tx, ty = self.calib_mgr.transformer.radar_to_bev(rx, ry)
                u, v = self.calib_mgr.transformer.bev_to_image(tx, ty)
                
                if 0 <= u < self.calib_mgr.camera.cx * 2 and 0 <= v < self.calib_mgr.camera.cy * 2:
                    # Use existing method in ImageViewport
                    self.image_vp.showTrajectoryProjection(u, v, rid)

        # Highlight Camera in Image
        if c_pt:
            # Draw box or cross
            u, v = c_pt['u'], c_pt['v']
            w, h = c_pt['w'], c_pt['h']
            # Draw rect? Helper addRect? 
            # ImageViewport doesn't have addRect. Use addPairMarker for center.
            self.image_vp.addPairMarker(u + w/2, v + h/2, f"C{cid}", QColor(255, 0, 255), size=8) # Magenta
            
        # Draw Connection if both present
        if r_pt and c_pt and self.calib_mgr and self.calib_mgr.is_calibrated:
            # Radar Img Proj
            tx, ty = self.calib_mgr.transformer.radar_to_bev(r_pt.get('x'), r_pt.get('y'))
            u_r, v_r = self.calib_mgr.transformer.bev_to_image(tx, ty)
            
            # Camera Img Point
            u_c = c_pt['u'] + c_pt['w']/2
            v_c = c_pt['v'] + c_pt['h']/2
            self.image_vp.drawConnectionLine(u_r, v_r, u_c, v_c, QColor(255, 255, 0)) # Yellow link

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
    
    # =========================================================================
    # Trajectory Mode (轨迹模式)
    # =========================================================================
    
    def _onTrajectoryModeToggle(self, checked: bool):
        """Handle trajectory mode radio button toggle."""
        if checked:
            self._enterTrajectoryMode()
        else:
            self._exitTrajectoryMode()
    
    def _enterTrajectoryMode(self):
        """Enter trajectory viewing mode."""
        if not self.trajectory_db:
            QMessageBox.warning(self, "Error", "TrajectoryDB not available")
            self.radio_pair.setChecked(True)
            return
            
        if not self.data_mgr.data_root:
            QMessageBox.warning(self, "Warning", "Please load sync JSON first")
            self.radio_pair.setChecked(True)
            return
        
        self._trajectory_mode_active = True
        self.ops.mode = AppMode.TRAJECTORY_VIEW
        
        # Load all trajectories
        self.statusbar.showMessage("Loading trajectories...")
        QApplication.processEvents()
        
        count = self.trajectory_db.load_all_radar_files(self.data_mgr.data_root)
        if count == 0:
            QMessageBox.warning(self, "Warning", "No trajectory data found")
            self._exitTrajectoryMode()
            return
        
        # Clear normal display
        self.image_vp.clearRadarMarkers()
        self.image_vp.clearPairMarkers()
        self.bev_vp.clearAll()
        
        # Load radar trajectories to BEV (solid lines, circles)
        trajectories = self.trajectory_db.get_all_trajectories()
        self.bev_vp.loadTrajectories(trajectories)
        
        # Load camera trajectories to BEV (dashed lines, squares)
        camera_trajectories = self.trajectory_db.get_all_camera_trajectories()
        self.bev_vp.loadCameraTrajectories(camera_trajectories)
        
        self.bev_vp.setTrajectoryMode(True)
        
        n_targets = self.trajectory_db.get_target_count()
        n_frames = self.trajectory_db.get_frame_count()
        self.statusbar.showMessage(f"🔍 Trajectory Mode: {n_targets} targets, {n_frames} frames | Radar: ●solid  Camera: ■dashed")
        self.lbl_mode.setText("🔍 Trajectory")
        self.lbl_mode.setStyleSheet("padding: 6px 12px; background: #664400; border-radius: 4px; font-weight: bold;")
        
        # Show trajectory ID list and populate it
        self.trajectory_group.setVisible(True)
        self.trajectory_list.clear()
        self.trajectory_list.addItem("★ All Targets")
        
        # Load saved pairs from DB (persistent)
        # self.trajectory_db.load_pairs_from_disk() # Removed: using DB directly
        saved_pairs = self.trajectory_db.get_matched_pairs()
        print(f"[DEBUG] _enterTrajectoryMode: Found {len(saved_pairs)} saved pairs in DB")
        
        # Cache for playback
        self._matched_pairs_cache = {r: c for r, c in saved_pairs}
        
        for r, c in saved_pairs:
            label = f"🔗 R{r}↔C{c}"
            self.trajectory_list.addItem(label)
        
        # Store all trajectories for filtering
        self._all_trajectories = trajectories
        self._all_camera_trajectories = camera_trajectories
        self._current_trajectory_id = None
    
    def _exitTrajectoryMode(self):
        """Exit trajectory mode and restore normal display."""
        self._trajectory_mode_active = False
        
        # Hide trajectory ID list
        self.trajectory_group.setVisible(False)
        self.trajectory_list.clear()
        
        # Clear trajectory display
        self.bev_vp.clearTrajectories()
        self.bev_vp.setTrajectoryMode(False)
        self.image_vp.clearTrajectoryProjection()
        
        # Restore normal display
        self._loadBatch(self.data_mgr.current_batch)
        self._updateModeUI()
    
    def _onTrajectoryIdSelected(self, item_or_id):
        """Handle selection of a target ID from the list (Item or int ID)."""
        if not self._trajectory_mode_active:
            return
            
        rid = None
        cid = None
        
        # 1. Determine RID/CID based on input type
        if isinstance(item_or_id, int):
            rid = item_or_id
            cid = self._matched_pairs_cache.get(rid)
        elif hasattr(item_or_id, 'text'):
            text = item_or_id.text()
            if text.startswith("★"):  # All targets
                rid = None
            else:
                # Extract IDs from "🔗 R{r}↔C{c}" format
                import re
                match = re.match(r'🔗 R(\d+)↔C(\d+)', text)
                if match:
                    rid = int(match.group(1))
                    cid = int(match.group(2))
        
        # 2. Update Display
        self.bev_vp.clearTrajectories()
        
        if rid is None:
            # Show all trajectories
            self._current_trajectory_id = None
            self.bev_vp.loadTrajectories(self._all_trajectories)
            self.bev_vp.loadCameraTrajectories(self._all_camera_trajectories)
            self.statusbar.showMessage(f"🔍 Showing all {len(self._all_trajectories)} targets (Radar + Camera)")
        else:
            self._current_trajectory_id = rid
            
            # Filter to show only this pair
            filtered_radar = {rid: self._all_trajectories.get(rid, [])}
            filtered_camera = {}
            if cid is not None:
                filtered_camera = {cid: self._all_camera_trajectories.get(cid, [])}
            
            self.bev_vp.loadTrajectories(filtered_radar)
            self.bev_vp.loadCameraTrajectories(filtered_camera)
            
            radar_pts = len(self._all_trajectories.get(rid, []))
            camera_pts = len(self._all_camera_trajectories.get(cid, [])) if cid is not None else 0
            self.statusbar.showMessage(f"🔍 Trajectory R{rid}" + (f"↔C{cid}" if cid is not None else "") + f": Radar {radar_pts}pts")

    # ... (handlers for click/dialog open/preview omitted as they are unchanged) ...

    def _onMatchPairSelected(self, radar_id: int, camera_id: int):
        """Handle pair selection from match dialog."""
        print(f"\n\n[DEBUG] >>> _onMatchPairSelected TRIGGERED with R{radar_id}, C{camera_id} <<<\n\n")
        
        if not self._trajectory_mode_active:
            print("[DEBUG] Trajectory mode not active, ignoring.")
            return
            
        # Add custom pair to list and save to DB
        if radar_id >= 0 and camera_id >= 0:
            label = f"🔗 R{radar_id}↔C{camera_id}"
            
            # Check if already exists
            found = False
            for i in range(self.trajectory_list.count()):
                if self.trajectory_list.item(i).text() == label:
                    self.trajectory_list.setCurrentRow(i)
                    found = True
                    break
            
            if not found:
                self.trajectory_list.insertItem(1, label)  # Insert after "All Targets"
                self.trajectory_list.setCurrentRow(1)
                
                # Save to DB (Persistent table)
                print(f"[DEBUG] Saving matched pair R{radar_id}-C{camera_id} to DB...")
                try:
                    self.trajectory_db.add_matched_pair(radar_id, camera_id)
                    # Verify immediately?
                    self.trajectory_db.cursor.execute(
                        "SELECT * FROM matched_pairs WHERE radar_id=? AND camera_id=?", 
                        (radar_id, camera_id)
                    )
                    row = self.trajectory_db.cursor.fetchone()
                    print(f"[DEBUG] Verification: Saved pair row = {row}")
                    
                    # Show confirmation
                    QMessageBox.information(self, "Saved", f"Matched Pair R{radar_id}↔C{camera_id} Saved!\nCheck Terminal for verification.")
                    
                except Exception as e:
                    print(f"[ERROR] Failed to save pair: {e}")
                    QMessageBox.critical(self, "DB Error", f"Failed to save matched pair: {e}")
                
        # Clear current display
        self.bev_vp.clearTrajectories()
        
        if radar_id == -1 and camera_id == -1:
            # Show all
            self.bev_vp.loadTrajectories(self._all_trajectories)
            self.bev_vp.loadCameraTrajectories(self._all_camera_trajectories)
            self.statusbar.showMessage(f"🔍 Showing all {len(self._all_trajectories)} trajectories")
            
            # Reset selection to "All Targets"
            self.trajectory_list.setCurrentRow(0)
        else:
            # Show selected pair
            if radar_id >= 0:
                filtered_radar = {radar_id: self._all_trajectories.get(radar_id, [])}
                self.bev_vp.loadTrajectories(filtered_radar)
            
            if camera_id >= 0:
                filtered_camera = {camera_id: self._all_camera_trajectories.get(camera_id, [])}
                self.bev_vp.loadCameraTrajectories(filtered_camera)
            
            radar_pts = len(self._all_trajectories.get(radar_id, []))
            camera_pts = len(self._all_camera_trajectories.get(camera_id, []))
            self.statusbar.showMessage(f"🔗 Matched Pair: R{radar_id}({radar_pts}pts) ↔ C{camera_id}({camera_pts}pts)")
    
    def _onTrajectoryPointClicked(self, target_id: int, frame_id: int):
        """Handle click on trajectory point in BEV."""
        if not self._trajectory_mode_active:
            return
            
        # Get radar point data
        radar_point = self.trajectory_db.get_point_at_frame(target_id, frame_id)
        if not radar_point:
            return
            
        x_radar, y_radar, range_val, velocity, rcs = radar_point
        
        # Get camera point data (if available)
        camera_point = self.trajectory_db.get_camera_point_at_frame(target_id, frame_id)
        
        # Load corresponding batch/frame
        self.slider.setValue(frame_id)
        self._loadBatch(frame_id)
        
        # Clear previous projections
        self.image_vp.clearTrajectoryProjection()
        
        # Show radar projection (calculated via homography)
        projection_result = None
        if self.calib_mgr:
            x_bev, y_bev = self.calib_mgr.radar_to_bev(x_radar, y_radar)
            projection_result = self.calib_mgr.bev_to_image(x_bev, y_bev)
            
            if projection_result:
                u, v = projection_result
                self.image_vp.showTrajectoryProjection(u, v, target_id)
        
        # Show camera detection position (ground truth from detection)
        if camera_point:
            cam_u, cam_v = camera_point
            self.image_vp.showCameraDetection(cam_u, cam_v, target_id)
            
            # Draw connecting line if both exist
            if projection_result:
                u, v = projection_result
                self.image_vp.drawConnectionLine(u, v, cam_u, cam_v)
        
        # Highlight the point in BEV
        self.bev_vp.highlightTrajectoryPoint(target_id, frame_id)
        
        # Status message with both radar and camera info
        msg = f"Target {target_id} @ Frame {frame_id}: Radar(x={x_radar:.1f}m, y={y_radar:.1f}m, v={velocity:.1f}m/s)"
        if camera_point:
            msg += f" | Camera(u={camera_point[0]:.0f}, v={camera_point[1]:.0f})"
        self.statusbar.showMessage(msg)
    
    def _onOpenMatchDialog(self):
        """Open the trajectory matching dialog."""
        if not self._trajectory_mode_active:
            return
            
        if not TrajectoryMatchDialog:
            QMessageBox.warning(self, "Error", "TrajectoryMatchDialog not available")
            return
        
        # Create dialog if not exists
        if not hasattr(self, '_match_dialog') or self._match_dialog is None:
            self._match_dialog = TrajectoryMatchDialog(self.trajectory_db, self)
            self._match_dialog.pairSelected.connect(self._onMatchPairSelected)
            self._match_dialog.radarPreview.connect(self._onRadarPreview)
            self._match_dialog.cameraPreview.connect(self._onCameraPreview)
            # Direct Callback (Backup)
            self._match_dialog.set_on_pair_selected(self._onMatchPairSelected)
        
        self._match_dialog.refresh()
        self._match_dialog.show()
        self._match_dialog.raise_()
    
    def _onRadarPreview(self, radar_id: int):
        """Preview radar trajectory when selected in match dialog."""
        if not self._trajectory_mode_active:
            return
        
        # Clear and show only this radar trajectory
        self.bev_vp.clearTrajectories()
        filtered_radar = {radar_id: self._all_trajectories.get(radar_id, [])}
        self.bev_vp.loadTrajectories(filtered_radar)
        
        # Show first frame of this trajectory
        traj = self._all_trajectories.get(radar_id, [])
        if traj:
            frame_id = traj[0][0]  # First frame
            self.slider.setValue(frame_id)
            self._loadBatch(frame_id)
        
        pts = len(traj)
        self.statusbar.showMessage(f"👁 Preview Radar R{radar_id}: {pts} points")
    
    def _onCameraPreview(self, camera_id: int):
        """Preview camera trajectory when selected in match dialog."""
        if not self._trajectory_mode_active:
            return
        
        # Clear and show only this camera trajectory
        self.bev_vp.clearTrajectories()
        filtered_camera = {camera_id: self._all_camera_trajectories.get(camera_id, [])}
        self.bev_vp.loadCameraTrajectories(filtered_camera)
        
        # Show first frame of this trajectory
        traj = self._all_camera_trajectories.get(camera_id, [])
        if traj:
            frame_id = traj[0][0]  # First frame
            self.slider.setValue(frame_id)
            self._loadBatch(frame_id)
            
            # Show camera detection on image
            u, v = traj[0][1], traj[0][2]
            self.image_vp.clearTrajectoryProjection()
            self.image_vp.showCameraDetection(u, v, camera_id)
        
        pts = len(traj)
        self.statusbar.showMessage(f"👁 Preview Camera C{camera_id}: {pts} points")
    
    def _onMatchPairSelected(self, radar_id: int, camera_id: int):
        """Handle pair selection from match dialog."""
        if not self._trajectory_mode_active:
            return
            
        # Add custom pair to list
        if radar_id >= 0 and camera_id >= 0:
            label = f"🔗 R{radar_id}↔C{camera_id}"
            
            # Check if already exists
            found = False
            for i in range(self.trajectory_list.count()):
                if self.trajectory_list.item(i).text() == label:
                    self.trajectory_list.setCurrentRow(i)
                    found = True
                    break
            
            if not found:
                self.trajectory_list.insertItem(1, label)  # Insert after "All Targets"
                self.trajectory_list.setCurrentRow(1)
        
        # Clear current display
        self.bev_vp.clearTrajectories()
        
        if radar_id == -1 and camera_id == -1:
            # Show all
            self.bev_vp.loadTrajectories(self._all_trajectories)
            self.bev_vp.loadCameraTrajectories(self._all_camera_trajectories)
            self.statusbar.showMessage(f"🔍 Showing all {len(self._all_trajectories)} trajectories")
            
            # Reset selection to "All Targets"
            self.trajectory_list.setCurrentRow(0)
        else:
            # Show selected pair
            if radar_id >= 0:
                filtered_radar = {radar_id: self._all_trajectories.get(radar_id, [])}
                self.bev_vp.loadTrajectories(filtered_radar)
            
            if camera_id >= 0:
                filtered_camera = {camera_id: self._all_camera_trajectories.get(camera_id, [])}
                self.bev_vp.loadCameraTrajectories(filtered_camera)
            
            radar_pts = len(self._all_trajectories.get(radar_id, []))
            camera_pts = len(self._all_camera_trajectories.get(camera_id, []))
            self.statusbar.showMessage(f"🔗 Matched Pair: R{radar_id}({radar_pts}pts) ↔ C{camera_id}({camera_pts}pts)")


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
