#!/usr/bin/env python3
"""
Radar-Camera Fusion Calibration System (Refactored)
雷达-相机融合标定系统 (重构版)

Uses new modular packages: data/, calibration/, trajectory/, ui/, visualization/

Author: caiusy
"""

import sys
import os
import json
from pathlib import Path
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

from data.sync_dataset import SyncDataset
from data.radar_loader import extract_radar_points
from calibration.calibration_session import CalibrationSession
from calibration.projector import project_single_point
from calibration.calibration_io import save_calibration_json, load_calibration_json
from trajectory.trajectory_db import TrajectoryDB
from ui.image_view import ImageViewport
from ui.bev_view import BEVViewport
from ui.operations import OperationsController, AppMode
from visualization.color_map import UI_COLORS, PAIR_COLORS

MAX_POINT_PAIRS = 10

# === Dark theme stylesheet ===
QSS = """
QMainWindow { background: #1E1E1E; }
QFrame#TopBar { background: #252526; border-bottom: 1px solid #3E3E42; }
QGroupBox { color: #D4D4D4; border: 1px solid #3E3E42; border-radius: 4px;
    margin-top: 8px; padding-top: 12px; font-weight: bold; }
QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
QPushButton { background: #2D2D30; color: #D4D4D4; border: 1px solid #3E3E42;
    border-radius: 4px; padding: 6px 14px; font-size: 12px; }
QPushButton:hover { background: #3E3E42; border-color: #007ACC; }
QPushButton:pressed { background: #007ACC; color: white; }
QLabel { color: #D4D4D4; font-size: 11px; }
QSlider::groove:horizontal { height: 6px; background: #3E3E42; border-radius: 3px; }
QSlider::handle:horizontal { background: #007ACC; width: 14px; margin: -4px 0;
    border-radius: 7px; }
QDoubleSpinBox, QSpinBox { background: #2D2D30; color: #D4D4D4;
    border: 1px solid #3E3E42; border-radius: 3px; padding: 2px; }
QRadioButton { color: #D4D4D4; }
QListWidget { background: #2a2a2a; color: #fff; font-size: 11px;
    border: 1px solid #3E3E42; }
QStatusBar { background: #007ACC; color: white; font-size: 11px; }
QSplitter::handle { background: #3E3E42; }
"""


class MainWindow(QMainWindow):
    """Main application window - refactored version."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Radar-Camera Fusion Calibration System")
        self.setMinimumSize(1280, 720)

        # Core modules
        self.dataset = SyncDataset()
        self.calib_session = CalibrationSession()
        self.trajectory_db = TrajectoryDB()
        self.ops = OperationsController()

        # Play timer
        self._play_timer = QTimer(self)
        self._play_timer.timeout.connect(self._play_step)

        # State
        self._trajectory_mode_active = False
        self._current_trajectory_id = None
        self._all_trajectories = {}
        self._all_camera_trajectories = {}
        self._matched_pairs_cache = {}
        self._current_radar_objects = []  # raw dicts for click interaction

        self._setupUI()
        self._connectSignals()
        self._updateModeUI()

    # =========================================================================
    # UI Setup
    # =========================================================================
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

        # Data
        g1 = QGroupBox("Data")
        l1 = QHBoxLayout(g1)
        self.btn_sync = QPushButton("📂 Sync JSON")
        l1.addWidget(self.btn_sync)
        lay.addWidget(g1)

        # Batch Navigation
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

        # Mode
        g3 = QGroupBox("Mode")
        l3 = QHBoxLayout(g3)
        self.radio_pair = QRadioButton("Point Pair")
        self.radio_pair.setChecked(True)
        self.radio_lane = QRadioButton("Lane")
        self.radio_trajectory = QRadioButton("🔍 Trajectory")
        l3.addWidget(self.radio_pair)
        l3.addWidget(self.radio_lane)
        l3.addWidget(self.radio_trajectory)
        lay.addWidget(g3)

        # Actions
        g5 = QGroupBox("Actions")
        l5 = QHBoxLayout(g5)
        self.btn_undo = QPushButton("↩ Undo")
        l5.addWidget(self.btn_undo)
        self.btn_clear = QPushButton("🗑 Clear")
        l5.addWidget(self.btn_clear)
        self.btn_save = QPushButton("💾 Save")
        l5.addWidget(self.btn_save)
        self.btn_compute = QPushButton("📐 Compute H")
        self.btn_compute.setToolTip("Compute homography from point pairs")
        l5.addWidget(self.btn_compute)
        lay.addWidget(g5)

        # Stats
        self.lbl_pairs = QLabel("Pairs: 0/10")
        lay.addWidget(self.lbl_pairs)
        self.lbl_lanes = QLabel("Lanes: 0")
        lay.addWidget(self.lbl_lanes)

        # Trajectory panel (hidden by default)
        self.trajectory_group = QGroupBox("Trajectory")
        self.trajectory_group.setVisible(False)
        traj_lay = QVBoxLayout(self.trajectory_group)
        self.trajectory_list = QListWidget()
        self.trajectory_list.setMaximumHeight(120)
        traj_lay.addWidget(self.trajectory_list)
        self.btn_play = QPushButton("▶ Play")
        traj_lay.addWidget(self.btn_play)
        self.spin_fps = QSpinBox()
        self.spin_fps.setRange(1, 60)
        self.spin_fps.setValue(10)
        self.spin_fps.setSuffix(" FPS")
        self.spin_fps.setFixedWidth(80)
        traj_lay.addWidget(self.spin_fps)
        lay.addWidget(self.trajectory_group)

        lay.addStretch()

        self.lbl_mode = QLabel("Ready")
        self.lbl_mode.setStyleSheet(
            "padding: 6px 12px; background: #333; border-radius: 4px; font-weight: bold;"
        )
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
        self.btn_sync.clicked.connect(self._onLoadSync)
        self.btn_prev.clicked.connect(lambda: self._goBatch(-1))
        self.btn_next.clicked.connect(lambda: self._goBatch(1))
        self.slider.valueChanged.connect(self._onSlider)

        self.radio_pair.toggled.connect(self._onModeSwitch)
        self.radio_lane.toggled.connect(self._onModeSwitch)
        self.radio_trajectory.toggled.connect(self._onTrajectoryModeToggle)

        self.btn_undo.clicked.connect(self._onUndo)
        self.btn_clear.clicked.connect(self._onClear)
        self.btn_save.clicked.connect(self._onSave)
        self.btn_compute.clicked.connect(self._onComputeHomography)
        self.btn_play.clicked.connect(self._togglePlayback)

        # Viewport interactions
        self.image_vp.radarClicked.connect(self._onRadarClicked)
        self.image_vp.imageClicked.connect(self._onImageClicked)
        self.image_vp.clicked.connect(self._onViewportClicked)

        # Trajectory
        self.trajectory_list.itemClicked.connect(self._onTrajectoryIdSelected)

    # =========================================================================
    # Data Loading
    # =========================================================================
    def _onLoadSync(self):
        folder = QFileDialog.getExistingDirectory(self, "Select Dataset Root")
        if not folder:
            return

        sync_dir = Path(folder) / "sync"
        if not sync_dir.exists():
            QMessageBox.warning(self, "Error", f"No 'sync' folder found in:\n{folder}")
            return

        try:
            self.dataset.load(folder)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load dataset:\n{e}")
            return

        n = self.dataset.num_frames
        self.slider.setMaximum(n - 1)
        self.slider.setEnabled(n > 0)
        self.btn_prev.setEnabled(n > 1)
        self.btn_next.setEnabled(n > 1)
        self.slider.setValue(0)

        self._loadCurrentFrame()
        self.statusbar.showMessage(f"✅ Loaded {n} frames from {folder}")

    def _goBatch(self, delta):
        new_idx = self.slider.value() + delta
        if 0 <= new_idx < self.dataset.num_frames:
            self.slider.setValue(new_idx)

    def _onSlider(self, value):
        self.dataset.go_to(value)
        self._loadCurrentFrame()

    def _loadCurrentFrame(self):
        if self.dataset.num_frames == 0:
            return

        idx = self.dataset.current_index
        self.lbl_batch.setText(f"{idx + 1}/{self.dataset.num_frames}")
        self.ops.current_batch = idx

        # Load image
        img_frame = self.dataset.get_image()
        if img_frame:
            self.image_vp.loadImage(str(img_frame.file_path))

        # Load radar
        self.image_vp.clearRadarMarkers()
        self.bev_vp.clearRadarMarkers()
        self._current_radar_objects = []

        radar_frame = self.dataset.get_radar()
        if radar_frame and radar_frame.objects:
            for obj in radar_frame.objects:
                target = {
                    'id': obj.track_id,
                    'x': obj.x,
                    'y': obj.y,
                    'class': obj.obj_class,
                    'speed': obj.speed,
                }
                self._current_radar_objects.append(target)

                # BEV display
                self.bev_vp.addRadarPoint(obj.x, obj.y, target)

                # Image projection (if homography available)
                if self.calib_session.has_homography:
                    u, v = project_single_point(obj.x, obj.y, self.calib_session.H)
                    if img_frame and 0 <= u < img_frame.width and 0 <= v < img_frame.height:
                        self.image_vp.addRadarProjection(u, v, target)

        # Redraw existing pairs for this batch
        self._redrawPairs()

    # =========================================================================
    # Point Pair Mode
    # =========================================================================
    def _onModeSwitch(self):
        if self._trajectory_mode_active:
            return
        self.ops.cancel()
        self.image_vp.clearPreview()
        self.image_vp.clearPendingRadar()
        self.bev_vp.clearPendingRadar()

        if self.radio_pair.isChecked():
            self.ops.start_pair_selection()
        elif self.radio_lane.isChecked():
            self.ops.start_lane_drawing()

        self._updateModeUI()

    def _onRadarClicked(self, target: dict):
        if self.ops.mode != AppMode.SELECT_RADAR:
            return
        if not self.ops.can_add_pair:
            self.statusbar.showMessage("Max 10 pairs reached!")
            return

        proj_u, proj_v = 0.0, 0.0
        if self.calib_session.has_homography:
            proj_u, proj_v = project_single_point(
                target['x'], target['y'], self.calib_session.H
            )

        if self.ops.select_radar_point(target, proj_u, proj_v):
            self.image_vp.highlightPendingRadar(target)
            self.bev_vp.highlightRadarMarker(target)
            self.statusbar.showMessage(
                f"Selected R{target.get('id')}. Now click corresponding image point."
            )
            self._updateModeUI()

    def _onImageClicked(self, u: float, v: float):
        if self.ops.mode != AppMode.SELECT_IMAGE:
            return

        pair = self.ops.select_image_point(u, v)
        if pair:
            idx = self.ops.num_pairs - 1
            self.image_vp.clearPendingRadar()
            self.bev_vp.clearPendingRadar()
            self.image_vp.clearPreview()

            self.image_vp.addPairMarker(pair.radar_u, pair.radar_v, idx, is_radar=True)
            self.image_vp.addPairMarker(pair.pixel_u, pair.pixel_v, idx, is_radar=False)
            self.bev_vp.addPairMarker(pair.radar_x, pair.radar_y, idx)

            self.statusbar.showMessage(
                f"Pair {idx+1} created: R{pair.radar_id} ↔ ({u:.0f}, {v:.0f})"
            )
            self._updateModeUI()

    def _onViewportClicked(self, x: float, y: float):
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

    def _redrawPairs(self):
        self.image_vp.clearPairMarkers()
        self.bev_vp.clearPairs()
        for i, pair in enumerate(self.ops.pairs):
            self.image_vp.addPairMarker(pair.radar_u, pair.radar_v, i, is_radar=True)
            self.image_vp.addPairMarker(pair.pixel_u, pair.pixel_v, i, is_radar=False)
            self.bev_vp.addPairMarker(pair.radar_x, pair.radar_y, i)

    # =========================================================================
    # Calibration
    # =========================================================================
    def _onComputeHomography(self):
        if self.ops.num_pairs < 4:
            QMessageBox.warning(
                self, "Warning",
                f"Need at least 4 point pairs, have {self.ops.num_pairs}."
            )
            return

        self.calib_session.clear_points()
        for pair in self.ops.pairs:
            self.calib_session.add_point(
                pair.radar_x, pair.radar_y, pair.pixel_u, pair.pixel_v
            )

        try:
            result = self.calib_session.compute()
            self.statusbar.showMessage(
                f"✅ Homography computed! Mean error: {result.mean_error:.2f}px, "
                f"Max error: {result.max_error:.2f}px"
            )
            # Refresh to show projections
            self._loadCurrentFrame()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Homography computation failed:\n{e}")

    # =========================================================================
    # Undo / Clear / Save
    # =========================================================================
    def _onUndo(self):
        if self.ops.undo_pending():
            self.image_vp.clearPendingRadar()
            self.image_vp.clearPreview()
            self.image_vp.undoLastLanePoint()
            self.statusbar.showMessage("Pending selection cancelled")
            self._updateModeUI()
            return

        if self.radio_pair.isChecked():
            pair = self.ops.undo_last_pair()
            if pair:
                self._redrawPairs()
                self.statusbar.showMessage(f"Undid pair with R{pair.radar_id}")
                self._updateModeUI()
        else:
            lane = self.ops.undo_last_lane()
            if lane:
                self.image_vp.clearLaneMarkers()
                self.statusbar.showMessage("Undid last lane")
                self._updateModeUI()

    def _onClear(self):
        reply = QMessageBox.question(
            self, "Confirm Clear", "Clear all point pairs and lanes?",
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
        if not self.calib_session.has_homography:
            QMessageBox.information(self, "Info", "No homography computed yet.")
            return

        path, _ = QFileDialog.getSaveFileName(
            self, "Save Calibration JSON", "calibration.json", "JSON (*.json)"
        )
        if path:
            try:
                save_calibration_json(Path(path), self.calib_session.result)
                self.statusbar.showMessage(f"💾 Saved to {path}")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Save failed:\n{e}")

    # =========================================================================
    # Trajectory Mode
    # =========================================================================
    def _onTrajectoryModeToggle(self, checked: bool):
        if checked:
            self._enterTrajectoryMode()
        else:
            self._exitTrajectoryMode()

    def _enterTrajectoryMode(self):
        if not self.dataset.dataset_root:
            QMessageBox.warning(self, "Warning", "Please load sync JSON first")
            self.radio_pair.setChecked(True)
            return

        self._trajectory_mode_active = True
        self.ops.mode = AppMode.TRAJECTORY_VIEW
        self.statusbar.showMessage("Loading trajectories...")
        QApplication.processEvents()

        data_root = str(self.dataset.dataset_root)

        # Use cached load — skips if same dataset
        count = self.trajectory_db.load_all_radar_files(data_root)

        if not self.trajectory_db.loaded:
            QMessageBox.warning(self, "Warning", "No trajectory data found")
            self._exitTrajectoryMode()
            return

        # Clear normal display
        self.image_vp.clearRadarMarkers()
        self.image_vp.clearPairMarkers()
        self.bev_vp.clearAll()

        # Load trajectories
        self._all_trajectories = self.trajectory_db.get_all_trajectories()
        self._all_camera_trajectories = self.trajectory_db.get_all_camera_trajectories()

        self.bev_vp.loadTrajectories(self._all_trajectories)
        self.bev_vp.loadCameraTrajectories(self._all_camera_trajectories)
        self.bev_vp.setTrajectoryMode(True)

        n_targets = self.trajectory_db.get_target_count()
        n_frames = self.trajectory_db.get_frame_count()
        loaded_str = "(cached)" if count == 0 else f"({count} pts)"
        self.statusbar.showMessage(
            f"🔍 Trajectory Mode: {n_targets} targets, {n_frames} frames {loaded_str}"
        )
        self.lbl_mode.setText("🔍 Trajectory")
        self.lbl_mode.setStyleSheet(
            "padding: 6px 12px; background: #664400; border-radius: 4px; font-weight: bold;"
        )

        # Show trajectory panel
        self.trajectory_group.setVisible(True)
        self.trajectory_list.clear()
        self.trajectory_list.addItem("★ All Targets")

        saved_pairs = self.trajectory_db.get_matched_pairs()
        self._matched_pairs_cache = {r: c for r, c in saved_pairs}
        for r, c in saved_pairs:
            self.trajectory_list.addItem(f"🔗 R{r}↔C{c}")

        self._current_trajectory_id = None

    def _exitTrajectoryMode(self):
        self._trajectory_mode_active = False
        if self._play_timer.isActive():
            self._play_timer.stop()
            self.btn_play.setText("▶ Play")

        self.trajectory_group.setVisible(False)
        self.trajectory_list.clear()
        self.bev_vp.clearTrajectories()
        self.bev_vp.setTrajectoryMode(False)
        self.image_vp.clearTrajectoryProjection()

        self._loadCurrentFrame()
        self._updateModeUI()

    def _onTrajectoryIdSelected(self, item):
        if not self._trajectory_mode_active:
            return

        text = item.text()
        self.bev_vp.clearTrajectories()

        if text.startswith("★"):
            self._current_trajectory_id = None
            self.bev_vp.loadTrajectories(self._all_trajectories)
            self.bev_vp.loadCameraTrajectories(self._all_camera_trajectories)
            self.statusbar.showMessage(
                f"🔍 Showing all {len(self._all_trajectories)} targets"
            )
        else:
            import re
            match = re.match(r'🔗 R(\d+)↔C(\d+)', text)
            if match:
                rid = int(match.group(1))
                cid = int(match.group(2))
                self._current_trajectory_id = rid
                filtered_radar = {rid: self._all_trajectories.get(rid, [])}
                filtered_camera = {cid: self._all_camera_trajectories.get(cid, [])}
                self.bev_vp.loadTrajectories(filtered_radar)
                self.bev_vp.loadCameraTrajectories(filtered_camera)
                self.statusbar.showMessage(f"🔗 Showing R{rid} ↔ C{cid}")

    def _togglePlayback(self):
        if self._play_timer.isActive():
            self._play_timer.stop()
            self.btn_play.setText("▶ Play")
        else:
            fps = self.spin_fps.value()
            self._play_timer.start(1000 // fps)
            self.btn_play.setText("⏸ Pause")

    def _play_step(self):
        curr = self.slider.value()
        if curr < self.slider.maximum():
            self.slider.setValue(curr + 1)
        else:
            self._togglePlayback()

    # =========================================================================
    # Mode UI
    # =========================================================================
    def _updateModeUI(self):
        mode = self.ops.mode

        mode_styles = {
            AppMode.NORMAL: ("Ready", "#333"),
            AppMode.SELECT_RADAR: ("🎯 Click Radar Point", UI_COLORS['radar']),
            AppMode.SELECT_IMAGE: ("📍 Click Image Point", UI_COLORS['radar_pending']),
            AppMode.LANE_START: ("🚗 Click Lane Start", UI_COLORS['lane']),
            AppMode.LANE_END: ("🚗 Click Lane End", UI_COLORS['lane']),
        }

        text, bg = mode_styles.get(mode, ("Ready", "#333"))
        self.lbl_mode.setText(text)
        self.lbl_mode.setStyleSheet(
            f"padding: 6px 12px; background: {bg}; border-radius: 4px; font-weight: bold;"
        )

        image_mode_map = {
            AppMode.NORMAL: 'normal',
            AppMode.SELECT_RADAR: 'select_radar',
            AppMode.SELECT_IMAGE: 'select_image',
            AppMode.LANE_START: 'lane_start',
            AppMode.LANE_END: 'lane_end',
        }
        self.image_vp.setMode(image_mode_map.get(mode, 'normal'))

        self.lbl_pairs.setText(f"Pairs: {self.ops.num_pairs}/{MAX_POINT_PAIRS}")
        self.lbl_lanes.setText(f"Lanes: {self.ops.num_lanes}")


def main():
    try:
        from PyQt6.QtCore import Qt
        QApplication.setHighDpiScaleFactorRoundingPolicy(
            Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
        )
    except Exception:
        pass

    app = QApplication(sys.argv)
    app.setStyleSheet(QSS)
    win = MainWindow()
    win.showMaximized()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
