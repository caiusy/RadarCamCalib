"""
trajectory_dialog.py - Dialog for matching radar and camera trajectories
轨迹匹配对话框 - 用于选择雷达和相机目标配对

Provides a popup dialog with two lists for selecting radar and camera targets
to create matched pairs for visualization.
"""

from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QListWidget, QListWidgetItem,
    QPushButton, QLabel, QGroupBox, QSplitter, QFrame
)
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QColor, QFont

from config import PAIR_COLORS


class TrajectoryMatchDialog(QDialog):
    """Dialog for selecting radar-camera trajectory pairs."""
    
    # Signal: (radar_id, camera_id) when a pair is confirmed
    pairSelected = pyqtSignal(int, int)
    
    # Signal: (radar_id) for live preview when radar is selected
    radarPreview = pyqtSignal(int)
    
    # Signal: (camera_id) for live preview when camera is selected
    cameraPreview = pyqtSignal(int)
    
    def __init__(self, trajectory_db, parent=None):
        super().__init__(parent)
        self.trajectory_db = trajectory_db
        self.selected_radar_id = None
        self.selected_camera_id = None
        
        self.setWindowTitle("🔗 Trajectory Matching")
        self.setModal(False)  # Non-modal so user can interact with main window
        self.setMinimumSize(400, 500)
        
        self._setupUI()
        self._loadData()
        self._connectSignals()
        self._pair_callback = None
        
    def set_on_pair_selected(self, callback):
        """Set direct callback for pair selection."""
        self._pair_callback = callback
    
    def _setupUI(self):
        layout = QVBoxLayout(self)
        
        # Title
        title = QLabel("Click target to preview. Click 'Show Pair' to confirm.")
        title.setFont(QFont("Arial", 11))
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setWordWrap(True)
        layout.addWidget(title)
        
        # Splitter for two lists
        splitter = QSplitter(Qt.Orientation.Horizontal)
        
        # Radar list
        radar_group = QGroupBox("📡 Radar Targets")
        radar_lay = QVBoxLayout(radar_group)
        self.radar_list = QListWidget()
        self.radar_list.setStyleSheet("background: #2a2a2a; color: #fff; font-size: 13px;")
        radar_lay.addWidget(self.radar_list)
        splitter.addWidget(radar_group)
        
        # Camera list
        camera_group = QGroupBox("📷 Camera Targets")
        camera_lay = QVBoxLayout(camera_group)
        self.camera_list = QListWidget()
        self.camera_list.setStyleSheet("background: #2a2a2a; color: #fff; font-size: 13px;")
        camera_lay.addWidget(self.camera_list)
        splitter.addWidget(camera_group)
        
        layout.addWidget(splitter)
        
        # Selection display
        self.lbl_selection = QLabel("Selection: None (click targets to preview)")
        self.lbl_selection.setStyleSheet("padding: 8px; background: #333; border-radius: 4px; font-weight: bold;")
        layout.addWidget(self.lbl_selection)
        
        # Buttons
        btn_layout = QHBoxLayout()
        
        self.btn_show = QPushButton("🔗 Bind & Show Pair")
        self.btn_show.setEnabled(False)
        self.btn_show.setStyleSheet("padding: 8px 16px; font-weight: bold;")
        btn_layout.addWidget(self.btn_show)
        
        self.btn_show_all = QPushButton("📊 Show All")
        self.btn_show_all.setStyleSheet("padding: 8px 16px;")
        btn_layout.addWidget(self.btn_show_all)
        
        self.btn_close = QPushButton("✖ Close")
        self.btn_close.setStyleSheet("padding: 8px 16px;")
        btn_layout.addWidget(self.btn_close)
        
        layout.addLayout(btn_layout)
    
    def _loadData(self):
        """Load radar and camera target IDs into lists."""
        if not self.trajectory_db or not self.trajectory_db.loaded:
            return
        
        # Get all radar target IDs
        radar_ids = self.trajectory_db.get_all_target_ids()
        for rid in radar_ids:
            traj = self.trajectory_db.get_trajectory(rid)
            item = QListWidgetItem(f"R{rid} ({len(traj)} frames)")
            item.setData(Qt.ItemDataRole.UserRole, rid)
            color = QColor(PAIR_COLORS[rid % len(PAIR_COLORS)])
            item.setForeground(color)
            self.radar_list.addItem(item)
        
        # Get all camera target IDs
        for cid in radar_ids:
            traj = self.trajectory_db.get_camera_trajectory(cid)
            if traj:
                item = QListWidgetItem(f"C{cid} ({len(traj)} frames)")
                item.setData(Qt.ItemDataRole.UserRole, cid)
                color = QColor(PAIR_COLORS[cid % len(PAIR_COLORS)])
                item.setForeground(color)
                self.camera_list.addItem(item)
    
    def _connectSignals(self):
        # Use currentItemChanged for more robust selection detection
        self.radar_list.currentItemChanged.connect(self._onRadarSelected)
        self.camera_list.currentItemChanged.connect(self._onCameraSelected)
        self.btn_show.clicked.connect(self._onShowPair)
        self.btn_show_all.clicked.connect(self._onShowAll)
        self.btn_close.clicked.connect(self.close)
    
    def _onRadarSelected(self, current, previous):
        if current:
            self.selected_radar_id = current.data(Qt.ItemDataRole.UserRole)
            self._updateSelection()
            # Emit live preview signal
            self.radarPreview.emit(self.selected_radar_id)
    
    def _onCameraSelected(self, current, previous):
        if current:
            self.selected_camera_id = current.data(Qt.ItemDataRole.UserRole)
            self._updateSelection()
            # Emit live preview signal
            self.cameraPreview.emit(self.selected_camera_id)
    
    def _updateSelection(self):
        radar_str = f"R{self.selected_radar_id}" if self.selected_radar_id is not None else "?"
        camera_str = f"C{self.selected_camera_id}" if self.selected_camera_id is not None else "?"
        self.lbl_selection.setText(f"Selection: {radar_str} ↔ {camera_str}")
        
        # Enable show button if both selected
        both_selected = self.selected_radar_id is not None and self.selected_camera_id is not None
        self.btn_show.setEnabled(both_selected)
        
        if both_selected:
            self.lbl_selection.setStyleSheet("padding: 8px; background: #446600; border-radius: 4px; font-weight: bold;")
        else:
            self.lbl_selection.setStyleSheet("padding: 8px; background: #333; border-radius: 4px; font-weight: bold;")
    
    def _onShowPair(self):
        if self.selected_radar_id is not None and self.selected_camera_id is not None:
            print(f"[DEBUG] TrajectoryDialog: Emitting pairSelected({self.selected_radar_id}, {self.selected_camera_id})")
            self.pairSelected.emit(self.selected_radar_id, self.selected_camera_id)
            if self._pair_callback:
                print(f"[DEBUG] TrajectoryDialog: Calling callback...")
                self._pair_callback(self.selected_radar_id, self.selected_camera_id)
    
    def _onShowAll(self):
        # Emit -1, -1 to indicate "show all"
        self.pairSelected.emit(-1, -1)
    
    def refresh(self):
        """Refresh data from trajectory database."""
        self.radar_list.clear()
        self.camera_list.clear()
        self.selected_radar_id = None
        self.selected_camera_id = None
        self._loadData()
        self._updateSelection()
