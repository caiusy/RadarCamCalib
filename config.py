"""
config.py - Configuration, constants, and styles for Radar-Camera Fusion System
"""

# =============================================================================
# Color Palette
# =============================================================================
COLORS = {
    'bg': '#1E1E1E',
    'panel': '#252526',
    'border': '#3E3E42',
    'text': '#D4D4D4',
    'text_dim': '#888888',
    'accent': '#007ACC',
    'success': '#4CAF50',
    'error': '#D16D6D',
    'warning': '#FFA500',
    
    # Radar/Point colors
    'radar': '#FF00FF',           # Magenta - radar points
    'radar_pending': '#FFFF00',   # Yellow - selected radar awaiting image point
    'image_preview': '#00FFFF',   # Cyan - preview circle at mouse
    'lane': '#FF6600',            # Orange - lane lines
}

# 10 distinct colors for completed point pairs
PAIR_COLORS = [
    '#FF4444',  # Red
    '#44FF44',  # Green
    '#4444FF',  # Blue
    '#FFFF44',  # Yellow
    '#FF44FF',  # Magenta
    '#44FFFF',  # Cyan
    '#FF8844',  # Orange
    '#FF44AA',  # Pink
    '#88FF44',  # Lime
    '#AA44FF',  # Purple
]

# =============================================================================
# Qt Style Sheet
# =============================================================================
QSS = """
QWidget {
    background-color: #1E1E1E;
    color: #D4D4D4;
    font-family: Arial, Helvetica, sans-serif;
    font-size: 13px;
}

QFrame#TopBar {
    background-color: #252526;
    border-bottom: 1px solid #3E3E42;
}

QGroupBox {
    border: 1px solid #3E3E42;
    border-radius: 4px;
    margin-top: 10px;
    padding-top: 10px;
    font-weight: bold;
}

QGroupBox::title {
    subcontrol-origin: margin;
    left: 8px;
    padding: 0 4px;
    color: #888;
}

QPushButton {
    background-color: #333;
    border: 1px solid #3E3E42;
    border-radius: 4px;
    padding: 6px 12px;
    color: white;
    min-width: 60px;
}

QPushButton:hover {
    background-color: #444;
    border-color: #555;
}

QPushButton:pressed {
    background-color: #007ACC;
}

QPushButton:disabled {
    background-color: #252526;
    color: #555;
}

QPushButton#UndoBtn {
    background-color: #664400;
}

QPushButton#UndoBtn:hover {
    background-color: #886600;
}

QPushButton#ClearBtn {
    background-color: #660000;
}

QPushButton#ClearBtn:hover {
    background-color: #880000;
}

QSlider::groove:horizontal {
    height: 8px;
    background: #333;
    border-radius: 4px;
}

QSlider::handle:horizontal {
    background: #007ACC;
    width: 18px;
    height: 18px;
    margin: -5px 0;
    border-radius: 9px;
}

QSlider::sub-page:horizontal {
    background: #007ACC;
    border-radius: 4px;
}

QSplitter::handle {
    background: #3E3E42;
}

QSplitter::handle:hover {
    background: #007ACC;
}

QGraphicsView {
    border: 1px solid #3E3E42;
    background: #000;
}

QRadioButton::indicator {
    width: 14px;
    height: 14px;
}

QRadioButton::indicator:checked {
    background: #007ACC;
    border: 2px solid #007ACC;
    border-radius: 8px;
}

QRadioButton::indicator:unchecked {
    background: #333;
    border: 2px solid #555;
    border-radius: 8px;
}

QDoubleSpinBox, QSpinBox {
    background: #333;
    border: 1px solid #3E3E42;
    border-radius: 3px;
    padding: 3px;
}

QStatusBar {
    background: #252526;
    border-top: 1px solid #3E3E42;
}

QLabel#StatusReady {
    color: #4CAF50;
}

QLabel#StatusPending {
    color: #FFA500;
}
"""

# =============================================================================
# Application Constants
# =============================================================================
MAX_POINT_PAIRS = 10
MARKER_SIZE_RADAR = 20       # Radar point marker size
MARKER_SIZE_IMAGE = 16       # Image point marker size
MARKER_SIZE_PREVIEW = 24     # Preview circle size
LANE_POINT_SIZE = 10         # Lane endpoint size
