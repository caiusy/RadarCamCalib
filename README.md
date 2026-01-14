# RadarCamCalib

**Radar-Camera Calibration & Annotation Tool**

A PyQt6-based desktop application for calibrating radar-camera systems and annotating point correspondences for autonomous driving sensor fusion.

![Python](https://img.shields.io/badge/Python-3.9+-blue.svg)
![PyQt6](https://img.shields.io/badge/PyQt6-6.0+-green.svg)
![License](https://img.shields.io/badge/License-MIT-yellow.svg)

![Screenshot](assets/preview_v2.png)

---

## 📅 Latest Updates (2026-01-15)

- **Pitch Optimization**: Implemented global optimization algorithm to refine camera pitch using all collected point pairs (`Optimize` button).
- **JSON Export**: Enhanced parameter saving to include full **Homography Matrices** (`radar_to_bev` and `camera_to_bev`) for direct coordinate mapping.
- **Coordinate System**: Standardized to **Y-Axis Forward** (Vehicle Frame) for clearer BEV visualization and calibration.
- **UI Improvements**: Added auto-refresh of pixel projections after optimization and fixed saving bugs.

---


## 🎯 Features

- **Radar-to-Image Projection**: Real-time projection of radar targets onto camera images
- **BEV Visualization**: Top-down Bird's Eye View with real-time radar/image comparison
- **Point Pair Annotation**: Click radar points (magenta) → click image points (yellow) → create matched pairs
- **Auto-Pitch Calculation**: Compute camera pitch automatically from lane lines or vanishing points
- **Lane Line Drawing**: Draw lane lines with 2-point (start/end) mode
- **Multi-Batch Support**: Navigate through synchronized image/radar frames
- **Undo/Redo**: Undo last operation with one click
- **Pitch Optimization**: Global optimization of pitch using all collected point pairs
- **Parameter Export**: Save intrinsics, extrinsics, and homography matrices to JSON
- **Dark Engineering Theme**: Low eye-strain UI for long annotation sessions
- **Zoom & Pan**: Scroll to zoom, right-click drag to pan canvas

---

## 📦 Installation

### Prerequisites

- Python 3.9+
- pip

### Install Dependencies

```bash
pip install PyQt6 numpy opencv-python
```

### Clone & Run

```bash
git clone https://github.com/caiusy/RadarCamCalib.git
cd RadarCamCalib
python main.py
```


---

## 🚀 Quick Start

### 1. Prepare Your Data

Your data should follow this structure:

```
dataset/
├── images/           # Camera images (.jpg, .png)
│   ├── 000.jpg
│   ├── 001.jpg
│   └── ...
├── radar/            # Radar target JSON files
│   ├── 000.json      # {"targets": [{"id": 0, "x": 20.5, "y": -3.2, ...}, ...]}
│   ├── 001.json
│   └── ...
├── data_sync.json    # Synchronization mapping
└── calibration_points.txt  # Initial calibration (4+ point pairs)
```

**data_sync.json** format:
```json
[
  {"image_path": "images/000.jpg", "radar_json": "radar/000.json"},
  {"image_path": "images/001.jpg", "radar_json": "radar/001.json"}
]
```

**calibration_points.txt** format (space-separated):
```
# radar_x radar_y pixel_u pixel_v
20.00 -5.00 540.00 620.00
20.00  5.00 740.00 620.00
40.00 -5.00 540.00 380.00
40.00  5.00 740.00 380.00
```

### 2. Run the Application

```bash
python main.py
```

### 3. Load Data

1. Click **📂 Sync JSON** → Select `data_sync.json`
2. Click **📐 Coarse TXT** → Select `calibration_points.txt`

### 4. Optimize & Save
1. Add point pairs across multiple frames.
2. Click **✨ Optimize** to refine pitch globally.
3. Click **📄 JSON** to save calibration results (including Homography matrices).
   - Status indicator turns **green ●** when loaded successfully

### 4. Annotate Point Pairs

1. Ensure **Point Pair** mode is selected
2. Status bar shows **"🎯 Click Radar Point"** (purple)
3. Click a **magenta radar circle** on the image → it turns **yellow**
4. Status changes to **"📍 Click Image Point"** (yellow)
5. Move mouse to see **preview circle**, click to confirm
6. Both points turn the **same color** (red/green/blue...) indicating a completed pair
7. Repeat for up to 10 pairs

### 5. Draw Lane Lines

1. Select **Lane (2pts)** mode
2. Click **start point** of lane
3. Move mouse to see preview line
4. Click **end point** to complete

### 6. Save Data

- Click **💾 Save** to export:
  - `point_pairs_YYYYMMDD_HHMMSS.txt`
  - `all_lanes_YYYYMMDD_HHMMSS.txt`

---

## 🖱️ Controls

| Action | Control |
|--------|---------|
| **Zoom** | Mouse scroll wheel |
| **Pan** | Right-click + drag |
| **Select Point** | Left-click |
| **Undo** | Click **↩ Undo** button |
| **Clear All** | Click **🗑 Clear** button |

---

## 📁 Project Structure

```
RadarCamCalib/
├── main.py           # Main application entry
├── config.py         # Colors, styles, constants
├── backend.py        # Data loading, calibration, export
├── viewports.py      # Image & BEV viewport widgets
├── operations.py     # Point pair & lane logic with undo
├── generate_dummy_data.py  # Generate test data
└── README.md
```

---

## 🔧 Output File Formats

### point_pairs_*.txt

```
# pixel_u, pixel_v, radar_id, radar_x, radar_y, range, velocity, rcs, batch
540.00, 620.00, 0, 20.00, -5.00, 20.62, 5.50, 15.20, 0
```

### all_lanes_*.txt

```
# lane_id, start_u, start_v, end_u, end_v
1, 100.00, 500.00, 1100.00, 500.00
```

---

## 📦 Packaging as Executable (Windows/macOS)

### Using PyInstaller

1. Install PyInstaller:
   ```bash
   pip install pyinstaller
   ```

2. Create executable:
   ```bash
   # Single file executable
   pyinstaller --onefile --windowed --name RadarCamCalib main.py
   
   # Or with icon (Windows)
   pyinstaller --onefile --windowed --icon=icon.ico --name RadarCamCalib main.py
   ```

3. Find the executable in `dist/` folder

### Recommended PyInstaller Options

```bash
pyinstaller --onefile \
            --windowed \
            --name RadarCamCalib \
            --add-data "config.py:." \
            --add-data "backend.py:." \
            --add-data "viewports.py:." \
            --add-data "operations.py:." \
            main.py
```

### Troubleshooting

- **Missing modules**: Add `--hidden-import=PyQt6.sip`
- **Large file size**: Use `--exclude-module` to remove unused modules
- **macOS signing**: May need to disable Gatekeeper for unsigned apps

---

## 🧪 Generate Test Data

Run the included script to generate synthetic test data:

```bash
python generate_dummy_data.py
```

This creates a `dataset/` folder with sample images, radar JSON, and calibration files.

---

## 📝 License

MIT License - feel free to use, modify, and distribute.

---

## 🤝 Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

---

## 📧 Contact

For questions or issues, please open a GitHub issue.
