# Radar-Camera Fusion Calibration System
# 雷达-相机融合标定可视化系统

> Author: **caiusy**

A modular Python toolkit for radar-camera calibration, BEV visualization, 
trajectory analysis, and radar range estimation.

## 功能概览

- **数据加载** — 同步 JSON + 雷达 JSON + 图像自动关联
- **点对标定** — 交互式选择雷达→图像对应点，RANSAC Homography 计算
- **车道线标定** — 绘制平行车道线自动估计消失点和 Pitch 角
- **标定优化** — 基于点对自动优化 Pitch 参数
- **轨迹可视化** — 雷达/相机多目标轨迹 BEV 显示、配对和回放
- **感知范围估计** — DBSCAN 去噪 + 栅格密度 + 轮廓提取
- **模拟数据** — 一键生成带车道线和车辆的仿真数据集
- **JSON 导出** — 标定结果和感知范围 JSON 标准输出

## 项目结构

```
RadarCamCalib-main/
├── main.py                  # 旧版入口 (保留兼容)
├── requirements.txt
│
├── data/                    # 数据层
│   ├── data_models.py       #   数据结构 (dataclass)
│   ├── sync_loader.py       #   同步 JSON 解析
│   ├── radar_loader.py      #   雷达 JSON 解析 (含中→英类别映射)
│   ├── image_loader.py      #   图像加载
│   └── sync_dataset.py      #   数据集封装
│
├── calibration/             # 标定算法层
│   ├── homography_solver.py #   Homography + RANSAC
│   ├── calibration_session.py # 标定会话管理
│   ├── projector.py         #   坐标投影 (Radar↔Image)
│   └── calibration_io.py    #   标定 JSON 读写
│
├── trajectory/              # 轨迹处理层
│   ├── trajectory_models.py #   轨迹数据结构
│   ├── trajectory_builder.py #  从 Sync 构建轨迹
│   └── trajectory_filter.py #   轨迹筛选/展平
│
├── range_estimation/        # 感知范围估计层
│   ├── point_filter.py      #   基础过滤 + DBSCAN
│   ├── density_grid.py      #   栅格密度分析
│   ├── contour_extractor.py #   轮廓 → 多边形
│   └── range_estimator.py   #   完整 Pipeline
│
├── visualization/           # 可视化层 (纯 OpenCV, 无 Qt)
│   ├── color_map.py         #   颜色定义
│   ├── image_overlay.py     #   图像叠加绘制
│   ├── bev_renderer.py      #   BEV 渲染
│   └── radar_drawer.py      #   雷达投影绘制
│
├── ui/                      # UI 层 (PyQt6)
│   ├── image_view.py        #   图像视口 (点击交互)
│   ├── bev_view.py          #   BEV 视口 (轨迹/标定)
│   └── operations.py        #   操作状态机 (点对/车道线/轨迹)
│
├── export/                  # 导出层
│   └── json_exporter.py     #   JSON + 文本报告导出
│
├── tools/                   # 工具
│   └── generate_mock_dataset.py # 模拟数据集生成器
│
├── tests/                   # 测试
│   ├── test_data.py         #   数据层测试
│   └── test_calibration.py  #   标定算法测试
│
├── configs/                 # 配置
│   └── default_config.yaml  #   默认参数配置
│
└── dataset/                 # 数据集 (不提交)
    └── ...
```

## 快速开始

### 安装依赖

```bash
pip install -r requirements.txt
```

### 生成模拟数据

```bash
python tools/generate_mock_dataset.py --output mock_dataset --num_frames 100
```

### 运行主程序 (旧版兼容入口)

```bash
python main.py
```

### 运行测试

```bash
python -m pytest tests/ -v
```

## 模块使用示例

### 加载数据集

```python
from data import SyncDataset

ds = SyncDataset()
ds.load("path/to/dataset")
img, radar = ds.get_current_data()
```

### 计算标定

```python
from calibration.calibration_session import CalibrationSession

session = CalibrationSession()
session.add_point(radar_x, radar_y, image_u, image_v)
# ... 添加至少4个点对
result = session.compute()
print(f"Mean error: {result.mean_error:.2f} px")
```

### 投影雷达到图像

```python
from calibration.projector import project_radar_to_image
import numpy as np

pts = np.array([[3.5, 42.0], [-2.0, 60.0]])
projected = project_radar_to_image(pts, result.H)
```

### 感知范围估计

```python
from range_estimation.range_estimator import estimate_radar_range
import numpy as np

points = np.array(...)  # Nx2 all trajectory points
range_result = estimate_radar_range(points)
```

## 设计原则

1. **分层解耦** — 数据/算法/可视化/UI 四层分离
2. **无 UI 可测试** — 所有算法模块可独立 import 和测试
3. **中文兼容** — 雷达 JSON 中文类别自动转英文
4. **渐进迁移** — 旧 `main.py` 保留可运行，新模块逐步替换
