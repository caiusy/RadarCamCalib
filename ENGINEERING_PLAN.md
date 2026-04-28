# RadarCamCalib 工程设计与分步实施方案

## 1. 目标

把当前的 `RadarCamCalib` 从“可用的单体 PyQt 工具”整理成“可持续演进的工程化项目”。

重点不是立刻重写，而是按阶段完成这 4 件事：

1. 把业务模型和 UI 彻底分开。
2. 把标定、轨迹、导出、状态持久化变成清晰模块。
3. 把“点对标注 / 车道线 / 轨迹播放 / 参数优化”统一到一条数据流里。
4. 给后续扩展留出空间，比如多车型、多雷达、多相机、批量评估和测试。

## 2. 现状判断

当前仓库已经具备完整功能闭环，但工程上有几个明显问题：

### 2.1 当前优势

- `calibration.py` 已经有比较完整的坐标变换与俯仰角计算逻辑。
- `operations.py` 已经抽出了点对、车道线、模式状态这些核心交互对象。
- `trajectory_db.py` / `trajectory_dialog.py` 已经形成轨迹子功能。
- `README.md` 和 `USER_MANUAL.md` 已覆盖安装和使用流程。

### 2.2 当前主要问题

- `main.py` 过大，承担了 UI、状态编排、业务调用、持久化恢复等多种职责。
- `backend.py`、`calibration.py`、`calib_manager.py` 的边界不够清楚，标定职责分散。
- 数据结构主要靠 `dict` 在模块间传递，长期容易出现字段漂移和隐式耦合。
- “点对标注 / 车道线 / 轨迹查看”本质上都是状态机驱动，但当前状态切换分散在窗口逻辑里。
- 缺少一套明确的数据契约：输入数据格式、缓存格式、导出格式、评估指标都还偏“工具式”。
- 测试比较薄，当前更像功能验证，不像回归保护。

## 3. 更合理的目标架构

不建议一次性重写。建议先按“分层重构”演进到下面这个结构。

```text
RadarCamCalib/
├── app/
│   ├── main.py                  # 程序入口
│   ├── bootstrap.py             # 组装依赖、初始化配置
│   └── state_store.py           # 应用级状态与事件分发
├── domain/
│   ├── models.py                # Frame、RadarTarget、PointPair、Lane、Trajectory
│   ├── calibration_service.py   # 标定核心算法
│   ├── projection_service.py    # Radar/Image/BEV 坐标变换
│   ├── optimization_service.py  # Pitch 优化、误差统计
│   └── evaluation_service.py    # 标定误差评估、可视化指标
├── infra/
│   ├── sync_repository.py       # data_sync.json / 图片 / radar json 加载
│   ├── annotation_repository.py # 点对、车道线、状态文件
│   ├── trajectory_repository.py # 轨迹 JSON / SQLite / cache
│   └── export_service.py        # txt / json 导出
├── ui/
│   ├── main_window.py           # 主窗口编排
│   ├── panels.py                # 右侧控制区
│   ├── image_viewport.py        # 图像视图
│   ├── bev_viewport.py          # BEV 视图
│   ├── trajectory_dialog.py     # 轨迹交互窗口
│   └── presenters.py            # 视图模型适配
├── tests/
│   ├── test_projection.py
│   ├── test_pitch_optimization.py
│   ├── test_data_loading.py
│   └── test_trajectory_mapping.py
├── docs/
│   ├── architecture.md
│   ├── data_contracts.md
│   └── release_checklist.md
└── README.md
```

## 4. 核心设计原则

### 4.1 先固化数据模型，再谈功能堆叠

先把这些对象变成稳定模型，而不是继续在 `dict` 上加字段：

- `FrameSample`
- `RadarTarget`
- `ImagePoint`
- `PointPair`
- `LaneAnnotation`
- `TrajectoryTrack`
- `CalibrationParams`
- `CalibrationResult`

这样做的收益很直接：

- 字段有约束，避免 UI 和算法层各自理解不同。
- 导出格式更容易版本化。
- 后续接数据库、缓存、接口都更稳。

### 4.2 单一坐标系定义必须写死

当前文档里已经多次提到 BEV 坐标系切换，说明这是历史痛点。后续必须把坐标契约固化：

- `BEV`: `X = Right`, `Y = Forward`, `Z = Up`
- `Radar Raw`: `X = Forward`, `Y = Left`
- `Image`: `u = right`, `v = down`

并把每个变换写成单测：

- `radar_to_bev`
- `bev_to_radar`
- `image_to_bev`
- `bev_to_image`
- `radar_to_image`

### 4.3 状态机统一

交互模式本质上是有限状态机，建议统一收口：

- `Idle`
- `SelectingRadarPoint`
- `SelectingImagePoint`
- `DrawingLaneStart`
- `DrawingLaneEnd`
- `TrajectoryInspecting`
- `PlaybackRunning`

所有鼠标点击、撤销、切换 batch、切换模式，都通过状态机转移，不直接散落在主窗口槽函数里。

### 4.4 “计算”与“显示”分离

视图层只关心：

- 当前要画什么
- 当前高亮什么
- 当前鼠标悬停什么

计算层只关心：

- 给一个雷达点，投影到图像哪里
- 给一个像素点，反投影到 BEV 哪里
- 给一组点对，优化出怎样的 pitch

不要让 `QGraphicsItem` 反向驱动业务状态。

## 5. 建议的数据流

### 5.1 标定主链路

```text
Sync JSON / Radar JSON / Image
    -> FrameSample
    -> ProjectionService
    -> ImageViewport + BEVViewport
    -> 用户点击形成 PointPair / LaneAnnotation
    -> AnnotationRepository 持久化
    -> OptimizationService 计算 pitch / H / 误差
    -> CalibrationResult
    -> UI 刷新 + JSON/TXT 导出
```

### 5.2 轨迹链路

```text
Radar frames / DB / cache
    -> TrajectoryRepository
    -> TrajectoryTrack[]
    -> BEV playback model
    -> 点击轨迹点
    -> 跳转 FrameSample
    -> 同步刷新 Image + BEV
```

### 5.3 评估链路

```text
PointPair[] + CalibrationParams
    -> EvaluationService
    -> reprojection error / BEV error / batch statistics
    -> metrics panel / export report
```

## 6. 分阶段实施方案

下面这套顺序更适合现有项目，不会因为“大重构”把可用功能打断。

### Phase 0：冻结基线

目标：先把现状稳定下来，避免后续改动没有对照物。

要做的事：

1. 清理仓库中的运行残留文件策略。
2. 固定一套可复现的示例数据。
3. 补一份 `docs/data_contracts.md`，定义当前输入输出格式。
4. 把主要功能录一段 GIF 或截图，作为回归基线。

验收标准：

- 新机器按 README 能启动。
- 示例数据能完成一遍完整标定流程。

### Phase 1：抽取领域模型

目标：停止在模块间直接传裸 `dict`。

要做的事：

1. 新增 `domain/models.py`。
2. 把 `PointPair`、`Lane`、`RadarTarget`、`CalibrationParams` 全部标准化。
3. `backend.py` 和 `operations.py` 改为返回领域对象，而不是散乱字典。

验收标准：

- 主流程不再依赖 `target.get(...)` 这种隐式字段访问。
- 导出逻辑统一从模型序列化。

### Phase 2：重组标定模块

目标：把“加载 H”和“几何投影 / pitch 优化”放到清晰边界里。

建议拆分：

- `projection_service.py`
  - 负责 `radar <-> bev <-> image`
- `calibration_service.py`
  - 负责 vanishing point、pitch 估计、homography 求解
- `optimization_service.py`
  - 负责基于全部点对的全局搜索与误差最小化

验收标准：

- `main.py` 不再直接拼算法细节。
- 所有标定入口都通过 service 调用。

### Phase 3：收口仓储层

目标：把“文件格式”和“业务对象”分开。

要做的事：

1. `sync_repository.py` 负责加载 `data_sync.json`、图片路径、radar json。
2. `annotation_repository.py` 负责点对、车道线、状态恢复。
3. `trajectory_repository.py` 负责轨迹 JSON / SQLite / cache。
4. 所有 repository 返回领域对象，不在 UI 层拼路径。

验收标准：

- UI 不再知道磁盘组织细节。
- 后续替换缓存实现时，不影响视图层。

### Phase 4：重构主窗口

目标：把 `main.py` 从“大总管”降级成“编排器”。

建议拆分：

- `main_window.py`: 主界面组装
- `toolbar_controller.py`: 顶部操作栏
- `batch_controller.py`: 批次切换
- `mode_controller.py`: 交互模式切换
- `playback_controller.py`: 轨迹播放控制

验收标准：

- `main.py` 只保留入口与依赖注入。
- 单个文件长度显著下降。

### Phase 5：统一状态机与事件流

目标：减少“点一下这里，别处状态跟着变”的隐式副作用。

建议做法：

1. 定义统一 `AppState`。
2. 所有用户动作都抽成事件：
   - `LoadSyncRequested`
   - `BatchChanged`
   - `RadarPointSelected`
   - `ImagePointSelected`
   - `LaneStarted`
   - `LaneFinished`
   - `OptimizeRequested`
   - `TrajectoryPlaybackStarted`
3. 由 reducer/controller 修改状态，再通知视图刷新。

验收标准：

- 撤销、切 batch、恢复缓存、轨迹跳转不会互相污染状态。

### Phase 6：补评估面板

目标：让系统不仅“能标”，还要“知道标得怎么样”。

建议指标：

- 图像重投影误差均值 / P95
- BEV 平面误差
- 每个 batch 的误差分布
- 每个 radar target 的稳定性
- 优化前后误差对比

验收标准：

- 用户能看到当前标定是否比上一次更好。
- 优化按钮不再只是“黑盒按钮”。

### Phase 7：测试体系

最少要补这几类测试：

1. 坐标变换单测
2. Pitch 计算单测
3. Homography 求解单测
4. 点对导入导出测试
5. 轨迹点击跳帧测试

验收标准：

- 修 BEV 或坐标系时，测试能第一时间报错。

### Phase 8：发布与 GitHub 工程化

建议补齐：

1. `requirements.txt` 校准
2. `README` 中加入数据格式、架构图、常见问题
3. `CHANGELOG.md`
4. GitHub release checklist
5. 可选：GitHub Actions 做基础 lint / test

## 7. 推荐的实施优先级

如果资源有限，建议按这个顺序推进：

1. `Phase 0` 冻结基线
2. `Phase 1` 领域模型
3. `Phase 2` 标定模块重组
4. `Phase 4` 主窗口拆分
5. `Phase 5` 状态机统一
6. `Phase 6` 误差评估
7. `Phase 7` 自动化测试
8. `Phase 8` GitHub 工程化

原因很简单：

- 没有模型，后面所有改动都会继续耦合。
- 没有统一标定服务，后面很难稳步加功能。
- 没有主窗口拆分，任何 UI 修改成本都会越来越高。

## 8. 当前仓库到目标架构的映射关系

| 当前文件 | 现状职责 | 建议去向 |
|---|---|---|
| `main.py` | UI + 编排 + 状态 + 调用 | `app/main.py` + `ui/main_window.py` + controller |
| `backend.py` | 数据加载 + H 计算 + 导出 | `infra/*_repository.py` + `domain/calibration_service.py` |
| `calibration.py` | 几何变换与 pitch 计算 | `domain/projection_service.py` + `domain/calibration_service.py` |
| `operations.py` | 点对/车道线/模式状态 | `domain/models.py` + `app/state_store.py` |
| `viewports.py` | 图像与 BEV 视图 | `ui/image_viewport.py` + `ui/bev_viewport.py` |
| `trajectory_db.py` | 轨迹缓存与读取 | `infra/trajectory_repository.py` |
| `trajectory_dialog.py` | 轨迹交互窗口 | `ui/trajectory_dialog.py` |

## 9. 风险与规避

### 风险 1：一边重构一边改功能，主流程失稳

规避：

- 每个 phase 结束都保证主流程可运行。
- 不做“大爆炸式迁移”。

### 风险 2：坐标系再次混乱

规避：

- 所有转换函数单测化。
- 坐标系定义写进 `docs/data_contracts.md`。

### 风险 3：UI 拆分后信号连接变复杂

规避：

- 用 controller / presenter 收口信号。
- View 只发事件，不直接调用深层服务。

### 风险 4：轨迹功能与标定功能状态互相污染

规避：

- 轨迹模式单独建 playback state。
- 禁止视图层直接读写对方内部状态。

## 10. 建议的近期执行清单

如果现在就开始做，我建议按下面 7 步落地：

1. 新建 `docs/architecture.md` 与 `docs/data_contracts.md`
2. 抽出统一 `domain/models.py`
3. 为 `calibration.py` 写第一批变换测试
4. 把 `backend.py` 中加载和导出拆开
5. 把 `main.py` 中轨迹相关逻辑先抽成独立 controller
6. 给优化结果增加误差统计输出
7. 最后再改 README 和 GitHub Actions

## 11. 上传到 GitHub 的建议流程

建议不要直接在大重构后一次性上传，而是按小步提交：

```bash
git checkout -b docs/engineering-plan
git add README.md ENGINEERING_PLAN.md
git commit -m "docs: add engineering plan for staged refactor"
git push -u origin docs/engineering-plan
```

如果你要直接推到 `main`，至少保证：

1. 文档能和当前代码对应
2. README 链接不失效
3. 仓库里不带运行期垃圾文件

## 12. 最终结论

这个项目现在最需要的不是继续堆功能，而是：

- 先把数据模型稳定下来
- 再把标定链路与 UI 解耦
- 然后把轨迹、优化、评估并到统一状态流

这样后面无论是加“多雷达”“批量标定”“自动匹配初值”“误差报告”，都会顺得多。
