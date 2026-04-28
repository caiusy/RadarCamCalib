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

## 6. 执行原则

为了做到“稳扎稳打”，后续每一阶段都按下面的规则执行。

### 6.1 一次只解决一个问题

每个阶段只允许有一个主目标：

- 要么整理模型
- 要么整理标定模块
- 要么整理主窗口
- 不要一边拆 UI 一边改算法一边改数据格式

### 6.2 每个阶段都必须可回退

每个阶段结束时都要满足：

- 主程序还能启动
- 基础数据还能加载
- 至少一条核心路径还能走通

如果达不到，就说明阶段切得太大。

### 6.3 每个阶段都要有验证

验证分成 3 层：

1. `静态验证`
   - 文件结构、命名、导入关系是否清晰
2. `功能验证`
   - 主流程是否仍然可用
3. `回归验证`
   - 之前能工作的能力没有退化

### 6.4 每个阶段都单独提交

建议每个阶段一个分支、一个 PR、一个清晰提交主题，例如：

```bash
git checkout -b refactor/phase-1-domain-models
git commit -m "refactor: introduce domain models for annotations"
```

### 6.5 先文档后重构

只要一个阶段会引入新约定，就先写文档，再动代码：

- 新的数据结构
- 新的目录布局
- 新的状态机
- 新的导出格式

## 7. 分阶段实施路线图

下面这套顺序更适合现有项目，不会因为“大重构”把可用功能打断。

### Phase 0：冻结基线

目标：

- 固定当前可工作的版本，建立后续所有重构的对照基线。

验证方式：

- 新机器或新环境按照 README 可以启动。
- 用仓库内示例数据完成一次完整流程：加载同步数据、加载粗标定、标点、画线、导出、轨迹播放。
- 保存一组基线截图，后续重构后对照检查。

可执行步骤：

1. 盘点仓库中的运行时文件和临时文件。
2. 确认 `.gitignore` 是否屏蔽 `app.log`、缓存文件、运行状态文件。
3. 固定一套最小可复现数据集，写入 README 或 `docs/data_contracts.md`。
4. 记录一遍当前主流程的人工测试脚本。
5. 导出一份基线截图或短录屏。

完成定义：

- 你可以明确回答“现在这个版本什么是好的”。
- 后面每个阶段都能拿这套基线做回归。

建议提交：

- `docs: freeze baseline workflow and dataset contract`

### Phase 1：抽取领域模型

目标：

- 停止在模块之间裸传 `dict`，把核心业务对象收敛成稳定模型。

验证方式：

- 主流程中的点对、车道线、雷达点、帧数据都能以 dataclass 或 typed model 传递。
- 主窗口不再到处写 `xxx.get("id")`、`xxx.get("x")`。
- 导出逻辑从模型序列化，不再拼接散乱字段。

可执行步骤：

1. 新建 `domain/models.py`。
2. 定义 `FrameSample`、`RadarTarget`、`PointPair`、`LaneAnnotation`、`CalibrationParams`。
3. 先从 `operations.py` 开始替换，风险最低。
4. 再把 `backend.py` 的输出改成领域对象。
5. 保留一层兼容转换函数，避免一次性改太多调用点。
6. 给模型写最小单测或序列化测试。

完成定义：

- 代码里核心路径不再依赖隐式字段访问。
- 新字段的引入只需要改模型，不需要全项目手动找字典键。

建议提交：

- `refactor: introduce domain models for core calibration entities`

### Phase 2：重组标定模块

目标：

- 把投影、求解、优化拆开，避免标定逻辑散在多个文件里。

验证方式：

- `main.py` 不再包含标定公式或参数搜索细节。
- 可以清楚回答“哪个文件负责投影、哪个负责 pitch、哪个负责优化”。
- 标定相关函数能在脱离 UI 的情况下单独调用。

可执行步骤：

1. 从 `calibration.py` 中抽出 `projection_service.py`。
2. 把 vanishing point 与 pitch 相关逻辑收敛到 `calibration_service.py`。
3. 把全局优化逻辑收敛到 `optimization_service.py`。
4. 为每个 service 提供最小公开接口。
5. 在旧调用点保留适配层，先跑通功能，再删旧接口。
6. 补第一批算法单测：坐标变换、pitch、homography。

完成定义：

- 标定层可单测、可复用、可脱离 UI 运行。
- 后续要换优化算法时，不必改主窗口。

建议提交：

- `refactor: split projection calibration and optimization services`

### Phase 3：收口仓储层

目标：

- 让文件系统、JSON、SQLite 的细节退出 UI 层。

验证方式：

- UI 不再负责拼接路径、读取 JSON、判断缓存文件是否存在。
- 数据来源变化时，只改 repository，不改主窗口。

可执行步骤：

1. 新建 `infra/sync_repository.py`。
2. 把 `backend.py` 中与数据加载相关逻辑迁移过去。
3. 新建 `annotation_repository.py` 管理点对和车道线持久化。
4. 新建 `trajectory_repository.py` 封装轨迹数据和缓存。
5. 给 repository 层写文件读写测试。
6. 最后再让 UI 改调 repository 接口。

完成定义：

- 路径拼接和文件格式判断不再散落在界面代码里。
- 后续接数据库或远程接口时，主流程不需要大改。

建议提交：

- `refactor: isolate repositories for sync annotation and trajectory data`

### Phase 4：重构主窗口

目标：

- 把 `main.py` 从“全能文件”降级成“初始化和装配入口”。

验证方式：

- `main.py` 不再直接包含大段槽函数和业务流程。
- 主要 UI 职责能按窗口、面板、控制器拆开定位。

可执行步骤：

1. 先创建 `ui/main_window.py`，把窗口搭建迁过去。
2. 再把顶部工具栏相关逻辑抽成 controller。
3. 再把 batch 切换逻辑抽成 controller。
4. 再把轨迹播放逻辑抽成 controller。
5. 最后把 `main.py` 缩减为入口和依赖注入。
6. 每拆一步都跑一次手工回归。

完成定义：

- `main.py` 文件显著缩小。
- 任何一个交互问题都能快速定位到对应 controller。

建议提交：

- `refactor: split main window into ui and controllers`

### Phase 5：统一状态机与事件流

目标：

- 消灭分散状态，减少隐藏副作用。

验证方式：

- 任意模式切换都能通过统一状态对象解释。
- 撤销、切 batch、切模式、轨迹跳转不再互相污染。
- 可以打印出当前状态，方便调试。

可执行步骤：

1. 定义 `AppState`。
2. 抽象事件类型：`BatchChanged`、`RadarPointSelected`、`ImagePointSelected` 等。
3. 增加 reducer 或 state store。
4. 让 UI 只发事件，不直接改深层状态。
5. 把 `operations.py` 中的模式迁移为状态机的一部分。
6. 给关键状态流写测试：点对标注、车道线、轨迹跳转。

完成定义：

- 你能画出一张状态转移图。
- 出现交互 bug 时，可以先从状态流排查，而不是全局搜槽函数。

建议提交：

- `refactor: unify app state and interaction event flow`

### Phase 6：补评估面板

目标：

- 让标定结果可量化，而不是只靠肉眼看对不对。

验证方式：

- 每次优化前后都能看到误差变化。
- 用户能知道是“整体更好”还是“某些 batch 更差”。

可执行步骤：

1. 实现 `evaluation_service.py`。
2. 输出基础指标：均值、中位数、P95、最大误差。
3. 增加按 batch 的误差分布。
4. 增加优化前后对比。
5. 在 UI 中先做文本面板，再考虑图表。
6. 支持导出评估结果 JSON 或 TXT。

完成定义：

- `Optimize` 不再是黑盒按钮。
- 标定结果可比较、可回归、可记录。

建议提交：

- `feat: add calibration evaluation metrics and report export`

### Phase 7：测试体系

目标：

- 建立最小可依赖的回归防线。

验证方式：

- 关键模块每次修改后都能自动跑测试。
- 改坐标系、改投影、改导出格式时，测试能立刻报错。

可执行步骤：

1. 先引入 `pytest`。
2. 优先写纯算法测试，不先碰 UI 自动化。
3. 依次补：
   - 坐标变换单测
   - pitch 计算单测
   - homography 求解单测
   - repository 读写测试
   - 轨迹映射测试
4. 把示例数据做成测试夹具。
5. 后面再决定是否补 Qt 交互测试。

完成定义：

- 关键逻辑不再只能靠人工试。
- 每次重构前后都能快速知道有没有破坏主链路。

建议提交：

- `test: add projection calibration and repository regression tests`

### Phase 8：发布与 GitHub 工程化

目标：

- 把项目从“本地能跑”升级成“别人也能接手和发布”。

验证方式：

- 新人能根据 README 跑起来。
- GitHub 上能看懂版本变化、架构、数据格式和测试状态。

可执行步骤：

1. 校准 `requirements.txt`。
2. 增加 `CHANGELOG.md`。
3. 补 `docs/architecture.md`、`docs/data_contracts.md`、`docs/release_checklist.md`。
4. 增加 GitHub Actions 跑基础测试。
5. 整理 issue / PR 模板。
6. 需要的话补打包说明和 release 流程。

完成定义：

- 仓库具备基本开源工程质量。
- 别人可以接手，不依赖作者口头说明。

建议提交：

- `chore: add release docs changelog and github workflow`

## 8. 推荐的实施优先级

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

## 9. 每阶段的验证清单

每个阶段提交前，至少做下面这些检查。

### 9.1 通用检查

1. 程序能启动
2. 示例数据能加载
3. 当前 batch 能显示图像和雷达点
4. 点对标注还能工作
5. 车道线绘制还能工作
6. 导出功能没坏
7. 轨迹功能如果未触及，也必须保持可用

### 9.2 如果改了坐标或标定逻辑

1. 检查 `radar_to_bev`
2. 检查 `image_to_bev`
3. 检查 `bev_to_image`
4. 对比优化前后误差
5. 对比基线截图

### 9.3 如果改了 UI 编排

1. 检查按钮仍然正确触发
2. 检查模式切换没有卡死
3. 检查撤销不会误删其他状态
4. 检查切 batch 不丢状态或乱状态

## 10. 当前仓库到目标架构的映射关系

| 当前文件 | 现状职责 | 建议去向 |
|---|---|---|
| `main.py` | UI + 编排 + 状态 + 调用 | `app/main.py` + `ui/main_window.py` + controller |
| `backend.py` | 数据加载 + H 计算 + 导出 | `infra/*_repository.py` + `domain/calibration_service.py` |
| `calibration.py` | 几何变换与 pitch 计算 | `domain/projection_service.py` + `domain/calibration_service.py` |
| `operations.py` | 点对/车道线/模式状态 | `domain/models.py` + `app/state_store.py` |
| `viewports.py` | 图像与 BEV 视图 | `ui/image_viewport.py` + `ui/bev_viewport.py` |
| `trajectory_db.py` | 轨迹缓存与读取 | `infra/trajectory_repository.py` |
| `trajectory_dialog.py` | 轨迹交互窗口 | `ui/trajectory_dialog.py` |

## 11. 风险与规避

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

## 12. 建议的近期执行清单

如果现在立刻开工，我建议先做前 3 个阶段，不要一下推进到 UI 大拆分。

### 第一轮冲刺：只做基础稳定化

1. 新建 `docs/data_contracts.md`
2. 固化基线数据与人工回归脚本
3. 清理 `.gitignore` 和运行时残留策略

### 第二轮冲刺：只做模型收敛

1. 新建 `domain/models.py`
2. 先替换 `operations.py` 的核心对象
3. 再替换 `backend.py` 的输出对象

### 第三轮冲刺：只做标定模块边界

1. 拆 `projection_service.py`
2. 拆 `calibration_service.py`
3. 拆 `optimization_service.py`
4. 写第一批算法测试

如果这三轮都稳定，再进入主窗口拆分。

## 13. 上传到 GitHub 的建议流程

建议按阶段分支推进，不要把所有 phase 混成一个 PR：

```bash
git checkout -b refactor/phase-0-baseline
git add .
git commit -m "docs: freeze baseline and data contracts"
git push -u origin refactor/phase-0-baseline
```

下一阶段：

```bash
git checkout -b refactor/phase-1-domain-models
git add .
git commit -m "refactor: add domain models"
git push -u origin refactor/phase-1-domain-models
```

当前这个仓库更适合：

1. 每个阶段一个分支
2. 每个阶段一个主题提交
3. 每个阶段一份验证记录
4. 验证通过后再合并

## 14. 最终结论

这个项目现在最需要的不是继续堆功能，而是：

- 先把数据模型稳定下来
- 再把标定链路与 UI 解耦
- 然后把轨迹、优化、评估并到统一状态流

如果你的目标是“一步一步稳扎稳打逐步实现”，那最合理的路线不是直接大重构，而是：

1. `Phase 0` 先冻结基线
2. `Phase 1` 再收敛模型
3. `Phase 2` 再拆标定服务
4. `Phase 4/5` 最后再动主窗口和状态机

这样每一步都能验证、能回退、能持续上传到 GitHub，而不是一次性把风险放大。
