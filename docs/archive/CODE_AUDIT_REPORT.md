# HRC-PalletizingPlanner 系统性代码审计报告

> **审计日期**: 2026-02-25
> **审计范围**: 全部源代码 (C++ header-only库、测试、Python脚本、MATLAB仿真、CMake构建、文档、版本控制)
> **审计方法**: 静态分析 + `-Wall -Wextra` 编译 + 运行时测试 + 逐文件人工审查

---

## 目录

- [一、严重问题 (Critical)](#一严重问题-critical)
- [二、重要问题 (Major)](#二重要问题-major)
- [三、一般问题 (Minor)](#三一般问题-minor)
- [四、信息与改进建议 (Info)](#四信息与改进建议-info)
- [五、统计汇总](#五统计汇总)
- [六、优先修复清单](#六优先修复清单)

---

## 一、严重问题 (Critical)

### C-01: `PathPlannerOptimized.hpp` int16_t 节点ID溢出

| 属性 | 值 |
|------|-----|
| **文件** | `include/PalletizingPlanner/PathPlannerOptimized.hpp:598` |
| **影响** | int16_t 最大值 32767, maxIterations=20000 时极有可能溢出 |
| **后果** | 节点ID回绕为负数 → 树拓扑错误 → 规划失败/崩溃 |

```cpp
// 当前代码 (严重缺陷)
int16_t newNodeId = static_cast<int16_t>(nodes_.size());   // ❌ 溢出

// 正确代码
int32_t newNodeId = static_cast<int32_t>(nodes_.size());   // ✅
```

**说明**: `maxIterations` 配置默认 20000, quality 配置可达 50000。超过 32767 个节点后 `int16_t` 回绕, `parentId` 指向错误节点, 导致路径提取产生垃圾结果或段错误。虽然 `parentId` 字段本身是 `int32_t`, 但赋值时已被截断。

---

### C-02: FK 返回单位文档严重错误 (m ≠ mm)

| 属性 | 值 |
|------|-----|
| **文件** | `include/PalletizingPlanner/CollisionCheckerSO.hpp:116,472` |
| **文件** | `ArmCollisionModel/s50_collision_matlab.h:101-102` |
| **文件** | `.github/copilot-instructions.md` 单位约定表 |
| **影响** | 文档声称 FK 输出 `mm, deg`, 但实际输出 `m, deg` |

**证据**: `testS50PalletizingSO.cpp:110` 中:
```cpp
Eigen::Vector3d pos(tcp.X*1000, tcp.Y*1000, tcp.Z*1000);  // ← 乘1000证明FK返回m
```

所有 `CollisionCheckerSO.hpp`, `s50_collision_matlab.h`, `copilot-instructions.md` 中 FK 接口单位均标注为 `mm, deg`, **但实际 `.so` 返回 `m, deg`**。这是最容易导致集成 bug 的错误源 — 任何信任文档的新代码都会产生 1000x 缩放错误。

---

### C-03: `visualize_s50_stl.py` STL 目录路径硬编码错误

| 属性 | 值 |
|------|-----|
| **文件** | `scripts/visualize_s50_stl.py:520` |
| **影响** | STL 可视化脚本永远无法找到 STL 文件 |

```python
# 当前代码 (错误路径)
stl_dir = os.path.join(project_dir, 'S50_ros2', 'S50_meshes_dir', 'S50')

# 实际STL位置
stl_dir = os.path.join(project_dir, 'ArmCollisionModel', 'model', 'meshes', 'S50')
```

脚本在找不到 STL 时静默跳过渲染, 用户只会看到空白的 3D 图, 无 STL 机器人模型。

---

### C-04: TimingStats FK 计时累计逻辑错误 → 负值显示

| 属性 | 值 |
|------|-----|
| **文件** | `include/PalletizingPlanner/CollisionCheckerSO.hpp:610-618` |
| **运行证据** | `testTrajectoryOptimality` 输出: `正运动学: -415146.50 μs/次` |

**根因**: `getTimingStats()` 将 `fkTime_us` 除以 `callCount` (碰撞检测调用次数)。但 FK 耗时通过 `forwardKinematics()` 方法独立累计 (不增加 `callCount`), 与碰撞检测调用计数无关。当 `forwardKinematics()` 被外部频繁调用 (如 IK 迭代中的数百次 FK), 大量 FK 时间累计后除以少量碰撞检测次数, 产生失真甚至异常的平均值。

```
实际输出: 正运动学: -415146.50 μs/次  ← 显然错误
```

**修复**: FK 调用应有独立计数器, 或将 FK/IK 耗时从碰撞检测计时中分离。

---

## 二、重要问题 (Major)

### M-01: Python 脚本 FK 模型与 C++ 不一致

| 文件 | 问题 |
|------|------|
| `scripts/visualize_palletizing.py:53-80` | `forward_kinematics()` 使用简化 DH, 与 `RobotModel.hpp` 不同 |
| `scripts/visualize_path.py:54-62` | `forward_kinematics_simple()` 用近似公式代替 |
| `scripts/visualize_s50_stl.py:160-168` | `fk_dh()` 使用负 a 值 DH 参数 |
| `scripts/visualize_scene.py:100-108` | `fk_s50()` 使用不同的 theta 偏移量 |

4 个 Python 脚本各自实现了不同的 FK, 且没有任何一个与 `RobotModel.hpp` 或 `.so` FK 完全一致。TCP 可视化位置与实际碰撞检测位置不匹配。

---

### M-02: Python 脚本单位混淆 (弧度 vs 角度)

| 文件 | 行号 | 问题 |
|------|------|------|
| `visualize_palletizing.py` | 105-109 | `np.radians(q_deg)` 但输入文件存储弧度 → **双重转换** |
| `visualize_s50_stl.py` | 249 | 碰撞距离 `dist*1000` 将 mm 值再乘1000 → **1000x 缩放错误** |

数据文件按照约定存储 **弧度(rad)**, 但脚本假设为度数并调用 `np.radians()`, 导致 FK 结果完全错误。

---

### M-03: Examples 使用已废弃的静态库栈

| 文件 | 问题 |
|------|------|
| `examples/basic_planning_example.cpp` | 仅使用 `PalletizingPlanner.hpp` (静态库栈), 不支持 SO 栈 |
| `examples/palletizing_example.cpp` | 依赖 `TaskSequencer + PalletizingPlanner` (纯静态库) |
| `examples/README.md` | 无 SO 依赖说明 |

两个示例均依赖 `libHRCInterface.a + libCmpAgu.a + libhansKinematics.a`, 与当前推荐的 SO 栈 (`dlopen libHRCInterface.so`) 完全脱节。无任何 SO 栈使用示例。

---

### M-04: 文档全面过时 — 缺失 SO 栈内容

| 文档 | 缺失内容 |
|------|----------|
| `docs/API.md` | 完全缺失 `CollisionCheckerSO`, `PathPlannerSO`, `CollisionGeometry` 类 |
| `docs/PROJECT_ANALYSIS.md` | 架构图仅含静态库栈, 文件数量过时 (称17个头文件, 实际19个) |
| `docs/SIMULATION_PROCESS.md` | 标题 "v3.0 / v13.0", 引用 `testS50_Palletizing_v13.m`, 最新为 v15 |
| `README.md` | 架构图和模块表仅描述静态库栈; CI badge 是静态链接非实际CI |

---

### M-05: CHANGELOG.md 严重滞后

| 属性 | 值 |
|------|-----|
| **文件** | `CHANGELOG.md` |
| **最后条目** | `[1.2.0] - 2026-02-01` |
| **缺失内容** | `CollisionCheckerSO`, `PathPlannerSO`, `CollisionGeometry`, `testTrajectoryOptimality`, IK 求解, 环境碰撞体系统, MATLAB v13-v15, S-曲线集成, 自由TCP模式 |

从 1.2.0 到当前至少应有 v1.3 ~ v2.0 的多个版本记录。

---

### M-06: MATLAB v15 硬编码绝对路径

| 属性 | 值 |
|------|-----|
| **文件** | `ArmCollisionModel/testS50_Palletizing_v15.m:32` |
| **代码** | `SO_PATH = '/home/ara/文档/collision/HansAlgorithmExport/bin/libHRCInterface.so'` |

在其他机器上必须手动修改路径才能运行 MATLAB 仿真。应使用 `getenv('HRC_LIB_PATH')` 或相对路径搜索。

---

### M-07: MATLAB v15 碰撞体可视化坐标系偏移

| 属性 | 值 |
|------|-----|
| **文件** | `ArmCollisionModel/testS50_Palletizing_v15.m:810-828` |
| **问题** | `.so` 的 `getUIInfoMationInterface` 返回 **机器人基座坐标系** 的碰撞几何, 但 STL 渲染经过了 Tbase 偏移 (Z=0.8m)。碰撞胶囊可视化和STL机器人有 0.8m 垂直偏差 |

---

### M-08: MATLAB v15 多处版本标签未更新

| 行号 | 当前标签 | 应为 |
|------|----------|------|
| 696-702 | `v4.0 碰撞统计` | `v5.0` |
| 1383-1385 | `v4.0统计` | `v5.0` |
| 1453-1457 | `C++ v4.0 Pipeline Stats` | `C++ v5.0` |
| 1549 | `v4.0 碰撞统计` | `v5.0` |
| 1676 | `drawInfoPanel_v14` | `drawInfoPanel_v15` |
| 1845 | `drawConveyor_v14` | `drawConveyor_v15` |

函数名和标签仍引用旧版本号。

---

### M-09: `PathPlanner.hpp` O(n) 线性扫描瓶颈

| 属性 | 值 |
|------|-----|
| **文件** | `include/PalletizingPlanner/PathPlanner.hpp:533-558` |
| **问题** | `findNearest()` 和 `findNear()` 均为 O(n) 线性扫描 |
| **影响** | 10000+ 节点时性能急剧下降 |

`PathPlannerOptimized.hpp` 和 `PathPlannerSO.hpp` 已使用 KD-Tree, 此文件为向后兼容保留, 但任何使用它的代码 (包括 `PalletizingPlanner.hpp` 顶层API) 都会受性能影响。

---

### M-10: `PathPlannerOptimized.hpp` propagateCostUpdate O(n²)

| 属性 | 值 |
|------|-----|
| **文件** | `include/PalletizingPlanner/PathPlannerOptimized.hpp:698-712` |
| **问题** | 遍历所有节点查找子节点 O(n), 对队列中每个节点重复 → O(n²) |
| **对比** | `PathPlannerSO.hpp` 使用 `children` 向量, O(子节点数) |

---

### M-11: IK 雅可比计算缺少 NaN 防护

| 属性 | 值 |
|------|-----|
| **文件** | `test/testS50PalletizingSO.cpp:105-130` |
| **问题** | FK 扰动可能失败 (返回 false), 但代码未检查返回值即使用 `tp` 计算雅可比列 |
| **后果** | NaN 注入 → 阻尼最小二乘解发散 → IK 静默失败 |

```cpp
// 当前代码无检查
checker.forwardKinematics(qp, tp);  // ← 可能失败
J.col(j) = ...;  // ← 使用可能无效的 tp
```

---

### M-12: `TaskSequencer.hpp` 配置与实际场景不匹配

| 属性 | 值 |
|------|-----|
| **文件** | `include/PalletizingPlanner/TaskSequencer.hpp:62-65,251-259` |
| **问题** | `PalletPattern::palletOrigin` 默认 (-1.0, 0.5, 0.1) m, 与实际码垛框架 (Y=375..1025mm) 不一致 |
| **问题** | `generateTasks()` 用 atan2 近似代替 IK, 生成非物理关节配置 |

---

### M-13: `stub_dependencies.c` ABI 不匹配风险

| 属性 | 值 |
|------|-----|
| **文件** | `lib/stub_dependencies.c` |
| **问题** | 所有桩函数声明为 `void fun(void)` 但真实签名含参数 |
| **后果** | C 调用约定下传参不匹配导致未定义行为 |

---

### M-14: `TimeParameterizationOptimized.hpp` SCurveTable 死代码

| 属性 | 值 |
|------|-----|
| **文件** | `include/PalletizingPlanner/TimeParameterizationOptimized.hpp:51-76` |
| **问题** | `scurveTable_` 在构造函数中计算但从未被任何方法引用 |
| **问题** | `computeSCurveTime()` 仅返回 `trapTime * 1.2`, 不使用物理 jerk 限制 |

---

### M-15: `README.md` 单位约定表错误

| 属性 | 值 |
|------|-----|
| **文件** | `README.md:112` |
| **问题** | HRC 碰撞距离返回值标注为 "m (米)", 未区分 SO 栈 (返回 mm) 和静态库栈 (返回 m) |

---

### M-16: 52 个 data/ 文件被 git 追踪

| 属性 | 值 |
|------|-----|
| **文件** | `.gitignore` + 52个 `data/*.txt` / `data/*.csv` |
| **问题** | `.gitignore` 规则在文件提交后才添加, 这些已追踪文件不会自动忽略 |
| **修复** | `git rm --cached data/*.txt data/*.csv` |

另有 `ArmCollisionModel/matlab.mat` (二进制大文件) 被追踪, 缺少 `*.mat` gitignore 规则。

---

### M-17: 三套路径规划器重复实现

| 文件 | 说明 |
|------|------|
| `PathPlanner.hpp` | 基础 RRT* (O(n) 最近邻) |
| `PathPlannerOptimized.hpp` | KD-Tree + 缓存优化 |
| `ParallelPathPlanner.hpp` | 预设模式 + KD-Tree (与 Optimized 大部分重复) |
| `PathPlannerSO.hpp` | ★ SO 栈 Free-TCP RRT* (推荐) |

4 套独立的 RRT* 实现, 维护成本高。建议保留 `PathPlannerSO.hpp` 为主, 其余标记 `[[deprecated]]`。

---

## 三、一般问题 (Minor)

### m-01: 编译器警告 (共计 ~50 条)

使用 `-Wall -Wextra` 编译产生以下类型警告:

| 警告类型 | 数量 | 示例文件 |
|----------|------|----------|
| `-Wunused-parameter` | ~15 | `PathOptimizer.hpp:446 (curvatureWeight)`, `PathPlannerSO.hpp:445 (result)`, `TaskSequencer.hpp:277 (robotModel)` |
| `-Wunused-variable` | ~8 | `testPalletizingPlanner.cpp:258 (elapsed)`, `testCollisionSimulation.cpp:564 (prevSegEnd)` |
| `-Wmisleading-indentation` | 1 | `testS50PalletizingSO.cpp:573` |
| `-Wformat` | 4 | `testCollisionDetectionTime.cpp:114` (`%llu` for `uint64_t*`) |
| `-Wstringop-overflow` | 4 | HRC 静态库头文件中 memset 溢出 |
| `-Wattributes` | 16 | `testRMLProbe.cpp:48-63` (function attribute 被忽略) |

---

### m-02: Python 脚本硬编码文件路径

| 文件 | 问题 |
|------|------|
| `visualize_palletizing.py:173` | 加载 `path_raw.txt` 无 `data/` 前缀 |
| `visualize_path.py:99` | `os.chdir(project_dir)` 后加载相对路径 |
| `visualize_trajectory.py:107-108` | 默认文件 `trajectory_viz.txt` 在项目中不存在 |

---

### m-03: Python 字体回退警告

| 文件 | 行号 | 问题 |
|------|------|------|
| `visualize_path.py` | 8-9 | `SimHei` 是 Windows 字体, Linux 不可用, 产生回退警告 |

---

### m-04: CMake 缺少编译警告标志

| 文件 | 问题 |
|------|------|
| `CMakeLists.txt` | 仅 `add_compile_options(-fPIC)`, 无 `-Wall -Wextra -Wpedantic` |
| `test/CMakeLists.txt` | 同上 |

建议开发期间启用警告, Release 构建可选禁用。

---

### m-05: `KDTree.hpp` 存储外部指针无所有权语义

| 属性 | 值 |
|------|-----|
| **文件** | `include/PalletizingPlanner/KDTree.hpp:97` |
| **问题** | `points_` 存储 `const std::vector<JointConfig>*` 原始指针, 无生命周期保证 |

如果外部 vector resize 导致迭代器失效, KD-Tree 访问野指针。应文档约束或改用引用。

---

### m-06: `CollisionCache.hpp` 边界不连续

| 属性 | 值 |
|------|-----|
| **文件** | `include/PalletizingPlanner/CollisionCache.hpp:246-250` |
| **问题** | `makeKey()` 使用 `floor(deg * invRes + 0.5)` (等效 round), 两个相差 0.001° 的配置可能映射到不同 key |

设计如此, 但未文档化, 可导致缓存"miss"假阳性。

---

### m-07: `PathPlannerOptimized.hpp` 并行功能被禁用

| 属性 | 值 |
|------|-----|
| **文件** | `include/PalletizingPlanner/PathPlannerOptimized.hpp:283` |
| **代码** | `useParallel = false  // 暂时禁用，避免线程安全问题` |

配置字段和线程相关代码存在但永远不启用, 属于死代码。

---

### m-08: `PathPlannerOptimized.hpp` float 精度降低

| 属性 | 值 |
|------|-----|
| **文件** | `include/PalletizingPlanner/PathPlannerOptimized.hpp:51` |
| **问题** | `float costFromStart` 代替 `double`, 路径总代价 >1000 rad 时丢失 ~3 位有效数字 |

---

### m-09: `ParallelPathPlanner.hpp` KD-Tree 周期性重建存在陈旧窗口

| 属性 | 值 |
|------|-----|
| **文件** | `include/PalletizingPlanner/ParallelPathPlanner.hpp:230-232` |
| **问题** | 每 100 次迭代重建一次, 中间 99 次最近邻搜索使用过时的树 |

---

### m-10: `testS50PalletizingSO.cpp` 大量代码重复

| 属性 | 值 |
|------|-----|
| **文件** | `test/testS50PalletizingSO.cpp:145-262` |
| **问题** | `executeP2P` 和 `executeRRTStar` 共享 ~70% 相同代码 (轨迹迭代/碰撞检查/文件输出) |

---

### m-11: `s50_collision_matlab.h` 函数名拼写错误

| 行号 | 当前拼写 | 正确拼写 |
|------|----------|----------|
| 131 | `setCPToolCollisonCapsuleShapeInterface` | `Collision` |
| 140 | `removeCPToolCollisonInterface` | `Collision` |

注: 拼写来自上游 `.so` 接口, 无法修改, 但应添加注释说明。`CollisionCheckerSO.hpp` 中的 `TRY_LOAD` 宏也使用了这个错误拼写名。

---

### m-12: `TimeParameterizationOptimized.hpp` 注释与实现不符

| 属性 | 值 |
|------|-----|
| **文件** | `include/PalletizingPlanner/TimeParameterizationOptimized.hpp:66-70` |
| **注释** | "7段S曲线的归一化形式" |
| **实际** | 五次多项式 Hermite 平滑 ($10t^3 - 15t^4 + 6t^5$), 非七段S曲线 |

---

### m-13: MATLAB v15 `VIEW_AZ/VIEW_EL` 常量未一致使用

| 属性 | 值 |
|------|-----|
| **文件** | `ArmCollisionModel/testS50_Palletizing_v15.m:87-88` |
| **问题** | 声明 `VIEW_AZ=135, VIEW_EL=25` 但多个 subplot 手动指定不同视角 `view(ax,140,30)` |

---

### m-14: `testCollisionDetectionTime.cpp` 格式说明符错误

| 属性 | 值 |
|------|-----|
| **文件** | `test/testCollisionDetectionTime.cpp:114` |
| **警告** | `%llu` 期望 `long long unsigned int*` 但实参为 `uint64_t*` |
| **修复** | 使用 `PRIu64` 或 `%zu` |

---

### m-15: 无测试框架 — 测试可靠性依赖返回值

| 问题描述 |
|----------|
| 所有测试为独立可执行文件, 通过 `return 0/1` 表示成功/失败。无断言宏、无自动测试发现、无测试结果聚合。一个测试内部多个子测试, 中间某个失败可能被后续成功覆盖。建议至少引入简单的 `ASSERT_*` 宏。|

---

## 四、信息与改进建议 (Info)

### I-01: `README.md` CI badge 为静态图片

CI-passing badge 指向静态 shields.io URL, 非真实 CI 系统。成功率 badge 标注 99.5%, 但实际 `testTrajectoryOptimality` 实现 100%。

### I-02: `test/deprecated/` 缺少弃用说明

`testS50PalletizingSO_v4_backup.cpp` 无注释说明为何弃用、被什么替代。`@file` 标签仍为 `testS50PalletizingSO.cpp`。

### I-03: `lib/` 目录混合存放真实库和桩库

`libCmpHansAlgorithmLib.so`, `libCmpHansFreeDriveMotion.so` (真实), `libCmpRMLStubs.so` (桩), `stub_dependencies.c` (桩源码) 混放, 无 README 说明依赖关系。

### I-04: `PalletizingPlanner.hpp` SceneConfig 场景参数使用米制

Scene 配置中 `obstacleHeight = 2.0 (m)`, `obstacleDepth = 0.6 (m)` 等字段使用米, 与 `testS50PalletizingSO.cpp` (使用 mm) 不一致。两套栈独立维护场景参数, 易出不同步。

### I-05: Python `visualize_trajectory.py` 默认数据文件不存在

`trajectory_viz.txt` 和 `scene_obstacles.txt` 在项目中均不存在, 脚本无法开箱运行。

### I-06: `s50_tcp_stubs.c` C/C++ 名称修饰风险

桩用 C 链接导出, 但 `.so` 中实际符号可能为 C++ mangled。仅因 `LD_PRELOAD` 拦截机制碰巧匹配。

### I-07: IK 阻尼参数 λ=5.0 固定不可调

`testS50PalletizingSO.cpp:100` 中阻尼最小二乘 λ 固定为 5.0, 接近奇异点时收敛变慢。单前场景工作正常, 但通用性不足。

### I-08: `PathOptimizerOptimized.hpp` 中 `totalStart` 变量设置后未使用

```
警告: variable 'totalStart' set but not used [-Wunused-but-set-variable]
```

---

## 五、统计汇总

| 严重级别 | 数量 | 分类 |
|----------|------|------|
| **Critical** | **4** | int16_t 溢出, FK 单位文档错误, STL 路径错误, 计时逻辑错误 |
| **Major** | **17** | Python FK不一致, 单位混淆, 文档过时, 代码重复, 场景参数不匹配 |
| **Minor** | **15** | 编译器警告, 硬编码路径, 死代码, 常量不一致 |
| **Info** | **8** | CI badge, 弃用说明, 混合库, 固定参数 |
| **合计** | **44** | — |

### 按模块分布

| 模块 | Critical | Major | Minor | Info |
|------|----------|-------|-------|------|
| C++ 核心库 (`include/`) | 2 | 5 | 5 | 1 |
| 测试代码 (`test/`) | — | 2 | 3 | 1 |
| Python 脚本 (`scripts/`) | 1 | 2 | 3 | 1 |
| MATLAB (`ArmCollisionModel/`) | — | 3 | 1 | — |
| 文档 (`docs/`, `README.md`, `CHANGELOG.md`) | — | 3 | — | 1 |
| 构建/版本控制 | 1 | 2 | 2 | 2 |
| 示例 (`examples/`) | — | 1 | 1 | 1 |
| 接口头文件 (`HRCInterface/`, `*.h`) | — | — | 1 | 1 |

---

## 六、优先修复清单

### P0 — 立即修复 (Critical)

| 编号 | 修复内容 | 工作量 |
|------|----------|--------|
| C-01 | `PathPlannerOptimized.hpp:598` → `int32_t` | 1 行 |
| C-02 | 修正 FK 单位文档 (CollisionCheckerSO.hpp, s50_collision_matlab.h, copilot-instructions.md) | 5 行 |
| C-04 | 分离 FK/IK 独立计时计数器, 或仅在碰撞调用中累计 | ~20 行 |

### P1 — 本周修复 (Major - 高影响)

| 编号 | 修复内容 | 工作量 |
|------|----------|--------|
| M-01 | 统一 Python FK 为调用 C++ 库或使用一致 DH 参数 | 2h |
| M-02 | 修正 Python 单位假设 (移除 np.radians, 修正 *1000) | 30min |
| M-06 | MATLAB SO_PATH 改用 getenv 回退 | 15min |
| M-11 | IK 雅可比 FK 返回值检查 | 15min |
| M-16 | `git rm --cached data/*.txt data/*.csv ArmCollisionModel/matlab.mat` | 5min |

### P2 — 两周内 (Major - 中影响)

| 编号 | 修复内容 | 工作量 |
|------|----------|--------|
| M-04 | 更新 API.md / SIMULATION_PROCESS.md / README.md | 2h |
| M-05 | 更新 CHANGELOG.md (补充 v1.3 ~ v2.0) | 1h |
| M-08 | 更新 MATLAB v15 版本标签 (v4.0 → v5.0) | 30min |
| M-03 | 添加 SO 栈示例程序 | 1h |
| m-04 | CMakeLists.txt 添加 `-Wall -Wextra` | 5min |

### P3 — 后续迭代 (Minor/架构)

| 编号 | 修复内容 | 工作量 |
|------|----------|--------|
| M-17 | 合并/废弃冗余规划器 (`PathPlanner`, `PathPlannerOptimized`, `ParallelPathPlanner`) | 4h |
| m-01 | 修复全部编译器警告 | 1h |
| m-10 | 重构 `executeP2P`/`executeRRTStar` 提取公共方法 | 1h |
| m-15 | 引入轻量断言宏或 catch2 测试框架 | 2h |

---

> **审计结论**: 核心 SO 栈 (CollisionCheckerSO + PathPlannerSO) 功能正确, 码垛仿真结果通过验证 (12/12 任务, 0 碰撞, 0.098s)。主要问题集中在 **(1) FK 单位文档错误** — 可能导致未来集成事故, **(2) 外围工具链 (Python/文档/示例) 严重落后于核心 C++ 代码**, **(3) 遗留代码清理不彻底** (4 套规划器, 废弃文件)。建议以 P0 修复为最高优先级, 同时系统性更新文档和辅助工具链。
