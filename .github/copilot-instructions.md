# Copilot Instructions - HRC Robot Motion Planning System

> **Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.**

## 核心业务逻辑

本系统为 HR_S50-2000 工业协作机器人实现**码垛场景最优运动规划**。核心逻辑链：

1. **碰撞检测模块** (`CollisionCheckerSO.hpp`) — 调用封装好的 `libHRCInterface.so` 碰撞检测模块（闭源，包含自碰撞检测、环境碰撞检测、FK/IK 求解）
2. **自由TCP路径规划** (`PathPlannerSO.hpp`) — Informed RRT* 算法，**纯关节空间代价**，运动过程中TCP位姿可自由变化（吸盘吸力足够大）
3. **路径优化** (`PathOptimizer.hpp`) — B-Spline 平滑 + 捷径优化，确保路径平滑且无碰撞
4. **时间参数化** — S曲线轨迹生成，实际执行使用 `libCmpRML.so`（华数上位机真实采用的S曲线库，未来直接在实际场景运行）
5. **MATLAB仿真验证** (`ArmCollisionModel/`) — 利用 URDF/STL 模型（`S50_ros2/`）进行码垛场景3D可视化 + 碰撞距离验证 + 轨迹动画

**关键设计约束**：
- 碰撞模块为 `.so` 动态库（`libHRCInterface.so`），通过 `dlopen` 运行时加载。里面包含各种碰撞检测、IK、FK 求解等完整功能
- **TCP自由变化模式 (freeTcpDuringTransit=true)**：码垛场景中吸盘吸力足够大，运动过程中TCP位姿可自由变化，仅起止点TCP被约束。这扩大了可行配置空间，产生更短路径和更强避障能力，同时消除规划过程中的FK调用开销
- **性能优化是核心目标**：通过逐层分析各部分运行时间（`TimingStats` + `PipelineTimingReport`）来定位和优化瓶颈，降低求解时间
- `libCmpRML.so` 是真实的上位机 S 曲线执行库，未来需在实际机器人上直接运行

**数据流 Pipeline**:
```
用户输入 TCP Pose (mm, deg) / JointConfig (deg)
    │
    ▼ IK 求解 (若输入为TCP位姿)
    │   └── 多起点数值IK (Damped Least-Squares, ~0.1ms/次)
    │
    ▼ PathPlannerSO.plan(start, goal)
    │   ├── Informed RRT* (纯关节空间代价, TCP自由变化)
    │   ├── IncrementalKDTree6D 最近邻加速 (O(log n))
    │   ├── CollisionCheckerSO 碰撞检测 (.so dlopen)
    │   └── 路径提取 + 重连优化 (rewiring)
    │
    ▼ PathOptimizer.optimize(rawPath)
    │   ├── 捷径优化 (碰撞验证)
    │   ├── B-Spline 拟合 (De Boor)
    │   └── 碰撞安全性校验
    │
    ▼ 时间参数化 (libCmpRML.so S曲线 / TimeParameterization 内置)
    │   ├── 关节速度/加速度/加加速度限位
    │   └── 4ms周期采样 (250 Hz, 与上位机一致)
    │
    ▼ 输出轨迹 → data/*.txt (rad, 空格分隔)
    │
    ▼ MATLAB 仿真验证 → ArmCollisionModel/pic/ (3D渲染+碰撞距离曲线)
```

## 架构概览

三层系统: **C++17 header-only 规划器** (`include/PalletizingPlanner/*.hpp`) + **HRC 碰撞库** (闭源 .so 动态库) + **MATLAB/Python 可视化**

### 主要技术栈 (SO栈 — 推荐)

```
CollisionCheckerSO.hpp   ← dlopen(libHRCInterface.so), 含FK/IK/环境碰撞
├── PathPlannerSO.hpp    ← Free-TCP Informed RRT* + IncrementalKDTree6D
├── PathOptimizer.hpp    ← B-Spline 平滑 (De Boor算法)
├── TimeParameterization ← S曲线 (内置) / libCmpRML.so (真实执行)
├── CollisionGeometry.hpp← 统一S50碰撞包络参数
├── RobotModel.hpp       ← DH正运动学 + 数值Jacobian
└── Types.hpp            ← JointConfig, Path, BSpline, PlanningResult
```

### 辅助技术栈 (静态库栈 — 向后兼容)

```
PalletizingPlanner.hpp   ← 顶层API (使用 CollisionChecker + 静态库)
├── PathPlannerOptimized ← KDTree + CollisionCache 优化 RRT*
├── HighPerformancePlanner ← 全优化流水线
├── KDTree.hpp           ← 批量构建 KD-Tree
├── CollisionCache.hpp   ← FNV-1a 哈希 + LRU 缓存
└── TaskSequencer.hpp    ← TSP 2-opt 任务序列优化
```

## ⚠️ 关键单位约定（必读 - 最常见错误源）

| 位置 | 关节角 | 坐标/距离 |
|------|--------|----------|
| 用户API `JointConfig::fromDegrees()` | **deg** | — |
| 内部存储 `config.q[i]` / 路径文件 | **rad** | — |
| HRC `.so` 接口 (update/FK/IK) | **deg** | **mm** |
| CollisionGeometry.hpp 碰撞几何 | — | **mm** |
| `SceneConfig` / `OBBObstacle.lwh` | — | **m** |
| HRC碰撞距离返回值 (SO) | — | **mm** |
| HRC碰撞距离返回值 (静态库) | — | **m** |

**API模式**: 所有公共API接受度(deg), 内部立即转换为弧度(rad)。文件输出使用rad便于数值计算。

## 项目结构

```
include/PalletizingPlanner/   ← Header-only C++17 库 (全部代码)
├── CollisionGeometry.hpp      ← ★ 统一碰撞包络参数 (CollisionChecker/SO 共享)
├── CollisionCheckerSO.hpp     ← ★ 主碰撞检测 (dlopen .so, 含FK/IK)
├── PathPlannerSO.hpp          ← ★ TCP-Aware Informed RRT*
├── Types.hpp                  ← 核心类型 (JointConfig, Path, BSpline)
├── RobotModel.hpp             ← DH正运动学 + Jacobian
├── PathOptimizer.hpp          ← B-Spline路径平滑
├── TimeParameterization.hpp   ← 内置S曲线 (五次多项式近似)
├── CollisionChecker.hpp       ← 静态库碰撞封装 (向后兼容)
├── PathPlannerOptimized.hpp   ← 优化RRT* (KDTree+Cache)
├── KDTree.hpp                 ← 批量KD-Tree
├── CollisionCache.hpp         ← FNV-1a + LRU碰撞缓存
├── HighPerformancePlanner.hpp ← 全优化流水线
├── PalletizingPlanner.hpp     ← 顶层API (静态库栈)
├── PathPlanner.hpp            ← 基础RRT* (向后兼容)
├── PathOptimizerOptimized.hpp ← 优化版B-Spline (向后兼容)
├── TimeParameterizationOptimized.hpp ← 优化版S曲线 (LUT加速)
├── ParallelPathPlanner.hpp    ← 带预设模式的RRT*
└── TaskSequencer.hpp          ← TSP 2-opt任务序列
test/                          ← 独立可执行测试 (无框架)
examples/                      ← 使用示例
scripts/                       ← Python可视化脚本
ArmCollisionModel/             ← MATLAB仿真 (@RobotCollisionModel + URDF/STL)
S50_ros2/                      ← ROS2 URDF + STL 模型 (MATLAB可引用)
lib/                           ← 预编译库 (.a 静态 + .so 存根)
HRCInterface/                  ← HRC C接口头文件
data/                          ← 运行时输出 (轨迹/路径/碰撞数据)
docs/                          ← 项目文档
```

## 构建与测试

```bash
mkdir -p build && cd build && cmake .. && make -j    # 输出到 bin/

# SO栈测试 (推荐, 需要 libHRCInterface.so)
HRC_LIB_PATH=/path/to/libHRCInterface.so ./bin/testS50PalletizingSO
HRC_LIB_PATH=/path/to/libHRCInterface.so ./bin/testS50CollisionSO
HRC_LIB_PATH=/path/to/libHRCInterface.so ./bin/testTrajectoryOptimality  # ★ 系统性6套测试

# 静态库栈测试
./bin/testPalletizingPlanner       # 综合功能 (7个子测试)
./bin/testPerformanceBenchmark     # KD-Tree/缓存 30x加速验证
./bin/testHighPerformance          # 优化前后对比
./bin/testRobustnessValidation     # 鲁棒性泛化 (99.5%成功率)
./bin/testCollisionDetectionTime   # HRC库资源消耗 (S50)

# MATLAB仿真
cd ArmCollisionModel
matlab -nodesktop -nosplash -batch "testS50_Palletizing_v15"  # 码垛仿真
matlab -nodesktop -nosplash -batch "testS50_Dynamic"          # 动态轨迹动画
```

**CMake链接**:
- 静态库栈: `libHRCInterface.a` → `libCmpAgu.a` → `libhansKinematics.a` → `stdc++` → `m` [→ `pthread`]
- SO栈: `stdc++` → `m` → `pthread` → `dl` (运行时 dlopen)

依赖: Eigen3 (系统 `/usr/include/eigen3`)。仅 Linux x86_64。

## C++ 代码约定

- **命名**: 类 PascalCase, 方法 camelCase, 成员变量 `trailing_underscore_`, 结构体字段 camelCase 无下划线
- **头文件**: `#pragma once`, Doxygen 文件头 (`@file`, `@brief`, `@date`), 中文注释
- **错误处理**: **无异常**。用返回值: `PlanningResult.status` 枚举 + `errorMessage` 字符串; `bool initialize()` 返回成功/失败
- **核心类型**: `JointVector = Eigen::Matrix<double, 6, 1>`; `Pose6D` 含 `Position3D` + `Quaterniond`
- **C/C++边界**: `.so` 通过 `dlopen`/`dlsym` 动态加载; 静态库通过 `extern "C" { #include <algorithmLibInterface.h> }`
- **碰撞参数**: 统一引用 `CollisionGeometry.hpp` 中的 `S50CollisionGeometry` 静态常量
- **数据文件**: 空格分隔纯文本, `#` 注释头标明格式和单位, 输出到 `data/`

## 性能优化指导

### 分层计时分析

SO栈内置 `TimingStats` 和 `PipelineTimingReport`，可逐层定位瓶颈：

```cpp
// 碰撞检测各阶段耗时
auto stats = checker.getTimingStats();
printf("%s", stats.toString().c_str());
// 输出: update耗时, 自碰撞耗时, 环境碰撞耗时, FK耗时, IK耗时

// 规划管线各阶段耗时
auto timing = planner.getTimingReport();
printf("  规划: %.1f ms  优化: %.1f ms  时间参数化: %.1f ms\n",
       timing.planningTime_ms, timing.optimizationTime_ms, timing.parameterizationTime_ms);
```

### 关键优化手段
- **自由TCP模式** (默认): 消除规划过程中的FK调用, 纯关节空间代价
- **KD-Tree 最近邻** (30x 加速): `IncrementalKDTree6D` / `KDTree6D`
- **碰撞缓存** (18x 加速): `CollisionCache` FNV-1a 哈希 + LRU
- **缓存行对齐**: `alignas(64)` 用于高频访问结构
- **惰性边验证**: 碰撞检测延迟到实际需要时
- **批量碰撞检测**: `isPathCollisionFree()` 单次加锁批量检查

### 性能基准

| 指标 | 实测 | 验证测试 |
|------|------|----------|
| 简单场景规划 | **0.04 ms** | testPerformanceBenchmark |
| 完整流水线 | **135 ms** | testPalletizingPlanner |
| KD-Tree 加速比 | **30x** | testPerformanceBenchmark |
| 碰撞缓存加速比 | **18x** | testPerformanceBenchmark |
| 规划成功率 | **100%** | testTrajectoryOptimality |
| 碰撞安全率 | **100%** | testTrajectoryOptimality |
| 路径一致性 (Ↄ0) | **20/20** | testTrajectoryOptimality TestF |
| TCP-to-TCP全链 | **~2.5s** | testTrajectoryOptimality TestC |
| IK求解 | **0.1 ms** | testTrajectoryOptimality TestC |

### 当前瓶颈: rewiring占规97%规划时间

分层计时显示 rewiring (findNear + 邻域父节点选择 + 重连) 占规划总时间的97%。
碰撞检测(77ms/43K次)和采样(2ms)已很快。优化方向:
- 减少 rewire 半径 或 尺对数量
- 异步/延迟 rewire
- 早期终止: 找到质量足够好的解后停止迭代

## HRC 碰撞库

### SO 动态库 (推荐)
```cpp
#include "PalletizingPlanner/CollisionCheckerSO.hpp"
palletizing::CollisionCheckerSO checker(robot);
checker.initialize();  // 自动搜索: HRC_LIB_PATH → ../lib/ → LD_LIBRARY_PATH

// 碰撞检测 + FK + IK + 环境障碍物 + 工具碰撞球
checker.isCollisionFree(config);
checker.forwardKinematics(config, pose);
checker.inverseKinematics(pose, refConfig, result);
checker.addEnvObstacleBall(id, center_mm, radius_mm);
```

### 静态库 (向后兼容)
```cpp
extern "C" { #include <algorithmLibInterface.h> }
// initACAreaConstrainPackageInterface(robType=1, dh[8], 碰撞几何...)
// updateACAreaConstrainPackageInterface(jointPos_deg, jointVel)
// checkCPSelfCollisionInterface(pair, &dist) → RTS_BOOL
```

碰撞几何: 统一参数在 `CollisionGeometry.hpp`
- 胶囊体 `[sx,sy,sz, ex,ey,ez, radius]` (7个double, mm)
- 球体 `[cx,cy,cz, radius]` (4个double, mm)

## HR_S50-2000 参数

DH(mm): `d1=296.5, d2=336.2, d3=239.0, d4=158.5, d5=158.5, d6=134.5, a2=900.0, a3=941.5`
关节限位(deg): J1:±360, J2:-190~+10, J3:±165, J4-J6:±360

## MATLAB 仿真

### 核心工作流

1. C++ 规划 (`testS50PalletizingSO`) → 输出轨迹到 `data/so_palletizing_trajectory.txt` + profile + summary
2. MATLAB 加载轨迹 → STL网格精确可视化 + `urdfFK()` 正运动学链
3. `libHRCInterface.so` 实时碰撞距离验证 → .so vs C++ 交叉检验
4. 3D 动画生成 → GIF / PNG 序列 (9张分析图 + 动画)

### STL可视化 vs 碰撞体计算架构

| 层次 | 用途 | 模型 | 精度 |
|------|------|------|------|
| **STL网格** | 3D渲染/动画 | `model/meshes/S50/elfin_*.STL` (73K面→22K降面) | 毫米级真实外形 |
| **碰撞胶囊体** | 碰撞检测 | `CollisionGeometry.hpp` (4胶囊+1球) | 简化包络, ~10μs/次 |
| **环境碰撞体** | 环境碰撞 | 胶囊/球 (电箱4+传送带3+框架4+已放箱12) | 简化包络 |

**注意**: 在某些紧凑构型（如自碰撞距离65.8mm的S6/S7场景），STL网格可能视觉上重叠，
但碰撞检测基于简化胶囊体是安全的。这是设计选择而非Bug。

### 关键文件
- **`ArmCollisionModel/testS50_Palletizing_v15.m`** — ★ 最新码垛场景仿真 (1,903行, 9图+GIF)
- **`ArmCollisionModel/@RobotCollisionModel/`** — 机器人可视化类 (未在v15使用)
- **`ArmCollisionModel/testS50_Dynamic.m`** — 动态轨迹动画
- **`S50_ros2/S50_urdf/S50/`** — URDF xacro + STL网格
- **`ArmCollisionModel/model/meshes/S50/`** — 7个STL文件 (base+link1-6)
- **`ArmCollisionModel/model/collideConfig/S50_collision.json`** — 碰撞配置

### MATLAB Headless 模式
```matlab
isHeadless = ~usejava('desktop');
if isHeadless
    set(0, 'DefaultFigureVisible', 'off');
end
```

### S曲线轨迹执行
- `libCmpRML.so` 是华数上位机真实采用的 S 曲线轨迹执行库
- MATLAB 仿真中使用此库或等价内置 S 曲线进行轨迹生成
- 未来直接在实际机器人上运行，需确保参数与上位机一致

## 码垛场景参数

### 场景布局 (mm, 机器人基座坐标系)

```
         +Y (里)
          │            ┌──────────────────────────┐
          │            │     框架 (1200×650×2000)    │ Y=375..1025
          │            │  ┌─────────┐  ┌─────────┐  │
          │            │  │BK-L     │  │BK-R     │  │ Y≈850 (紧靠框架后壁)
          │            │  │ L1→L2→L3│  │ L1→L2→L3│  │
          │            │  └─────────┘  └─────────┘  │
          │            │  ┌─────────┐  ┌─────────┐  │
          │            │  │FR-L     │  │FR-R     │  │ Y≈550
          │            │  │ L1→L2→L3│  │ L1→L2→L3│  │
          │            │  └─────────┘  └─────────┘  │
          │            └──────────────────────────┘
   ───────┼──[ROBOT]──────────────────────────── +X
          │  (0,0,800)
     ┌────┼────┐
     │ 电箱     │ Y=-325..+325
     │550×650   │
     │×800      │
     └─────────┘
       传送带 ──────────►  X=475..1025, Y=-2550..950
```

### 码垛顺序
**列优先**: 里→外(BK→FR), 左→右(L→R), 下→上(L1→L3)
每个XY位置先堆满3层再移到下一个位置:
```
BK-L: L1→L2→L3 → BK-R: L1→L2→L3 → FR-L: L1→L2→L3 → FR-R: L1→L2→L3
```

### 环境碰撞体

| ID范围 | 物体 | 类型 | 半径(mm) |
|--------|------|------|----------|
| 10-13 | 电箱4条边 | 胶囊 | 80 |
| 15-17 | 传送带(左右侧+皮带) | 胶囊 | 80/275 |
| 30-33 | 框架4根立柱 | 胶囊 | 50 |
| 34-45 | 已放置箱子(动态) | 球 | 250 |
| 6 | 搬运工具球(动态) | 球 | 225 |

### 码垛仿真性能

| 指标 | 数值 |
|------|------|
| IK求解 (12位置) | **0.6 ms** |
| 路径规划 (86段) | **0.0 ms** (全P2P直达) |
| 时间参数化 | **1.9 ms** |
| 碰撞检测 (5363次) | **68.4 ms** (9.97μs/次) |
| 全链路计算总耗时 | **0.098 s** |
| S曲线执行时间 (12箱) | **209.6 s** |
| 自碰撞 | **0** |
| 环境碰撞 | **0** |
| 最小自碰撞距离 | **10.5 mm** |

## 测试编写模式

无测试框架。每个测试是独立可执行文件:
```cpp
int main() {
    try {
        testFoo();  // void testXxx() 命名
    } catch (const std::exception& e) {
        std::cerr << "异常: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
```
新增测试需在 `test/CMakeLists.txt` 添加 `add_executable` + 链接配置。

## 常见陷阱

### 单位转换
```cpp
// ❌ 错误: 直接使用度数
config.q[0] = 90;  // 内部期望弧度!

// ✅ 正确
JointConfig config = JointConfig::fromDegrees({90, -50, 70, 0, 80, 0});
```

### SO 库路径
```bash
# 设置环境变量
export HRC_LIB_PATH=/path/to/libHRCInterface.so
# 或放到 lib/ 目录下
cp libHRCInterface.so /path/to/project/lib/
```

### CMake链接错误
静态库严格顺序: `libHRCInterface.a` → `libCmpAgu.a` → `libhansKinematics.a` → `stdc++` → `m` [→ `pthread`]

### 碰撞检测初始化
SO栈自动配置正确的S50参数 (来自 `CollisionGeometry.hpp`)。只需:
```cpp
CollisionCheckerSO checker(robot);
checker.initialize();  // 默认参数从 S50CollisionGeometry 加载
```

## 关键文件索引

| 文件 | 说明 |
|------|------|
| `include/PalletizingPlanner/CollisionGeometry.hpp` | ★ 统一S50碰撞包络参数 (新增) |
| `include/PalletizingPlanner/CollisionCheckerSO.hpp` | ★ SO碰撞检测 (推荐, 含FK/IK) |
| `include/PalletizingPlanner/PathPlannerSO.hpp` | ★ Free-TCP RRT* (含IncrementalKDTree6D) |
| `include/PalletizingPlanner/Types.hpp` | 核心类型定义 |
| `include/PalletizingPlanner/PalletizingPlanner.hpp` | 顶层API (静态库栈) |
| `include/PalletizingPlanner/CollisionChecker.hpp` | 静态库碰撞封装 |
| `test/testS50PalletizingSO.cpp` | ★ 码垛场景仿真 (IK+环境碰撞+动态箱子) |
| `test/testTrajectoryOptimality.cpp` | ★ 系统性轨迹最优性测试 (6套测试) |
| `ArmCollisionModel/testS50_Palletizing_v15.m` | ★ 最新MATLAB码垛仿真 |
| `docs/PROJECT_STRUCTURE_ANALYSIS.md` | 项目结构分析报告 |
