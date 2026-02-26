# 🤖 HRC 协作机器人运动规划系统

<p align="center">
  <a href="https://www.huayan-robotics.com">
    <img src="https://www.huayan-robotics.com/media/upload/index/Huayan%20Elfin%20index%20banner.jpg" alt="Huayan Robotics" width="600">
  </a>
</p>

<p align="center">
  <a href="LICENSE"><img src="https://img.shields.io/badge/License-MIT-green.svg" alt="License"></a>
  <a href="https://isocpp.org/"><img src="https://img.shields.io/badge/C++-17-blue.svg" alt="C++"></a>
  <a href="https://www.linux.org/"><img src="https://img.shields.io/badge/Platform-Linux%20x86__64-lightgrey.svg" alt="Platform"></a>
  <a href="https://www.huayan-robotics.com"><img src="https://img.shields.io/badge/Huayan-Robotics-orange.svg" alt="Huayan"></a>
  <a href="#性能指标"><img src="https://img.shields.io/badge/Pipeline-98ms-blueviolet.svg" alt="Pipeline"></a>
  <a href="#性能指标"><img src="https://img.shields.io/badge/Success_Rate-100%25-success.svg" alt="Success Rate"></a>
</p>

<p align="center">
  <b>HR_S50-2000 工业协作机器人码垛场景最优运动规划系统</b><br>
  专为码垛场景设计的高性能路径规划与轨迹生成解决方案<br>
  <code>C++17 Header-Only</code> · <code>9,600+ 行核心代码</code> · <code>18 个模块</code> · <code>98ms 端到端</code>
</p>

<p align="center">
  <a href="https://www.huayan-robotics.com">🌐 官网</a> •
  <a href="mailto:yuesj@huayan-robotics.com">📧 联系我们</a> •
  <a href="docs/API.md">📖 API 文档</a> •
  <a href="examples/">💡 示例</a> •
  <a href="CHANGELOG.md">📋 变更日志</a>
</p>

---

## 📋 目录

- [项目概述](#项目概述)
- [⚠️ 关键单位约定](#️-关键单位约定)
- [系统架构](#系统架构)
  - [双栈架构](#双栈架构)
  - [SO栈 (推荐)](#so栈-推荐)
  - [静态库栈 (向后兼容)](#静态库栈-向后兼容)
  - [数据流管线](#数据流管线)
- [核心特性](#核心特性)
- [性能指标](#性能指标)
- [快速开始](#快速开始)
- [API 文档](#api-文档)
- [碰撞检测架构](#碰撞检测架构)
- [配置与调优](#配置与调优)
- [测试验证](#测试验证)
- [技术规格](#技术规格)
- [目录结构](#目录结构)
- [MATLAB 仿真验证](#matlab-仿真验证)
- [Python 可视化工具](#python-可视化工具)
- [数据文件格式规范](#数据文件格式规范)
- [开发指南](#开发指南)
- [调试与故障排除](#调试与故障排除)
- [版本历史](#版本历史)
- [许可证](#许可证)

---

## 项目概述

本系统为 **HR_S50-2000 工业协作机器人**实现**码垛场景最优运动规划**。采用三层架构：

### 1. C++17 Header-Only 运动规划器 ⭐

18 个头文件、9,600+ 行纯头文件实现，零编译依赖，用户仅需 `#include` 即可使用。

**双栈设计**：
- **SO 动态库栈 (推荐)** — `dlopen(libHRCInterface.so)` 运行时加载，含 FK/IK/环境碰撞
- **静态库栈 (向后兼容)** — `extern "C"` 链接 `.a` 静态库

### 2. HRC 碰撞检测库 (闭源)

人机协作 (Human-Robot Collaboration) 场景实时碰撞检测。
- **`libHRCInterface.so`** (SO栈): 动态库，运行时加载，含完整 FK/IK/碰撞检测
- **`libHRCInterface.a` + `libCmpAgu.a` + `libhansKinematics.a`** (静态库栈): 预编译静态库

### 3. MATLAB / Python 可视化模块

- **MATLAB**: URDF/STL 精确 3D 渲染 + `libHRCInterface.so` 碰撞距离交叉验证 (v15.0)
- **Python**: matplotlib / PyVista 可视化，含 STL 网格渲染、轨迹动画

---

## ⚠️ 关键单位约定

> **这是开发中最常见的错误源，请务必仔细阅读！**

| 位置 | 关节角 | 坐标/距离 |
|------|--------|----------|
| 用户 API `JointConfig::fromDegrees()` | **deg** | — |
| 内部存储 `config.q[i]` / 路径文件 | **rad** | — |
| HRC `.so` 接口 (update/FK/IK) | **deg** | **mm** |
| HRC `.so` FK 输出位置 | — | **m** (⚠️ 非 mm) |
| HRC `.so` FK 输出姿态 | **deg** | — |
| CollisionGeometry.hpp 碰撞几何 | — | **mm** |
| `SceneConfig` / `OBBObstacle.lwh` | — | **m** |
| HRC 碰撞距离返回值 (SO) | — | **mm** |
| HRC 碰撞距离返回值 (静态库) | — | **m** |
| `.hard` 硬件配置 | **deg** | **mm** |

```cpp
// ❌ 错误: 直接使用度数（内部期望弧度）
JointConfig config;
config.q[0] = 90;  // BUG! 内部存储的是弧度

// ✅ 正确: 使用工厂方法
JointConfig config = JointConfig::fromDegrees({90, -50, 70, 0, 80, 0});
```

**核心原则**: 所有公共 API 接受度 (deg)，内部立即转换为弧度 (rad)，文件输出使用 rad。

---

## 系统架构

### 双栈架构

本项目提供两套技术栈，**推荐使用 SO 栈**：

```
┌─────────────────────────────────────────────────────────────────────────┐
│                  ★ SO 动态库栈 (推荐)                                     │
│                                                                         │
│  CollisionCheckerSO.hpp   ← dlopen(libHRCInterface.so), 含FK/IK/环境碰撞 │
│  ├── PathPlannerSO.hpp    ← Free-TCP Informed RRT* + IncrementalKDTree6D │
│  ├── PathOptimizer.hpp    ← B-Spline 平滑 (De Boor算法)                  │
│  ├── TimeParameterization ← S曲线 (内置) / libCmpRML.so (真实执行)        │
│  ├── CollisionGeometry.hpp← 统一S50碰撞包络参数                           │
│  ├── RobotModel.hpp       ← DH正运动学 + 数值Jacobian                    │
│  └── Types.hpp            ← JointConfig, Path, BSpline, PlanningResult   │
├─────────────────────────────────────────────────────────────────────────┤
│                  静态库栈 (向后兼容)                                       │
│                                                                         │
│  PalletizingPlanner.hpp   ← 顶层API (使用 CollisionChecker + 静态库)     │
│  ├── PathPlannerOptimized ← KDTree + CollisionCache 优化 RRT*            │
│  ├── HighPerformancePlanner ← 全优化流水线                                │
│  ├── KDTree.hpp           ← 批量构建 KD-Tree                             │
│  ├── CollisionCache.hpp   ← FNV-1a 哈希 + LRU 缓存                      │
│  └── TaskSequencer.hpp    ← TSP 2-opt 任务序列优化                       │
└─────────────────────────────────────────────────────────────────────────┘
```

### SO栈 (推荐)

**核心创新 — 自由TCP模式**：码垛场景中吸盘吸力足够大，运动过程中 TCP 位姿可自由变化 (`freeTcpDuringTransit=true`)，仅起止点 TCP 被约束。这：
- 扩大了可行配置空间，产生更短路径
- 增强避障能力
- 消除规划过程中的 FK 调用开销（纯关节空间代价）

```
CollisionCheckerSO (726行)    ← dlopen libHRCInterface.so, 含TimingStats
├── PathPlannerSO (913行)     ← Free-TCP Informed RRT* + IncrementalKDTree6D
├── PathOptimizer (460行)     ← B-Spline 路径平滑
├── TimeParameterization      ← S曲线 (内置430行) / libCmpRML.so
├── CollisionGeometry (70行)  ← 统一S50碰撞包络
├── RobotModel (329行)        ← DH正运动学 + Jacobian
└── Types (487行)             ← 核心数据类型
```

### 静态库栈 (向后兼容)

```
PalletizingPlanner (418行)    ← 顶层 Facade API
├── PathPlannerOptimized (862行) ← alignas(64) + 椭球采样 + 懒惰碰撞
├── PathOptimizerOptimized (649行) ← 自适应捷径 + 增量B-Spline
├── TimeParameterizationOptimized (480行) ← S曲线查表加速
├── CollisionChecker (901行)  ← extern "C" 三层碰撞检测
├── CollisionCache (342行)    ← FNV-1a + LRU 缓存
├── KDTree (293行)            ← 6D批量KD-Tree
├── HighPerformancePlanner (532行) ← 全优化流水线
├── ParallelPathPlanner (558行) ← v3.0 超高性能
└── TaskSequencer (457行)     ← TSP 2-opt
```

### 数据流管线

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

---

## 核心特性

### 🚀 自由TCP运动规划 (SO栈核心)

- **Free-TCP Informed RRT\*** — 纯关节空间代价函数，运动过程中TCP位姿可自由变化
- **IncrementalKDTree6D** — 增量式6D KD-Tree，O(log n) 最近邻
- **数值IK求解** — 多起点 Damped Least-Squares，~0.1ms/次
- **动态环境碰撞体** — 运行时添加/移除球体和胶囊体（电箱、传送带、框架、已放箱子）

### 🛡️ 碰撞检测 (双模式)

**SO栈**: `dlopen` 加载 `libHRCInterface.so`
- 自碰撞检测 (连杆对胶囊体/球体距离)
- 环境碰撞体 (动态添加球体/胶囊体)
- 工具碰撞球 (搬运工具保护)
- 碰撞距离返回值: **mm**

**静态库栈**: `extern "C"` 链接 `.a` 库
- 三层: 连杆自碰撞 + 虚拟墙 + TCP安全区域
- FNV-1a + LRU 碰撞缓存 (18x 加速)
- 碰撞距离返回值: **m**

### 📈 路径优化

- **自适应随机捷径** — 碰撞验证下的快速路径缩短
- **5次 B-Spline 平滑** — De Boor 算法保证 C³ 连续（加速度连续）
- **曲率分析** — 确保路径可执行性

### ⏱️ 时间参数化

- **libCmpRML.so** — 华数上位机真实 S 曲线执行库，未来直接在实际机器人上运行
- **内置 S 曲线** — 五次多项式近似七段式 + 查表加速
- 4ms 采样周期 (250 Hz, 与上位机一致)

### 🔧 分层性能分析

```cpp
// 碰撞检测各阶段耗时
auto stats = checker.getTimingStats();
printf("%s", stats.toString().c_str());
// 输出: update耗时, 自碰撞耗时, 环境碰撞耗时, FK耗时(独立计数), IK耗时(独立计数)

// 规划管线各阶段耗时
auto timing = planner.getTimingReport();
printf("  规划: %.1f ms  优化: %.1f ms  时间参数化: %.1f ms\n",
       timing.planningTime_ms, timing.optimizationTime_ms, timing.parameterizationTime_ms);
```

---

## 性能指标

### SO栈性能 (码垛场景)

| 指标 | 实测 | 验证测试 |
|------|------|----------|
| IK 求解 (12位置) | **0.6 ms** | testTrajectoryOptimality |
| 碰撞检测 (5363次) | **68.4 ms** (9.97 μs/次) | testS50PalletizingSO |
| FK 单次耗时 | **0.54 μs** | testTrajectoryOptimality |
| 码垛全链路计算 (12箱) | **0.098 s** | testS50PalletizingSO |
| S曲线执行总时间 (12箱) | **209.6 s** | testS50PalletizingSO |
| 规划成功率 | **100%** | testTrajectoryOptimality |
| 碰撞安全率 | **100%** | testTrajectoryOptimality |
| 最小自碰撞距离 | **10.5 mm** | testS50PalletizingSO |
| 路径一致性 (20次重复) | **100%** | testTrajectoryOptimality |

### 静态库栈性能

| 指标 | 实测 | 验证测试 |
|------|------|----------|
| 简单场景规划 | **0.04 ms** | testPerformanceBenchmark |
| 完整流水线 (规划+优化+参数化) | **~135 ms** | testPalletizingPlanner |
| KD-Tree 加速比 | **30x** | testPerformanceBenchmark |
| 碰撞缓存加速比 | **18x** | testPerformanceBenchmark |
| 规划成功率 | **99.5%** | testRobustnessValidation |

### 当前瓶颈

分层计时显示 rewiring (findNear + 邻域父节点选择 + 重连) 占规划总时间的 97%。碰撞检测 (77ms/43K次) 和采样 (2ms) 已很快。优化方向：
- 减少 rewire 半径或邻域数量
- 异步/延迟 rewire
- 早期终止：找到质量足够好的解后停止迭代

---

## 快速开始

### 环境要求

| 依赖 | 最低版本 | 说明 |
|------|----------|------|
| **操作系统** | Linux x86_64 | 预编译库仅支持此平台 |
| **GCC** | 7+ | 需支持 C++17 |
| **CMake** | 3.14+ | 项目构建系统 |
| **Eigen3** | 3.3+ | 线性代数库 (`/usr/include/eigen3`) |
| **libHRCInterface.so** | — | SO栈碰撞检测 (设置 `HRC_LIB_PATH` 环境变量) |

### 构建步骤

```bash
# 1. 克隆仓库
git clone https://github.com/huayan-robotics/HRC-PalletizingPlanner.git
cd HRC-PalletizingPlanner

# 2. 安装 Eigen3
sudo apt-get install libeigen3-dev

# 3. 构建
mkdir -p build && cd build
cmake ..
make -j$(nproc)

# 4. 可执行文件输出到 bin/
ls ../bin/
```

### 运行测试

```bash
# ─── SO栈测试 (推荐, 需 libHRCInterface.so) ───────────────────
export HRC_LIB_PATH=/path/to/libHRCInterface.so

# 码垛完整仿真 (12箱, IK+环境碰撞+动态障碍物)
./bin/testS50PalletizingSO

# 碰撞检测验证 (7场景)
./bin/testS50CollisionSO

# ★ 系统性轨迹最优性测试 (6套测试)
./bin/testTrajectoryOptimality

# ─── 静态库栈测试 ────────────────────────────────────────────
# 综合功能测试 (7个子测试)
./bin/testPalletizingPlanner

# 性能基准测试 (KD-Tree 30x / 缓存 18x 加速验证)
./bin/testPerformanceBenchmark

# 鲁棒性验证测试 (1000+ 随机配置)
./bin/testRobustnessValidation
```

### SO栈最小示例

```cpp
#include "PalletizingPlanner/CollisionCheckerSO.hpp"
#include "PalletizingPlanner/PathPlannerSO.hpp"
using namespace palletizing;

int main() {
    // 创建碰撞检测器 (自动搜索: HRC_LIB_PATH → ../lib/ → LD_LIBRARY_PATH)
    RobotModel robot;
    CollisionCheckerSO checker(robot);
    if (!checker.initialize()) return -1;

    // 创建规划器
    PathPlannerSO planner(checker, robot);

    // IK 求解: TCP位姿 → 关节角
    Pose6D targetPose;  // 设置目标位姿...
    JointConfig refConfig = JointConfig::fromDegrees({0, -90, 30, 0, -60, 0});
    std::vector<double> ikResult;
    if (checker.inverseKinematics(targetPose, refConfig, ikResult)) {
        // IK 成功
    }

    // 规划: 关节空间 P2P
    JointConfig start = JointConfig::fromDegrees({0, -90, 30, 0, -60, 0});
    JointConfig goal  = JointConfig::fromDegrees({45, -60, 45, 30, -45, 45});
    auto result = planner.plan(start, goal);

    if (result.status == PlanningStatus::Success) {
        printf("路径长度: %.3f rad, 规划时间: %.1f ms\n",
               result.pathLength, result.planningTime_ms);
    }
    return 0;
}
```

### 静态库栈最小示例

```cpp
#include "PalletizingPlanner/PalletizingPlanner.hpp"
using namespace palletizing;

int main() {
    PalletizingPlanner planner;
    if (!planner.initialize()) return -1;

    JointConfig start = JointConfig::fromDegrees({0, -90, 30, 0, -60, 0});
    JointConfig goal  = JointConfig::fromDegrees({45, -60, 45, 30, -45, 45});
    PlanningResult result = planner.planPointToPoint(start, goal);

    if (result.isSuccess()) {
        printf("路径长度: %.3f rad, 规划时间: %.3f s\n",
               result.pathLength, result.planningTime);
    }
    return 0;
}
```

---

## API 文档

### SO栈核心接口

#### CollisionCheckerSO

```cpp
class CollisionCheckerSO {
public:
    explicit CollisionCheckerSO(const RobotModel& robot);
    bool initialize();  // 自动搜索 .so: HRC_LIB_PATH → ../lib/ → LD_LIBRARY_PATH

    // 碰撞检测
    bool isCollisionFree(const JointConfig& config);
    bool isPathCollisionFree(const Path& path);  // 批量检查, 单次加锁

    // 正/逆运动学 (位置: m, 姿态: deg)
    bool forwardKinematics(const JointConfig& config, Pose6D& pose);
    bool inverseKinematics(const Pose6D& pose, const JointConfig& ref,
                           std::vector<double>& result);

    // 环境碰撞体 (动态添加/移除)
    void addEnvObstacleBall(int id, const Eigen::Vector3d& center_mm, double radius_mm);
    void addEnvObstacleCapsule(int id, const Eigen::Vector3d& start_mm,
                               const Eigen::Vector3d& end_mm, double radius_mm);
    void removeEnvObstacle(int id);

    // 工具碰撞球
    void setToolBall(int id, const Eigen::Vector3d& offset_mm, double radius_mm);

    // 性能分析
    TimingStats getTimingStats() const;  // FK/IK/碰撞独立计时
};
```

#### PathPlannerSO

```cpp
class PathPlannerSO {
public:
    PathPlannerSO(CollisionCheckerSO& checker, const RobotModel& robot);

    PlanningResult plan(const JointConfig& start, const JointConfig& goal);

    // 配置
    void setMaxIterations(int n);
    void setStepSize(double rad);
    void setFreeTcpDuringTransit(bool enable);  // 默认 true

    // 性能报告
    PipelineTimingReport getTimingReport() const;
};
```

### 静态库栈接口

#### PalletizingPlanner (Facade)

```cpp
class PalletizingPlanner {
public:
    explicit PalletizingPlanner(const RobotDHParams& = RobotDHParams::fromHRConfig());
    bool initialize(const SceneConfig& = SceneConfig::defaultPalletizing());

    PlanningResult planPointToPoint(const JointConfig& start, const JointConfig& goal);
    PalletizingResult planPickAndPlace(const PalletizingTask& task);
    std::vector<PalletizingResult> planTaskSequence(
        const std::vector<PalletizingTask>& tasks,
        const JointConfig& homeConfig,
        ProgressCallback callback = nullptr);

    void setConfig(const PlannerConfig& config);
    bool isConfigValid(const JointConfig& config) const;
    void savePathToFile(const Path& path, const std::string& filename) const;
};
```

### 核心数据类型

| 类型 | 说明 | 关键方法 |
|------|------|----------|
| `JointConfig` | 6DOF 关节配置 (内部 rad) | `fromDegrees()`, `toDegrees()`, `distanceTo()`, `interpolate()` |
| `JointVector` | `Eigen::Matrix<double, 6, 1>` | 内部计算用 |
| `Pose6D` | 6D 位姿 (Position3D + Quaternion) | SLERP 插值, 欧拉角构造 |
| `Path` | `Waypoint` 序列 | `totalLength()`, `updatePathParameters()` |
| `BSpline` | B-Spline 曲线 | `evaluate(t)` — De Boor 算法 |
| `PlanningResult` | 规划输出 | `isSuccess()`, `pathLength`, `planningTime` |
| `TimingStats` | 碰撞检测计时 | FK/IK 独立计数器 |
| `PipelineTimingReport` | 管线计时 | 各阶段耗时分解 |

---

## 碰撞检测架构

### SO栈碰撞检测 (推荐)

```
CollisionCheckerSO
├── dlopen("libHRCInterface.so")       ← 运行时加载
├── 初始化: S50CollisionGeometry 自动配置碰撞参数
├── 自碰撞检测 (4胶囊体 + 1球体)
├── 环境碰撞体 (动态 ID 系统)
│   ├── ID 10-13: 电箱4条边 (胶囊, R=80mm)
│   ├── ID 15-17: 传送带 (胶囊, R=80/275mm)
│   ├── ID 30-33: 框架立柱 (胶囊, R=50mm)
│   ├── ID 34-45: 已放置箱子 (球, R=250mm) — 动态更新
│   └── ID 6:     搬运工具 (球, R=225mm) — 动态更新
├── FK/IK: forwardKinematics / inverseKinematics
└── TimingStats: FK/IK/碰撞 各自独立计时
```

碰撞几何参数统一定义在 `CollisionGeometry.hpp`：

```cpp
namespace S50CollisionGeometry {
    // 胶囊体: [sx, sy, sz, ex, ey, ez, radius] (7值, mm)
    // 球体:   [cx, cy, cz, radius] (4值, mm)
    static constexpr double capsule1[] = {...};
    // ...
}
```

### 静态库栈碰撞检测

```
CollisionChecker
├── extern "C" { #include <algorithmLibInterface.h> }
├── initACAreaConstrainPackageInterface(robType=1, dh, geom)
├── setCPSelfColliderLinkModelOpenStateInterface(true)
├── updateACAreaConstrainPackageInterface(jointPos_deg, vel)
├── 第一层: 连杆-连杆自碰撞
├── 第二层: 连杆-虚拟墙环境碰撞
└── 第三层: TCP-安全区域入侵检测
```

### SO库路径搜索

```bash
# 设置环境变量 (推荐)
export HRC_LIB_PATH=/path/to/libHRCInterface.so

# 或放到项目 lib/ 目录
cp libHRCInterface.so /path/to/project/lib/

# 自动搜索顺序: HRC_LIB_PATH → ../lib/ → LD_LIBRARY_PATH
```

---

## 配置与调优

### SO栈配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `freeTcpDuringTransit` | `true` | 自由TCP模式 (码垛场景推荐) |
| `maxIterations` | 10,000 | RRT* 最大迭代 |
| `stepSize` | 0.2 rad | 采样步长 |
| `goalBias` | 0.10 | 目标偏向 |
| `rewireRadius` | 1.0 rad | 重连半径 |

### 静态库栈配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `plannerType` | `InformedRRTStar` | 算法选择 |
| `useLazyCollision` | `true` | 懒惰碰撞检测 |
| `useCache` | `true` | FNV-1a + LRU 缓存 |
| `useKDTree` | `true` | KD-Tree 加速 |

### S曲线参数 (libCmpRML.so)

| 参数 | J1-J3 | J4-J6 | 单位 |
|------|-------|-------|------|
| 最大速度 | 120 | 180 | °/s |
| 最大加速度 | 121 | 121 | °/s² |
| 最大 Jerk | 860 | 860 | °/s³ |
| 采样周期 | 4 | 4 | ms |

---

## 测试验证

### SO栈测试 (需 `HRC_LIB_PATH`)

| 测试程序 | 说明 | 结果 |
|----------|------|------|
| `testS50PalletizingSO` | 12箱码垛完整仿真 (IK+环境碰撞+动态障碍物) | 12/12 ✅, 0碰撞 |
| `testS50CollisionSO` | 7场景碰撞检测 (安全/碰撞/极限) | 7/7 ✅ |
| `testTrajectoryOptimality` | ★ 系统性测试 (运动学/P2P/TCP全链/环境碰撞/安全距离/重复性) | 6/6 ✅, 20/20 重复 |

### 静态库栈测试

| 测试程序 | 说明 | 结果 |
|----------|------|------|
| `testPalletizingPlanner` | 综合功能 (7子测试: 运动学/碰撞/RRT*/优化/S曲线/BIT*/码垛) | 7/7 ✅ |
| `testPerformanceBenchmark` | KD-Tree 30x / 缓存 18x 加速验证 | ✅ |
| `testRobustnessValidation` | 1000+ 随机配置, 99.5% 成功率 | ✅ |
| `testHighPerformance` | 优化前后对比, 全流水线 | ✅ |
| `testCollisionDetectionTime` | HRC库 CPU/内存/堆栈资源分析 | ✅ |

### 测试编写规范

无测试框架，每个测试是独立可执行文件：

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

新增测试需在 `test/CMakeLists.txt` 添加：
- **SO栈**: 链接 `stdc++ m pthread dl` (运行时 dlopen)
- **静态库栈**: 严格顺序 `libHRCInterface.a → libCmpAgu.a → libhansKinematics.a → stdc++ → m [→ pthread]`

---

## 技术规格

### HR_S50-2000 参数

| 参数 | 值 | 单位 |
|------|-----|------|
| 自由度 | 6 | DOF |
| 最大负载 | 50 | kg |
| 臂展 | 2000 | mm |
| 重复定位精度 | ±0.05 | mm |

### DH 参数 (mm)

| d1 | d2 | d3 | d4 | d5 | d6 | a2 | a3 |
|---|---|---|---|---|---|---|---|
| 296.5 | 336.2 | 239.0 | 158.5 | 158.5 | 134.5 | 900.0 | 941.5 |

### 关节限位 (deg)

| | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| Min | -360 | -190 | -165 | -360 | -360 | -360 |
| Max | +360 | +10 | +165 | +360 | +360 | +360 |
| 最大速度 (°/s) | 120 | 120 | 120 | 180 | 180 | 180 |

---

## 目录结构

```
HRC-PalletizingPlanner/
├── CMakeLists.txt                  # CMake 3.14+, C++17, -Wall -Wextra
├── README.md
├── CHANGELOG.md
├── CONTRIBUTING.md
├── LICENSE                         # MIT
├── HR_S50-2000.hard                # 完整硬件配置
│
├── include/PalletizingPlanner/     # ★ Header-Only C++17 (18文件, ~9,600行)
│   │
│   │  ── SO栈 (推荐) ──
│   ├── CollisionCheckerSO.hpp      #  726行 — dlopen碰撞检测 (FK/IK/环境碰撞/TimingStats)
│   ├── PathPlannerSO.hpp           #  913行 — Free-TCP Informed RRT* + IncrementalKDTree6D
│   ├── CollisionGeometry.hpp       #   70行 — 统一S50碰撞包络参数
│   │
│   │  ── 共享模块 ──
│   ├── Types.hpp                   #  487行 — 核心类型 (JointConfig/Path/BSpline/PlanningResult)
│   ├── RobotModel.hpp              #  329行 — DH正运动学/Jacobian/关节限位
│   ├── PathOptimizer.hpp           #  460行 — B-Spline路径平滑 (De Boor)
│   ├── TimeParameterization.hpp    #  430行 — 内置S曲线
│   │
│   │  ── 静态库栈 (向后兼容) ──
│   ├── PalletizingPlanner.hpp      #  418行 — 顶层Facade API
│   ├── CollisionChecker.hpp        #  901行 — extern "C" 三层碰撞检测
│   ├── PathPlannerOptimized.hpp    #  862行 — KDTree+缓存优化RRT*
│   ├── PathOptimizerOptimized.hpp  #  649行 — 增量B-Spline
│   ├── TimeParameterizationOptimized.hpp  # 480行 — S曲线查表加速
│   ├── CollisionCache.hpp          #  342行 — FNV-1a + LRU
│   ├── KDTree.hpp                  #  293行 — 6D批量KD-Tree
│   ├── HighPerformancePlanner.hpp  #  532行 — 全优化流水线
│   ├── ParallelPathPlanner.hpp     #  558行 — v3.0超高性能
│   ├── PathPlanner.hpp             #  702行 — 基础RRT*
│   └── TaskSequencer.hpp           #  457行 — TSP 2-opt
│
├── HRCInterface/                   # HRC C接口头文件
│   ├── algorithmLibInterface.h
│   ├── InterfaceDataStruct.h
│   └── stack_utils.h
│
├── lib/                            # 预编译库 (.a 静态库)
│   ├── libHRCInterface.a
│   ├── libCmpAgu.a
│   └── libhansKinematics.a
│
├── test/                           # 测试程序 (13个)
│   ├── CMakeLists.txt
│   ├── testS50PalletizingSO.cpp    # ★ SO栈码垛仿真 (12箱)
│   ├── testS50CollisionSO.cpp      # ★ SO栈碰撞检测 (7场景)
│   ├── testTrajectoryOptimality.cpp # ★ 系统性轨迹测试 (6套)
│   ├── testPalletizingPlanner.cpp   # 静态库综合测试
│   ├── testPerformanceBenchmark.cpp
│   ├── testRobustnessValidation.cpp
│   ├── testHighPerformance.cpp
│   ├── testCollisionDetectionTime.cpp
│   ├── testCollisionSimulation.cpp
│   ├── testS50PalletizingRML.cpp
│   ├── testS50CollisionRML.cpp
│   ├── testPalletizingScenarioRML.cpp
│   └── testRMLProbe.cpp
│
├── examples/                       # 使用示例
│   ├── basic_planning_example.cpp
│   └── palletizing_example.cpp
│
├── scripts/                        # Python 可视化 (5个)
│   ├── visualize_s50_stl.py        #  589行 — STL网格3D渲染 + 碰撞距离曲线
│   ├── visualize_scene.py          #  674行 — 完整码垛工作站渲染
│   ├── visualize_palletizing.py    #  296行 — 码垛路径动画 (FuncAnimation)
│   ├── visualize_path.py           #  203行 — 路径对比 (原始 vs B-Spline)
│   └── visualize_trajectory.py     #  159行 — 带障碍物轨迹可视化
│
├── ArmCollisionModel/              # MATLAB 仿真模块
│   ├── testS50_Palletizing_v15.m   # ★ 码垛工作站仿真 v15.0 (~1,900行)
│   ├── testS50_Dynamic.m           #   动态轨迹动画
│   ├── testS50.m                   #   静态碰撞测试
│   ├── s50_collision_matlab.h      #   .so MEX接口
│   ├── @RobotCollisionModel/       #   机器人可视化类
│   ├── model/meshes/S50/           #   7个STL文件 (base+link1-6)
│   └── pic/                        #   输出图像
│
├── S50_ros2/                       # ROS2 URDF + STL 模型
├── robotModel/                     # 独立运动学模型
├── docs/                           # 文档
├── data/                           # 运行时数据输出 (gitignored)
├── build/                          # 构建中间产物
└── bin/                            # 可执行文件输出
```

---

## MATLAB 仿真验证

### 核心工作流

1. C++ 规划 → 输出轨迹到 `data/so_palletizing_trajectory.txt`
2. MATLAB 加载轨迹 → STL 网格精确可视化 + `urdfFK()` 正运动学链
3. `libHRCInterface.so` 实时碰撞距离验证 → .so vs C++ 交叉检验
4. 3D 动画生成 → GIF / PNG 序列

### 码垛仿真 v15.0

**`testS50_Palletizing_v15.m`** (~1,900行) 实现完整码垛工作站仿真：

#### 场景布局 (mm, 机器人基座坐标系)

```
         +Y (里)
          │            ┌──────────────────────────┐
          │            │     框架 (1200×650×2000)    │
          │            │  ┌─────────┐  ┌─────────┐  │
          │            │  │BK-L     │  │BK-R     │  │
          │            │  │ L1→L2→L3│  │ L1→L2→L3│  │
          │            │  └─────────┘  └─────────┘  │
          │            │  ┌─────────┐  ┌─────────┐  │
          │            │  │FR-L     │  │FR-R     │  │
          │            │  │ L1→L2→L3│  │ L1→L2→L3│  │
          │            │  └─────────┘  └─────────┘  │
          │            └──────────────────────────┘
   ───────┼──[ROBOT]──────────────────────────── +X
          │  (0,0,800)
     ┌────┼────┐
     │ 电箱     │
     └─────────┘
       传送带 ──────────►
```

#### 码垛顺序
**列优先**: 里→外(BK→FR), 左→右(L→R), 下→上(L1→L3)

#### STL可视化 vs 碰撞体

| 层次 | 用途 | 模型 |
|------|------|------|
| STL网格 | 3D渲染/动画 | `model/meshes/S50/` (73K面→22K降面) |
| 碰撞胶囊体 | 碰撞检测 | `CollisionGeometry.hpp` (4胶囊+1球) |
| 环境碰撞体 | 环境碰撞 | 胶囊/球 (电箱+传送带+框架+已放箱) |

### SO_PATH 配置

MATLAB v15 自动搜索 `libHRCInterface.so`：

```matlab
% 搜索顺序: HRC_LIB_PATH环境变量 → 项目lib/ → 硬编码路径
SO_PATH = getenv('HRC_LIB_PATH');
if isempty(SO_PATH)
    projLib = fullfile(fileparts(mfilename('fullpath')), '..', 'lib', 'libHRCInterface.so');
    if exist(projLib, 'file'), SO_PATH = projLib; end
end
```

### 运行方式

```bash
export HRC_LIB_PATH=/path/to/libHRCInterface.so
cd ArmCollisionModel
matlab -nodesktop -nosplash -batch "testS50_Palletizing_v15"
```

---

## Python 可视化工具

### 脚本一览

| 脚本 | 行数 | 功能 |
|------|------|------|
| `visualize_s50_stl.py` | 589 | STL网格3D渲染 + RML碰撞距离曲线 |
| `visualize_scene.py` | 674 | 完整码垛工作站渲染 (碰撞模型) |
| `visualize_palletizing.py` | 296 | 码垛路径 FuncAnimation 动画 |
| `visualize_path.py` | 203 | 路径对比 (原始 vs B-Spline) |
| `visualize_trajectory.py` | 159 | 带障碍物轨迹可视化 |

### 安装

```bash
pip install numpy matplotlib
pip install pyvista  # 可选: 高质量渲染
```

### DH 参数同步

每个 Python 脚本独立定义 DH 参数和 `fk_s50()` 函数。修改 DH 参数时需同步：
- `RobotModel.hpp`
- `scripts/*.py`
- `model/collideConfig/*.json`

---

## 数据文件格式规范

| 格式 | 列数 | 单位 | 用途 |
|------|------|------|------|
| 路径文件 `.txt` | 6 | rad | 关节空间路径 |
| 轨迹文件 `.txt` | 13 | rad + rad/s | 时间+位置+速度 |
| 仿真 CSV | 19 | deg + deg/s + deg/s² | 250Hz 采样轨迹 |
| 碰撞 profile CSV | 14+ | deg, mm | 碰撞距离曲线 |

通用约定：空格分隔 (`.txt`) 或逗号 (`.csv`)，`#` 注释头标明格式和单位，输出到 `data/`。

---

## 开发指南

### C++ 代码规范

| 规则 | 示例 |
|------|------|
| 命名空间 | `palletizing` |
| 类名 | PascalCase: `PathPlannerSO` |
| 方法名 | camelCase: `isCollisionFree()` |
| 成员变量 | `trailing_underscore_`: `robot_`, `timing_` |
| 结构体字段 | camelCase 无下划线: `maxIterations` |
| 头文件 | `#pragma once`, Doxygen (`@file`, `@brief`) |
| 错误处理 | **无异常** — 返回值 (`PlanningResult.status` + `errorMessage`) |
| 注释 | 中文注释 |

### CMake 链接配置

```cmake
# SO栈 (运行时 dlopen)
target_link_libraries(myTest stdc++ m pthread dl)

# 静态库栈 (严格顺序!)
target_link_libraries(myTest
  libHRCInterface.a libCmpAgu.a libhansKinematics.a stdc++ m pthread)
```

---

## 调试与故障排除

### 常见问题

| 问题 | 原因 | 解决 |
|------|------|------|
| 单位错误 | `config.q[i]=90` (应为rad) | 使用 `JointConfig::fromDegrees()` |
| SO加载失败 | 找不到 .so | 设置 `HRC_LIB_PATH` 环境变量 |
| 静态库链接错误 | 链接顺序不对 | 按 `HRC→Agu→Kin→stdc++→m` 顺序 |
| FK输出单位 | 位置是 m 不是 mm | 见单位约定表 |
| "---1---" 输出 | HRC库内部调试信息 | `2>/dev/null` 抑制 |
| MATLAB找不到URDF | 工作目录错误 | `cd ArmCollisionModel && addpath(genpath('.'))` |

### 性能调试

```bash
# SO栈: 使用 TimingStats + PipelineTimingReport 分层分析
HRC_LIB_PATH=/path/to/so ./bin/testTrajectoryOptimality

# 静态库栈: 基准 → 对比 → 资源分析
./bin/testPerformanceBenchmark
./bin/testHighPerformance
./bin/testCollisionDetectionTime
```

---

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| 3.0.0 | 2026-02-12 | ★ SO动态库架构、自由TCP模式、IK求解器、环境碰撞体、代码审计修复 |
| 2.1.0 | 2026-02-09 | 代码质量升级: mt19937、int32_t、锁优化 |
| 2.0.0 | 2026-02-05 | 码垛工作站仿真 v7.0 |
| 1.4.0 | 2026-02-04 | Python 3D 场景可视化 |
| 1.3.1 | 2026-01-30 | 动态碰撞动画、无头模式 |
| 1.3.0 | 2026-01-30 | MATLAB 碰撞可视化模块 |
| 1.2.0 | 2026-02-01 | 项目结构优化、CI/CD |
| 1.1.0 | 2026-01-29 | KD-Tree 30x + 缓存 18x 加速 |
| 1.0.0 | 2026-01-29 | 初始版本 |

---

## 依赖说明

### 外部依赖

| 依赖 | 版本 | 必需 | 安装 | 用途 |
|------|------|------|------|------|
| Eigen3 | 3.3+ | ✅ | `apt install libeigen3-dev` | 线性代数/四元数 |
| pthread | — | 可选 | 系统自带 | 多线程规划 |

### MATLAB 依赖

| 依赖 | 版本 | 用途 |
|------|------|------|
| MATLAB | R2019b+ | 碰撞模型可视化 |

### Python 依赖

```bash
pip install numpy matplotlib         # 必需
pip install pyvista                   # 可选: 高质量 3D 渲染
```

---

## 许可证

本项目采用 [MIT 许可证](LICENSE)。

HRC 碰撞检测库 (`libHRCInterface.so`/`.a`, `libCmpAgu.a`, `libhansKinematics.a`) 为广东华沿机器人有限公司专有软件。

---

## 联系方式

| 渠道 | 联系方式 |
|------|----------|
| 📧 技术支持 | yuesj@huayan-robotics.com |
| 📧 商务合作 | marketing@huayan-robotics.com |
| 📞 咨询热线 | 400-852-9898 |
| 🌐 官方网站 | https://www.huayan-robotics.com |
| 🐛 Issues | [GitHub Issues](https://github.com/huayan-robotics/HRC-PalletizingPlanner/issues) |

---

<p align="center">
  <b>广东华沿机器人有限公司 | Guangdong Huayan Robotics Co., Ltd.</b><br>
  <i>用机器人技术为人类服务</i>
</p>
