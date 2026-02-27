# Copilot Instructions - HRC Robot Motion Planning System

> **Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.**

## 项目目标

本项目为 HR_S50-2000 工业协作机器人实现**码垛场景避障运动规划**。最终交付物是一个 `.so` 动态库模块，输入预定义碰撞环境 + 机器人模型数据，输出无碰撞轨迹。MATLAB仿真仅用作验证/可视化工具。

## ⚠️ 必读核心约束

### 1. TCP必须保持水平 (constrainTcpHorizontal)

运动过程中TCP（吸盘）必须保持水平朝下，仅允许绕Z轴旋转。重物吸附在平面吸盘上，若TCP倾斜则重物脱落。

- `constrainTcpHorizontal = true` (默认)
- 每个新采样点通过FK检查TCP Z轴是否接近 (0,0,-1)
- 容差: `orientTolerance_deg = 30.0`
- 吸盘可能不居中抓取箱子，偏移量影响碰撞建模

### 2. FK/IK 仅通过 SO 库

所有正/逆运动学计算必须且只能通过 `CollisionCheckerSO` (即 `libHRCInterface.so`) 完成。`RobotModel.hpp` 仅提供参数查询和关节空间工具函数（限位检查、随机采样、插值），不包含任何FK/IK实现。

### 3. 无魔法数字

所有常量必须来自参数文件或 `RobotDHParams` 结构体，不允许硬编码。自定义数值必须有明确来源和注释。

### 4. 单位约定（最常见错误源）

| 位置 | 关节角 | 坐标/距离 |
|------|--------|-----------|
| 用户API `JointConfig::fromDegrees()` | **deg** | — |
| 内部存储 `config.q[i]` / 路径文件 | **rad** | — |
| HRC `.so` 接口 (update/FK/IK) | **deg** | **mm** |
| **FK2 (`forwardKinematics2`) 输出** | — | **m** (不是mm!) |
| **FK2 TCP朝向输出 (A,B,C)** | **deg** | — |
| CollisionGeometry.hpp 碰撞几何 | — | **mm** |
| `getUIInfoMationInterface` 输出 | — | **mm** |
| `SceneConfig` / `OBBObstacle.lwh` | — | **m** |
| HRC碰撞距离返回值 (SO) | — | **mm** |
| MATLAB `fk2Skeleton()` 返回值 | — | **m** |
| C++ 轨迹文件 TCP坐标 | **deg** | **mm** |
| MATLAB 世界坐标系 (3D渲染) | — | **m** |

## 数据流 Pipeline

```
用户输入 TCP Pose (mm, deg) / JointConfig (deg)
    │
    ▼ IK 求解 → CollisionCheckerSO.inverseKinematics() / NumericalIK
    │
    ▼ PathPlannerSO.plan(start, goal)
    │   ├── Informed RRT* (关节空间代价 + TCP水平约束)
    │   ├── IncrementalKDTree6D 最近邻 O(log n)
    │   ├── CollisionCheckerSO 碰撞检测 (.so dlopen)
    │   ├── FK检查TCP水平 → 拒绝不满足约束的配置
    │   └── 路径提取 + rewiring
    │
    ▼ PathOptimizer.optimize(rawPath)
    │   ├── 捷径优化 (碰撞验证)
    │   ├── B-Spline 拟合 (De Boor)
    │   └── 碰撞安全性校验
    │
    ▼ 时间参数化 (libCmpRML.so S曲线, 华数上位机真实执行库)
    │   ├── 关节速度/加速度/加加速度限位
    │   └── 4ms周期采样 (250 Hz)
    │
    ▼ 输出轨迹 → data/*.txt (rad, 空格分隔)
    │
    ▼ MATLAB 仿真验证 (仅验证/可视化, 非核心)
```

## 项目结构

```
include/PalletizingPlanner/   ← Header-only C++17 库 (全部代码)
├── CollisionCheckerSO.hpp     ← ★ 碰撞检测 (dlopen .so, 含FK/IK, 线程安全)
├── PathPlannerSO.hpp          ← ★ TCP水平约束 Informed RRT* + KDTree6D
├── PathOptimizer.hpp          ← B-Spline路径平滑
├── Types.hpp                  ← 核心类型 (JointConfig, Path, BSpline, Config)
├── CollisionGeometry.hpp      ← 统一S50碰撞包络参数
├── RobotModel.hpp             ← 机器人参数 + 关节空间工具 (无FK/IK!)
├── NumericalIK.hpp            ← DLS数值IK (使用SO FK)
└── TimeParameterization.hpp   ← 内置S曲线 (开发用, 生产用libCmpRML.so)
test/
├── testS50CollisionSO.cpp     ← SO碰撞 + S曲线 + 计时
├── testS50PalletizingSO.cpp   ← ★ 完整码垛仿真
└── testTrajectoryOptimality.cpp ← ★ 6套系统性测试 (A-F)
examples/
└── basic_planning_example_so.cpp
ArmCollisionModel/             ← MATLAB仿真 (验证/可视化)
├── testS50_Palletizing_v15.m  ← ★ 最新码垛仿真 (~2083行, 9图+GIF)
├── testS50_Dynamic.m          ← 动态轨迹动画
├── testS50_Quick.m            ← 快速验证
├── diagnostic_so_frames.m     ← SO库诊断 (FK2/getUIInfo对比)
├── s50_collision_matlab.h     ← MATLAB用SO库C头文件
├── s50_tcp_stubs.c            ← TCP桩函数源码
├── dlopen_global.c            ← MEX加载器源码 (RTLD_GLOBAL)
├── dlopen_global.mexa64       ← 编译好的MEX函数
├── model/meshes/S50/          ← 7个STL文件
└── pic/                       ← 输出图片
scripts/                       ← Python可视化
├── visualize_palletizing.py   ← 码垛路径3D可视化
├── visualize_path.py          ← 路径 + B-Spline效果
├── visualize_s50_stl.py       ← STL网格 + 碰撞分析
├── visualize_scene.py         ← 码垛工作站3D场景
└── visualize_trajectory.py    ← 轨迹动画
lib/                           ← 预编译库
├── libHRCInterface.so         ← ★ 闭源碰撞/FK/IK库
├── libCmpRML.so               ← ★ S曲线执行库 (上位机真实库)
├── libHRCInterface.a          ← 静态库版本
├── libCmpAgu.a                ← 辅助算法库
└── libhansKinematics.a        ← 运动学库
HRCInterface/
└── algorithmLibInterface.h    ← SO库完整C接口声明
S50_ros2/                      ← ROS2 URDF + STL模型
data/                          ← C++运行时输出
docs/                          ← 项目文档
```

## 构建

```bash
mkdir -p build && cd build && cmake .. && make -j

# 运行测试
HRC_LIB_PATH=/path/to/libHRCInterface.so ./bin/testS50PalletizingSO
HRC_LIB_PATH=/path/to/libHRCInterface.so ./bin/testTrajectoryOptimality
```

CMake >= 3.14, C++17, 仅Linux x86_64。SO栈链接: `stdc++` -> `m` -> `pthread` -> `dl`。依赖: Eigen3。

## 三阶段路径规划

每个箱子搬运分三个阶段，碰撞配置不同：

### 阶段1: HOME -> 传送带 (无箱子)

- 碰撞环境: 电箱 + 传送带 + 框架 + 已放置箱子
- TCP无负载, 不启用工具碰撞体
- 自碰撞检测正常

### 阶段2: 搬运箱子到托盘 (★核心优化阶段)

- TCP吸附箱子: **箱子作为TCP碰撞体延伸** (球体近似, toolIdx=6)
- 箱子面积远大于吸盘面积, 碰撞建模必须考虑箱子外形
- TCP必须保持水平 (`constrainTcpHorizontal=true`)
- 碰撞环境: 电箱 + 传送带 + 框架 + 已放置箱子
- Z轴旋转后箱子也跟着旋转 -> 自碰撞需要考虑旋转后的箱子与机械臂碰撞

### 阶段3: 返回HOME (无箱子)

- 释放箱子后, 移除工具碰撞体 (toolIdx=6)
- 将刚放置的箱子添加为环境障碍 (球体近似)
- 碰撞环境: 电箱 + 传送带 + 框架 + 已放置箱子 (含刚放置的)

### 运动段定义 (segment)

| seg | 动作 | 阶段 | 工具碰撞 |
|-----|------|------|----------|
| 0 | HOME->PickApproach | 1 | 无 |
| 1 | PickApproach->Pick | 1 | 无 |
| 2 | Pick->PickApproach | 2 | 启用工具球 |
| 3 | PickApproach->PlaceApproach | 2 | 启用工具球 |
| 4 | PlaceApproach->Place | 2 | 启用工具球 |
| 5 | Place->PlaceApproach | 3 | 移除工具球+添加已放置球 |
| 6 | PlaceApproach->HOME | 3 | 无 |

## 环境碰撞模型

### 框架 (Frame) — 4根近端立柱 + 2根顶梁 + 2块侧挡板

框架顶部是**开放**的（不是封闭的）。碰撞建模：
- 4根近端垂直立柱 (靠近机器人侧, 胶囊体, r=50mm)
- 2根顶梁 (平行于传送带方向, 胶囊体)
- 2块侧挡板 (框架两侧, 胶囊体近似)
- 远端立柱/横梁不在机械臂工作范围内, 可忽略

### 箱子碰撞

- **搬运中**: 箱子作为TCP延伸, 用工具碰撞球 (toolIdx=6, r=225mm) 近似
- **已放置**: 每放一箱添加环境障碍球 (envId=46+i, r=250mm)
- 吸盘不一定居中抓取, 偏移影响球心位置

### 电箱 (Cabinet) — 4条顶部边

| ID | 物体 | 类型 | 半径(mm) |
|----|------|------|----------|
| 10-13 | 电箱顶部4条边 | 胶囊 | 80 |

### 传送带 (Conveyor) — 顶部边

| ID | 物体 | 类型 | 半径(mm) |
|----|------|------|----------|
| 15-17 | 传送带(左右侧+皮带) | 胶囊 | 80/275 |

### 框架立柱

| ID | 物体 | 类型 | 半径(mm) |
|----|------|------|----------|
| 1-4 | 4根近端立柱 | 胶囊 | 50 |

### 动态碰撞体

> **⚠️ envId 30-45 是 SO 库内部保留范围，注册会失败 (返回1)**。有效范围: 1-29 和 46+。

| ID | 物体 | 类型 | 半径(mm) | 说明 |
|----|------|------|----------|------|
| 46-57 | 已放置箱子 | 球 | 250 | 每放一箱添加一个 |
| 6 | 搬运工具球 | 球 | 225 | 搬运中启用, 到达后移除 |

## C++ 核心 API

### CollisionCheckerSO (唯一碰撞/FK/IK接口)

```cpp
CollisionCheckerSO checker(robot);
checker.initialize();  // dlopen libHRCInterface.so

// 碰撞检测
checker.isCollisionFree(config);
checker.getCollisionReport(config, computeTcp);
checker.getCollisionDistanceMm(config);
checker.isPathCollisionFree(start, end, resolution);

// FK / IK (唯一正确的FK/IK来源)
checker.forwardKinematics(config, pose);  // 输出: 位置m, 朝向deg
checker.inverseKinematics(pose, refConfig, result);

// 环境障碍物 (全部mm单位)
checker.addEnvObstacleBall(id, center_mm, radius_mm);
checker.addEnvObstacleCapsule(id, start_mm, end_mm, radius_mm);
checker.removeEnvObstacle(id);

// 工具碰撞体
checker.setToolBall(toolIdx, offset_mm, radius_mm);
checker.setToolCapsule(toolIdx, start_mm, end_mm, radius_mm);
checker.removeTool(toolIdx);

// 安全裕度
checker.setSafetyMarginMm(10.0);
```

### CollisionCheckerSO 完整API

| 方法 | 参数 | 返回 | 说明 |
|------|------|------|------|
| `initialize(soPath?)` | 可选路径 | `bool` | 加载SO+初始化+设置碰撞几何 |
| `isCollisionFree(config)` | JointConfig(rad) | `bool` | 自碰撞+环境碰撞 |
| `getCollisionReport(config, computeTcp?)` | JointConfig, bool | `CollisionReport` | 距离/碰撞对/TCP |
| `getCollisionDistanceMm(config)` | JointConfig | `double` | 最小自碰撞距离(mm) |
| `isPathCollisionFree(start, end, res?)` | 2xJointConfig, double | `bool` | 路径批量检测 |
| `forwardKinematics(config, &pose)` | JointConfig, Pose6D& | `bool` | FK2: 位置m, 朝向deg |
| `inverseKinematics(pose, ref, &result)` | Pose6D, JointConfig, JointConfig& | `bool` | IK |
| `addEnvObstacleBall(id, center, r)` | int, Position3D(mm), double(mm) | `bool` | 球形环境障碍 |
| `addEnvObstacleCapsule(id, s, e, r)` | int, 2xPos(mm), double(mm) | `bool` | 胶囊环境障碍 |
| `removeEnvObstacle(id)` | int | `bool` | 移除环境障碍 |
| `setToolBall(idx, offset, r)` | int(6/7), Pos(mm), double(mm) | `bool` | 工具碰撞球 |
| `setToolCapsule(idx, s, e, r)` | int, 2xPos(mm), double(mm) | `bool` | 工具碰撞胶囊 |
| `removeTool(idx)` | int | `bool` | 移除工具碰撞体 |
| `setSafetyMarginMm(margin)` | double | `void` | 自碰撞裕度(mm) |
| `setEnvSafetyMarginMm(margin)` | double | `void` | 环境碰撞裕度(mm) |
| `getTimingStats()` | -- | `TimingStats` | 各阶段微秒计时 |

### CollisionReport 结构

```cpp
struct CollisionReport {
    bool selfCollision;           // 是否自碰撞
    bool envCollision;            // 是否环境碰撞
    double selfDistanceMm;        // 自碰撞距离(mm)
    double envDistanceMm;         // 环境碰撞距离(mm)
    std::pair<int,int> selfPair;  // 自碰撞对 (连杆index)
    std::pair<int,int> envPair;   // 环境碰撞对 (连杆,障碍ID)
    Pose6D tcpPose;               // TCP位姿 (仅computeTcp=true)
};
```

### TimingStats 结构

```cpp
struct TimingStats {
    uint64_t update_us, selfCollision_us, envCollision_us, fk_us, ik_us;
    uint32_t updateCalls, selfCollCalls, envCollCalls, fkCalls, ikCalls;
    std::string toString() const;
};
```

### PathPlannerSO

```cpp
RobotModel robot;
CollisionCheckerSO checker(robot);
checker.initialize();

TCPPlannerConfig cfg;
cfg.constrainTcpHorizontal = true;  // 默认: TCP保持水平
cfg.orientTolerance_deg = 30.0;     // 水平容差

PathPlannerSO planner(robot, checker, cfg);
PlanningResult result = planner.plan(start, goal);
PipelineTimingReport timing = planner.getTimingReport();
```

### TCPPlannerConfig 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `constrainTcpHorizontal` | `true` | TCP保持水平 (码垛必须) |
| `desiredTcpAxis` | `(0,0,-1)` | TCP Z轴期望方向 (朝下) |
| `orientTolerance_deg` | `30.0` | 朝向容差 (度) |
| `tcpPoseWeight` | `0.0` | TCP代价权重 (0=纯关节空间) |
| `maxTcpDeviation_mm` | `500.0` | 最大TCP偏移容差(mm) |

### PlannerConfig 基础参数

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| `stepSize` | `0.1` | rad | RRT扩展步长 |
| `goalBias` | `0.15` | 概率 | 目标采样偏置 |
| `rewireRadius` | `0.5` | rad | 重连半径上限 |
| `maxIterations` | `50000` | -- | 最大迭代次数 |
| `maxPlanningTime` | `10.0` | s | 最大规划时间 |
| `useInformedSampling` | `true` | -- | Informed RRT*椭球采样 |
| `informedThreshold` | `1.5` | 比率 | 椭球激活阈值 |
| `collisionResolution` | `0.02` | rad | 碰撞检测插值分辨率 |
| `shortcutIterations` | `200` | -- | 捷径优化迭代 |
| `smoothIterations` | `100` | -- | 平滑迭代 |
| `splineDegree` | `5` | -- | B-Spline阶数 |
| `splineResolution` | `100` | -- | B-Spline采样点数 |

### RobotModel (仅参数和工具函数, 无FK/IK!)

```cpp
RobotModel robot;
robot.isWithinLimits(config);
robot.clampToLimits(config);
robot.randomConfig();
robot.interpolate(start, end, t);
robot.getParams();  // DH参数, 关节限位, 速度限制
```

## 核心类型 (Types.hpp)

| 类型 | 定义 | 说明 |
|------|------|------|
| `JointVector` | `Eigen::Matrix<double, 6, 1>` | 6轴关节向量 |
| `JointConfig` | `{JointVector q}` (内部rad) | `fromDegrees()`, `toDegrees()`, `distanceTo()`, `interpolate()` |
| `Pose6D` | `{Position3D, Orientation}` | `fromEulerZYX()`, `toMatrix()`, `interpolate()` (SLERP) |
| `Path` | `{vector<Waypoint>}` | `totalLength()`, `updatePathParameters()` |
| `Waypoint` | `{JointConfig, optional<Pose6D>, double pathParam}` | 路径点 |
| `BSpline` | degree-3 clamped uniform | `initUniform()`, `evaluate()` (De Boor), `derivative()` |
| `PlanningStatus` | enum | Success, Failure, Timeout, InvalidStart/Goal, CollisionAtStart/Goal, NoPath |
| `PlanningResult` | struct | status, rawPath, optimizedPath, smoothedSpline, timings, errorMessage |
| `PlannerType` | enum | RRT, RRTStar, InformedRRTStar (默认), BITStar |
| `OptimizerType` | enum | Shortcut, BSplineSmooth, Hybrid (默认) |

## PipelineTimingReport

| 字段 | 说明 |
|------|------|
| `collisionInit_ms` | 碰撞模块初始化 |
| `planningTotal_ms` | 规划总时间 |
| `sampling_ms` | RRT采样时间 |
| `nearestSearch_ms` | KD-Tree近邻搜索 |
| `collisionCheck_ms` | 碰撞检测时间 |
| `tcpCostEval_ms` | TCP约束检查 + FK时间 |
| `rewiring_ms` | 重连优化时间 |
| `optimizationTotal_ms` | 路径优化总时间 |
| `shortcut_ms` | 捷径优化 |
| `bspline_ms` | B-Spline拟合 |
| `validation_ms` | 碰撞验证 |
| `parameterization_ms` | 时间参数化 |
| `totalPipeline_ms` | 端到端总时间 |
| `planIterations` | 迭代次数 |
| `nodesExplored` | 树节点数 |
| `collisionChecks` | 碰撞检测调用次数 |
| `fkCalls` | FK调用次数 |

## HRC SO 底层 C 接口

完整声明: `HRCInterface/algorithmLibInterface.h`

| 函数 | 参数 | 返回 | 单位 |
|------|------|------|------|
| `initACAreaConstrainPackageInterface` | `robType, dh[8], baseGeo, lowerGeo, elbowGeo, upperGeo, wristGeo, initJoint` | void | mm,deg |
| `updateACAreaConstrainPackageInterface` | `joint[6], vel[6], acc[6]` | void | deg |
| `checkCPSelfCollisionInterface` | `pair[2], *dist` | SO_INT(1=碰撞) | mm |
| `forwardKinematics2` | `joint[6], *tcp` | void | **位置m**, 朝向deg |
| `inverseKinematics` | `*targetTcp, ref[6], *result` | SO_INT | m, deg |
| `getUIInfoMationInterface` | `idx[7], type[7], data[63], radius[7]` | void | mm |
| `setCPToolCollisionBallShapeInterface` | `toolIdx, offset[3], radius` | SO_INT | mm |
| `setCPToolCollisonCapsuleShapeInterface` | `toolIdx, start[3], end[3], radius` | SO_INT | mm |
| `removeCPToolCollisonInterface` | `toolIdx` | SO_INT | -- |
| `addEnvObstacleBallInterface` | `envId, center[3], radius` | SO_INT | mm |
| `addEnvObstacleCapsuleInterface` | `envId, start[3], end[3], radius` | SO_INT | mm |
| `removeEnvObstacleInterface` | `envId` | SO_INT | -- |
| `setCPSelfColliderLinkModelOpenStateInterface` | `flags[3]` | void | -- |
| `setLinkEnvCollisionEnabledInterface` | `flags[7]` | void | -- |

### MC_COORD_REF 结构体

```c
typedef struct {
    SO_LREAL X, Y, Z, A, B, C;  // 位置(m) + 朝向(deg)
} MC_COORD_REF;
```

### DH数组顺序 (注意非标准)

```
dh[8] = [d1, d2, d3, d4, d5, d6, a2, a3]
       = [296.5, 336.2, 239, 158.5, 158.5, 134.5, 900, 941.5]
```

> **注意**: MATLAB仿真中使用 `[d1, d2, a2, d3, a3, d4, d5, d6]` 顺序,
> 但C接口 `algorithmLibInterface.h` 文档明确定义 S系列机器人为
> `dh = {d1, d2, d3, d4, d5, d6, a2, a3}`。C++代码遵循此接口定义。

### 类型映射

| C类型 | SO类型 | MATLAB类型 |
|-------|--------|------------|
| `double` | `SO_LREAL` | `double` |
| `short` | `SO_INT` | `int16` |
| `int` | `SO_DINT` | `int32` |
| `long long` | `SO_LINT` | `int64` |
| `signed char` | `SO_BOOL` | `int8` |

## HR_S50-2000 机器人参数

### DH参数 (mm)

```
d1=296.5  d2=336.2  d3=239.0  d4=158.5  d5=158.5  d6=134.5
a2=900.0  a3=941.5
```

### 关节限位 (deg)

| 关节 | 下限 | 上限 |
|------|------|------|
| J1 | -360 | +360 |
| J2 | -190 | +10 |
| J3 | -165 | +165 |
| J4 | -360 | +360 |
| J5 | -360 | +360 |
| J6 | -360 | +360 |

### HOME位 (deg)

`[0, -90, 0, 0, 90, 0]` -- 手臂水平伸出 (+X方向), TCP Z朝下

### 碰撞几何 (CollisionGeometry.hpp, mm)

| 碰撞体 | 几何值 | 半径(mm) |
|---------|--------|----------|
| 基座胶囊 | `{0,0,20, 0,0,330, 160}` | 160 |
| 大臂胶囊 | `{0,0,340, 900,0,340, 140}` | 140 |
| 肘部胶囊 | `{-10,0,60, 941.5,0,60, 120}` | 120 |
| 小臂胶囊 | `{0,0,-50, 0,0,100, 100}` | 100 |
| 腕部球 | `{0,0,20, 140}` | 140 |
| 安全裕度(自碰撞) | `10.0` | -- |
| 安全裕度(环境) | `5.0` | -- |

## 四套FK实现与约定差异（关键陷阱）

系统中存在四套FK实现，约定**互不兼容**：

| FK实现 | ZERO构型TCP位置 | 可信度 |
|--------|----------------|--------|
| FK2 (`forwardKinematics2`) | (0, 0, 1175)mm -- 竖直 | 唯一正确 |
| `getUIInfoMation` | 不同约定 | 仅碰撞体 |
| C++ DH FK (已移除) | 偏差~2284mm | X |
| Python DH FK | 偏差~2984mm | X |

### FK2 经验骨架模型

通过对 SO 库 FK2 输出系统性圆拟合导出的精确骨架:

```
H=220mm (肩高), L1=380mm (大臂), L2=420mm (小臂), L3=155mm (腕->TCP)

arm_dir = [cos(q1), sin(q1), 0]
a2 = -q2,  a23 = -q2+q3,  a235 = -q2+q3+q5

Base     = [0, 0, 0]
Shoulder = [0, 0, H]
Elbow    = Shoulder + L1 * [sin(a2)*arm_dir, cos(a2)]
Wrist    = Elbow    + L2 * [sin(a23)*arm_dir, cos(a23)]
TCP      = Wrist    + L3 * [sin(a235)*arm_dir, cos(a235)]
```

- J4/J6 只影响TCP**朝向**，不影响TCP**位置**
- q2符号取反 (`a2 = -q2`)
- H=220mm != d1=296.5mm -- FK2内部使用非标准DH约定
- 此模型**仅用于MATLAB可视化**, 所有C++路径规划通过SO FK

### FK使用规则

1. C++ 所有FK/IK -> `CollisionCheckerSO.forwardKinematics()`
2. MATLAB TCP位置 -> `fk2Skeleton()` (FK2骨架模型)
3. MATLAB STL渲染 -> `urdfFK()` (URDF关节驱动)
4. `getUIInfoMation` -> 仅碰撞体几何, 不用于TCP
5. **切勿混用**不同FK的坐标

## 码垛场景布局 (机器人基座坐标系)

```
         +Y (里)
          |            +------------------------------+
          |            |     框架 (1200x650x2000mm)      | Y=375..1025
          |            |  +---------+  +---------+    |
          |            |  |BK-L     |  |BK-R     |    | Y~850
          |            |  +---------+  +---------+    |
          |            |  +---------+  +---------+    |
          |            |  |FR-L     |  |FR-R     |    | Y~550
          |            |  +---------+  +---------+    |
          |            +------------------------------+
   -------+--[ROBOT]------------------------------- +X
          |  (0,0,800mm)
     +----+----+
     | 电箱     |
     |550x650mm|
     +---------+
       传送带 -------->  X=475..1025, Y=-2550..950
```

### 场景元素

| 元素 | 尺寸 | 说明 |
|------|------|------|
| 电箱 | 550x650x800 mm | 基座一侧, -X方向 |
| 框架 | 1200x650x2000 mm, 管径30mm | +Y方向 |
| 托盘 | 1000x600x500 mm | 框架内底部 |
| 传送带 | 3500x550 mm, 高750mm | -Y方向 |
| 箱子 | 350x280x250 mm | 传送带->框架 |
| 基座高度 | 800 mm | 电箱顶部 |

码垛顺序 (列优先): BK-L(L1->L3) -> BK-R(L1->L3) -> FR-L(L1->L3) -> FR-R(L1->L3)

## C++ 代码约定

- **命名**: 类 PascalCase, 方法 camelCase, 成员变量 `trailing_underscore_`, 结构体字段 camelCase
- **头文件**: `#pragma once`, Doxygen (`@file`, `@brief`, `@date`), 中文注释
- **错误处理**: **无异常**。`PlanningResult.status` 枚举 + `errorMessage`; `bool initialize()`
- **线程安全**: `CollisionCheckerSO` 全部公共方法 `std::mutex`
- **C/C++边界**: `dlopen`/`dlsym` 动态加载
- **碰撞参数**: 引用 `CollisionGeometry.hpp` 的 `S50CollisionGeometry` 常量
- **数据文件**: 空格分隔纯文本, `#`注释, 输出到 `data/`
- **FK/IK**: **仅通过** `CollisionCheckerSO`, 不自行实现

## C++ 数据输出格式

### so_palletizing_trajectory.txt (19列, 空格分隔)

| 列 | 字段 | 单位 |
|----|------|------|
| 1 | task | -- |
| 2 | seg | -- |
| 3 | time | s |
| 4-9 | q1..q6 | deg |
| 10-15 | v1..v6 | deg/s |
| 16 | dist | mm |
| 17-19 | tcpX,Y,Z | mm |

### so_palletizing_profile.csv (16列CSV)

```
task,segment,step,time_s,q1..q6,selfDist_mm,selfCollision,envCollision,tcpX/Y/Z_mm
```

### so_palletizing_summary.txt (key:value)

关键: version, robot, positions, total_planning_ms, total_motion_s, self_collisions, env_collisions, min_dist_mm

## MATLAB 仿真 (testS50_Palletizing_v15.m)

### SO库加载流程

```matlab
% 1. 预加载桩函数 (RTLD_GLOBAL, 必须!)
dlopen_global(fullfile(scriptDir, 's50_tcp_stubs.so'));
% 2. 加载SO库
loadlibrary(SO_PATH, SO_HEADER, 'alias', 'libHRCInterface');
% 3. 初始化碰撞
dh = [296.5, 336.2, 900, 239, 941.5, 158.5, 158.5, 134.5];
calllib('libHRCInterface', 'initACAreaConstrainPackageInterface', ...
    int16(1), dh, baseGeo, lowerGeo, elbowGeo, upperGeo, wristGeo, homeJoint);
% 4. 开启碰撞通道
calllib('libHRCInterface', 'setCPSelfColliderLinkModelOpenStateInterface', int8([1,1,1]));
```

### MATLAB FK规则

| 用途 | 方法 |
|------|------|
| TCP位置 | `fk2Skeleton()` |
| STL渲染 | `urdfFK()` |
| 碰撞体位置 | `getUIInfoMation` |
| **切勿混用!** | |

### MC_COORD_REF 陷阱

```matlab
% 正确: 初始化所有字段 + [~,tcpS]捕获
tcpS = libstruct('MC_COORD_REF');
tcpS.X=0; tcpS.Y=0; tcpS.Z=0; tcpS.A=0; tcpS.B=0; tcpS.C=0;
[~, tcpS] = calllib('libHRCInterface', 'forwardKinematics2', q_deg, tcpS);

% 错误: 不初始化->段错误, 不捕获输出->tcpS不被修改
```

### v15 输出 (9张PNG + 1个GIF)

| 编号 | 内容 |
|------|------|
| 01 | 碰撞场景STL姿态 |
| 02 | 碰撞距离Profile+统计 |
| 03 | 碰撞几何+环境碰撞体 |
| 04 | 码垛3D场景(多视角)+TCP轨迹 |
| 05 | 关节角/速度/碰撞距离 |
| 06 | 碰撞距离综合分析 |
| 07 | TCP 3D轨迹+姿态 |
| 08 | 动态回放最终帧 |
| 09 | 综合仪表盘 |
| GIF | 码垛全流程动画 |

### v15 关键辅助函数

| 函数 | 用途 |
|------|------|
| `fk2Skeleton(q_deg) -> 5x3 (m)` | FK2骨架模型 |
| `urdfFK(JOINTS, q_rad) -> cell{8x4x4}` | URDF FK变换链 |
| `renderCapsuleRobotHandles(ax, q_deg, ...)` | FK2骨架渲染 |
| `renderSTLRobotHandles(ax, meshData, ...)` | STL网格渲染 |
| `drawCapsule3D(ax, p1, p2, r, col, alpha)` | 3D胶囊绘制 |
| `loadNumericData(filepath) -> matrix` | 加载数值文件 |
| `readSummaryFile(filepath) -> struct` | 解析summary |

## 性能优化

### 分层计时

```cpp
auto timing = planner.getTimingReport();
auto stats = checker.getTimingStats();
```

### 当前瓶颈: rewiring占~97%

优化方向: 减小rewire半径/邻域数, 异步/延迟rewire, 早期终止

### 关键优化手段

- **TCP水平约束**: 通过FK检查TCP方向, 作为硬约束filter拒绝不合规样本
- **KD-Tree最近邻** (30x加速): O(log n), 每500次自动重建
- **椭球采样**: Informed RRT* 限制采样区域
- **批量碰撞**: `isPathCollisionFree()` 单次加锁
- **惰性边验证**: 延迟碰撞检测
- **动态重连半径**: gamma*(ln(n+1)/(n+1))^(1/d), d=6

## 测试编写模式

无框架, 独立可执行文件:

```cpp
int main() {
    try { testFoo(); }
    catch (const std::exception& e) { std::cerr << e.what() << "\n"; return 1; }
    return 0;
}
```

CMakeLists.txt 添加:
```cmake
add_executable(testMyTest testMyTest.cpp)
target_include_directories(testMyTest PRIVATE ${EIGEN3_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/HRCInterface)
target_link_libraries(testMyTest stdc++ m pthread dl)
```

### testTrajectoryOptimality 6套测试

| 套 | 名称 | 内容 |
|----|------|------|
| A | FreeSpaceBaseline | 8组代表性关节对 |
| B | ObstacleAvoidance | 5组环境障碍 |
| C | TcpToTcpWorkflow | 4组TCP->IK->规划全链路 |
| D | ParameterSensitivity | 7组参数扫描 |
| E | PathQualityAnalysis | 3组路径质量 |
| F | Repeatability | 20次重复性 |

## 常见陷阱清单

| # | 陷阱 | 错误 | 正确 |
|---|------|------|------|
| 1 | 单位 | `config.q[0] = 90` | `JointConfig::fromDegrees(...)` |
| 2 | FK来源 | `robot.forwardKinematics()` | `checker.forwardKinematics()` |
| 3 | FK2单位 | 当作mm | 位置是**m**, 朝向是deg |
| 4 | TCP约束 | `freeTcpDuringTransit=true` | `constrainTcpHorizontal=true` |
| 5 | MATLAB SO | 直接loadlibrary | 先dlopen_global桩函数 |
| 6 | MC_COORD_REF | 不初始化/不捕获输出 | 逐字段初始化+[~,tcpS]捕获 |
| 7 | DH数组 | 标准d/a交替 | `[d1,d2,a2,d3,a3,d4,d5,d6]` |
| 8 | FK约定混用 | urdfFK算TCP | fk2Skeleton算TCP |
| 9 | getUIInfo | 用于TCP位置 | 仅用于碰撞体几何 |

## S曲线轨迹执行

- `libCmpRML.so`: 华数上位机真实S曲线库
- 4ms周期采样 (250Hz), 与上位机一致
- 内置 `TimeParameterization.hpp`: 五次多项式近似 (开发/测试用)
- 生产环境直接用 `libCmpRML.so`
