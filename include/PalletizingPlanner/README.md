# PalletizingPlanner — Header-Only C++17 运动规划库

> **Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd. (广东华沿机器人有限公司)**

## 概述

本目录包含 HR_S50-2000 协作机器人码垛运动规划系统的全部核心头文件。采用 **SO-only 架构**，通过 `dlopen` 运行时加载 `libHRCInterface.so` 实现碰撞检测、正/逆运动学。

**设计原则**:
- Header-only: 仅需 `#include` 即可使用，零编译依赖
- SO-only: 所有 FK/IK/碰撞检测通过 `libHRCInterface.so` 完成
- TCP 水平约束: 码垛场景默认 `constrainTcpHorizontal=true`
- 无魔法数字: 所有常量引用 `RobotDHParams` 或 `S50CollisionGeometry`

## 模块架构

```
CollisionCheckerSO.hpp     ← dlopen(libHRCInterface.so), 唯一FK/IK/碰撞接口
├── PathPlannerSO.hpp      ← TCP水平约束 Informed RRT* + IncrementalKDTree6D
├── PathOptimizer.hpp      ← B-Spline 路径平滑 (De Boor 算法)
├── TimeParameterization.hpp ← S曲线 (内置) / libCmpRML.so (生产)
├── CollisionGeometry.hpp  ← 统一 S50 碰撞包络参数 (胶囊体+球体)
├── NumericalIK.hpp        ← 多起点 Damped Least-Squares IK 求解器
├── RobotModel.hpp         ← 机器人 DH 参数 + 关节空间工具 (无FK/IK!)
└── Types.hpp              ← JointConfig, Path, BSpline, PlanningResult
```

## 模块说明

| 文件 | 行数 | 职责 |
|------|------|------|
| `CollisionCheckerSO.hpp` | ~724 | `dlopen` 加载 SO 库, 线程安全碰撞/FK/IK, 环境障碍物管理, TimingStats |
| `PathPlannerSO.hpp` | ~900 | Informed RRT* + 6D KD-Tree, TCP 水平约束过滤, 椭球采样, rewiring |
| `PathOptimizer.hpp` | ~471 | 自适应捷径优化 + 5 次 B-Spline 拟合 (De Boor) |
| `TimeParameterization.hpp` | ~443 | 七段式 S 曲线, 4ms 采样 (250 Hz), 兼容 libCmpRML.so |
| `CollisionGeometry.hpp` | ~100 | S50 碰撞包络: 4 胶囊体 + 1 球体, 安全裕度 |
| `NumericalIK.hpp` | ~161 | 多起点 DLS 数值 IK, ~0.1ms/次, 使用 SO FK |
| `RobotModel.hpp` | ~199 | DH 参数, 关节限位/速度/加速度, 随机采样, 插值 |
| `Types.hpp` | ~482 | `JointConfig`, `Pose6D`, `Path`, `BSpline`, `PlanningResult`, 配置结构体 |

## 快速使用

```cpp
#include "PalletizingPlanner/CollisionCheckerSO.hpp"
#include "PalletizingPlanner/PathPlannerSO.hpp"
using namespace palletizing;

RobotModel robot;
CollisionCheckerSO checker(robot);
checker.initialize();  // dlopen libHRCInterface.so

// TCP 水平约束规划 (码垛场景默认)
TCPPlannerConfig cfg;
cfg.constrainTcpHorizontal = true;
PathPlannerSO planner(robot, checker, cfg);

auto start = JointConfig::fromDegrees({0, -90, 0, 0, 90, 0});
auto goal  = JointConfig::fromDegrees({45, -60, 45, 30, -45, 45});
auto result = planner.plan(start, goal);
```

## 单位约定

| 位置 | 关节角 | 坐标/距离 |
|------|--------|----------|
| 用户 API `JointConfig::fromDegrees()` | **deg** | — |
| 内部存储 `config.q[i]` | **rad** | — |
| HRC SO 接口 (update/FK/IK) | **deg** | **mm** |
| FK2 输出位置 | — | **m** (非 mm!) |
| `CollisionGeometry.hpp` | — | **mm** |
| 碰撞距离返回值 | — | **mm** |

## 性能指标

| 指标 | 实测 |
|------|------|
| 碰撞检测 (单次) | **31 μs** |
| FK (单次) | **10 μs** |
| IK (单次) | **0.1 ms** |
| 码垛全链路 (12 箱) | **0.31 s** |
| 规划成功率 | **100%** |

## 参考文献

1. Gammell, J.D., et al. "Informed RRT\*: Optimal sampling-based path planning focused via direct sampling of an admissible ellipsoidal heuristic." *IROS 2014*.
2. De Boor, C. "A Practical Guide to Splines." *Springer, 2001*.
