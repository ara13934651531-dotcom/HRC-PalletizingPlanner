# API 文档 | API Documentation

> **HR_S50-2000 协作机器人码垛运动规划系统 — SO-only 架构**

详细的 API 文档请参阅各头文件中的 Doxygen 注释。

---

## 模块概览

### 核心类 (SO 架构)

| 类名 | 头文件 | 描述 |
|------|--------|------|
| `CollisionCheckerSO` | `CollisionCheckerSO.hpp` | `dlopen` 碰撞检测 + FK/IK + 环境障碍物 (唯一 FK/IK 来源) |
| `PathPlannerSO` | `PathPlannerSO.hpp` | TCP 水平约束 Informed RRT\* + IncrementalKDTree6D |
| `PathOptimizer` | `PathOptimizer.hpp` | 自适应捷径优化 + B-Spline 平滑 (De Boor) |
| `TimeParameterization` | `TimeParameterization.hpp` | 七段式 S 曲线 / libCmpRML.so 集成 |
| `NumericalIK` | `NumericalIK.hpp` | 多起点 Damped Least-Squares IK (~0.1ms/次) |
| `RobotModel` | `RobotModel.hpp` | DH 参数 + 关节空间工具 (无 FK/IK!) |

### 碰撞几何与类型

| 类/结构体 | 头文件 | 描述 |
|-----------|--------|------|
| `S50CollisionGeometry` | `CollisionGeometry.hpp` | S50 碰撞包络参数 (4 胶囊 + 1 球, mm) |
| `JointConfig` | `Types.hpp` | 6-DOF 关节配置 (内部 rad) |
| `Pose6D` | `Types.hpp` | 笛卡尔位姿 (Position3D + Orientation) |
| `Path` / `Waypoint` | `Types.hpp` | 路径点序列 |
| `BSpline` | `Types.hpp` | Clamped uniform B-Spline (De Boor) |
| `PlanningResult` | `Types.hpp` | 规划输出 (status + paths + timings) |
| `PlannerConfig` | `Types.hpp` | RRT\* 参数配置 |
| `TCPPlannerConfig` | `PathPlannerSO.hpp` | TCP 约束配置 |
| `CollisionReport` | `CollisionCheckerSO.hpp` | 碰撞检测详细报告 |
| `TimingStats` | `CollisionCheckerSO.hpp` | FK/IK/碰撞独立计时 |
| `PipelineTimingReport` | `PathPlannerSO.hpp` | 规划管线各阶段计时 |

---

## CollisionCheckerSO API

```cpp
// 初始化 (自动搜索 HRC_LIB_PATH → ../lib/ → LD_LIBRARY_PATH)
CollisionCheckerSO checker(robot);
bool ok = checker.initialize();

// 碰撞检测 (所有方法线程安全)
bool free = checker.isCollisionFree(config);
bool pathFree = checker.isPathCollisionFree(start, end, resolution);
CollisionReport report = checker.getCollisionReport(config, computeTcp);
double dist_mm = checker.getCollisionDistanceMm(config);

// FK/IK (唯一正确来源, 位置输出: m, 朝向: deg)
Pose6D pose;
checker.forwardKinematics(config, pose);
JointConfig result;
checker.inverseKinematics(pose, refConfig, result);

// 环境障碍物 (全部 mm 单位)
checker.addEnvObstacleBall(id, center_mm, radius_mm);
checker.addEnvObstacleCapsule(id, start_mm, end_mm, radius_mm);
checker.removeEnvObstacle(id);

// 工具碰撞体
checker.setToolBall(toolIdx, offset_mm, radius_mm);
checker.removeTool(toolIdx);

// 安全裕度
checker.setSafetyMarginMm(10.0);
checker.setEnvSafetyMarginMm(5.0);
```

## PathPlannerSO API

```cpp
TCPPlannerConfig cfg;
cfg.constrainTcpHorizontal = true;   // 默认: TCP 保持水平
cfg.orientTolerance_deg = 30.0;      // 朝向容差
PathPlannerSO planner(robot, checker, cfg);

PlanningResult result = planner.plan(start, goal);
PipelineTimingReport timing = planner.getTimingReport();
```

## 关键配置参数

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| `constrainTcpHorizontal` | `true` | — | TCP 保持水平 (码垛必须) |
| `orientTolerance_deg` | `30.0` | deg | TCP 朝向容差 |
| `maxIterations` | `50000` | — | RRT\* 最大迭代 |
| `stepSize` | `0.1` | rad | RRT 扩展步长 |
| `goalBias` | `0.15` | 概率 | 目标采样偏置 |
| `collisionResolution` | `0.02` | rad | 碰撞检测插值分辨率 |
| `shortcutIterations` | `200` | — | 捷径优化迭代 |
| `splineDegree` | `5` | — | B-Spline 阶数 |

---

## 快速链接

- [源码头文件](../include/PalletizingPlanner/)
- [示例程序](../examples/)
- [测试程序](../test/)
- [指导文件](../.github/copilot-instructions.md)

---

**广东华沿机器人有限公司 | Guangdong Huayan Robotics Co., Ltd.** | https://www.huayan-robotics.com
