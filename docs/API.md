# API 文档 | API Documentation

详细的 API 文档请参阅各头文件中的注释，或访问在线文档。

## 📚 模块概览

### 核心类

| 类名 | 头文件 | 描述 |
|------|--------|------|
| `PalletizingPlanner` | `PalletizingPlanner.hpp` | 顶层规划接口 |
| `PathPlanner` | `PathPlanner.hpp` | Informed RRT* 路径规划器 |
| `PathOptimizer` | `PathOptimizer.hpp` | B-Spline 路径优化器 |
| `TimeParameterizer` | `TimeParameterization.hpp` | S曲线时间参数化 |
| `TaskSequencer` | `TaskSequencer.hpp` | TSP 任务序列优化 |
| `CollisionChecker` | `CollisionChecker.hpp` | HRC 碰撞检测封装 |
| `RobotModel` | `RobotModel.hpp` | HR_S50-2000 运动学模型 |

### 数据类型

| 类型 | 描述 |
|------|------|
| `JointConfig` | 6-DOF 关节配置 |
| `Pose6D` | 笛卡尔位姿 (x, y, z, rx, ry, rz) |
| `Path` | 路径点序列 |
| `BSpline` | B样条曲线 |
| `Trajectory` | 带时间戳的轨迹 |

## 🔗 快速链接

- [源码头文件](../include/PalletizingPlanner/)
- [示例程序](../examples/)
- [测试程序](../test/)

---

**Huayan Robotics** | https://www.huayan-robotics.com
