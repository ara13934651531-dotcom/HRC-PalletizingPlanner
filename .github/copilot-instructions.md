# Copilot Instructions - HRC Robot Collision Detection Library

> **Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.**  
> **Website**: https://www.huayan-robotics.com  
> **Contact**: yuesj@huayan-robotics.com

## 项目概述

这是一个**人机协作(HRC)机器人碰撞检测算法库**的测试工程，包含两个主要组件：
1. **HRC碰撞检测库** - 用于验证碰撞检测算法的性能和资源消耗
2. **码垛运动规划系统** - 世界顶尖水平的协作机器人运动规划器（适用于 Elfin 系列协作机器人）

## 架构要点

### 核心组件
- **HRCInterface/** - 算法库的对外接口定义
  - `algorithmLibInterface.h` - 主接口，包含碰撞检测、安全平面、区域约束等全部 API（约 50+ 个函数）
  - `InterfaceDataStruct.h` - IEC 61131-3 兼容的数据类型定义（`RTS_IEC_*` 类型）
- **lib/** - 预编译静态库（`libHRCInterface.a`, `libCmpAgu.a`, `libhansKinematics.a`）
- **include/PalletizingPlanner/** - 码垛运动规划系统
  - `Types.hpp` - 核心数据类型（JointConfig, Pose6D, Path, BSpline）
  - `RobotModel.hpp` - HR_S50-2000运动学模型
  - `CollisionChecker.hpp` - HRC碰撞检测封装
  - `PathPlanner.hpp` - Informed RRT* / BIT* 路径规划
  - `PathOptimizer.hpp` - 捷径优化 + B-Spline平滑
  - `TimeParameterization.hpp` - S曲线时间参数化（预留TOPP接口）
  - `TaskSequencer.hpp` - TSP任务序列优化
  - `PalletizingPlanner.hpp` - 顶层接口
- **test/** - 测试程序

### 数据类型约定
接口使用 PLC/IEC 兼容类型，需注意单位转换：
```cpp
typedef double RTS_IEC_LREAL;  // 浮点数
typedef signed long int RTS_IEC_LINT;  // 长整型
// 坐标/姿态结构体
typedef struct { RTS_IEC_LREAL X, Y, Z, A, B, C; } MC_COORD_REF;
```

### 单位规范
- **位置坐标**: mm（毫米）
- **角度**: deg（度）用于 DH 参数和关节位置，rad（弧度）用于 TCP 姿态
- **速度**: mm/s
- **碰撞距离**: m（米）- 注意与位置单位不同！

## 构建系统

### 标准构建流程
```bash
cd /home/ara/桌面/X86_test
mkdir -p build && cd build
cmake ..
make
# 可执行文件输出到 bin/
```

### CMake 配置要点
- C++17 标准，启用 `-fPIC`
- 静态库链接顺序重要：`libHRCInterface.a` → `libCmpAgu.a` → `libhansKinematics.a`
- 依赖 Eigen3（`/usr/include/eigen3`）

## 碰撞检测 API 使用模式

### 1. 初始化（必须按顺序调用）
```cpp
// 1) 初始化算法包（传入机器人类型、DH参数、各连杆碰撞几何）
initACAreaConstrainPackageInterface(robType, dh, baseGeometry, ...);

// 2) 可选：设置工具坐标系
setACToolCoordinateInterface(toolCoord);

// 3) 可选：添加安全平面/OBB区域
addACHalfPlaneAreaInterface(pose, id);  // id: 0-7
addACOrientedBoundingBoxAreaDefindLWHInterface(pose, lwh, id);  // id: 8-12
```

### 2. 周期性更新（每控制周期调用）
```cpp
updateACAreaConstrainPackageInterface(jointPositions, jointVelocity);
```

### 3. 碰撞查询
```cpp
RTS_IEC_LINT colliderPair[2];
RTS_IEC_LREAL distance;
RTS_BOOL hasCollision = checkCPSelfCollisionInterface(colliderPair, &distance);
```

### 碰撞体索引枚举
```cpp
// collisionModelIndex 枚举值
Base=1, LowerArm=2, Elbow=3, UpperArm=4, Wrist=5, Tool1=6, Tool2=7
// 安全平面: Plane1-8 (10-17), OBB: Obb1-5 (20-24)
```

## 运动规划系统使用

### 基本规划流程
```cpp
#include "PalletizingPlanner/PalletizingPlanner.hpp"
using namespace palletizing;

// 1. 创建并初始化规划器
PalletizingPlanner planner;
planner.initialize();

// 2. 定义起点终点
JointConfig start = JointConfig::fromDegrees({0, -90, 30, 0, -60, 0});
JointConfig goal = JointConfig::fromDegrees({45, -60, 45, 30, -45, 45});

// 3. 规划路径
PlanningResult result = planner.planPointToPoint(start, goal);

// 4. 获取平滑样条用于控制
if (result.isSuccess()) {
    BSpline spline = result.smoothedSpline;
    for (double t = 0; t <= 1.0; t += 0.01) {
        JointConfig q = spline.evaluate(t);
    }
}
```

### 时间参数化
```cpp
TimeParameterizationConfig tpConfig = TimeParameterizationConfig::fromRobotParams(robot.getParams());
tpConfig.profileType = VelocityProfileType::SCurve;
TimeParameterizer parameterizer(tpConfig);
Trajectory trajectory = parameterizer.parameterize(result.optimizedPath);
```

## 测试程序说明

`testCollisionDetectionTime.cpp` 测量碰撞检测的：
- **内存使用** - 通过 `/proc/self/status` 读取 VmRSS
- **堆栈使用** - 通过汇编直接读取 `%rsp` 寄存器（x86_64 专用）
- **CPU 利用率** - 通过 `/proc/stat` 和 `/proc/self/stat`

`testPalletizingPlanner.cpp` 测试运动规划系统的：
- 机器人运动学模型
- 碰撞检测集成
- Informed RRT* 规划
- B-Spline 路径优化
- S曲线时间参数化

运行测试：
```bash
./bin/testCollisionDetectionTime
./bin/testPalletizingPlanner
```

## 开发注意事项

1. **接口是 C 语言绑定** - 使用 `extern "C"` 包装，不要传递 C++ 对象
2. **碰撞几何参数格式**：
   - 球体: `[x, y, z, radius]`（长度 4）
   - 胶囊体: `[start_x, start_y, start_z, end_x, end_y, end_z, radius]`（长度 7）
3. **平台限制** - 堆栈监控代码仅支持 x86_64 + GCC
4. **库不开源** - 只能通过接口文档理解行为，无法查看实现
5. **规划器命名空间** - 所有规划相关类型在 `palletizing` 命名空间
6. **HR_S50-2000 参数** - DH参数配置在 `RobotDHParams::fromHRConfig()`
