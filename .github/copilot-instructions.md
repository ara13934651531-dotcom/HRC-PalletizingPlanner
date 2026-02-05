# Copilot Instructions - HRC Robot Motion Planning System

> **Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.**

## 项目架构

三层系统: **C++ 规划器** (header-only) + **HRC 碰撞库** (闭源 C) + **MATLAB/Python 可视化**

```
PalletizingPlanner.hpp     ← 顶层API (palletizing命名空间)
├── PathPlanner.hpp        ← Informed RRT* / BIT*
├── PathOptimizer.hpp      ← B-Spline平滑
├── TimeParameterization.hpp ← S曲线时间参数化
├── CollisionChecker.hpp   ← 封装HRC C接口 (extern "C")
└── RobotModel.hpp         ← DH正运动学
```

## ⚠️ 关键单位约定（必读）

| 类型 | 单位 | 示例 |
|------|------|------|
| DH参数/坐标 | **mm** | `d1 = 296.5`, `a2 = 900.0` |
| 关节角（API） | **deg** | `JointConfig::fromDegrees({0, -90, 30, ...})` |
| 关节角（内部/HRC） | **rad** | `config.q[i]` 内部存储弧度 |
| HRC碰撞几何 | **mm** | 胶囊体`[7]`, 球体`[4]` |
| 碰撞距离 | **m** | `distance = 0.05` 表示 5cm |

## 构建与测试

```bash
# 构建 (输出到 bin/)
mkdir -p build && cd build && cmake .. && make -j

# 运行测试
./bin/testPalletizingPlanner          # 综合功能
./bin/testPerformanceBenchmark        # KD-Tree/缓存效率 (30x加速验证)
./bin/testCollisionDetectionTime      # HRC库资源消耗分析
```

**CMake链接顺序 (严格)**: `libHRCInterface.a` → `libCmpAgu.a` → `libhansKinematics.a`

## C++ 规划器使用

```cpp
#include "PalletizingPlanner/PalletizingPlanner.hpp"
using namespace palletizing;

PalletizingPlanner planner;
planner.initialize();

// 用户接口始终用度数
JointConfig start = JointConfig::fromDegrees({0, -90, 30, 0, -60, 0});
JointConfig goal = JointConfig::fromDegrees({45, -60, 45, 30, -45, 45});
PlanningResult result = planner.planPointToPoint(start, goal);

if (result.isSuccess()) {
    // result.optimizedPath, result.smoothedSpline 可用于控制
}
```

## HRC 碰撞检测库 (C接口)

```cpp
extern "C" {
#include <algorithmLibInterface.h>
}

// robType: 0=Elfin, 1=S-Serial (HR_S50)
RTS_IEC_LREAL dh[8] = {296.5, 336.2, 239.0, 158.5, 158.5, 134.5, 900.0, 941.5};
initACAreaConstrainPackageInterface(1, dh, baseGeom, lowerArmGeom, ...);

// 周期更新 (jointPos单位: deg)
updateACAreaConstrainPackageInterface(jointPos, jointVel);

// 碰撞查询
RTS_IEC_LINT pair[2]; RTS_IEC_LREAL dist;
RTS_BOOL collision = checkCPSelfCollisionInterface(pair, &dist);
```

**碰撞几何格式 (mm)**:
- 胶囊体: `[start_x,y,z, end_x,y,z, radius]` (长度7)
- 球体: `[center_x,y,z, radius]` (长度4)

## HR_S50-2000 参数

```cpp
// DH参数 (mm) - RobotDHParams::fromHRConfig()
d1=296.5, d2=336.2, d3=239.0, d4=158.5, d5=158.5, d6=134.5, a2=900.0, a3=941.5

// 关节限位 (deg)
J1: ±360, J2: -190~+10, J3: ±165, J4-J6: ±360
```

## 可视化工具

| 工具 | 用途 |
|------|------|
| `scripts/visualize_scene.py` | Python 3D场景 (matplotlib) |
| `ArmCollisionModel/testS50.m` | MATLAB 静态碰撞可视化 |
| `ArmCollisionModel/testS50_Dynamic.m` | MATLAB 动态轨迹GIF |

MATLAB碰撞模型配置: `ArmCollisionModel/model/collideConfig/S50_collision.json`

## 代码约定

1. **header-only**: 规划代码在 `include/PalletizingPlanner/*.hpp`
2. **Eigen**: `JointVector = Eigen::Matrix<double, 6, 1>`
3. **C/C++混合**: HRC接口用 `extern "C"` 包装，不传递 C++ 对象
4. **平台**: 仅 Linux x86_64，堆栈监控使用 `%rsp` 寄存器

## 关键文件

- [HRCInterface/algorithmLibInterface.h](HRCInterface/algorithmLibInterface.h) - 50+ 碰撞检测 API
- [include/PalletizingPlanner/Types.hpp](include/PalletizingPlanner/Types.hpp) - 核心数据类型
- [include/PalletizingPlanner/CollisionChecker.hpp](include/PalletizingPlanner/CollisionChecker.hpp) - HRC封装
- [examples/basic_planning_example.cpp](examples/basic_planning_example.cpp) - 使用示例
