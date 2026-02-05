# Copilot Instructions - HRC Robot Motion Planning System

> **Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.**

## 项目概述

HR_S50-2000 协作机器人运动规划系统，包含：
- **HRC碰撞检测库** (`lib/` 预编译库) - C接口，闭源
- **码垛运动规划系统** (`include/PalletizingPlanner/`) - header-only C++17 实现

## 架构层次

```
PalletizingPlanner.hpp    ← 顶层接口
├── PathPlanner.hpp       ← Informed RRT* / BIT* 规划
├── PathOptimizer.hpp     ← B-Spline平滑
├── TimeParameterization.hpp ← S曲线时间参数化
├── CollisionChecker.hpp  ← 封装HRC C接口
└── RobotModel.hpp        ← DH运动学
```

所有类型在 `palletizing` 命名空间内。

## ⚠️ 关键单位约定

| 类型 | 单位 | 示例 |
|------|------|------|
| DH参数/坐标 | **mm** | `d1 = 296.5` |
| 关节角（API） | **deg** | `JointConfig::fromDegrees({0, -90, 30, ...})` |
| 关节角（内部） | **rad** | `config.q[i]` 存储弧度 |
| 碰撞距离 | **m** | `distance = 0.05` 表示 5cm |

## 构建

```bash
mkdir -p build && cd build && cmake .. && make -j
# 输出: bin/testPalletizingPlanner, bin/testPerformanceBenchmark 等
```

**CMake链接顺序必须为**: `libHRCInterface.a` → `libCmpAgu.a` → `libhansKinematics.a`

## 典型使用模式

### 路径规划
```cpp
#include "PalletizingPlanner/PalletizingPlanner.hpp"
using namespace palletizing;

PalletizingPlanner planner;
planner.initialize();

JointConfig start = JointConfig::fromDegrees({0, -90, 30, 0, -60, 0});
JointConfig goal = JointConfig::fromDegrees({45, -60, 45, 30, -45, 45});
PlanningResult result = planner.planPointToPoint(start, goal);

if (result.isSuccess()) {
    // result.optimizedPath, result.smoothedSpline 可用于控制
}
```

### HRC碰撞检测库（C接口）
```cpp
extern "C" {
#include <algorithmLibInterface.h>
}

// 1. 初始化 (robType: 0=Elfin, 1=S-Serial)
RTS_IEC_LREAL dh[8] = {296.5, 336.2, 239.0, 158.5, 158.5, 134.5, 900.0, 941.5};
initACAreaConstrainPackageInterface(1, dh, baseGeom, lowerArmGeom, ...);

// 2. 周期更新
updateACAreaConstrainPackageInterface(jointPos, jointVel);

// 3. 碰撞查询
RTS_IEC_LINT pair[2]; RTS_IEC_LREAL dist;
RTS_BOOL collision = checkCPSelfCollisionInterface(pair, &dist);
```

**碰撞几何格式**:
- 球体: `[x, y, z, radius]` (长度4)
- 胶囊体: `[start_xyz, end_xyz, radius]` (长度7)

## 代码约定

1. **头文件**: 所有规划代码在 `include/PalletizingPlanner/*.hpp`，header-only 设计
2. **数据类型**: 使用 Eigen (`JointVector = Eigen::Matrix<double, 6, 1>`)
3. **角度转换**: 用户接口使用度，内部全部弧度
4. **C/C++混合**: HRC接口用 `extern "C"` 包装，不传递 C++ 对象
5. **平台**: 仅支持 Linux x86_64，堆栈监控使用 `%rsp` 寄存器

## HR_S50-2000 机器人参数

```cpp
// DH参数 (mm) - 见 RobotDHParams::fromHRConfig()
d1=296.5, d2=336.2, d3=239.0, d4=158.5, d5=158.5, d6=134.5, a2=900.0, a3=941.5

// 关节限位 (deg)
J1: ±360, J2: -190~+10, J3: ±165, J4-J6: ±360
```

## 测试程序

| 程序 | 用途 |
|------|------|
| `testPalletizingPlanner` | 综合功能测试 |
| `testPerformanceBenchmark` | KD-Tree/缓存效率测试 |
| `testRobustnessValidation` | 边界条件测试 |
| `testCollisionDetectionTime` | 碰撞检测资源消耗分析 |

## 文件结构参考

- `HRCInterface/algorithmLibInterface.h` - 50+个碰撞检测API
- `include/PalletizingPlanner/Types.hpp` - 核心数据类型定义
- `test/*.cpp` - 测试用例（可作为使用示例）
- `examples/*.cpp` - 简洁的使用示例
