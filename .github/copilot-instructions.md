# Copilot Instructions - HRC Robot Motion Planning System

> **Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.**

## 架构概览

三层系统: **C++17 header-only 规划器** (`include/PalletizingPlanner/*.hpp`) + **HRC 碰撞库** (闭源 C 静态库) + **MATLAB/Python 可视化**

```
PalletizingPlanner.hpp     ← 顶层API (palletizing 命名空间，全部扁平)
├── PathPlanner.hpp        ← Informed RRT* / BIT* (含 KDTree.hpp)
├── PathOptimizer.hpp      ← B-Spline平滑 (De Boor算法)
├── TimeParameterization.hpp ← S曲线七段式时间参数化
├── CollisionChecker.hpp   ← 封装HRC C接口 (extern "C")
├── CollisionCache.hpp     ← FNV-1a哈希 + LRU淘汰 (mutex线程安全)
├── TaskSequencer.hpp      ← TSP任务序列优化 (2-opt)
└── RobotModel.hpp         ← DH正运动学 (标准DH变换)
```

数据流: 用户传入 `JointConfig`(deg) → 内部转 rad → PathPlanner 采样+碰撞检测 → PathOptimizer B-Spline平滑 → TimeParameterization S曲线 → 输出轨迹

## ⚠️ 关键单位约定（必读 - 最常见错误源）

| 位置 | 关节角 | 坐标/距离 |
|------|--------|----------|
| 用户API `JointConfig::fromDegrees()` | **deg** | — |
| 内部存储 `config.q[i]` / 路径文件 | **rad** | — |
| HRC `updateACAreaConstrainPackageInterface()` | **deg** | — |
| DH参数 / 碰撞几何 | — | **mm** |
| `SceneConfig` / `OBBObstacle.lwh` | — | **m** |
| HRC碰撞距离返回值 | — | **m** |

## 构建与测试

```bash
mkdir -p build && cd build && cmake .. && make -j    # 输出到 bin/
./bin/testPalletizingPlanner       # 综合功能 (7个子测试)
./bin/testPerformanceBenchmark     # KD-Tree/缓存 30x加速验证
./bin/testCollisionDetectionTime   # HRC库资源消耗
./bin/testHighPerformance          # 优化前后对比 (需 pthread)
./bin/testRobustnessValidation     # 鲁棒性泛化 (需 pthread)
```

**CMake链接顺序(严格)**: `libHRCInterface.a` → `libCmpAgu.a` → `libhansKinematics.a` → `stdc++` → `m` [→ `pthread`]

依赖: Eigen3 (系统 `/usr/include/eigen3`), 无包管理器。仅 Linux x86_64。

## C++ 代码约定

- **命名**: 类 PascalCase, 方法 camelCase, 成员变量 `trailing_underscore_`, 结构体字段 camelCase 无下划线
- **头文件**: `#pragma once`, Doxygen 文件头 (`@file`, `@brief`, `@date`), 中文注释广泛使用
- **错误处理**: **无异常**。用返回值: `PlanningResult.status` 枚举 + `errorMessage` 字符串; `bool initialize()` 返回成功/失败
- **核心类型**: `JointVector = Eigen::Matrix<double, 6, 1>`; `Pose6D` 含 `Position3D` + `Quaterniond`
- **C/C++边界**: HRC接口 `extern "C" { #include <algorithmLibInterface.h> }`, 只传C数组不传C++对象; `robType`: 0=Elfin, 1=S-Serial(HR_S50)
- **数据文件**: 空格分隔纯文本, `#` 注释头标明格式和单位, 输出到 `data/` 或项目根目录

## 测试编写模式

无测试框架。每个测试是独立可执行文件，自定义 `main()`:
```cpp
int main() {
    try {
        testFoo();  // void testXxx() 命名
        testBar();
    } catch (const std::exception& e) {
        std::cerr << "异常: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
```
新增测试需在 `test/CMakeLists.txt` 添加 `add_executable` + 严格链接顺序。

## HRC 碰撞库 (C接口)

```cpp
extern "C" { #include <algorithmLibInterface.h> }
// 初始化: initACAreaConstrainPackageInterface(robType=1, dh[8], 碰撞几何...)
// 更新(每周期): updateACAreaConstrainPackageInterface(jointPos_deg, jointVel)
// 查询: checkCPSelfCollisionInterface(pair, &dist) → RTS_BOOL
```
碰撞几何(mm): 胶囊体 `[sx,sy,sz, ex,ey,ez, radius]`(7), 球体 `[cx,cy,cz, radius]`(4)

## HR_S50-2000 参数

DH(mm): `d1=296.5, d2=336.2, d3=239.0, d4=158.5, d5=158.5, d6=134.5, a2=900.0, a3=941.5`
关节限位(deg): J1:±360, J2:-190~+10, J3:±165, J4-J6:±360

## 可视化

- **Python**: `scripts/visualize_scene.py` — matplotlib, DH参数**独立硬编码**(需与C++同步), 后端 `Agg`(离线)/`TkAgg`(交互), 输出 `data/sim3d/`
- **MATLAB**: `ArmCollisionModel/testS50*.m` — 用 `@RobotCollisionModel` 类, 碰撞配置 `model/collideConfig/S50_collision.json`, 输出 `pic/`
- MATLAB Headless检测: `isHeadless = ~usejava('desktop')` → `set(0,'DefaultFigureVisible','off')`

## 关键文件索引

| 文件 | 说明 |
|------|------|
| `include/PalletizingPlanner/Types.hpp` | 所有核心类型: JointConfig, Path, BSpline, PlanningResult |
| `include/PalletizingPlanner/PalletizingPlanner.hpp` | 顶层API入口 |
| `include/PalletizingPlanner/CollisionChecker.hpp` | HRC C库封装, SceneConfig定义 |
| `HRCInterface/algorithmLibInterface.h` | 50+ 碰撞检测C接口声明 |
| `HRCInterface/InterfaceDataStruct.h` | RTS_IEC_LREAL/LINT/BOOL 类型定义 |
| `examples/basic_planning_example.cpp` | 完整使用示例 |
| `test/CMakeLists.txt` | 所有测试目标和链接配置 |
