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

**数据流**: 用户传入 `JointConfig`(deg) → 内部转 rad → PathPlanner 采样+碰撞检测 → PathOptimizer B-Spline平滑 → TimeParameterization S曲线 → 输出轨迹

**Pipeline时间**: PathPlanning ~66ms + Optimization ~16ms + TimeParam ~53ms = **~135ms 总计**

## ⚠️ 关键单位约定（必读 - 最常见错误源）

| 位置 | 关节角 | 坐标/距离 |
|------|--------|----------|
| 用户API `JointConfig::fromDegrees()` | **deg** | — |
| 内部存储 `config.q[i]` / 路径文件 | **rad** | — |
| HRC `updateACAreaConstrainPackageInterface()` | **deg** | — |
| DH参数 / 碰撞几何 | — | **mm** |
| `SceneConfig` / `OBBObstacle.lwh` | — | **m** |
| HRC碰撞距离返回值 | — | **m** |

**API模式**: 所有公共API接受度(deg), 内部立即转换为弧度(rad)。文件输出使用rad便于数值计算。

## 项目结构理念

- **header-only 库**: 所有规划器代码在 `include/PalletizingPlanner/`, 无需编译成库, 用户直接 `#include`
- **示例驱动**: `examples/` 包含完整使用示例, 从简单点对点到复杂码垛任务
- **数据输出**: 所有轨迹/路径输出到 `data/` 目录, 使用空格分隔文本格式便于Python/MATLAB读取
- **闭源依赖隔离**: HRC碰撞库预编译在 `lib/`, 头文件在 `HRCInterface/`, 通过 `extern "C"` 清晰封装

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

## 典型开发工作流

### 1. 添加新的规划算法
```cpp
// 1. 在 include/PalletizingPlanner/MyAlgorithm.hpp 创建 header-only 实现
namespace palletizing {
class MyAlgorithm {
    // ...实现...
};
}

// 2. 在 examples/test_my_algorithm.cpp 创建测试
#include "PalletizingPlanner/MyAlgorithm.hpp"
// ...测试代码...

// 3. 在 test/CMakeLists.txt 添加编译目标 (遵循严格链接顺序)
add_executable(testMyAlgorithm ${CMAKE_SOURCE_DIR}/examples/test_my_algorithm.cpp)
target_link_libraries(testMyAlgorithm libHRCInterface.a libCmpAgu.a libhansKinematics.a stdc++ m)

// 4. 编译运行
# cd build && make -j && ./bin/testMyAlgorithm
```

### 2. 调试碰撞检测问题
```bash
# 1. 启用详细日志 (在 CollisionChecker.hpp 中设置 VERBOSE=true)
# 2. 运行碰撞检测测试
./bin/testCollisionDetectionTime

# 3. 可视化当前配置 (修改 scripts/visualize_scene.py 中的 q_deg)
python3 scripts/visualize_scene.py  # 生成 data/sim3d/*.png

# 4. 精确验证 (MATLAB)
matlab -nodesktop -batch "testS50.m"  # 输出到 ArmCollisionModel/pic/
```

### 3. 性能分析流程
```bash
# 1. 基准性能测试 (30x KD-Tree加速, 18x缓存加速)
./bin/testPerformanceBenchmark

# 2. 高性能配置对比
./bin/testHighPerformance  # 优化前后对比, 需 pthread

# 3. 鲁棒性泛化测试 (99.5%成功率)
./bin/testRobustnessValidation  # 需 pthread, 测试1000+随机配置
```

## 性能基准 (世界顶尖水平)

| 指标 | 目标 | 实测 | 验证测试 |
|------|------|------|----------|
| 简单场景规划 | < 100 ms | **0.04 ms** | testPerformanceBenchmark |
| 中等距离规划 | < 500 ms | **20-37 ms** | testPalletizingPlanner |
| 大范围运动规划 | < 2 s | **238 ms** | testHighPerformance |
| 完整流水线 | - | **135 ms** | testPalletizingPlanner |
| KD-Tree 加速比 | > 10x | **30.01x** | testPerformanceBenchmark |
| 碰撞缓存加速比 | > 5x | **18.09x** | testPerformanceBenchmark |
| 规划成功率 | > 95% | **99.5%** | testRobustnessValidation |

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

## 常见陷阱与调试技巧

### 单位转换错误 (最常见)
```cpp
// ❌ 错误: 直接使用度数
JointConfig config;
config.q[0] = 90;  // 内部期望弧度!

// ✅ 正确: 使用 fromDegrees()
JointConfig config = JointConfig::fromDegrees({90, -50, 70, 0, 80, 0});
```

### Python/C++ DH参数不一致
**问题**: Python可视化与C++运动学结果不匹配
**诊断**: 对比 `scripts/visualize_scene.py` 中的 `DH = {...}` 与 `include/PalletizingPlanner/RobotModel.hpp` 的DH参数
**修复**: 确保两处定义完全一致 (d1, d2, d3, d4, d5, d6, a2, a3)

### 碰撞检测初始化失败
```cpp
// HRC库初始化顺序关键:
// 1. initACAreaConstrainPackageInterface(robType=1, dh, geom)
// 2. setCPSelfColliderLinkModelOpenStateInterface(true)  // 开启连杆碰撞
// 3. updateACAreaConstrainPackageInterface(jointPos_deg, vel)  // 每次查询前更新

// 检查返回值:
if (!initACAreaConstrainPackageInterface(...)) {
    // 失败原因: DH参数错误, 碰撞几何数组长度不匹配
}
```

### CMake链接错误
**症状**: `undefined reference to HRC function`
**原因**: 链接顺序错误或缺少库
**解决**: 严格按顺序: `libHRCInterface.a` → `libCmpAgu.a` → `libhansKinematics.a` → `stdc++` → `m` [→ `pthread`]

### 路径输出文件格式
所有输出文件使用**空格分隔**, **rad单位**, **`#`注释头**:
```
# Joint trajectory (rad)
# q1 q2 q3 q4 q5 q6
0.0 -1.5708 0.5236 0.0 -1.0472 0.0
0.1 -1.5000 0.6000 0.1 -1.1000 0.1
...
```

## 可视化

- **Python**: `scripts/visualize_scene.py` — matplotlib, DH参数**独立硬编码**(需与C++同步), 后端 `Agg`(离线)/`TkAgg`(交互), 输出 `data/sim3d/`
  - 运行: `python3 scripts/visualize_scene.py` (生成PNG到 `data/sim3d/`)
  - DH参数定义在脚本顶部 `DH = {...}` 和 `fk_s50()` 函数中, **必须与C++保持一致**
  - 颜色方案基于实物照片: `ROBOT_BODY = (0.92, 0.92, 0.94)` (白灰色), `JOINT_RING = (0.02, 0.02, 0.04)` (黑色)
- **MATLAB**: `ArmCollisionModel/testS50*.m` — 用 `@RobotCollisionModel` 类, 碰撞配置 `model/collideConfig/S50_collision.json`, 输出 `pic/`
  - 运行示例: `matlab -nodesktop -nosplash -batch "run('testS50_Palletizing.m')"`
  - 关键类: `@RobotCollisionModel` 使用 URDF + STL 网格模型进行精确可视化
  - 支持动态轨迹动画: `testS50_Dynamic.m` 生成 GIF 动画
- MATLAB Headless检测: `isHeadless = ~usejava('desktop')` → `set(0,'DefaultFigureVisible','off')`

## 可视化工作流示例

1. **C++规划** → 输出轨迹文件到 `data/trajectory.txt` (关节角rad, 空格分隔)
2. **Python快速预览** → `python3 scripts/visualize_path.py` 读取轨迹, 生成2D/3D图像
3. **MATLAB精确验证** → `testS50_Dynamic.m` 加载轨迹, 用STL模型渲染, 检查碰撞距离

## 可视化系统架构

### Python可视化 (matplotlib)
- **核心文件**: `scripts/visualize_scene.py`, `visualize_path.py`, `visualize_trajectory.py`
- **独立DH正运动学**: `fk_s50()` 函数, 标准DH变换, 必须与C++同步
- **碰撞几何**: 胶囊体/球体, LOCAL坐标系 → 世界坐标系变换
- **输出目录**: `data/sim3d/` (PNG图像)
- **后端切换**: `matplotlib.use('Agg')` (无显示) ↔ `matplotlib.use('TkAgg')` (交互式)

### MATLAB可视化 (Robotics Toolbox)
- **机器人类**: `@RobotCollisionModel` 封装 URDF + STL 网格
- **配置文件**: `model/collideConfig/S50_collision.json` (碰撞几何mm, LOCAL坐标)
- **测试脚本模式**:
  - `testS50.m` - 静态单姿态验证
  - `testS50_Dynamic.m` - 动态轨迹动画(生成GIF)
  - `testS50_Palletizing.m` - 码垛场景多姿态验证
- **输出目录**: `ArmCollisionModel/pic/{S50_sim, S50_scene_sim, S50_palletizing}/`
- **Headless模式**: 检测 `~usejava('desktop')` → 关闭图形窗口, 保存文件

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
