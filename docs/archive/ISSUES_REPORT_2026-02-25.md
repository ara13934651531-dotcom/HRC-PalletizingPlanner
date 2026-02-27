# HRC-PalletizingPlanner 系统性问题分析报告

> **日期**: 2026-02-25  
> **版本**: master 分支  
> **分析范围**: C++ header-only 库 / 测试文件 / CMake 配置 / MATLAB 仿真 / Python 脚本 / 运行时测试结果

---

## 目录

1. [严重问题 (Critical)](#1-严重问题-critical)
2. [重要问题 (Major)](#2-重要问题-major)
3. [中等问题 (Medium)](#3-中等问题-medium)
4. [轻微问题 (Minor)](#4-轻微问题-minor)
5. [架构与设计问题](#5-架构与设计问题)
6. [测试运行结果分析](#6-测试运行结果分析)
7. [文档与维护问题](#7-文档与维护问题)

---

## 1. 严重问题 (Critical)

### 1.1 `testHighPerformance` **成功率 0%** — 静态库栈完全不可用

**现象**: 运行 `./bin/testHighPerformance` 50 次试验，成功率 **0.0%**。

```
║ Trials:        50                   ║
║ Success Rate:  0.0    %             ║
```

**原因**: `HighPerformancePlanner.hpp` 依赖 `CollisionChecker.hpp` (静态库栈)，静态库初始化打印
`---------------------1-------------------` / `---------------------2-------------------` 这些调试输出来自底层HRC静态库，
但在当前测试配置下，碰撞检测结果始终将起点或终点判为碰撞 (`Collision at Start`)。

**影响**: 整个静态库栈 (CollisionChecker + PathPlannerOptimized + HighPerformancePlanner + PalletizingPlanner) 在实际使用中不可靠。

**文件**: `include/PalletizingPlanner/HighPerformancePlanner.hpp`, `test/testHighPerformance.cpp`

---

### 1.2 `testPalletizingPlanner` 成功率极低 (10%)

**现象**: 测试6 (随机起止点规划) 仅 1/10 成功：
```
性能统计:
  成功率: 1/10 (10.00%)
```
测试7 (S曲线时间参数化) 同样失败: `路径规划失败`。

**原因**: 与 1.1 相同，静态库碰撞检测将大多数配置误判为碰撞。

**文件**: `test/testPalletizingPlanner.cpp`

---

### 1.3 `testRobustnessValidation` 超时 (>60s 无输出)

**现象**: `./bin/testRobustnessValidation` 运行超过 60 秒后被 timeout 杀死，无任何输出。

**原因**: 静态库栈碰撞检测初始化可能死循环或极慢，导致整个验证程序卡死。

**文件**: `test/testRobustnessValidation.cpp`

---

### 1.4 `libHRCInterface.so` 不在项目 `lib/` 目录中

**现象**: SO 栈测试需要设置环境变量 `HRC_LIB_PATH`，因为 `lib/` 目录只有 `libHRCInterface.a` 而没有 `.so`。

```bash
$ ls lib/libHRCInterface*
lib/libHRCInterface.a     # 只有静态库
# 动态库在 /home/ara/文档/collision/HansAlgorithmExport/bin/libHRCInterface.so
```

**影响**: SO 栈测试无法开箱即用，CI/CD 也无法自动运行。`CollisionCheckerSO::initialize()` 搜索路径 `../lib/libHRCInterface.so` 会失败。

**建议**: 将 `.so` 复制到 `lib/` 目录，或在 CMake 中添加自动搜索逻辑。

---

### 1.5 `libCmpRML.so` 位置错误

**现象**: `libCmpRML.so` 放在项目根目录 `/home/ara/文档/X86_test/libCmpRML.so` 而非 `lib/` 目录。但 CMake 中 `testRMLProbe` 引用的是 `${CMAKE_SOURCE_DIR}/libCmpRML.so`（项目根目录）。

**影响**: 库文件组织混乱，其他 RML 测试可能找不到依赖。

**文件**: `test/CMakeLists.txt:107`

---

## 2. 重要问题 (Major)

### 2.1 `testTrajectoryOptimality` TestF 首次运行出现负时间 (-63554ms)

**现象**:
```
[  1] -63554.17     3.13    2.312   3002 ✅
[  2]  1289.80     2.84    2.312   3002 ✅
```

**原因**: TestF 的 `planTime_ms` 来自 `planner.getTimingReport().planningTotal_ms`。虽然 `plan()` 在开头执行 `timing_ = PipelineTimingReport()` 重置，但 `timing_` 是 `mutable` 的，且在多个 lambda/内部函数中通过 `msNow()` 累加。如果 TestE 的 `planner` 对象在 TestF 开始前未完全析构，`timing_` 中残存来自 TestE 的巨大值（约 65 秒）。首次 `plan()` 调用初始的 `ms(tStart, now)` 计算可能得到负值，因为 `tStart` 是新的而 `totalPipeline_ms` 使用的时间点不匹配。

更精确的原因：`PipelineTimingReport::planningTotal_ms` 通过 `ms(tPlan, tPlanEnd)` 计算，理论上应该正确。但 `runOnePlan` 中 `r.planTime_ms = timing.planningTotal_ms` 使用的是内部计时而非外部 `t0-t1` 差值。由于 `start.distanceTo(goal) < 1e-8` 的短路逻辑或 rewiring 的内部 `msNow()` 累加逻辑，某种情况下会产生负值。

**文件**: `include/PalletizingPlanner/PathPlannerSO.hpp:367`, `test/testTrajectoryOptimality.cpp:148`

---

### 2.2 TestF 所有 20 次规划产生**完全相同的路径** (2.312 rad)

**现象**:
```
路径长度: μ=2.312rad σ=0.000rad 变异系数: 路径=0.0%
```

RRT* 是随机算法，20 次完全相同的路径长度 (σ=0) 极不正常。

**可能原因**:
1. `std::random_device{}()` 在某些 Linux 环境下可能返回固定种子
2. 起止点距离 < `stepSize` 导致直连，跳过了 RRT* 搜索
3. `maxIterations=3000`、`maxPlanningTime=2.0s` 的情况下所有运行都收敛到相同最优解

**文件**: `test/testTrajectoryOptimality.cpp:610`, `include/PalletizingPlanner/PathPlannerSO.hpp:909`

---

### 2.3 `CollisionCheckerSO::getCollisionReport` 中 `selfMinDist_mm` 处理不一致

**碰撞时**:
```cpp
report.selfMinDist_mm = coll ? 0.0 : dist;   // Line ~384
```

但在 `isCollisionFree()` 中：
```cpp
if (coll) return false;
if (dist < safetyMarginMm_) return false;     // Line ~334
```

**问题**: 当 `coll=0` (无碰撞) 但 `dist < safetyMarginMm_=10mm` 时，`isCollisionFree` 返回 false，但 `getCollisionReport` 中设置 `report.collisionFree = !selfCollision && ... && selfMinDist_mm > safetyMarginMm_`。两者逻辑一致，但 `getCollisionReport` 在计算 `collisionFree` 之前没有检查关节限位 (`isWithinLimits`)，而 `isCollisionFree` 会检查。

**文件**: `include/PalletizingPlanner/CollisionCheckerSO.hpp:350-406`

---

### 2.4 `CollisionCheckerSO` 计时统计在 `getTimingStats()` 中除法不安全

```cpp
TimingStats avg = timing_;
if (avg.callCount > 0) {
    avg.updateTime_us    /= avg.callCount;   // 修改了原始timing_的副本
    ...
}
```

**问题**: 虽然这是副本上操作，但 `timing_` 本身是 `mutable` 的并且在多线程环境 (通过 `std::lock_guard<std::mutex>` 保护) 下被并发修改。`getTimingStats()` 未加锁就读取了 `timing_`，存在**数据竞争**。

**文件**: `include/PalletizingPlanner/CollisionCheckerSO.hpp:616-633`

---

### 2.5 `PathPlannerSO` 中 `propagateCostUpdate` 使用纯关节距离而非混合代价

```cpp
void propagateCostUpdate(int nodeId) {
    ...
    double edgeDist = nodes_[cur].config.distanceTo(nodes_[ch].config);
    double nc = nodes_[cur].costFromStart + edgeDist;
    ...
}
```

**问题**: 在 `useTcpCost=true` 模式下，节点的 `costFromStart` 是混合代价 `(1-w)*jointDist + w*tcpCost`，但 `propagateCostUpdate` 仅用纯关节距离传播，导致代价不一致。

**文件**: `include/PalletizingPlanner/PathPlannerSO.hpp:854-865`

---

### 2.6 `PathOptimizer.hpp` 依赖 `CollisionChecker.hpp` (静态库) 但 `PathPlannerSO` 内部也实现了路径优化

**问题**: `PathPlannerSO` 在其 `optimizePath()` 方法中重新实现了 `shortcutOptimize`、`simplifyPath`、`subdividePath` 和 BSpline 拟合——这与 `PathOptimizer.hpp` 中的逻辑几乎完全重复。

**影响**: 代码维护困难，修改一处逻辑需同步改另一处。

**文件**: `include/PalletizingPlanner/PathPlannerSO.hpp:714-815`, `include/PalletizingPlanner/PathOptimizer.hpp:98-245`

---

### 2.7 `CollisionCheckerSO::toMmDisplay` 自动单位转换逻辑不可靠

```cpp
auto toMmDisplay = [](double v) {
    return std::abs(v) < 10.0 ? v * 1000.0 : v;
};
```

**问题**: 通过 `< 10.0` 判断是否为米制，对于 DH 参数中 `d6=134.5` 和 `a2=900.0` 等自然为 mm 的值可以正确保留，但对于确实 < 10 的 mm 值（如半径 `5mm`）会错误乘以 1000。此启发式判断不可靠。

**文件**: `include/PalletizingPlanner/CollisionCheckerSO.hpp:261-262`

---

## 3. 中等问题 (Medium)

### 3.1 编译警告未修复（6处）

| 文件 | 行号 | 类型 | 描述 |
|------|------|------|------|
| `testCollisionDetectionTime.cpp` | 114 | `warn_unused_result` | `fscanf` 返回值未检查 |
| `testPerformanceBenchmark.cpp` | 236 | `unused-variable` | `numRepeats` 未使用 |
| `testPalletizingPlanner.cpp` | 171 | `unused-but-set-variable` | `jacobian` 被赋值但未使用 |
| `testPalletizingPlanner.cpp` | 258 | `unused-variable` | `elapsed` 未使用 |
| `testPalletizingPlanner.cpp` | 386 | `unused-variable` | `totalTime` 未使用 |
| `testRobustnessValidation.cpp` | 862 | `unused-but-set-variable` | `isFree` 被赋值但未使用 |

---

### 3.2 `IKResult` 和 `numericalIK` 在两个测试文件中完全重复

`test/testS50PalletizingSO.cpp` 和 `test/testTrajectoryOptimality.cpp` 各自定义了完全相同的 `IKResult` 结构和 `numericalIK` 函数。

**建议**: 提取到公共头文件 (`include/PalletizingPlanner/NumericalIK.hpp`)。

---

### 3.3 `IncrementalKDTree6D::rebuild()` 会破坏迭代中的引用

```cpp
void rebuild() {
    ...
    root_ = buildBalanced(indices, 0, (int)indices.size(), 0);
}
```

`buildBalanced` 通过 `std::nth_element` 重排 `indices` 数组并修改 `nodes_[nIdx].left/right`，但左右子树指针的旧值可能被其他查询线程读取。虽然当前是单线程使用，但 `kdTree_` 被声明为 `mutable`，意味着可能在 `const` 方法中被调用。

**文件**: `include/PalletizingPlanner/PathPlannerSO.hpp:160-181`

---

### 3.4 `EllipsoidSamplerSO` Gram-Schmidt 正交化可能失败

```cpp
for (int i = 1; i < 6; ++i) {
    Eigen::VectorXd v = M.col(i);
    for (int j = 0; j < i; ++j) v -= v.dot(M.col(j)) * M.col(j);
    if (v.norm() > 1e-9) M.col(i) = v.normalized();
    // 如果 v.norm() <= 1e-9, M.col(i) 保持为单位矩阵的列
}
```

**问题**: 当 start 和 goal 沿某个关节轴方向对齐时，Gram-Schmidt 可能产生非正交基，导致椭球采样变形。缺少对退化情况的处理。

**文件**: `include/PalletizingPlanner/PathPlannerSO.hpp:293-298`

---

### 3.5 `TimeParameterization.hpp` 的 S 曲线实现是**近似**而非真正 S 曲线

```cpp
// 五次多项式平滑 (满足速度和加速度边界条件)
double s = utils::smootherStep(tau);
```

**问题**: 文件注释声称 "七段式 S 曲线"，但实际实现是五次多项式 (`smootherStep`)，这只是** quintic Hermite 插值**而非真正的分段S曲线。真正的S曲线应有7段（加加速-匀加速-减加速-匀速-加减速-匀减速-减减速），每段有不同的jerk值。

**影响**: 速度/加速度曲线不完全满足工业级 S 曲线标准，与 `libCmpRML.so` 的实际执行曲线有差异。

**文件**: `include/PalletizingPlanner/TimeParameterization.hpp:270-310`

---

### 3.6 `TimeParameterization::sCurveParameterization` 的时间放大因子 1.875 可能过于保守

```cpp
segmentTime *= 1.875;
```

注释说 "五次多项式 smootherStep 的峰值速度因子为 1.875"，但这导致轨迹时间被放大 87.5%。对于码垛这种时间敏感场景，过于保守的时间缩放会**显著增加节拍时间**。

**文件**: `include/PalletizingPlanner/TimeParameterization.hpp:297`

---

### 3.7 `BSpline::evaluate` 中 `k=n` 的处理可能越界

```cpp
if (t >= 1.0 - 1e-9) k = n;

// De Boor算法
std::vector<JointVector> d(degree + 1);
for (int j = 0; j <= degree; ++j) {
    d[j] = controlPoints[k - degree + j].q;   // k-degree+j 可能 < 0 或 >= size
}
```

当 `t ≈ 1.0` 且 `k = n` 时，`k - degree + 0 = n - degree`。如果 `n < degree`（虽然 `initUniform` 会避免），索引可能无效。此外，`knotVector` 的访问 `knotVector[k + 1 + j - r]` 在 `k=n, j=degree, r=1` 时为 `n + degree`，而 `knotVector.size() = m+1 = n+degree+2`，勉强合法但边界紧凑。

**文件**: `include/PalletizingPlanner/Types.hpp:238-260`

---

### 3.8 `RobotModel::computeFK` 的 DH 模型可能与 URDF 不一致

MATLAB `testS50_Palletizing_v15.m` 中使用 `JOINTS` 数组定义了与 URDF 对应的坐标系变换：
```matlab
JOINTS = [
    0,       0, 0.2833,        0,      0,  pi/2;
   -0.3345,  0, 0,          pi/2,      0, -pi/2;
   ...
```

但 C++ `RobotModel::computeFK` 使用 DH 参数 `d1=296.5mm, d2=336.2mm`，而 URDF 的 `base_link → link1` 距离是 `0.2833m = 283.3mm`（与 `d1=296.5mm` 不同）。

**影响**: MATLAB 仿真的 FK 结果可能与 C++ `.so` FK 结果有微小偏差（虽然 MATLAB 使用了自己的 `urdfFK` 函数）。

**文件**: `include/PalletizingPlanner/RobotModel.hpp:117-178`, `ArmCollisionModel/testS50_Palletizing_v15.m:62`

---

### 3.9 `CollisionChecker.hpp` (静态库) 和 `CollisionCheckerSO.hpp` (动态库) 碰撞距离单位不同

按照 copilot-instructions.md：
- SO 碰撞距离返回值: **mm**
- 静态库碰撞距离返回值: **m**

但 `CollisionReport::selfMinDistance` (静态库) 注释写 `[m]`，而 `CollisionReportSO::selfMinDist_mm` (SO) 注释写 `[mm]`。
`PathOptimizer.hpp` 使用 `CollisionChecker` (静态库)，若混用 SO 栈会导致单位错误。

**文件**: `include/PalletizingPlanner/CollisionChecker.hpp:92`, `include/PalletizingPlanner/CollisionCheckerSO.hpp:115`

---

## 4. 轻微问题 (Minor)

### 4.1 枚举值声明了 `LazyPRM` / `ABITStar` / `STOMP` / `CHOMP` 但完全未实现

```cpp
enum class PlannerType {
    ...
    LazyPRM,       // [未实现 - 预留]
    ABITStar       // [未实现 - 预留]
};
enum class OptimizerType {
    ...
    STOMP,         // [未实现 - 预留]
    CHOMP,         // [未实现 - 预留]
};
```

**影响**: 用户可能选择这些枚举值但不会有任何效果，也没有运行时错误提示。

**文件**: `include/PalletizingPlanner/Types.hpp:344-353`

---

### 4.2 `BSplineFitter::fitLeastSquares` 实际上不是最小二乘拟合

```cpp
// 构造B-Spline基函数矩阵
// 这里使用简化方法：均匀分布控制点
for (int j = 0; j < m; ++j) {
    double t = static_cast<double>(j) / (m - 1);
    // 找到最近的路径点
    ...
    controlPoints[j] = points[nearestIdx];
}
```

**问题**: 函数名为 `fitLeastSquares` 但实际实现只是最近邻采样，并非基于 B-Spline 基函数矩阵的最小二乘优化。这是误导性命名。

**文件**: `include/PalletizingPlanner/PathOptimizer.hpp:397-438`

---

### 4.3 `PathOptimizer::fitWithCurvatureOptimization` — TODO 未完成

```cpp
BSpline fitWithCurvatureOptimization(...) {
    BSpline spline = fitLeastSquares(points, numControlPoints);
    // TODO: 添加曲率优化迭代
    return spline;
}
```

**文件**: `include/PalletizingPlanner/PathOptimizer.hpp:445-453`

---

### 4.4 `PlannerConfig::enableParallel` 和 `numThreads` 字段在所有规划器中均未使用

```cpp
bool enableParallel = true;    // 启用并行计算
int numThreads = 4;            // 线程数
```

`PathPlannerSO`、`PathPlanner`、`PathPlannerOptimized` 均为单线程实现，这些配置字段无效。

**文件**: `include/PalletizingPlanner/Types.hpp:393-394`

---

### 4.5 `CMakeLists.txt` 顶层文件逻辑过于简单

```cmake
cmake_minimum_required(VERSION 3.14)
project(HRC-PalletizingPlanner LANGUAGES CXX)
add_compile_options(-fPIC -Wall -Wextra -Wno-unused-parameter -Wno-attributes)
...
add_subdirectory(test)
```

**缺失**:
1. 没有 `find_package(Eigen3 REQUIRED)` — 直接硬编码 `/usr/include/eigen3`
2. 没有 install target
3. 没有 CTest 集成 (`enable_testing()` + `add_test()`)
4. 没有 Release/Debug 构建类型区分（缺少 `-O2` / `-DNDEBUG`）
5. `examples/` 的编译定义在 `test/CMakeLists.txt` 中而非独立的 CMakeLists

**文件**: `CMakeLists.txt`, `test/CMakeLists.txt`

---

### 4.6 `RobotModel::computeJacobian` 使用数值差分而非解析

```cpp
const double eps = 1e-8;
// 数值微分计算雅可比
```

对于已知 DH 参数的6自由度机器人，解析雅可比矩阵可以精确计算且更快。数值差分不仅精度受 `eps` 选择影响，还需要额外 6 次 FK 调用。

**文件**: `include/PalletizingPlanner/RobotModel.hpp:195-225`

---

### 4.7 `BSpline::derivative` 使用有限差分近似，精度受限

```cpp
double h = 1e-6;
if (order == 1) {
    auto p1 = evaluate(t + h);
    auto p0 = evaluate(t - h);
    return (p1.q - p0.q) / (2 * h);
}
```

B-Spline 导数应直接通过差分控制点和降阶节点向量精确计算，而非数值差分。高阶导数递归调用使得误差放大。

**文件**: `include/PalletizingPlanner/Types.hpp:269-303`

---

### 4.8 `generateSmoothControlPoints` 是空操作

```cpp
std::vector<JointConfig> generateSmoothControlPoints(
        const std::vector<JointConfig>& pathPoints) {
    std::vector<JointConfig> controlPoints;
    controlPoints.push_back(pathPoints.front());
    for (size_t i = 1; i < pathPoints.size() - 1; ++i) {
        controlPoints.push_back(pathPoints[i]);
    }
    controlPoints.push_back(pathPoints.back());
    return controlPoints;
}
```

**问题**: 注释写 "使用 Catmull-Rom 到 B-Spline 转换思想"，但实际只是原样拷贝了所有路径点，没有任何平滑处理。

**文件**: `include/PalletizingPlanner/PathOptimizer.hpp:346-363`

---

### 4.9 `TimingStats::toString` 中缓存命中率永远为 0%

SO 栈没有实现碰撞缓存（`CollisionCache` 仅用于静态库栈），但 `TimingStats::cacheHits` 默认为 0，打印时显示 `0.0%`。这对用户有误导性。

**文件**: `include/PalletizingPlanner/CollisionCheckerSO.hpp:87-96`

---

### 4.10 `PlannerConfig::collisionResolution` 默认值 0.02 rad (≈1.1°) 可能不足

对于 S50 臂展 ~1.8m 的机器人，0.02 rad 的步进在末端可产生约 36mm 的位移，对于某些紧凑环境可能遗漏碰撞。

**文件**: `include/PalletizingPlanner/Types.hpp:385`

---

## 5. 架构与设计问题

### 5.1 静态库栈和 SO 栈的代码严重重复

| 功能 | 静态库栈文件 | SO 栈文件 | 重复度 |
|------|-------------|----------|--------|
| 碰撞检测 | `CollisionChecker.hpp` (902行) | `CollisionCheckerSO.hpp` (727行) | ~60% |
| 路径规划 | `PathPlanner.hpp` + `PathPlannerOptimized.hpp` | `PathPlannerSO.hpp` (914行) | ~50% |
| 路径优化 | `PathOptimizer.hpp` + `PathOptimizerOptimized.hpp` | `PathPlannerSO::optimizePath()` | ~70% |
| KD-Tree | `KDTree.hpp` | `PathPlannerSO::IncrementalKDTree6D` | ~40% |

**影响**: 修Bug需要改多处，功能不一致的风险高。

---

### 5.2 Header-only 库设计导致编译时间长

所有 `.hpp` 文件包含完整实现，每个测试文件编译都需要处理所有模板和内联代码。建议关键类使用 `.hpp` + `.cpp` 分离，或使用 inline 控制。

---

### 5.3 缺少统一的碰撞检测接口抽象

`CollisionChecker` 和 `CollisionCheckerSO` 有相似但不兼容的 API：
- `CollisionChecker::isPathCollisionFree(..., double resolution = 0.02)`
- `CollisionCheckerSO::isPathCollisionFree(..., double resolution = 0.02)`

但它们不继承自公共基类，`PathOptimizer` 硬绑定了 `CollisionChecker`，无法与 `CollisionCheckerSO` 一起使用。

---

### 5.4 `PalletizingPlanner` 顶层 API 只支持静态库栈

`PalletizingPlanner.hpp` 构造函数创建 `PathPlannerOptimized` 和 `PathOptimizer`，均依赖 `CollisionChecker` (静态库)。没有对应的 SO 版本顶层 API。

**文件**: `include/PalletizingPlanner/PalletizingPlanner.hpp`

---

### 5.5 MATLAB 仿真与 C++ 的数据耦合脆弱

MATLAB v15 依赖 C++ 输出的特定文件格式：
- `data/so_palletizing_trajectory.txt` (19列)
- `data/so_palletizing_profile.csv` (16列)
- `data/so_palletizing_summary.txt`

如果 C++ 输出格式改变，MATLAB 代码会静默产生错误结果而非报错。缺少版本号检查。

---

## 6. 测试运行结果分析

### SO 栈测试 (✅ 正常工作)

| 测试 | 结果 | 备注 |
|------|------|------|
| `testS50PalletizingSO` | ✅ 通过 | 12箱码垛, 0碰撞, 最小距离10.5mm, 总耗时0.09s |
| `testTrajectoryOptimality` | ⚠️ 基本通过 | TestF首次异常负时间, 路径零方差 |
| `testS50CollisionSO` | ✅ 通过 | 碰撞检测功能正常 |

### 静态库栈测试 (❌ 严重问题)

| 测试 | 结果 | 备注 |
|------|------|------|
| `testHighPerformance` | ❌ 0% 成功率 | 50次全部失败 |
| `testPalletizingPlanner` | ❌ 10% 成功率 | 碰撞检测误判 |
| `testRobustnessValidation` | ❌ 超时 | >60s无输出 |
| `testPerformanceBenchmark` | ✅ 通过 | 仅基准测试, 不涉及实际规划 |

### 需要额外环境的测试

| 测试 | 依赖 | 状态 |
|------|------|------|
| `testRMLProbe` | `libCmpRML.so` + PLC运行时 | 未测试 |
| `testS50CollisionRML` | 静态库 + RML | 未测试 |
| `testS50PalletizingRML` | 静态库 + RML | 未测试 |

---

## 7. 文档与维护问题

### 7.1 `copilot-instructions.md` 中性能基准已过时

文档声称：
```
| 简单场景规划 | 0.04 ms |
| 完整流水线   | 135 ms  |
```

但实际 SO 栈码垛仿真总耗时 0.09s (90ms)，且规划时间始终显示 0.0ms（因为全是 P2P 直达），这些基准数字不反映实际复杂场景性能。`testTrajectoryOptimality` 中单次规划实测约 1.2-1.4 秒。

---

### 7.2 项目中存在废弃/冗余目录

- `ArmCollisionModelaa/` — 非标准命名，内含 README 和空 output/scripts 目录
- `test/deprecated/` — 废弃测试文件
- `data/` 目录中有大量中间输出文件（40+），缺少 `.gitignore` 管理

---

### 7.3 `README.md` 的构建说明中 SO 栈用法信息不足

README 中只提到：
```bash
HRC_LIB_PATH=/path/to/libHRCInterface.so ./bin/testS50PalletizingSO
```

但未说明：
1. `.so` 文件的获取方式
2. `.so` 不在 `lib/` 目录中的事实
3. 真实路径 `/home/ara/文档/collision/HansAlgorithmExport/bin/libHRCInterface.so`
4. 需要哪些运行时依赖 (如 glibc 版本)

---

### 7.4 Python 可视化脚本的 FK 可能与 C++/MATLAB 不一致

`scripts/visualize_palletizing.py` 中有注释 `# 简化的DH模型 (UR类型)`，其 FK 实现未经验证是否与 C++ `RobotModel::computeFK` 一致。

**文件**: `scripts/visualize_palletizing.py:48`

---

## 总结

| 严重程度 | 数量 | 关键词 |
|---------|------|--------|
| **Critical** | 5 | 测试0%成功率、缺少.so文件、超时 |
| **Major** | 9 | 负时间、代价不一致、代码重复、单位混乱 |
| **Medium** | 10 | 编译警告、近似S曲线、越界风险 |
| **Minor** | 10 | 未实现枚举、空函数、误导命名 |
| **架构** | 5 | 栈重复、缺少抽象、耦合脆弱 |
| **文档维护** | 4 | 过时基准、冗余目录 |

**优先修复建议**:
1. 调查静态库栈为何碰撞检测全面失败 → 可能是初始化参数问题
2. 将 `libHRCInterface.so` 复制到 `lib/` 目录或建立符号链接
3. 统一 SO/静态库碰撞检测接口为公共抽象基类
4. 修复 `getTimingStats()` 的数据竞争
5. 修复 `propagateCostUpdate` 的代价一致性问题
6. 清除编译警告
