# HR_S50-2000 码垛仿真系统 — 完整流程详解

> **版本**: v3.0 / v13.0  
> **机器人**: HR_S50-2000 (6-DOF, 50kg 载荷, ~2m 臂展)  
> **日期**: 2026-02-24  
> **Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.**

---

## 目录

1. [系统架构概览](#1-系统架构概览)
2. [机器人模型与参数](#2-机器人模型与参数)
3. [碰撞检测系统](#3-碰撞检测系统)
4. [C++ 后端流水线 (testS50PalletizingSO.cpp)](#4-c-后端流水线)
5. [MATLAB 前端可视化 (testS50_Palletizing_v13.m)](#5-matlab-前端可视化)
6. [数据文件格式](#6-数据文件格式)
7. [性能分析与实测数据](#7-性能分析与实测数据)
8. [已知问题与改进方向](#8-已知问题与改进方向)

---

## 1. 系统架构概览

### 1.1 三层架构

整个仿真系统由三层构成：

```
┌─────────────────────────────────────────────────────────────┐
│                  MATLAB 可视化前端 (v13)                      │
│  loadlibrary .so → STL网格 → 碰撞验证 → 9张图表 + GIF动画    │
├─────────────────────────────────────────────────────────────┤
│                   数据交换层 (文本文件)                        │
│  so_palletizing_trajectory.txt    19列 空格分隔               │
│  so_palletizing_profile.csv       15列 逗号分隔               │
│  so_collision_trajectory.txt      27列 空格分隔               │
│  so_palletizing_summary.txt       key: value 键值对           │
│  so_collision_summary.txt         key: value 键值对           │
├─────────────────────────────────────────────────────────────┤
│                   C++ 后端流水线 (v3.0)                       │
│  RobotModel → CollisionCheckerSO (.so) → PathPlannerSO       │
│  (TCP-aware Informed RRT*) → TimeParameterizer (S-curve)      │
└─────────────────────────────────────────────────────────────┘
```

### 1.2 端到端数据流

```
用户定义码垛布局 (12个位置, 度数)
         │
         ▼
  ┌─── C++ 后端 ───────────────────────────────────────┐
  │  1) 初始化: RobotModel + CollisionCheckerSO + Planner │
  │  2) TSP序列优化: 贪心+2-opt 确定任务顺序               │
  │  3) 逐任务执行 (每任务7段运动):                         │
  │     ├─ 长距段 → TCP-aware Informed RRT*                │
  │     └─ 短距段 → 直线P2P                                │
  │  4) 每段: 路径→S曲线参数化(4ms/250Hz)→碰撞监测         │
  │  5) 输出: trajectory.txt + profile.csv + summary.txt    │
  └────────────────────────────────────────────────────┘
         │
    文本文件 (rad/deg, 空格/逗号分隔)
         │
         ▼
  ┌─── MATLAB v13 ─────────────────────────────────────┐
  │  1) loadlibrary 加载 libHRCInterface.so              │
  │  2) 加载 STL 网格 (含70%降面优化)                     │
  │  3) 读取 C++ 输出的轨迹/碰撞数据                      │
  │  4) .so实时碰撞验证 (对比C++预计算结果)               │
  │  5) 全轨迹扫描: 碰撞距离 + TCP姿态(FK)分析            │
  │  6) 生成9张PNG + 1个GIF动画                           │
  │  7) 性能分层计时报告                                   │
  └────────────────────────────────────────────────────┘
```

---

## 2. 机器人模型与参数

### 2.1 基本参数

| 参数 | 值 |
|------|-----|
| 型号 | HR_S50-2000 |
| 自由度 | 6-DOF |
| 载荷 | 50 kg |
| 臂展 | ~2000 mm |
| 类型代码 (robType) | 1 (S-Serial) |
| 控制周期 | 4 ms (250 Hz) |

### 2.2 DH 参数 (标准DH, 单位: mm)

| 参数 | 值 | 说明 |
|------|-----|------|
| d1 | 296.5 | 基座到关节1偏移 |
| d2 | 336.2 | 关节1到关节2偏移 |
| d3 | 239.0 | 关节2到关节3偏移 |
| d4 | 158.5 | 关节3到关节4偏移 |
| d5 | 158.5 | 关节4到关节5偏移 |
| d6 | 134.5 | 关节5到关节6偏移 |
| a2 | 900.0 | 下臂连杆长度 |
| a3 | 941.5 | 上臂连杆长度 |

DH参数数组在C++和MATLAB中的定义：
```cpp
// C++ (CollisionCheckerSO 初始化)
double dh[8] = {296.5, 336.2, 239.0, 158.5, 158.5, 134.5, 900.0, 941.5};
```
```matlab
% MATLAB (v13第35行)
DH_MM = [296.5, 336.2, 239.0, 158.5, 158.5, 134.5, 900.0, 941.5];
```

### 2.3 URDF 关节参数

MATLAB 中使用 URDF 风格的关节定义进行正运动学计算（单位：m, rad）：

| 关节 | x | y | z | roll | pitch | yaw |
|------|------|------|--------|------|-------|------|
| J1 | 0 | 0 | 0.2833 | 0 | 0 | π/2 |
| J2 | -0.3345 | 0 | 0 | π/2 | 0 | -π/2 |
| J3 | -0.9 | 0 | -0.239 | 0 | 0 | π |
| J4 | 0.9415 | 0 | 0 | 0 | 0 | 0 |
| J5 | 0 | 0 | 0.1585 | -π/2 | 0 | 0 |
| J6 | 0 | 0 | 0.1585 | π/2 | 0 | 0 |

FK 算法：
```matlab
function T_all = urdfFK(JOINTS, q_rad)
    T_all = cell(7,1);
    T_all{1} = eye(4);                       % 基座
    for i = 1:6
        xyz = JOINTS(i,1:3);  rpy = JOINTS(i,4:6);
        T_all{i+1} = T_all{i} * Trans(xyz) * RotRPY(rpy) * RotZ(q_rad(i));
    end
end
% T_all{7} = TCP在基坐标系下的4×4齐次变换矩阵
```

### 2.4 关节限位

| 关节 | 范围 (deg) |
|------|------------|
| J1 | ±360 |
| J2 | -190 ~ +10 |
| J3 | ±165 |
| J4 | ±360 |
| J5 | ±360 |
| J6 | ±360 |

### 2.5 碰撞几何模型

碰撞检测使用简化的胶囊体(Capsule)和球体(Ball)包围体，单位为 **mm**，在连杆**局部坐标系**中定义：

| 部位 | 类型 | 参数 [sx,sy,sz, ex,ey,ez, radius] 或 [cx,cy,cz, radius] | 半径(mm) |
|------|------|-----------------------------------------------------------|----------|
| 基座 (base) | 胶囊体 | [0, 0, 20, 0, 0, 330, 160] | 160 |
| 下臂 (lowerArm) | 胶囊体 | [0, 0, 340, 900, 0, 340, 140] | 140 |
| 肘部 (elbow) | 胶囊体 | [-10, 0, 60, 941.5, 0, 60, 120] | 120 |
| 上臂 (upperArm) | 胶囊体 | [0, 0, -50, 0, 0, 100, 100] | 100 |
| 腕部 (wrist) | 球体 | [0, 0, 20, 140] | 140 |

### 2.6 STL 网格模型

可视化使用精确的 STL 网格模型，存放在 `ArmCollisionModel/model/meshes/S50/`：

| 文件名 | 原始面数 | 低模面数 | 降面率 | 说明 |
|--------|---------|---------|--------|------|
| elfin_base.STL | - | - | ~70% | 基座 |
| elfin_link1.STL | - | - | ~70% | 连杆1 |
| elfin_link2.STL | - | - | ~70% | 连杆2 |
| elfin_link3.STL | - | - | ~70% | 连杆3 |
| elfin_link4.STL | - | - | ~70% | 连杆4 |
| elfin_link5.STL | - | - | ~70% | 连杆5 |
| elfin_link6.STL | - | - | ~70% | 连杆6 |
| **合计** | **73,076** | **21,918** | **70%** | 7个连杆 |

- STL文件单位为 **mm**，加载时自动检测(`maxCoord > 10`)并换算为 **m**
- 低模通过 MATLAB `reducepatch()` 生成，目标为原面数的 30%（`MESH_REDUCE_RATIO = 0.3`）
- 原始模型用于静态图渲染，低模用于动画以提高帧率

---

## 3. 碰撞检测系统

### 3.1 碰撞库来源

碰撞检测核心来自华数机器人闭源预编译共享库：

```
/home/ara/文档/collision/HansAlgorithmExport/bin/libHRCInterface.so
```

该库为 CDS (Collision Detection System) 层接口，通过 `extern "C"` 导出 C-linkage 函数。

### 3.2 C++ 端调用方式

C++ 后端通过 `dlopen` / `dlsym` 延迟加载：

```cpp
// CollisionCheckerSO 内部使用 dlopen(RTLD_LAZY) 加载
void* handle = dlopen("libHRCInterface.so", RTLD_LAZY);
// RTLD_LAZY: 延迟符号解析 — 只有被调用的函数才会被解析
// 这规避了 initTCPPosition/updateTCPPosition 等未实现符号
```

### 3.3 MATLAB 端调用方式

MATLAB 使用 `loadlibrary` 加载，但其内部使用 `RTLD_NOW`（立即符号解析），导致必须为缺失的符号提供桩函数：

```matlab
% 1. 需要 LD_PRELOAD 预加载桩库
%    LD_PRELOAD=/path/to/s50_tcp_stubs.so matlab
% 
% 2. 加载 .so
loadlibrary(SO_PATH, SO_HEADER, 'alias', 'libHRCInterface');
%
% 3. SO_HEADER = s50_collision_matlab.h (简化C头文件，供MATLAB解析)
```

桩函数 (`s50_tcp_stubs.c`)：
```c
void initTCPPosition(void) { }      // 声明但从未实现的函数
void updateTCPPosition(void) { }    // 同上
```

### 3.4 核心API调用序列

#### 3.4.1 初始化 (调用一次)

```c
// 1. 初始化碰撞算法包
initACAreaConstrainPackageInterface(
    robType,        // SO_INT: 1 = S-Serial (S50)
    dh[8],          // 8×SO_LREAL: DH参数 (mm)
    baseGeo[7],     // 基座胶囊体 (mm)
    lowerArmGeo[7], // 下臂胶囊体 (mm)
    elbowGeo[7],    // 肘部胶囊体 (mm)
    upperArmGeo[7], // 上臂胶囊体 (mm)
    wristGeo[4],    // 腕部球体 (mm)
    initJoint[6]    // 初始关节角 (deg)
);

// 2. 启用全部碰撞检测连杆对
setCPSelfColliderLinkModelOpenStateInterface(flags[3]);  // [1,1,1] = 全开
```

#### 3.4.2 实时碰撞查询 (每周期调用)

```c
// Step 1: 更新机器人状态 (必须在查询前调用)
updateACAreaConstrainPackageInterface(
    jointPos[6],    // 关节位置 (deg)
    jointVel[6],    // 关节速度 (deg/s)
    jointAcc[6]     // 关节加速度 (deg/s²)
);

// Step 2: 查询自碰撞
SO_BOOL result = checkCPSelfCollisionInterface(
    &pair[2],       // [输出] 碰撞连杆对索引 (SO_LINT)
    &dist           // [输出] 最小碰撞距离 (SO_LREAL, mm)
);
// result: 0 = 无碰撞, 1 = 有碰撞
// dist: 最近连杆对之间的最小距离 (mm)
```

#### 3.4.3 正运动学 (TCP位姿)

```c
// 通过.so内置FK计算TCP位姿
MC_COORD_REF tcp;  // {X, Y, Z, A, B, C} (mm, deg)
forwardKinematics2(jointPos[6], &tcp);
// X,Y,Z: TCP在基坐标系下的位置 (mm)
// A,B,C: TCP姿态角 (deg)
```

#### 3.4.4 碰撞几何可视化查询

```c
// 获取当前碰撞体位姿 (用于可视化叠加)
getUIInfoMationInterface(
    collIdx[7],      // [输出] 碰撞体索引 (SO_DINT)
    collType[7],     // [输出] 类型: 1=球, 2=胶囊 (SO_DINT)
    dataList[63],    // [输出] 9×7 世界坐标数据 (SO_LREAL)
    radiusList[7]    // [输出] 半径 (SO_LREAL, m)
);
// dataList reshape 为 7×9 矩阵:
//   胶囊: [p1x,p1y,p1z, p2x,p2y,p2z, ...]
//   球体: [cx,cy,cz, ...]
```

### 3.5 类型映射

| .so 内部类型 | C类型定义 | MATLAB映射 | 位宽 |
|-------------|-----------|------------|------|
| SO_LREAL | double | double | 64 bit |
| SO_BOOL | signed char (int8) | int8 | 8 bit |
| SO_INT | short | int16 | 16 bit |
| SO_LINT | long long | int64 | 64 bit |
| SO_DINT | int | int32 | 32 bit |
| MC_COORD_REF | struct{6×double} | libstruct | 48 byte |

### 3.6 单位约定汇总

| 接口 / 上下文 | 关节角 | 坐标/距离 |
|---------------|--------|----------|
| 用户 API (JointConfig::fromDegrees) | **deg** | — |
| 内部存储 / 路径文件 | **rad** | — |
| HRC .so `updateACAreaConstrainPackageInterface` | **deg** | — |
| HRC .so `checkCPSelfCollisionInterface` 输出 | — | **mm** |
| HRC .so `forwardKinematics2` 输出 | **deg** (A,B,C) | **mm** (X,Y,Z) |
| DH参数 / 碰撞几何定义 | — | **mm** |
| SceneConfig / 场景障碍物 | — | **m** |
| STL 文件源 | — | **mm** (自动→m) |
| 轨迹文件输出 | **deg** | **mm** |

---

## 4. C++ 后端流水线

> 源文件: `test/testS50PalletizingSO.cpp` (714 行)

### 4.1 总体流程

```
┌──────────────────────────────────────────────────────┐
│               阶段1: 系统初始化                        │
│  RobotModel → CollisionCheckerSO(.so) → PathPlannerSO │
│  → TimeParameterizer(S-curve)                          │
├──────────────────────────────────────────────────────┤
│               阶段2: 码垛布局定义                      │
│  HOME位 + 安全过渡位 + 取料位 + 12个放料位              │
├──────────────────────────────────────────────────────┤
│               阶段3: TSP序列优化                       │
│  贪心最近邻 + 2-opt 局部搜索                           │
├──────────────────────────────────────────────────────┤
│               阶段4: 码垛仿真执行                      │
│  HOME→安全(P2P) → 12任务×7段 → 返回HOME(RRT*)         │
├──────────────────────────────────────────────────────┤
│               阶段5: 统计报告 + 文件输出                │
│  分层计时 + 逐层分析 + 碰撞统计 + 文件写入              │
└──────────────────────────────────────────────────────┘
```

### 4.2 阶段1: 系统初始化 (详解)

#### 4.2.1 机器人模型初始化

```cpp
RobotModel robot;
// 内部初始化 DH 参数、关节限位
// 提供 FK/IK 计算接口和参数查询
```

#### 4.2.2 碰撞检测器初始化

```cpp
CollisionCheckerSO checker(robot);
bool collisionOk = checker.initialize();
// 内部流程:
// 1. dlopen("libHRCInterface.so", RTLD_LAZY) 加载共享库
// 2. dlsym 获取函数指针: initACArea..., updateACArea..., checkCPSelfCollision..., forwardKinematics2
// 3. 调用 initACAreaConstrainPackageInterface(1, dh, base, lowerArm, elbow, upperArm, wrist, homeJoint)
// 4. 调用 setCPSelfColliderLinkModelOpenStateInterface([1,1,1]) 全开碰撞对
// 耗时: ~1.9 ms
```

#### 4.2.3 路径规划器配置

```cpp
TCPPlannerConfig planConfig;
planConfig.maxIterations     = 3000;    // RRT*最大迭代
planConfig.maxPlanningTime   = 1.2;     // 单段规划超时 (s)
planConfig.stepSize          = 0.15;    // RRT*步长 (rad)
planConfig.goalBias          = 0.2;     // 目标偏置采样概率 (20%)
planConfig.tcpPoseWeight     = 0.3;     // TCP姿态代价权重 (30%)
planConfig.shortcutIterations = 60;     // 快捷路径优化迭代数
planConfig.splineResolution  = 35;      // B-Spline分辨率
planConfig.collisionResolution = 0.03;  // 碰撞检测分辨率 (rad)

// TCP姿态约束 (码垛场景)
planConfig.constrainTcpOrientation = true;
planConfig.desiredTcpAxis = Eigen::Vector3d(0, 0, -1);  // Z轴朝下
planConfig.orientTolerance_deg = 45.0;                    // 允许45°偏差

PathPlannerSO planner(robot, checker, planConfig);
```

#### 4.2.4 轨迹参数化器

```cpp
auto tpConfig = TimeParameterizationConfig::fromRobotParams(robot.getParams());
tpConfig.profileType    = VelocityProfileType::SCurve;  // 七段S曲线
tpConfig.samplePeriod   = 0.004;                         // 4ms (250Hz)
tpConfig.velocityScaling = 1.0;                          // 100%速度
TimeParameterizer parameterizer(tpConfig);
```

### 4.3 阶段2: 码垛布局定义

#### 4.3.1 关键姿态

| 姿态 | 关节角 (deg) | 用途 |
|------|-------------|------|
| HOME | [0, -90, 0, 0, 90, 0] | 起始/终止位 |
| SAFE_TRANSIT | [0, -70, 40, 0, 30, 0] | 安全过渡位 |
| PICK_APPROACH | [-60, -40, 80, 0, -40, -60] | 取料接近位 |
| PICK_POS | [-60, -30, 90, 0, -60, -60] | 取料位 |

#### 4.3.2 放料位 (12个 = 3层 × 2行 × 2列)

| 位置编号 | 标签 | 接近位 (deg) | 放料位 (deg) |
|----------|------|-------------|-------------|
| 0 | L1-R1-C1 | [50,-50,70,0,-20,50] | [50,-40,80,0,-40,50] |
| 1 | L1-R1-C2 | [70,-50,70,0,-20,70] | [70,-40,80,0,-40,70] |
| 2 | L1-R2-C1 | [50,-50,70,30,-20,50] | [50,-40,80,30,-40,50] |
| 3 | L1-R2-C2 | [70,-50,70,30,-20,70] | [70,-40,80,30,-40,70] |
| 4 | L2-R1-C1 | [50,-55,65,0,-10,50] | [50,-45,75,0,-30,50] |
| 5 | L2-R1-C2 | [70,-55,65,0,-10,70] | [70,-45,75,0,-30,70] |
| 6 | L2-R2-C1 | [50,-55,65,30,-10,50] | [50,-45,75,30,-30,50] |
| 7 | L2-R2-C2 | [70,-55,65,30,-10,70] | [70,-45,75,30,-30,70] |
| 8 | L3-R1-C1 | [50,-60,60,0,0,50] | [50,-50,70,0,-20,50] |
| 9 | L3-R1-C2 | [70,-60,60,0,0,70] | [70,-50,70,0,-20,70] |
| 10 | L3-R2-C1 | [50,-60,60,30,0,50] | [50,-50,70,30,-20,50] |
| 11 | L3-R2-C2 | [70,-60,60,30,0,70] | [70,-50,70,30,-20,70] |

**层间命名规则**: L{层号}-R{行号}-C{列号}
- 第1层 (L1): position 0-3, J2 ≈ -40~-50°
- 第2层 (L2): position 4-7, J2 ≈ -45~-55°
- 第3层 (L3): position 8-11, J2 ≈ -50~-60°

### 4.4 阶段3: TSP 序列优化

目的：确定12个放料位的执行顺序，使总关节空间移动距离最短。

#### 4.4.1 贪心最近邻 (初始解)

```
算法:
1. 起点 = HOME
2. 对每一步:
   a. 遍历所有未访问的放料位
   b. 选择关节空间 L2 距离最小的
   c. 标记为已访问, 移动到该位置
3. 输出访问顺序
```

度量函数: `JointConfig::distanceTo()` — 6维关节空间欧氏距离 (rad)

#### 4.4.2 2-opt 局部优化

```
算法:
repeat:
  improved = false
  for i = 0..n-2:
    for j = i+2..n-1:
      d1 = dist(order[i]→order[i+1]) + dist(order[j]→order[j+1])
      d2 = dist(order[i]→order[j])   + dist(order[i+1]→order[j+1])
      if d2 < d1 - ε:
        reverse(order[i+1..j])  // 翻转子序列
        improved = true
until not improved
```

### 4.5 阶段4: 码垛仿真执行 (核心)

#### 4.5.1 总体运动结构

```
HOME → SAFE_TRANSIT (P2P)
│
├── 任务 1 (7段运动):  
│   ├── Seg 0: 安全→取料接近    [RRT*] — 长距, 需避障
│   ├── Seg 1: 取料接近→取料    [P2P]  — 短距, 直线
│   ├── Seg 2: 取料→取料抬升    [P2P]  — 短距, 直线
│   ├── Seg 3: 取料抬升→安全    [RRT*] — 长距, 需避障
│   ├── Seg 4: 安全→放料接近    [RRT*] — 长距, 需避障
│   ├── Seg 5: 放料接近→放料    [P2P]  — 短距, 直线
│   └── Seg 6: 放料→放料抬升    [P2P]  — 短距, 直线
│
├── 任务 2~12: 同上结构
│
└── 最后一个放料位 → HOME (RRT*)
```

**运动段总数**: 1(HOME→安全) + 12×7(任务) + 1(返回HOME) = **86段**

#### 4.5.2 P2P 执行流程 (executeP2P)

用于短距运动（取料接近→取料、放料接近→放料等）：

```
输入: start_config, target_config
│
├── 1. 构建直线路径
│   path = {Waypoint(start, param=0.0), Waypoint(target, param=1.0)}
│
├── 2. S曲线参数化
│   Trajectory traj = parameterizer.parameterize(path)
│   // 七段S曲线: 加加速→匀加速→减加速→匀速→加减速→匀减速→减减速
│   // 采样周期: 4ms (250Hz)
│   // 输出: 每个时间步的 {config, velocity, acceleration, time}
│
├── 3. 碰撞检测 (每10步采样)
│   for i = 0, 10, 20, ..., traj.size():
│     report = checker.getCollisionReport(traj[i].config, withFK=true)
│     // 调用 updateACArea... + checkCPSelfCollision... + forwardKinematics2
│     记录: selfMinDist_mm, selfCollision, tcpPose
│
├── 4. 数据输出
│   每10步 → CSV (15列)
│   每20步 → 轨迹文件 (19列, 含FK TCP)
│
└── 输出: SegmentResult {totalTime_s, minSelfDist_mm, collisionCount, ...}
```

#### 4.5.3 RRT* 执行流程 (executeRRTStar)

用于长距运动（安全位→取料接近、安全位→放料接近等）：

```
输入: start_config, target_config
│
├── 1. 快速直通检查
│   if checker.isPathCollisionFree(start, target, resolution=0.03):
│     // 起终点可直线无碰撞到达, 跳过RRT* ← 极大加速!
│     path = {Waypoint(start), Waypoint(target)}
│     跳至S曲线参数化
│
├── 2. TCP-aware Informed RRT* 规划 (仅当直通失败)
│   planResult = planner.plan(start, target)
│   │
│   ├── 2a. 核心循环 (最多3000次迭代, 超时1.2s):
│   │   ├── 随机采样 (80%随机 + 20%目标偏置)
│   │   ├── KD-Tree 最近邻搜索
│   │   ├── 碰撞检测 (是否可达)
│   │   ├── TCP姿态代价评估 (权重30%):
│   │   │   cost = (1-w)*pathCost + w*tcpOrientCost
│   │   │   tcpOrientCost = acos(dot(tcpZaxis, [0,0,-1]))
│   │   ├── 最优父节点选择 + 重连
│   │   └── 目标到达检测
│   │
│   ├── 2b. 路径后处理:
│   │   ├── 快捷路径优化 (60次迭代): 随机选两点, 若可直连则删除中间段
│   │   ├── B-Spline平滑 (De Boor算法, 分辨率35)
│   │   └── 碰撞验证 (分辨率0.03 rad)
│   │
│   └── 2c. 分层计时输出:
│       采样 | 邻搜 | 碰撞 | TCP代价 | 重连 | 捷径 | B曲线 | 验证
│
├── 3. 路径回退策略
│   optimizedPath有效 → 使用优化路径
│   rawPath有效 → 使用原始路径
│   全部失败 → 回退到直线P2P (保证可达性)
│
├── 4. S曲线参数化 (同P2P)
│
├── 5. 碰撞检测 (同P2P, 每10步)
│
└── 6. 数据输出 (同P2P)
```

#### 4.5.4 RRT* 分层计时报告 (每段输出)

对于每个RRT*规划段，输出细粒度计时：

```
— RRT*分层: 采样X.Xms 邻搜X.Xms 碰X.Xms TCPX.Xms 重连X.Xms | 捷径X.Xms B曲线X.Xms 验证X.Xms | iter=N nodes=N fk=N coll=N
```

| 计时项 | 说明 |
|--------|------|
| sampling_ms | 随机采样耗时 |
| nearestSearch_ms | KD-Tree 最近邻搜索 |
| collisionCheck_ms | 碰撞检测 (RRT*内部) |
| tcpCostEval_ms | TCP姿态代价计算 |
| rewiring_ms | RRT*重连操作 |
| shortcut_ms | 快捷路径优化 |
| bspline_ms | B-Spline 平滑 |
| validation_ms | 最终碰撞验证 |

### 4.6 阶段5: 统计报告

#### 4.6.1 全流水线分层计时

| 层级 | 典型耗时 | 说明 |
|------|---------|------|
| 碰撞.so初始化 | 1.9 ms | dlopen + initACArea |
| RRT*规划+优化 | 700.7 ms | 所有段合计 |
| S曲线参数化 | 388.8 ms | 所有段合计 |
| 碰撞运行时监测 | 397.6 ms | 28,798次调用 |
| **总计(含IO)** | **1.519 s** | 端到端 |

#### 4.6.2 碰撞检测统计

| 指标 | 值 |
|------|-----|
| 初始化耗时 | 1.915 ms |
| update平均耗时 | 6.991 μs/call |
| selfCheck平均耗时 | 0.336 μs/call |
| FK平均耗时 | 0.783 μs/call |
| 总计平均耗时 | 7.861 μs/call |
| 总调用次数 | 28,798 |

#### 4.6.3 逐层分析

| 层 | 任务数 | 运动时间 | 规划耗时(平均) | 参数化(平均) | 碰撞监测(平均) |
|----|--------|---------|--------------|------------|--------------|
| L1 | 4 | - | - | - | - |
| L2 | 4 | - | - | - | - |
| L3 | 4 | - | - | - | - |

---

## 5. MATLAB 前端可视化

> 源文件: `ArmCollisionModel/testS50_Palletizing_v13.m` (1,574 行)

### 5.1 总体流程

```
┌──────────────────────────────────────────────────────────┐
│  1. 系统初始化: 字体检测 + Headless模式 + 路径设置          │
│                                                            │
│  2. 加载碰撞.so: loadlibrary → initACArea → HOME验证       │
│                                                            │
│  3. 加载STL网格: 7个连杆 + 70%降面(reducepatch)             │
│                                                            │
│  4. 加载C++数据: 轨迹文件 + 碰撞文件 + 摘要文件             │
│                                                            │
│  5. 碰撞验证(关键姿态): C++ vs .so 逐场景对比               │
│                                                            │
│  6. 全轨迹扫描: 每5步 update+check+FK → 碰撞距离+TCP方向    │
│                                                            │
│  7. 9张图表生成 + 1个GIF动画                                │
│                                                            │
│  8. 性能报告: 分层计时 + 碰撞统计 + 优化效果                │
└──────────────────────────────────────────────────────────┘
```

### 5.2 步骤详解

#### 5.2.1 系统初始化

```matlab
% 中文字体检测 (优先级)
CJK_FONT 候选列表 = {
    'Noto Sans CJK SC',      % 首选
    'Noto Serif CJK SC',
    'SimHei',
    'WenQuanYi Micro Hei',
    'Arial Unicode MS'
};

% Headless模式检测
isHeadless = ~usejava('desktop');
if isHeadless
    set(0, 'DefaultFigureVisible', 'off');  % 不显示窗口, 只保存文件
end

% 输出目录
outputDir = './pic/S50_palletizing_v13';
```

#### 5.2.2 碰撞.so加载与初始化

```matlab
% 文件路径
SO_PATH   = '/home/ara/.../libHRCInterface.so'
SO_HEADER = 's50_collision_matlab.h'   % 简化C头文件

% 加载流程
loadlibrary(SO_PATH, SO_HEADER, 'alias', 'libHRCInterface')

% 初始化碰撞模型
calllib('libHRCInterface', 'initACAreaConstrainPackageInterface', ...
    int16(1),                               % robType = 1 (S50)
    DH_MM,                                  % [296.5, 336.2, ...]
    [0,0,20, 0,0,330, 160],               % 基座胶囊
    [0,0,340, 900,0,340, 140],            % 下臂胶囊
    [-10,0,60, 941.5,0,60, 120],          % 肘部胶囊
    [0,0,-50, 0,0,100, 100],              % 上臂胶囊
    [0,0,20, 140],                         % 腕部球体
    [0, -90, 0, 0, 90, 0])               % HOME (deg)

% 开启碰撞对
calllib('libHRCInterface', 'setCPSelfColliderLinkModelOpenStateInterface', int8([1,1,1]))

% HOME验证: collision=0, min_dist=698.1mm
```

#### 5.2.3 STL网格加载与降面

```matlab
meshNames = {'elfin_base','elfin_link1',...,'elfin_link6'};  % 7个
MESH_REDUCE_RATIO = 0.3;  % 保留30%面数

for i = 1:7
    tr = stlread(fn);
    
    % 单位检测: maxCoord > 10 → 文件为mm, 乘0.001转m
    maxCoord = max(abs(tr.Points(:)));
    if maxCoord > 10, sc = 0.001; else, sc = 1.0; end
    
    meshData{i} = {V*sc, F};            % 原始 (静态图)
    meshDataLow{i} = reducepatch(F,V,targetFaces);  % 低模 (动画)
end
% 效果: 73,076 → 21,918 faces (70%减少), 动画提速约3倍
```

#### 5.2.4 C++数据加载

```matlab
% 码垛轨迹: 19列 空格分隔
pall_raw = loadNumericData('data/so_palletizing_trajectory.txt');
% [task, seg, time, q1..q6, v1..v6, dist, tcpX, tcpY, tcpZ]

% 碰撞场景轨迹: 27列 空格分隔
coll_raw = loadNumericData('data/so_collision_trajectory.txt');
% [scenario, time, q1..q6, v1..v6, a1..a6, selfDist, tcpX, tcpY, tcpZ, tcpA, tcpB, tcpC]

% 摘要文件: key: value 格式
pallSummary = readSummaryFile('data/so_palletizing_summary.txt');
collSummary = readSummaryFile('data/so_collision_summary.txt');
```

#### 5.2.5 碰撞验证: C++ vs .so (关键姿态)

对每个碰撞场景 (7个)，提取最低碰撞距离处的关节角，用 .so 重新计算：

```matlab
for si = 1:nScenarios
    q_deg = coll_keyQ(si,:);  % 提取该场景中最小距离处的关节角
    
    % .so 碰撞距离
    calllib(..., 'updateACAreaConstrainPackageInterface', q_deg, zeros(1,6), zeros(1,6));
    [~, ~, distVal] = calllib(..., 'checkCPSelfCollisionInterface', int64([0,0]), 0.0);
    
    % .so FK
    tcpS = libstruct('MC_COORD_REF');
    [~, tcpS] = calllib(..., 'forwardKinematics2', q_deg, tcpS);
    
    % 对比 C++ 预计算值
    diff = abs(cpp_dist - so_dist);  % 实测: ALL Δ = 0.0 mm (完美匹配)
end
```

**实测结果**: 全部7个场景，C++ vs .so 碰撞距离差异为 **0.0 mm**，确认两端使用完全相同的底层库。

#### 5.2.6 全轨迹扫描: 碰撞 + TCP 分析

```matlab
SCAN_STRIDE = 5;  % 每5步检查一次

for ri = 1:SCAN_STRIDE:nPall
    q_deg = pall_raw(ri, 4:9);
    vel   = pall_raw(ri, 10:15);
    
    % .so碰撞距离
    calllib(..., 'updateACAreaConstrainPackageInterface', q_deg, vel, zeros(1,6));
    [~, ~, distVal] = calllib(..., 'checkCPSelfCollisionInterface', ...);
    so_pall_dist(ri) = distVal;
    
    % .so FK → TCP位姿
    [~, tcpS] = calllib(..., 'forwardKinematics2', q_deg, tcpS);
    so_pall_tcp(ri,:) = [tcpS.X, tcpS.Y, tcpS.Z, tcpS.A, tcpS.B, tcpS.C];
    
    % URDF FK → TCP Z轴方向 (姿态质量评估)
    T_all = urdfFK(JOINTS, deg2rad(q_deg));
    Rend = T_all{7}(1:3,1:3);
    tcpZaxis = Rend(:,3);  % 末端Z轴方向
    
    % TCP姿态偏差 = arccos(dot(tcpZaxis, [0,0,-1]))
end

% 插值填充未扫描的点
so_pall_dist = interp1(validIdx, ..., 'linear', 'extrap');

% TCP姿态统计:
% mean = 81.7°, max = 171.7°, within 45° = 0%
% → 结论: TCP Z轴并未指向下方, 需改进规划器的TCP约束实现
```

### 5.3 图表生成 (9张PNG + 1个GIF)

#### Figure 1: 碰撞场景 STL 姿态 (01_collision_poses_so.png)

- **布局**: 2×4 子图 (7个场景姿态 + 1个汇总柱图)
- **内容**: 每个子图渲染一个碰撞场景的关键姿态 (最小距离处)
- **渲染**: 原始STL网格 + 关节标记 + TCP标记
- **标注**: 场景编号、.so距离值、颜色编码 (绿>200, 黄>100, 红<100 mm)
- **汇总**: 水平柱图对比 C++ vs .so 碰撞距离

#### Figure 2: .so vs C++ 碰撞对比 + TCP姿态 (02_collision_verification.png)

- **布局**: 2×3 子图
- **子图 2a**: 码垛轨迹碰撞距离 — C++ (灰) vs .so (蓝) 时序对比
- **子图 2b**: 碰撞场景距离 — C++ vs .so 时序对比
- **子图 2c**: 差异分布直方图 (mean, std)
- **子图 2d**: TCP姿态偏差时序 (含容差线)
- **子图 2e**: TCP XZ空间散点 (色彩编码=碰撞距离)
- **子图 2f**: TCP姿态角 A,B,C 时序

#### Figure 3: 碰撞几何可视化 (03_collision_geometry.png)

- **布局**: 1×3 子图 (3个关键姿态)
- **内容**: STL网格 (30% opacity) + 碰撞体叠加 (胶囊体红色, 球体蓝色, 25% opacity)
- **碰撞体**: 从 `getUIInfoMationInterface` 实时获取世界坐标数据
- **标注**: 场景编号 + 碰撞距离

#### Figure 4: 码垛3D场景 (04_scene_stl_overview.png)

- **布局**: 2×2 子图 (4个任务/视角)
- **场景元素**:
  - 地面 (浅灰色方形)
  - 电控柜 (0.55m×0.65m×0.80m, 机器人基座在其上)
  - 码垛框架 (1.20m×1.15m×2.00m 管柱结构)
  - 托盘 (1.00m×1.05m×0.55m)
  - 传送带 (2.00m×0.55m×0.75m, 含滚筒和皮带)
  - 箱子 (0.35m×0.28m×0.25m, 棕色)
  - STL机器人 (含TCP轨迹线)
- **基座**: Tbase = [0, 0, 0.80m] (电控柜高度)

#### Figure 5: 关节角/速度/碰撞联合分析 (05_joint_collision_analysis.png)

- **布局**: 2×3 子图
- **子图 5a**: Task 1 — 6轴关节角度 vs 时间
- **子图 5b**: Task 1 — 6轴关节速度 vs 时间 (展示S曲线特征)
- **子图 5c**: 碰撞距离 vs 关节角 (双Y轴)
- **子图 5d**: Collision S1 — 关节角度
- **子图 5e**: Collision S1 — 关节速度
- **子图 5f**: J2 相图 (速度 vs 加速度, 色彩=时间) — 展示S曲线相空间特性

#### Figure 6: 碰撞距离综合分析 (06_collision_distance.png)

- **布局**: 2×2 子图
- **子图 6a (大)**: 全碰撞场景距离时序 (7条线, 含50mm/100mm警戒线)
- **子图 6b**: 码垛碰撞距离分布直方图
- **子图 6c**: 各任务安全裕度柱图 (颜色编码)

#### Figure 7: TCP轨迹 + 姿态向量 (07_tcp_trajectory_orientation.png)

- **布局**: 1×2 子图
- **子图 7a**: 码垛 TCP 3D路径 — 每任务一条线, 颜色渐变
- **子图 7b**: TCP Z轴方向向量 — quiver3 箭头图
  - 绿色: 偏差 < 15°
  - 黄色: 偏差 15°~30°
  - 红色: 偏差 > 30°

#### Figure 8: 动态回放 (08_dynamic_replay.png + palletizing_v13.gif)

- **布局**: 左侧3D视图 (64%宽) + 右侧信息面板 (30%宽)
- **3D视图**:
  - 完整码垛场景 (地面+电控柜+框架+托盘+传送带)
  - 低模STL机器人 (逐帧删除/重建)
  - TCP轨迹线 (当前任务)
  - 箱子管理:
    - 传送带上的箱子 (取完消失)
    - 搬运中的箱子 (跟随TCP)
    - 已放置的箱子 (放下后出现在托盘上)
  - .so实时碰撞距离 (每帧调用)
- **信息面板 (drawInfoPanel_v13)**:
  - 版本标识: "v13 REAL .so COLLISION"
  - 碰撞源: "libHRCInterface.so (实时)"
  - 进度条: Task N / Total (带百分比)
  - 碰撞距离: 大字体 + 颜色编码
  - TCP坐标: X/Y/Z (mm)
  - 6轴关节角: 数值 + 条形图
  - 时间戳
- **动画参数**:
  - `ANIM_SUBSAMPLE = 10`: 帧采样率
  - `GIF_SUBSAMPLE = 30`: GIF帧采样率
  - GIF延迟 = 120ms/帧
  - 使用 `drawnow limitrate` 限速

#### Figure 9: 综合仪表盘 (09_dashboard_profiling.png)

- **布局**: 2×3 子图
- **子图 9a**: 安全+质量6维评分柱图
  - 碰撞安全 = min(soCollDist)/700×100%
  - 码垛安全 = min(soPallDist)/700×100%
  - 零碰撞 = 100% (0次碰撞)
  - 平均裕度 = mean(dist)/5
  - S曲线 = 95% (固定)
  - TCP姿态 = 100 - mean(error)/1.8
- **子图 9b**: C++ vs .so 碰撞距离水平柱图
- **子图 9c**: 码垛任务安全裕度柱图
- **子图 9d**: 性能分析文本面板 (MATLAB自身计时)
- **子图 9e**: 耗时分解饼图
- **子图 9f**: C++ Pipeline 统计文本面板 (从summary文件读取)

### 5.4 辅助函数清单

| 函数 | 行数 | 说明 |
|------|------|------|
| `urdfFK(JOINTS, q_rad)` | ~10 | URDF正运动学, 输出7个4×4齐次矩阵 |
| `renderSTLRobot(ax, ...)` | ~15 | 静态STL渲染 (原始模型) |
| `renderSTLRobotOnBase(ax, ...)` | ~15 | 带基座偏移的STL渲染 |
| `renderSTLRobotHandles(ax, ...)` | ~20 | 返回handles的STL渲染 (动画用, 可删除) |
| `drawCapsule3D(ax, p1, p2, r, ...)` | ~15 | 3D胶囊体渲染 (圆柱+两端球) |
| `drawInfoPanel_v13(ax, ...)` | ~50 | 动画信息面板 (进度+碰撞+关节角) |
| `loadNumericData(filepath)` | ~20 | 加载空格分隔数值文件 (跳过#注释) |
| `readSummaryFile(filepath)` | ~15 | 解析 key: value 摘要文件 |
| `drawGround_v11(ax, ...)` | ~3 | 地面矩形 |
| `drawCabinet_v11(ax, ...)` | ~5 | 电控柜长方体 |
| `drawFrame_v11(ax, ...)` | ~15 | 码垛框架 (4立柱+横撑) |
| `drawPallet_v11(ax, ...)` | ~5 | 托盘长方体 |
| `drawConveyor_v11(ax, ...)` | ~30 | 传送带 (侧板+腿+滚筒+皮带) |
| `drawBox_v11(ax, pos, box)` | ~5 | 箱子长方体 |
| `saveFig(fig, dir, name)` | ~3 | 保存PNG (150dpi) |

---

## 6. 数据文件格式

### 6.1 so_palletizing_trajectory.txt (19列, 空格分隔)

```
# HR_S50-2000 码垛v3.0 (.so + TCP-aware RRT*)
# task seg time q1..q6 v1..v6 dist tcpX tcpY tcpZ
```

| 列号 | 名称 | 单位 | 说明 |
|------|------|------|------|
| 1 | task | - | 任务编号 (0=HOME→安全, 1~12=码垛任务, 13=返HOME) |
| 2 | seg | - | 运动段编号 (0~6) |
| 3 | time | s | 段内时间 |
| 4-9 | q1~q6 | deg | 关节角度 |
| 10-15 | v1~v6 | deg/s | 关节速度 |
| 16 | dist | mm | 当前累计最小自碰撞距离 |
| 17-19 | tcpX, tcpY, tcpZ | mm | TCP在基坐标系下的位置 (来自.so FK) |

采样: 每20步输出一行 (即每80ms一个数据点)

### 6.2 so_palletizing_profile.csv (15列, 逗号分隔)

```csv
task,segment,step,time_s,q1,q2,q3,q4,q5,q6,selfDist_mm,selfCollision,tcpX_mm,tcpY_mm,tcpZ_mm
```

采样: 每10步输出一行 (即每40ms一个数据点)

### 6.3 so_collision_trajectory.txt (27列, 空格分隔)

| 列号 | 名称 | 单位 |
|------|------|------|
| 1 | scenario | - |
| 2 | time | s |
| 3-8 | q1~q6 | deg |
| 9-14 | v1~v6 | deg/s |
| 15-20 | a1~a6 | deg/s² |
| 21 | selfDist | mm |
| 22-24 | tcpX, tcpY, tcpZ | mm |
| 25-27 | tcpA, tcpB, tcpC | deg |

### 6.4 so_palletizing_summary.txt (key: value)

```
positions: 12
segments: 86
total_motion_s: 1145.6856
collisions: 0
min_dist_mm: 583.87

init_ms: 1.927
planning_total_ms: 700.738
param_total_ms: 388.803
collision_runtime_ms: 397.615
elapsed_s: 1.519

coll_init_ms: 1.915
coll_update_avg_us: 6.991
coll_self_avg_us: 0.336
coll_fk_avg_us: 0.783
coll_total_avg_us: 7.861
coll_calls: 28798
```

### 6.5 so_collision_summary.txt (key: value + per-scenario)

```
scenarios: 7
total_collisions: 0
global_min_dist_mm: 65.79

# Per-scenario (每行):
id "描述" time=Xs points=N minDist=Xmm paramMs=X collMs=X singleUs=X
```

---

## 7. 性能分析与实测数据

### 7.1 C++ 后端性能

#### 7.1.1 全流水线计时 (12位码垛, 86段)

| 阶段 | 耗时 | 平均/段 | 说明 |
|------|------|---------|------|
| .so初始化 | 1.927 ms | - | 一次性 (dlopen + init) |
| RRT*规划+优化 | 700.738 ms | 8.1 ms/段 | 含采样/邻搜/碰撞/TCP/重连/捷径/B-spline |
| S曲线参数化 | 388.803 ms | 4.5 ms/段 | 七段式S曲线 |
| 碰撞运行时监测 | 397.615 ms | 4.6 ms/段 | 28,798次 update+check |
| **总计(含IO)** | **1.519 s** | **17.7 ms/段** | 端到端 |

**实时比**: 1,145.7s运动 / 1.519s计算 = **754× 实时**

#### 7.1.2 碰撞.so 微观性能

| 操作 | 平均耗时 | 调用次数 |
|------|---------|---------|
| update (更新关节状态) | 6.991 μs | 28,798 |
| selfCheck (自碰撞查询) | 0.336 μs | 28,798 |
| FK (正运动学) | 0.783 μs | 28,798 |
| **总计** | **7.861 μs/call** | 28,798 |

#### 7.1.3 碰撞场景性能 (7场景, 289次调用)

| 场景 | 描述 | 运动时间 | 最小距离 | 碰撞评估 |
|------|------|---------|---------|---------|
| S1 | HOME→常规工作姿态 | 2.65 s | 694.67 mm | 极安全 |
| S2 | 常规工作→大角度运动 | 4.94 s | 408.75 mm | 安全 |
| S3 | 大角度→折叠近碰撞区 | 5.48 s | 325.59 mm | 安全 |
| S4 | 碰撞区→安全区恢复 | 3.60 s | 330.62 mm | 安全 |
| S5 | 全轴大范围运动 | 5.97 s | 478.13 mm | 安全 |
| S6 | 极限距离伸展 | 7.60 s | 65.79 mm | ⚠️ 需关注 |
| S7 | 返回HOME | 5.76 s | 65.79 mm | ⚠️ 需关注 |

全局最小碰撞距离: **65.79 mm** (S6/S7极限伸展场景)

### 7.2 MATLAB v13 性能

#### 7.2.1 分层计时 (含.so运行)

| 阶段 | 耗时 | 占比 |
|------|------|------|
| 碰撞.so初始化 | ~200 ms | <1% |
| STL网格加载 | ~800 ms | ~0.3% |
| C++数据加载 | ~150 ms | <1% |
| 碰撞检测(.so) | ~5,000 ms | ~2% |
| 静态图渲染 (Fig1-7,9) | ~42,300 ms | ~18% |
| 动画渲染 (Fig8) | ~187,600 ms | ~80% |
| **总计** | **~233 s** | 100% |

#### 7.2.2 .so调用统计

| 指标 | 值 |
|------|-----|
| 碰撞调用总数 | 2,360次 |
| 平均碰撞调用耗时 | 33 μs/call (含MATLAB calllib开销) |
| FK调用总数 | ~1,000次 |
| C++ vs .so 碰撞误差 | 0.0 mm (完美匹配) |

#### 7.2.3 v12 vs v13 性能对比

| 指标 | v12 | v13 | 改进 |
|------|-----|-----|------|
| 总耗时 | 332 s | 233 s | +30% |
| STL面数(动画) | 73,076 | 21,918 | 70%降面 |
| 碰撞检测源 | C++预计算 | .so实时 | 真正实时 |
| TCP姿态分析 | 无 | 有 | 新增 |
| 输出图表 | 11 PNG + 2 GIF | 9 PNG + 1 GIF | 更精炼 |
| 性能分析 | 基础 | 分层μs级 | 完整 |

### 7.3 安全评估

| 指标 | 码垛 | 碰撞场景 | 评价 |
|------|------|---------|------|
| 碰撞次数 | 0 | 0 | ✅ 完美 |
| 全局最小距离 | 583.87 mm | 65.79 mm | ✅ 安全 (>50mm) |
| C++ vs .so 一致性 | 0.0 mm | 0.0 mm | ✅ 完美匹配 |
| 规划成功率 | 100% | 100% | ✅ 所有段成功 |

---

## 8. 已知问题与改进方向

### 8.1 TCP 姿态问题 (已发现)

| 指标 | 实测值 | 目标值 | 状态 |
|------|--------|--------|------|
| 平均TCP Z轴偏差 | 81.7° | < 45° | ❌ 不达标 |
| 最大偏差 | 171.7° | < 90° | ❌ 严重 |
| 45°容差内比例 | 0% | > 80% | ❌ 完全不满足 |

**根因**: C++ PathPlannerSO 的 `tcpPoseWeight=0.3` (30%) 权重不足以有效约束TCP方向。RRT*采样是在关节空间进行的，仅通过代价函数软约束TCP姿态，无法保证硬约束。

**改进建议**:
1. 增大 `tcpPoseWeight` 至 0.6~0.8
2. 在 RRT* 采样时加入TCP姿态拒绝采样 (超过容差直接丢弃)
3. 对短距P2P段使用笛卡尔空间插值 (保持TCP姿态)
4. 引入面向任务的约束IK采样

### 8.2 .so加载桩函数需求

`libHRCInterface.so` 内部有两个声明但未实现的函数：
- `initTCPPosition()`
- `updateTCPPosition()`

C++ 使用 `dlopen(RTLD_LAZY)` 可规避，但 MATLAB `loadlibrary` 使用 `RTLD_NOW` 导致加载失败。

**当前方案**: 编译空桩函数 `s50_tcp_stubs.so`，通过 `LD_PRELOAD` 预加载。

**长期方案**: 修复 `libHRCInterface.so` 源码，实现或删除这两个函数声明。

### 8.3 S6/S7 场景低安全裕度

极限伸展场景 (S6, S7) 最小碰撞距离仅 65.79 mm，接近可能的危险阈值。

**建议**: 在规划器中增加最小安全距离约束 (>100mm)，或在碰撞检测后增加安全距离检查层。

### 8.4 MATLAB 动画性能

动画渲染占总耗时80%。主要瓶颈是每帧删除/重建 patch 对象。

**改进方向**:
1. 使用 `set(h,'Vertices',V_new)` 更新顶点替代删除重建
2. 进一步降低动画模型面数
3. 减小 GIF 帧数或分辨率
4. 考虑使用 Python/OpenGL 替代 MATLAB 进行实时渲染

---

## 附录

### A. 文件列表

| 文件 | 角色 | 行数 |
|------|------|------|
| `test/testS50PalletizingSO.cpp` | C++后端主程序 | 714 |
| `ArmCollisionModel/testS50_Palletizing_v13.m` | MATLAB前端主脚本 | 1,574 |
| `ArmCollisionModel/s50_collision_matlab.h` | MATLAB碰撞.so C头文件 | 208 |
| `ArmCollisionModel/s50_tcp_stubs.c` | 缺失符号桩函数 | ~10 |
| `include/PalletizingPlanner/CollisionCheckerSO.hpp` | C++碰撞检测封装 | - |
| `include/PalletizingPlanner/PathPlannerSO.hpp` | C++ TCP-aware RRT* | - |
| `include/PalletizingPlanner/TimeParameterization.hpp` | S曲线参数化 | - |
| `include/PalletizingPlanner/RobotModel.hpp` | DH正运动学 | - |

### B. 运行命令

```bash
# 1. 编译C++后端
cd build && cmake .. && make -j
./bin/testS50PalletizingSO       # 生成 data/so_palletizing_*.txt
./bin/testS50CollisionSO         # 生成 data/so_collision_*.txt

# 2. 运行MATLAB v13 (带.so碰撞)
cd /home/ara/文档/X86_test/ArmCollisionModel
LD_PRELOAD=/path/to/s50_tcp_stubs.so \
LD_LIBRARY_PATH=/home/ara/文档/collision/HansAlgorithmExport/bin:$LD_LIBRARY_PATH \
matlab -nodesktop -nosplash -batch "testS50_Palletizing_v13"

# 3. 输出位置
# C++ 数据:  data/so_palletizing_trajectory.txt 等
# MATLAB 图: ArmCollisionModel/pic/S50_palletizing_v13/*.png, *.gif
```

### C. 依赖

| 组件 | 版本/说明 |
|------|----------|
| 编译器 | GCC (C++17) |
| Eigen3 | 系统安装 (/usr/include/eigen3) |
| libHRCInterface.so | 闭源 (HansAlgorithmExport) |
| libCmpRML.so | 闭源 (S-curve参数化) |
| MATLAB | R2022b+ (需 stlread, reducepatch) |
| 平台 | Linux x86_64 only |
