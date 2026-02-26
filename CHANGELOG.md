# 更新日志 | Changelog

所有重要更改都将记录在此文件中。

格式基于 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.0.0/)，
版本号遵循 [语义化版本](https://semver.org/lang/zh-CN/)。

---

## [3.0.0] - 2026-02-12

### 新增 (SO 动态库架构 — 重大升级)
- 🏗️ **CollisionCheckerSO** — `dlopen` 运行时加载 `libHRCInterface.so`，统一碰撞/FK/IK 接口
- 🏗️ **PathPlannerSO** — Free-TCP Informed RRT* + IncrementalKDTree6D (纯关节空间代价)
- 🏗️ **CollisionGeometry** — 统一 S50 碰撞包络参数 (胶囊体+球体)
- 🎯 **自由TCP模式** (`freeTcpDuringTransit=true`) — 运动过程中TCP位姿可自由变化，扩大可行配置空间
- 🔧 **数值IK求解器** — 多起点 Damped Least-Squares (~0.1ms/次)
- 🌍 **环境碰撞体系统** — 动态添加/移除球体和胶囊体 (电箱/传送带/框架/已放箱子)
- 🔩 **工具碰撞球** — 搬运工具末端碰撞保护
- ⏱️ **libCmpRML.so S曲线集成** — 华数上位机真实S曲线轨迹执行库
- 🧪 **testTrajectoryOptimality** — 系统性6套测试 (运动学/点到点/TCP全链/环境碰撞/安全距离/重复性)
- 🧪 **testS50PalletizingSO** — 12箱码垛完整仿真 (IK+环境碰撞+动态障碍物)
- 🧪 **testS50CollisionSO** — 7场景碰撞检测验证
- 📊 **TimingStats 分层计时** — FK/IK/碰撞独立计数 + PipelineTimingReport

### 变更
- 📐 **FK单位文档修正** — FK返回位置单位为 m (非mm)，已修正所有相关文档
- 🔢 **int16_t → int32_t** — PathPlannerOptimized 节点ID修复溢出风险 (C-01)
- ⚡ **TimingStats FK/IK独立计时** — 修复除零错误导致的负数显示 (C-04)
- 🐍 **Python可视化修复** — STL路径修正、CSV列映射修正、单位转换修正 (C-03, M-02)
- 🔧 **MATLAB v15** — SO_PATH改为环境变量、版本标签统一更新 (M-06, M-08)
- ⚠️ **CMake编译警告** — 添加 -Wall -Wextra，警告从~50降至~14 (m-04)
- 📁 **Git仓库清理** — 移除89个跟踪数据文件和matlab.mat (M-16)
- 📝 **.gitignore更新** — 添加 `*.mat` 规则
- 🛡️ **IK NaN防护** — Jacobian计算中添加FK返回值检查 (M-11)

### 性能指标 (SO栈)
- ✅ 码垛全链路计算: **0.098 s** (12箱)
- ✅ IK求解 (12位置): **0.6 ms**
- ✅ 碰撞检测 (5363次): **68.4 ms** (9.97 μs/次)
- ✅ 规划成功率: **100%**
- ✅ 碰撞安全率: **100%**
- ✅ FK单次耗时: **0.54 μs**
- ✅ 路径一致性 (20/20 重复): **100%**

---

## [2.1.0] - 2026-02-09

### 变更
- 🔧 代码质量全面升级: mt19937随机数、int32_t节点ID、关节限位采样
- 🔧 pruneTree实现、OptimizedPathPlanner集成、锁粒度优化
- 📦 CMake 3.14 更新、零编译警告

---

## [1.2.0] - 2026-02-01

### 新增
- 📁 系统性优化项目结构
- 📄 LICENSE 文件 (MIT)
- 📄 CONTRIBUTING.md 贡献指南
- 🔧 GitHub Actions CI/CD 配置
- 📝 Issue 模板 (Bug Report / Feature Request)
- 📦 examples/ 示例程序目录
- 📁 docs/ 文档目录
- 📁 data/ 数据文件目录

### 变更
- 🔄 更新公司信息为 Huayan Robotics
- 📧 更新联系邮箱

---

## [1.1.0] - 2026-01-29

### 新增
- ⚡ KD-Tree 空间索引加速 (30x 性能提升)
- 💾 碰撞检测缓存 (18x 性能提升)
- 🔄 延迟碰撞检测优化
- 📊 高性能测试程序
- 📈 性能基准测试程序

### 变更
- 优化规划管道，总时间从 ~500ms 降至 ~135ms

---

## [1.0.0] - 2026-01-29

### 新增
- 🚀 核心运动规划系统
  - `Types.hpp` - 核心数据类型
  - `RobotModel.hpp` - HR_S50-2000 运动学模型
  - `CollisionChecker.hpp` - HRC 碰撞检测封装
  - `PathPlanner.hpp` - Informed RRT* 规划器
  - `PathOptimizer.hpp` - B-Spline 路径优化
  - `TimeParameterization.hpp` - S曲线时间参数化
  - `TaskSequencer.hpp` - TSP 任务序列优化
  - `PalletizingPlanner.hpp` - 顶层 API

- 🧪 测试程序
  - `testPalletizingPlanner.cpp` - 功能测试
  - `testRobustnessValidation.cpp` - 鲁棒性测试

- 📊 可视化工具
  - `visualize_path.py` - 路径可视化脚本

- 📄 文档
  - `README.md` - 项目说明
  - `.github/copilot-instructions.md` - AI 编码指南

### 性能指标
- ✅ 平均规划时间: 135ms
- ✅ 成功率: 100%
- ✅ 路径平滑度: 0.092 rad/mm
- ✅ 鲁棒性评分: 99.5% (★★★★★ WORLD-CLASS)

---

## 维护者

- **Huayan Robotics** - yuesj@huayan-robotics.com

## 链接

- **官网**: https://www.huayan-robotics.com
- **GitHub**: https://github.com/huayan-robotics/HRC-PalletizingPlanner

---

<p align="center">
  <b>🤖 广东华沿机器人有限公司</b><br>
  <i>用机器人技术为人类服务</i>
</p>
