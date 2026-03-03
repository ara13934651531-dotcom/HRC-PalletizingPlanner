/**
 * @file PathPlannerSO.hpp
 * @brief TCP水平约束路径规划器 (使用 CollisionCheckerSO)
 *
 * 核心理念 (码垛场景):
 *   运动过程中TCP必须保持水平 (吸盘朝下), 仅允许绕Z轴旋转。
 *   重物在运输中若TCP倾斜则可能脱落, 因此TCP姿态受约束。
 *   规划在关节空间进行, 但通过FK验证每个新节点满足水平约束。
 *
 * 关键特性:
 *   1. 碰撞检测: .so 动态库 (CollisionCheckerSO, dlopen)
 *   2. TCP水平约束 (默认): 每个新配置通过FK检查TCP Z轴是否朝下
 *   3. IncrementalKDTree6D: O(log n) 最近邻 + 周期重建
 *   4. 全链路分层计时 (planning / optimization / parameterization)
 *   5. FK/IK 仅通过 CollisionCheckerSO (libHRCInterface.so) 执行
 *
 * 数据流:
 *   输入 start/goal TCP位姿 → IK求解 → Informed RRT* (关节空间 + TCP约束)
 *     → Shortcut + BSpline 优化 → 输出 Path + 计时报告
 *
 * @author Guangdong Huayan Robotics Co., Ltd.
 * @version 4.0.0
 * @date 2026-02-26
 */

#pragma once

#include "Types.hpp"
#include "RobotModel.hpp"
#include "CollisionCheckerSO.hpp"

#include <random>
#include <queue>
#include <chrono>
#include <algorithm>
#include <cmath>



namespace palletizing {

// ============================================================================
// 轻量级增量式 KD-Tree (6维关节空间, 加速最近邻搜索)
// 注: 独立于 KDTree.hpp 中的批量构建版本, 此处为增量插入+周期重建版
// ============================================================================

class IncrementalKDTree6D {
public:
    struct KDNode {
        int pointId;        // 对应 RRTNodeSO 的 id
        JointVector q;
        int left = -1, right = -1;
        int splitDim;
    };
    
    IncrementalKDTree6D() { nodes_.reserve(4096); }
    
    void clear() { nodes_.clear(); root_ = -1; needsRebuild_ = false; insertCount_ = 0; }
    
    void insert(int pointId, const JointVector& q) {
        KDNode n;
        n.pointId = pointId;
        n.q = q;
        n.splitDim = 0;
        int nIdx = (int)nodes_.size();
        nodes_.push_back(n);
        
        if (root_ < 0) {
            root_ = nIdx;
            return;
        }
        
        // 增量插入
        int cur = root_;
        int depth = 0;
        while (true) {
            int dim = depth % 6;
            if (q[dim] < nodes_[cur].q[dim]) {
                if (nodes_[cur].left < 0) { nodes_[cur].left = nIdx; nodes_[nIdx].splitDim = (depth+1)%6; break; }
                cur = nodes_[cur].left;
            } else {
                if (nodes_[cur].right < 0) { nodes_[cur].right = nIdx; nodes_[nIdx].splitDim = (depth+1)%6; break; }
                cur = nodes_[cur].right;
            }
            depth++;
        }
        
        insertCount_++;
        // 退化检测: 过多插入后重建
        if (insertCount_ > 500 && insertCount_ % 500 == 0) {
            rebuild();
        }
    }
    
    // 最近邻搜索
    int findNearest(const JointVector& q, double& bestDist) const {
        if (root_ < 0) return -1;
        bestDist = 1e30;
        int bestId = -1;
        searchNearest(root_, q, 0, bestId, bestDist);
        return bestId;
    }
    
    // 范围搜索 (半径内所有点)
    void findNear(const JointVector& q, double radius, std::vector<int>& result) const {
        result.clear();
        if (root_ < 0) return;
        double r2 = radius * radius;
        searchNear(root_, q, 0, r2, result);
    }
    
    int size() const { return (int)nodes_.size(); }
    
private:
    void searchNearest(int nodeIdx, const JointVector& target, int depth,
                       int& bestId, double& bestDist) const {
        if (nodeIdx < 0) return;
        const auto& node = nodes_[nodeIdx];
        
        double d = (node.q - target).norm();
        if (d < bestDist) {
            bestDist = d;
            bestId = node.pointId;
        }
        
        int dim = depth % 6;
        double diff = target[dim] - node.q[dim];
        
        int first = diff < 0 ? node.left : node.right;
        int second = diff < 0 ? node.right : node.left;
        
        searchNearest(first, target, depth + 1, bestId, bestDist);
        
        // 剪枝: 分割面距离 < 当前最优
        if (std::abs(diff) < bestDist) {
            searchNearest(second, target, depth + 1, bestId, bestDist);
        }
    }
    
    void searchNear(int nodeIdx, const JointVector& target, int depth,
                    double r2, std::vector<int>& result) const {
        if (nodeIdx < 0) return;
        const auto& node = nodes_[nodeIdx];
        
        double d2 = (node.q - target).squaredNorm();
        if (d2 <= r2) result.push_back(node.pointId);
        
        int dim = depth % 6;
        double diff = target[dim] - node.q[dim];
        
        int first = diff < 0 ? node.left : node.right;
        int second = diff < 0 ? node.right : node.left;
        
        searchNear(first, target, depth + 1, r2, result);
        if (diff * diff <= r2) {
            searchNear(second, target, depth + 1, r2, result);
        }
    }
    
    void rebuild() {
        if (nodes_.empty()) return;
        std::vector<int> indices(nodes_.size());
        for (int i = 0; i < (int)nodes_.size(); i++) indices[i] = i;
        root_ = buildBalanced(indices, 0, (int)indices.size(), 0);
    }
    
    int buildBalanced(std::vector<int>& indices, int lo, int hi, int depth) {
        if (lo >= hi) return -1;
        int dim = depth % 6;
        int mid = (lo + hi) / 2;
        std::nth_element(indices.begin() + lo, indices.begin() + mid,
                         indices.begin() + hi,
                         [&](int a, int b) { return nodes_[a].q[dim] < nodes_[b].q[dim]; });
        int nIdx = indices[mid];
        nodes_[nIdx].splitDim = dim;
        nodes_[nIdx].left = buildBalanced(indices, lo, mid, depth + 1);
        nodes_[nIdx].right = buildBalanced(indices, mid + 1, hi, depth + 1);
        return nIdx;
    }
    
    std::vector<KDNode> nodes_;
    int root_ = -1;
    int insertCount_ = 0;
    bool needsRebuild_ = false;
};

// ============================================================================
// TCP-aware 规划配置
// ============================================================================

struct TCPPlannerConfig : public PlannerConfig {
    // ═══ 码垛核心: 运动过程中 TCP 保持水平 ═══
    // 吸盘吸附重物时, TCP必须保持水平(Z轴朝下), 否则重物脱落。
    // 仅允许绕TCP Z轴旋转 (yaw), 不允许倾斜。
    // 每个新采样点通过FK检查TCP朝向, 不满足约束的点被拒绝。
    bool constrainTcpHorizontal = true;

    // TCP Z轴期望方向 (世界坐标系, 朝下)
    Eigen::Vector3d desiredTcpAxis = Eigen::Vector3d(0, 0, -1);
    
    // TCP朝向容差 (度) — TCP Z轴与desiredTcpAxis的最大夹角
    double orientTolerance_deg = 30.0;

    // TCP 代价权重 (附加TCP平滑性代价, 0=纯关节空间代价)
    // 通常为0: 关节空间代价已足够, TCP约束通过filter保证
    double tcpPoseWeight      = 0.0;
    double tcpOrientWeight    = 0.15;  // 姿态变化惩罚
    double tcpPositionWeight  = 0.15;  // 位置变化惩罚

    // 最大TCP位置偏移容忍 (mm) — 超过此范围的采样直接拒绝
    double maxTcpDeviation_mm = 500.0;

    // ═══ TCP工作空间排除区 (AABB盒子) ═══
    // 用于阻止TCP进入框架墙面等禁区 (弥补SO库无link-env碰撞查询)
    // TCP位于任一盒子内部即被拒绝
    // 盒子定义: {minX, maxX, minY, maxY, minZ, maxZ} (mm)
    //   例: 后墙 → {-600, 600, 1025, 1e6, -800, 1200}
    struct TCPExclusionBox {
        double minX, maxX, minY, maxY, minZ, maxZ;
        bool contains(double x, double y, double z) const {
            return x >= minX && x <= maxX && y >= minY && y <= maxY
                && z >= minZ && z <= maxZ;
        }
    };
    std::vector<TCPExclusionBox> tcpExclusionBoxes;

    // ═══ 箱子-机械臂碰撞检测 (独立于SO工具碰撞体) ═══
    // SO库pair(6,4)限制导致工具球无法覆盖箱子实际体积(z=0~-250mm)
    // 此独立检测器通过getUIInfo获取碰撞体位置, 26点OBB采样精确检测
    BoxCollisionConfig boxCollision;
};

// ============================================================================
// 分层计时报告
// ============================================================================

struct PipelineTimingReport {
    double collisionInit_ms   = 0;
    double planningTotal_ms   = 0;
    double   sampling_ms      = 0;
    double   nearestSearch_ms = 0;
    double   collisionCheck_ms = 0;
    double   tcpCostEval_ms   = 0;
    double   rewiring_ms      = 0;
    double optimizationTotal_ms = 0;
    double   shortcut_ms      = 0;
    double   bspline_ms       = 0;
    double   validation_ms    = 0;
    double parameterization_ms = 0;
    double totalPipeline_ms   = 0;
    
    int    planIterations     = 0;
    int    nodesExplored      = 0;
    int    collisionChecks    = 0;
    int    fkCalls            = 0;
    int    boxCheckRejects    = 0;   ///< 箱子碰撞检测拒绝的采样点数
    double boxCheck_ms        = 0;   ///< 箱子碰撞检测总耗时 (ms)
    
    std::string toString() const {
        char buf[2048];
        snprintf(buf, sizeof(buf),
            "  ╔═══ 全流水线分层计时报告 ══════════════════════════════╗\n"
            "  ║ [1] 碰撞初始化:       %8.2f ms                    ║\n"
            "  ║ [2] 路径规划:         %8.2f ms  (%d 迭代, %d 节点)  ║\n"
            "  ║     ├ 采样:           %8.2f ms                    ║\n"
            "  ║     ├ 最近邻搜索:     %8.2f ms                    ║\n"
            "  ║     ├ 碰撞检测:       %8.2f ms  (%d 次)           ║\n"
            "  ║     ├ TCP代价评估:    %8.2f ms  (%d 次FK)         ║\n"
            "  ║     └ 重连优化:       %8.2f ms                    ║\n"
            "  ║ [3] 路径优化:         %8.2f ms                    ║\n"
            "  ║     ├ 捷径优化:       %8.2f ms                    ║\n"
            "  ║     ├ BSpline平滑:    %8.2f ms                    ║\n"
            "  ║     └ 验证:           %8.2f ms                    ║\n"
            "  ║ [4] 时间参数化:       %8.2f ms                    ║\n"
            "  ║ [5] 箱子碰撞:        %8.2f ms  (%d 拒绝)          ║\n"
            "  ╠════════════════════════════════════════════════════════╣\n"
            "  ║   总计:              %8.2f ms                     ║\n"
            "  ╚════════════════════════════════════════════════════════╝\n",
            collisionInit_ms,
            planningTotal_ms, planIterations, nodesExplored,
            sampling_ms, nearestSearch_ms,
            collisionCheck_ms, collisionChecks,
            tcpCostEval_ms, fkCalls,
            rewiring_ms,
            optimizationTotal_ms,
            shortcut_ms, bspline_ms, validation_ms,
            parameterization_ms,
            boxCheck_ms, boxCheckRejects,
            totalPipeline_ms);
        return std::string(buf);
    }
};

// ============================================================================
// 椭球采样器 (副本, 避免依赖旧 PathPlanner.hpp/CollisionChecker.hpp)
// ============================================================================

class EllipsoidSamplerSO {
public:
    EllipsoidSamplerSO(const JointConfig& start, const JointConfig& goal,
                       const RobotDHParams& params = RobotDHParams::fromHRConfig())
        : start_(start), goal_(goal) {
        auto [minLimits, maxLimits] = params.getJointLimits();
        jointMin_ = minLimits;
        jointMax_ = maxLimits;
        center_ = 0.5 * (start.q + goal.q);
        cMin_ = (goal.q - start.q).norm();
        Eigen::VectorXd a1 = (goal.q - start.q).normalized();
        Eigen::Matrix<double, 6, 6> M = Eigen::Matrix<double, 6, 6>::Identity();
        M.col(0) = a1;
        for (int i = 1; i < 6; ++i) {
            Eigen::VectorXd v = M.col(i);
            for (int j = 0; j < i; ++j) v -= v.dot(M.col(j)) * M.col(j);
            if (v.norm() > 1e-9) M.col(i) = v.normalized();
        }
        rotation_ = M;
    }

    JointConfig sample(double cBest, std::mt19937& gen) {
        if (cBest >= std::numeric_limits<double>::infinity()) return sampleUniform(gen);
        std::vector<double> r(6);
        r[0] = cBest / 2.0;
        double rr = std::sqrt(cBest * cBest - cMin_ * cMin_) / 2.0;
        for (int i = 1; i < 6; ++i) r[i] = rr;
        std::normal_distribution<double> normal(0.0, 1.0);
        JointVector xBall;
        for (int i = 0; i < 6; ++i) xBall[i] = normal(gen);
        double norm = xBall.norm();
        std::uniform_real_distribution<double> uniform(0.0, 1.0);
        double rad = std::pow(uniform(gen), 1.0 / 6.0);
        xBall = xBall / norm * rad;
        for (int i = 0; i < 6; ++i) xBall[i] *= r[i];
        JointVector xEllipsoid = rotation_ * xBall + center_;
        return JointConfig(xEllipsoid);
    }

private:
    JointConfig sampleUniform(std::mt19937& gen) {
        JointVector q;
        for (int i = 0; i < 6; ++i) {
            std::uniform_real_distribution<double> dist(jointMin_[i], jointMax_[i]);
            q[i] = dist(gen);
        }
        return JointConfig(q);
    }
    JointConfig start_, goal_;
    JointVector center_, jointMin_, jointMax_;
    double cMin_;
    Eigen::Matrix<double, 6, 6> rotation_;
};

// ============================================================================
// RRT 节点 (含TCP缓存)
// ============================================================================

struct RRTNodeSO {
    int id;
    JointConfig config;
    int parentId = -1;
    double costFromStart = 0.0;
    double tcpCost = 0.0;          // TCP平滑性代价累计
    std::vector<int> children;
    
    // TCP位姿缓存 (避免重复FK)
    mutable SO_COORD_REF tcpPose = {};
    mutable bool tcpCached = false;
    
    RRTNodeSO() : id(-1) {}
    RRTNodeSO(int nodeId, const JointConfig& cfg) : id(nodeId), config(cfg) {}
};

// ============================================================================
// TCP-Aware Path Planner
// ============================================================================

class PathPlannerSO {
public:
    PathPlannerSO(const RobotModel& robot,
                   CollisionCheckerSO& checker,
                   const TCPPlannerConfig& config = TCPPlannerConfig())
        : robot_(robot), checker_(checker), config_(config),
          gen_(std::random_device{}()) {}
    
    /**
     * @brief 规划路径 (TCP位姿感知)
     */
    PlanningResult plan(const JointConfig& start, const JointConfig& goal) {
        timing_ = PipelineTimingReport();
        auto tPipeline = std::chrono::high_resolution_clock::now();
        
        PlanningResult result;
        
        // 验证起止点
        if (!robot_.isWithinLimits(start)) {
            result.status = PlanningStatus::InvalidStart;
            return result;
        }
        if (!robot_.isWithinLimits(goal)) {
            result.status = PlanningStatus::InvalidGoal;
            return result;
        }
        if (!checker_.isCollisionFree(start)) {
            result.status = PlanningStatus::CollisionAtStart;
            return result;
        }
        if (!checker_.isCollisionFree(goal)) {
            result.status = PlanningStatus::CollisionAtGoal;
            return result;
        }

        if (start.distanceTo(goal) < 1e-8) {
            result.rawPath.waypoints = {Waypoint(start), Waypoint(goal)};
            result.rawPath.updatePathParameters();
            result.optimizedPath = result.rawPath;
            result.pathLength = 0.0;
            result.status = PlanningStatus::Success;
            result.iterations = 0;
            result.nodesExplored = 1;
            timing_.planningTotal_ms = 0.0;
            timing_.optimizationTotal_ms = 0.0;
            timing_.totalPipeline_ms = ms(tPipeline, std::chrono::high_resolution_clock::now());
            return result;
        }
        
        // ── 箱子碰撞自适应阈值 ──
        // 当起止点本身存在箱子-机械臂碰撞时 (IK解的固有限制):
        //   - 自动降低safetyMargin至起止点最差距离 - buffer
        //   - 确保中间路径不会比起止点更差
        //   - 规划完成后恢复原始margin
        double originalBoxMargin = config_.boxCollision.safetyMargin;
        if (config_.boxCollision.enabled) {
            auto bxS = checker_.getBoxCollisionReport(start, config_.boxCollision);
            auto bxG = checker_.getBoxCollisionReport(goal, config_.boxCollision);
            if (bxS.collision || bxG.collision) {
                // 使用安全关键距离 (仅Base+LowerArm) 进行自适应
                double worstDist = std::min(bxS.criticalMinDist_mm, bxG.criticalMinDist_mm);
                double adaptiveMargin = worstDist - 5.0;
                config_.boxCollision.safetyMargin = adaptiveMargin;
                printf("  ⚠️ 箱子碰撞自适应: start=%.1fmm(%s) goal=%.1fmm(%s) → margin=%.1fmm\n",
                       bxS.criticalMinDist_mm, BoxCollisionReport::colliderName(bxS.criticalClosest),
                       bxG.criticalMinDist_mm, BoxCollisionReport::colliderName(bxG.criticalClosest),
                       adaptiveMargin);
            }
        }

        // 规划
        auto tPlan = std::chrono::high_resolution_clock::now();
        Path rawPath = planInformedRRTStar(start, goal, result);
        auto tPlanEnd = std::chrono::high_resolution_clock::now();
        timing_.planningTotal_ms = ms(tPlan, tPlanEnd);

        // 恢复原始margin
        config_.boxCollision.safetyMargin = originalBoxMargin;
        
        if (rawPath.empty()) {
            result.status = PlanningStatus::NoPath;
            return result;
        }
        
        result.rawPath = rawPath;
        result.planningTime = timing_.planningTotal_ms / 1000.0;
        
        // 优化
        auto tOpt = std::chrono::high_resolution_clock::now();
        result.optimizedPath = optimizePath(rawPath);
        auto tOptEnd = std::chrono::high_resolution_clock::now();
        timing_.optimizationTotal_ms = ms(tOpt, tOptEnd);
        result.optimizationTime = timing_.optimizationTotal_ms / 1000.0;
        
        result.pathLength = result.optimizedPath.totalLength();
        result.status = PlanningStatus::Success;
        result.iterations = timing_.planIterations;
        result.nodesExplored = timing_.nodesExplored;
        
        auto tPipelineEnd = std::chrono::high_resolution_clock::now();
        timing_.totalPipeline_ms = ms(tPipeline, tPipelineEnd);
        
        return result;
    }
    
    PipelineTimingReport getTimingReport() const { return timing_; }
    
    void setConfig(const TCPPlannerConfig& config) { config_ = config; }
    const TCPPlannerConfig& getConfig() const { return config_; }
    
private:
    // ========================================================================
    // Informed RRT* (TCP-Aware)
    // ========================================================================
    
    Path planInformedRRTStar(const JointConfig& start, const JointConfig& goal,
                              PlanningResult& result) {
        nodes_.clear();
        nodes_.reserve(config_.maxIterations);
        kdTree_.clear();
        
        // TCP代价开关: tcpPoseWeight > 0 时启用TCP平滑性代价
        const bool useTcpCost = config_.tcpPoseWeight > 0;
        
        // TCP水平约束: 每个新节点需要FK检查TCP朝向
        const bool checkHorizontal = config_.constrainTcpHorizontal;
        
        RRTNodeSO startNode(0, start);
        startNode.costFromStart = 0.0;
        if (useTcpCost) cacheTcpPose(startNode);
        nodes_.push_back(startNode);
        kdTree_.insert(0, start.q);
        
        // 目标TCP位姿 (仅TCP代价模式使用)
        SO_COORD_REF goalTcp = {};
        if (useTcpCost) checker_.forwardKinematics(goal, goalTcp);
        
        EllipsoidSamplerSO ellipsoidSampler(start, goal, robot_.getParams());
        double cBest = std::numeric_limits<double>::infinity();
        int goalNodeId = -1;
        
        std::uniform_real_distribution<double> uniDist(0.0, 1.0);
        
        auto tStart = std::chrono::high_resolution_clock::now();
        
        for (int iter = 0; iter < config_.maxIterations; iter++) {
            auto now = std::chrono::high_resolution_clock::now();
            if (ms(tStart, now) > config_.maxPlanningTime * 1000) break;
            
            // 采样
            auto tSamp = std::chrono::high_resolution_clock::now();
            JointConfig qRand;
            if (uniDist(gen_) < config_.goalBias && goalNodeId < 0) {
                qRand = goal;
            } else if (config_.useInformedSampling && cBest < 1e30) {
                qRand = ellipsoidSampler.sample(cBest, gen_);
                qRand = robot_.clampToLimits(qRand);
            } else {
                qRand = robot_.randomConfig();
            }
            timing_.sampling_ms += msNow(tSamp);
            
            // 最近邻
            auto tNN = std::chrono::high_resolution_clock::now();
            int nearestId = findNearest(qRand);
            timing_.nearestSearch_ms += msNow(tNN);
            if (nearestId < 0) continue;
            
            // Steer
            JointConfig qNew = steer(nodes_[nearestId].config, qRand, config_.stepSize);
            if (!robot_.isWithinLimits(qNew)) continue;
            
            // 碰撞检测
            auto tColl = std::chrono::high_resolution_clock::now();
            bool pathClear = checker_.isPathCollisionFree(
                nodes_[nearestId].config, qNew, config_.collisionResolution);
            timing_.collisionCheck_ms += msNow(tColl);
            timing_.collisionChecks++;
            if (!pathClear) continue;

            // TCP位姿计算 (用于水平约束检查 + TCP代价评估)
            SO_COORD_REF qNewTcp = {};
            bool qNewTcpValid = false;
            if (checkHorizontal || useTcpCost) {
                qNewTcpValid = checker_.forwardKinematics(qNew, qNewTcp);
                timing_.fkCalls++;
                if (!qNewTcpValid) continue;
                
                // TCP水平约束检查: TCP Z轴必须接近朝下
                if (checkHorizontal) {
                    auto tOrientCheck = std::chrono::high_resolution_clock::now();
                    double a_r = qNewTcp.A * M_PI / 180.0;
                    double b_r = qNewTcp.B * M_PI / 180.0;
                    // TCP Z轴在世界坐标系 (由欧拉角ABC=ZYX intrinsic推导)
                    Eigen::Vector3d tcpZ(
                        -std::sin(b_r),
                        std::sin(a_r) * std::cos(b_r),
                        std::cos(a_r) * std::cos(b_r)
                    );
                    double dotProd = tcpZ.dot(config_.desiredTcpAxis);
                    double angleDev_deg = std::acos(std::clamp(std::abs(dotProd), 0.0, 1.0)) * 180.0 / M_PI;
                    timing_.tcpCostEval_ms += msNow(tOrientCheck);
                    
                    if (angleDev_deg > config_.orientTolerance_deg) continue;
                }
                
                // 注: TCP排除区检查仅在direct-path判定中使用 (executeRRTStar)
                // RRT采样不做硬排除 — 单树RRT难以通过大关节距的窄通道
                // RRT*自然倾向最短路径, 从而绕过后墙区域
            }

            // ── 箱子-机械臂碰撞检测 (独立于SO工具碰撞体) ──
            // 绕过SO pair(6,4)限制: 通过getUIInfo获取碰撞体世界坐标,
            // 26点OBB采样检查箱子与5个碰撞体(Base~Wrist)的距离
            if (config_.boxCollision.enabled) {
                auto tBoxCheck = std::chrono::high_resolution_clock::now();
                bool boxSafe = checker_.isBoxCollisionFree(qNew, config_.boxCollision);
                timing_.boxCheck_ms += msNow(tBoxCheck);
                if (!boxSafe) {
                    timing_.boxCheckRejects++;
                    continue;
                }
            }

            // TCP代价评估
            auto tTcp = std::chrono::high_resolution_clock::now();
            double jointDist = nodes_[nearestId].config.distanceTo(qNew);
            if (!nodes_[nearestId].tcpCached) cacheTcpPose(nodes_[nearestId]);
            double tcpCostIncr = computeTcpCostFromPoses(nodes_[nearestId].tcpPose, qNewTcp, goalTcp);
            timing_.tcpCostEval_ms += msNow(tTcp);
            
            // 混合代价 = (1-w)*关节距离 + w*TCP代价
            double w = config_.tcpPoseWeight;
            double edgeCost = (1.0 - w) * jointDist + w * tcpCostIncr;
            
            // 邻域寻找最佳父节点
            auto tRewire = std::chrono::high_resolution_clock::now();
            double rewireR = computeRewireRadius();
            std::vector<int> neighbors = findNear(qNew, rewireR);
            
            int bestParent = nearestId;
            double bestCost = nodes_[nearestId].costFromStart + edgeCost;
            
            for (int nId : neighbors) {
                double nJointDist = nodes_[nId].config.distanceTo(qNew);
                double nEdge = nJointDist;
                if (useTcpCost) {
                    if (!nodes_[nId].tcpCached) cacheTcpPose(nodes_[nId]);
                    double nTcpCost = computeTcpCostFromPoses(nodes_[nId].tcpPose, qNewTcp, goalTcp);
                    double w = config_.tcpPoseWeight;
                    nEdge = (1.0 - w) * nJointDist + w * nTcpCost;
                }
                double nCost = nodes_[nId].costFromStart + nEdge;
                
                if (nCost < bestCost) {
                    timing_.collisionChecks++;
                    if (checker_.isPathCollisionFree(nodes_[nId].config, qNew,
                                                     config_.collisionResolution)) {
                        bestParent = nId;
                        bestCost = nCost;
                    }
                }
            }
            
            // 添加节点
            int newId = (int)nodes_.size();
            RRTNodeSO newNode(newId, qNew);
            newNode.parentId = bestParent;
            newNode.costFromStart = bestCost;
            if (useTcpCost) { newNode.tcpPose = qNewTcp; newNode.tcpCached = true; }
            nodes_.push_back(newNode);
            kdTree_.insert(newId, qNew.q);
            nodes_[bestParent].children.push_back(newId);
            
            // Rewire
            for (int nId : neighbors) {
                if (nId == bestParent) continue;
                double nJointDist = qNew.distanceTo(nodes_[nId].config);
                double nEdge = nJointDist;
                if (useTcpCost) {
                    if (!nodes_[nId].tcpCached) cacheTcpPose(nodes_[nId]);
                    double nTcpCost = computeTcpCostFromPoses(qNewTcp, nodes_[nId].tcpPose, goalTcp);
                    double w = config_.tcpPoseWeight;
                    nEdge = (1.0 - w) * nJointDist + w * nTcpCost;
                }
                double newCost = newNode.costFromStart + nEdge;
                
                if (newCost < nodes_[nId].costFromStart) {
                    timing_.collisionChecks++;
                    if (checker_.isPathCollisionFree(qNew, nodes_[nId].config,
                                                     config_.collisionResolution)) {
                        int oldParent = nodes_[nId].parentId;
                        if (oldParent >= 0) {
                            auto& ch = nodes_[oldParent].children;
                            ch.erase(std::remove(ch.begin(), ch.end(), nId), ch.end());
                        }
                        nodes_[nId].parentId = newId;
                        nodes_[nId].costFromStart = newCost;
                        nodes_[newId].children.push_back(nId);
                        propagateCostUpdate(nId);
                    }
                }
            }
            timing_.rewiring_ms += msNow(tRewire);
            
            // 检查目标
            double distGoal = qNew.distanceTo(goal);
            if (distGoal < config_.stepSize) {
                timing_.collisionChecks++;
                if (checker_.isPathCollisionFree(qNew, goal, config_.collisionResolution)) {
                    double pathCost = newNode.costFromStart + distGoal;
                    
                    if (pathCost < cBest) {
                        cBest = pathCost;
                        if (goalNodeId < 0) {
                            goalNodeId = (int)nodes_.size();
                            RRTNodeSO gNode(goalNodeId, goal);
                            gNode.parentId = newId;
                            gNode.costFromStart = pathCost;
                            if (useTcpCost) cacheTcpPose(gNode);
                            nodes_.push_back(gNode);
                            kdTree_.insert(goalNodeId, goal.q);
                            nodes_[newId].children.push_back(goalNodeId);
                        } else if (pathCost < nodes_[goalNodeId].costFromStart) {
                            int oldP = nodes_[goalNodeId].parentId;
                            if (oldP >= 0) {
                                auto& ch = nodes_[oldP].children;
                                ch.erase(std::remove(ch.begin(), ch.end(), goalNodeId), ch.end());
                            }
                            nodes_[goalNodeId].parentId = newId;
                            nodes_[goalNodeId].costFromStart = pathCost;
                            nodes_[newId].children.push_back(goalNodeId);
                        }
                    }
                }
            }
            
            timing_.planIterations = iter + 1;
        }
        
        timing_.nodesExplored = (int)nodes_.size();
        
        if (goalNodeId >= 0) return extractPath(goalNodeId);
        return Path();
    }
    
    // ========================================================================
    // TCP 代价函数
    // ========================================================================
    
    double computeTcpCostFromPoses(const SO_COORD_REF& parentTcp,
                                   const SO_COORD_REF& childTcp,
                                   const SO_COORD_REF& goalTcp) const {
        // TCP位置变化量 (mm)
        double dx = childTcp.X - parentTcp.X;
        double dy = childTcp.Y - parentTcp.Y;
        double dz = childTcp.Z - parentTcp.Z;
        double posDist = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        // TCP姿态变化量 (deg)
        double da = childTcp.A - parentTcp.A;
        double db = childTcp.B - parentTcp.B;
        double dc = childTcp.C - parentTcp.C;
        double orientDist = std::sqrt(da*da + db*db + dc*dc);
        
        // 归一化: 位置用1000mm, 姿态用180deg
        double posCost = posDist / 1000.0 * config_.tcpPositionWeight;
        double orientCost = orientDist / 180.0 * config_.tcpOrientWeight;
        
        // 到目标的TCP距离贡献 (引导朝向目标TCP)
        double gdx = childTcp.X - goalTcp.X;
        double gdy = childTcp.Y - goalTcp.Y;
        double gdz = childTcp.Z - goalTcp.Z;
        double goalDist = std::sqrt(gdx*gdx + gdy*gdy + gdz*gdz) / 1000.0 * 0.1;
        
        return posCost + orientCost + goalDist;
    }
    
    void cacheTcpPose(const RRTNodeSO& node) const {
        if (node.tcpCached) return;
        // 所有FK通过SO库执行 (libHRCInterface.so)
        checker_.forwardKinematics(node.config, node.tcpPose);
        node.tcpCached = true;
        timing_.fkCalls++;
    }
    
    // ========================================================================
    // 路径优化 (TCP-aware)
    // ========================================================================
    
    Path optimizePath(const Path& rawPath) {
        if (rawPath.waypoints.size() < 3) return rawPath;
        
        // 1. Shortcut
        auto t1 = std::chrono::high_resolution_clock::now();
        Path shortcut = shortcutOptimize(rawPath, config_.shortcutIterations);
        timing_.shortcut_ms = msNow(t1);
        
        // 2. 简化
        Path simplified = simplifyPath(shortcut, 0.01);
        
        // 3. 细分
        Path subdivided = subdividePath(simplified, config_.stepSize);
        
        // 4. BSpline
        auto t2 = std::chrono::high_resolution_clock::now();
        BSpline spline;
        std::vector<JointConfig> cps;
        cps.reserve(subdivided.waypoints.size());
        for (auto& wp : subdivided.waypoints) cps.push_back(wp.config);
        int degree = std::min(config_.splineDegree, (int)cps.size() - 1);
        degree = std::max(degree, 1);
        spline.initUniform(cps, degree);
        
        Path smoothed = spline.sample(config_.splineResolution);
        timing_.bspline_ms = msNow(t2);
        
        // 5. 验证 (自碰撞 + 环境 + 箱子碰撞)
        auto t3 = std::chrono::high_resolution_clock::now();
        bool valid = true;
        for (size_t i = 0; i + 1 < smoothed.waypoints.size(); i++) {
            if (!checker_.isPathCollisionFree(smoothed.waypoints[i].config,
                                              smoothed.waypoints[i+1].config,
                                              config_.collisionResolution)) {
                valid = false;
                break;
            }
            // 箱子碰撞验证: 确保BSpline平滑后箱子仍不与机械臂碰撞
            if (config_.boxCollision.enabled) {
                if (!checker_.isPathBoxCollisionFree(smoothed.waypoints[i].config,
                                                     smoothed.waypoints[i+1].config,
                                                     config_.boxCollision,
                                                     config_.collisionResolution)) {
                    valid = false;
                    break;
                }
            }
        }
        timing_.validation_ms = msNow(t3);
        
        return valid ? smoothed : subdivided;
    }
    
    Path shortcutOptimize(const Path& path, int iterations) {
        if (path.waypoints.size() < 3) return path;
        Path opt = path;
        std::uniform_int_distribution<int> dist(0, (int)opt.waypoints.size() - 1);
        
        for (int iter = 0; iter < iterations; iter++) {
            if (opt.waypoints.size() < 3) break;
            int n = (int)opt.waypoints.size();
            int i = dist(gen_) % n;
            int j = dist(gen_) % n;
            if (i > j) std::swap(i, j);
            if (j - i < 2) continue;
            
            if (checker_.isPathCollisionFree(opt.waypoints[i].config,
                                              opt.waypoints[j].config,
                                              config_.collisionResolution)) {
                // 箱子碰撞检查: 捷径不允许穿过箱子碰撞区域
                bool boxOk = true;
                if (config_.boxCollision.enabled) {
                    boxOk = checker_.isPathBoxCollisionFree(opt.waypoints[i].config,
                                                            opt.waypoints[j].config,
                                                            config_.boxCollision,
                                                            config_.collisionResolution);
                }
                if (boxOk) {
                    std::vector<Waypoint> newWps;
                    for (int k = 0; k <= i; k++) newWps.push_back(opt.waypoints[k]);
                    for (int k = j; k < n; k++) newWps.push_back(opt.waypoints[k]);
                    opt.waypoints = std::move(newWps);
                    dist = std::uniform_int_distribution<int>(0, (int)opt.waypoints.size() - 1);
                }
            }
        }
        opt.updatePathParameters();
        return opt;
    }
    
    Path simplifyPath(const Path& path, double tolerance) {
        if (path.waypoints.size() < 3) return path;
        Path s;
        s.waypoints.push_back(path.waypoints.front());
        for (size_t i = 1; i < path.waypoints.size() - 1; i++) {
            JointVector v = path.waypoints[i+1].config.q - s.waypoints.back().config.q;
            JointVector w = path.waypoints[i].config.q - s.waypoints.back().config.q;
            double sq = v.squaredNorm();
            if (sq < 1e-18) { s.waypoints.push_back(path.waypoints[i]); continue; }
            double t = std::clamp(w.dot(v) / sq, 0.0, 1.0);
            double d = (path.waypoints[i].config.q - (s.waypoints.back().config.q + t*v)).norm();
            if (d > tolerance) s.waypoints.push_back(path.waypoints[i]);
        }
        s.waypoints.push_back(path.waypoints.back());
        s.updatePathParameters();
        return s;
    }
    
    Path subdividePath(const Path& path, double maxDist) {
        if (path.waypoints.empty()) return path;
        Path sub;
        for (size_t i = 0; i < path.waypoints.size() - 1; i++) {
            auto& s = path.waypoints[i].config;
            auto& e = path.waypoints[i+1].config;
            double d = s.distanceTo(e);
            int n = std::max(1, (int)std::ceil(d / maxDist));
            for (int j = 0; j < n; j++) {
                double t = (double)j / n;
                sub.waypoints.push_back(Waypoint(s.interpolate(e, t)));
            }
        }
        sub.waypoints.push_back(path.waypoints.back());
        sub.updatePathParameters();
        return sub;
    }
    
    // ========================================================================
    // 树操作
    // ========================================================================
    
    int findNearest(const JointConfig& q) const {
        // KD-Tree加速 O(log n) vs 线性 O(n)
        double bestDist;
        return kdTree_.findNearest(q.q, bestDist);
    }
    
    std::vector<int> findNear(const JointConfig& q, double radius) const {
        std::vector<int> result;
        kdTree_.findNear(q.q, radius, result);
        return result;
    }
    
    JointConfig steer(const JointConfig& from, const JointConfig& to, double step) const {
        JointVector dir = to.q - from.q;
        double d = dir.norm();
        if (d <= step) return to;
        return JointConfig(from.q + step * dir / d);
    }
    
    double computeRewireRadius() const {
        double n = (double)nodes_.size();
        double d = 6.0;
        auto [mi, mx] = robot_.getParams().getJointLimits();
        double vol = 1.0;
        for (int i = 0; i < 6; i++) vol *= (mx[i] - mi[i]);
        double gamma = 2.0 * std::pow(1.0+1.0/d, 1.0/d) * std::pow(vol/std::pow(M_PI,d/2.0), 1.0/d);
        return std::min(gamma * std::pow(std::log(n+1)/(n+1), 1.0/d), config_.rewireRadius);
    }
    
    void propagateCostUpdate(int nodeId) {
        // 注: 默认tcpPoseWeight=0, 边代价退化为纯关节距离。
        //     当 useTcpCost=true 时, 使用纯关节距离作为近似 (TCP代价需要FK, 开销太大),
        //     因为 propagate 只在 rewire 改善路径时调用, 关节距离是代价下界。
        std::queue<int> q;
        q.push(nodeId);
        while (!q.empty()) {
            int cur = q.front(); q.pop();
            for (int ch : nodes_[cur].children) {
                double edgeDist = nodes_[cur].config.distanceTo(nodes_[ch].config);
                double nc = nodes_[cur].costFromStart + edgeDist;
                if (nc < nodes_[ch].costFromStart) {
                    nodes_[ch].costFromStart = nc;
                    q.push(ch);
                }
            }
        }
    }
    
    Path extractPath(int goalId) const {
        Path path;
        std::vector<int> ids;
        int cur = goalId;
        while (cur >= 0) {
            ids.push_back(cur);
            cur = nodes_[cur].parentId;
        }
        std::reverse(ids.begin(), ids.end());
        for (int id : ids) {
            Waypoint wp(nodes_[id].config);
            path.waypoints.push_back(wp);
        }
        path.updatePathParameters();
        return path;
    }
    
    // ========================================================================
    // 计时辅助
    // ========================================================================
    
    static double ms(std::chrono::high_resolution_clock::time_point a,
                     std::chrono::high_resolution_clock::time_point b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
    }
    
    static double msNow(std::chrono::high_resolution_clock::time_point start) {
        return ms(start, std::chrono::high_resolution_clock::now());
    }
    
    const RobotModel& robot_;
    CollisionCheckerSO& checker_;
    TCPPlannerConfig config_;
    
    std::vector<RRTNodeSO> nodes_;
    mutable IncrementalKDTree6D kdTree_;
    std::mt19937 gen_;
    mutable PipelineTimingReport timing_;
};

} // namespace palletizing
