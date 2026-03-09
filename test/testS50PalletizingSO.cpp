/**
 * @file testS50PalletizingSO.cpp
 * @brief HR_S50-2000 码垛仿真 v8.0 — v3.1安全优先布局 + 收缩通过策略
 *
 * v8.0 核心变更 (2026-03-xx):
 *   1. 布局升级到 v3.1 安全优先优化 (S50_Layout_Optimization_Model_v3.md §10.1):
 *      - FRAME_CY=1054.4 (原680), CONV_CX=977.2 (原580),
 *        CONV_CY=176.7 (原30.6), PALLET_H=200mm (托盘在地面, 底面Z_world=0)
 *      - FRAME_GAP=404.4mm, CONV_GAP=427.2mm (安全间距显著提升)
 *      - 安全得分 +75.4%, 航路点循环路径 -11.3%
 *   2. ★ 收缩通过策略 (Side-pass, 替代顶部翻越):
 *      - 搬运路径经由框架前方开口进入 (Y=fNY), 而非翻越顶梁 (Z=1200+)
 *      - 框架入口航路点: (x_place, fNY, z_place+150mm), Z≤975mm
 *      - 垂直距离节省 ~450mm vs 顶部翻越, 路径更短更安全
 *      - 搬运段改用RRT* (TCP-Horizontal + box-env碰撞检测)
 *   3. 箱子放置: v3.1网格公式 (BK-L第一层)
 *   4. TCP Z轴旋转避障 + 箱子OBB碰撞检测 (保留v7.0特性)
 *
 * v7.0 变更: 优化布局(v2.0) + TCP旋转避障 + J4/J6搜索
 * v6.5 变更: J1短弧优化, 2段对角搬运
 *
 * @date 2026-03-06
 * @copyright Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.
 */

#include "PalletizingPlanner/RobotModel.hpp"
#include "PalletizingPlanner/CollisionCheckerSO.hpp"
#include "PalletizingPlanner/PathPlannerSO.hpp"
#include "PalletizingPlanner/TimeParameterization.hpp"
#include "PalletizingPlanner/NumericalIK.hpp"
#include "PalletizingPlanner/Types.hpp"

#include <cstdio>
#include <cstring>
#include <cmath>
#include <chrono>
#include <vector>
#include <algorithm>
#include <numeric>
#include <string>
#include <limits>
#include <array>

using namespace palletizing;

// ============================================================================
// 场景几何参数 (mm, 基于S50_Layout_Optimization_Model.md §10优化结果)
// ============================================================================
namespace scene {
    // === 物体尺寸 (固定参数, §3) ===
    constexpr double CAB_W = 550, CAB_D = 650, CAB_H = 800;
    constexpr double FRM_W = 1200, FRM_D = 650, FRM_H = 2000;
    constexpr double FRM_TUBE_R = 30;
    constexpr double PAL_W = 1000, PAL_D = 600;
    constexpr double CONV_LEN = 2000, CONV_W = 550, CONV_H = 750;
    constexpr double BOX_LX = 350, BOX_WY = 280, BOX_HZ = 250;
    constexpr double BOX_GAP = 10;   // §3: 箱间间隙10mm

    // === 布局参数 (v3.1 安全优先优化, S50_Layout_Optimization_Model_v3.md §10.1) ===
    // v3.1: 避碰优先, 路径次优先 — 不再贴紧约束边界, 主动拉大安全间距
    constexpr double FRAME_CY  = 1054.4;  // y_f: 框架中心Y (mm), v3.1安全优先 (v2.0: 680)
    constexpr double CONV_CX   = 977.2;   // x_c: 传送带中心X (mm), v3.1安全优先 (v2.0: 580)
    constexpr double CONV_CY   = 176.7;   // y_c: 传送带/拾取位Y (mm), v3.1安全优先 (v2.0: 30.6)
    constexpr double PALLET_H = 200.0;    // 托盘物理高度(mm), 底面在地面 (世界Z=0)

    constexpr double baseZ = CAB_H;  // 800mm, 机器人基座世界Z高度

    // v3.1 间距参数 (反算, 用于日志兼容)
    constexpr double FRAME_GAP = FRAME_CY - CAB_D/2 - FRM_D/2;  // = 1054.4-325-325 = 404.4mm
    constexpr double CONV_GAP  = CONV_CX  - CAB_W/2 - CONV_W/2; // = 977.2-275-275 = 427.2mm

    inline double frameCY() { return FRAME_CY; }
    inline double palletSurfBase() { return PALLET_H - baseZ; }  // 托盘面Z(基座坐标) = 200-800 = -600mm
    inline double convCX() { return CONV_CX; }
    inline double convSurfBase() { return CONV_H + 30 + 35 - baseZ; }  // 15mm
}

// ============================================================================
// 数据结构
// ============================================================================
struct SegmentResult {
    std::string name;
    double totalTime_s = 0; int totalPoints = 0;
    double minSelfDist_mm = 1e10; int collisionCount = 0;
    int envCollisionCount = 0; double peakVelocity_dps = 0;
    bool success = false;
    double planningTime_ms = 0, optimizationTime_ms = 0;
    double paramTime_ms = 0, collCheckTime_ms = 0, totalSegmentTime_ms = 0;
    int planIterations = 0, planNodes = 0, pathWaypoints = 0;
    double pathLength_rad = 0;
    int tcpExclusionViolations = 0;  // TCP进入排除区计数
};

struct TaskResult {
    int taskIndex; std::string description;
    std::vector<SegmentResult> segments;
    double totalTime_s = 0, minSelfDist_mm = 1e10;
    int totalCollisions = 0, totalEnvCollisions = 0;
    double totalPlanningMs = 0, totalParamMs = 0, totalCollCheckMs = 0;
};

struct BoxTarget {
    std::string label;
    Eigen::Vector3d pos_mm;
    Eigen::Vector3d approach_mm;
};

// ============================================================================
// IK求解: 使用共享 NumericalIK.hpp (消除代码重复)
// ============================================================================
// IKResult, numericalIK(), multiStartIK() 定义在 NumericalIK.hpp 中
using palletizing::IKResult;
using palletizing::numericalIK;
using palletizing::multiStartIK;

// ============================================================================
// SO库IK求解 (TCP保持水平: B=0, C=180° → Z轴朝下)
// ============================================================================
// NumericalIK 仅约束位置(3DOF)+关节投影, 极端配置下TCP朝向无法保证.
// SO IK 直接指定6D目标位姿(含朝向), 精确保证TCP水平.
//
// 原理:
//   FK2 Euler ZYX: R = Rz(A)·Ry(B)·Rx(C)
//   TCP Z-axis 分量 = -cos(B)·cos(C)  →  B=0, C=180° → Z=(0,0,-1) ✓
//   A角(yaw) 从种子FK获取, 保证臂膀方向一致
//
// 单位: target_mm(mm) → IK输入(m), FK验证(mm)
IKResult soIKHorizontal(CollisionCheckerSO& checker,
    const RobotModel& robot,
    const Eigen::Vector3d& target_mm,
    const std::vector<JointConfig>& seeds,
    double tol_mm = 3.0)
{
    IKResult best;
    best.posError_mm = 1e10;

    for (const auto& seed : seeds) {
        // 从种子FK获取自然A角(yaw), 保持臂膀方向
        SO_COORD_REF seedTcp;
        if (!checker.forwardKinematics(seed, seedTcp)) continue;

        // IK目标: 位置(mm) + TCP水平朝向(B=0, C=180°)
        // v1.0.0: FK2和IK统一使用mm
        SO_COORD_REF ikPose;
        ikPose.X = target_mm.x();
        ikPose.Y = target_mm.y();
        ikPose.Z = target_mm.z();
        ikPose.A = seedTcp.A;   // 种子的自然yaw角
        ikPose.B = 0.0;         // pitch = 0
        ikPose.C = 180.0;       // roll = 180° → TCP Z-down

        JointConfig result;
        bool ikOk = checker.inverseKinematics(ikPose, seed, result);

        // 关节限位检查 (SO IK可能返回超限角度)
        if (ikOk && !robot.isWithinLimits(result)) ikOk = false;
        if (!ikOk) continue;

        // FK验证: 位置精度 + TCP Z-down偏差
        SO_COORD_REF tcp;
        if (!checker.forwardKinematics(result, tcp)) continue;

        double err = std::sqrt(std::pow(tcp.X - target_mm.x(), 2) +
                               std::pow(tcp.Y - target_mm.y(), 2) +
                               std::pow(tcp.Z - target_mm.z(), 2));
        double Br = tcp.B * M_PI / 180, Cr = tcp.C * M_PI / 180;
        double zz = std::cos(Br) * std::cos(Cr);
        double downDev = std::acos(std::clamp(-zz, -1.0, 1.0)) * 180.0 / M_PI;

        if (err < tol_mm && downDev < 5.0 && err < best.posError_mm) {
            best.config = result;
            best.posError_mm = err;
            best.converged = true;
            best.iterations = 1;
        }
        if (best.converged && best.posError_mm < tol_mm * 0.5) break;
    }

    return best;
}

// ============================================================================
// 辅助: P2P执行
// ============================================================================
SegmentResult executeP2P(const char* name,
    const JointConfig& start, const JointConfig& target,
    RobotModel& robot, CollisionCheckerSO& checker,
    TimeParameterizer& param, FILE* csv, int ti, int si, FILE* traj)
{
    SegmentResult r; r.name = name; r.pathWaypoints = 2;
    r.pathLength_rad = start.distanceTo(target);
    auto tS = std::chrono::high_resolution_clock::now();
    Path path; Waypoint w0(start); w0.pathParam=0; Waypoint w1(target); w1.pathParam=1;
    path.waypoints.push_back(w0); path.waypoints.push_back(w1);
    auto t0 = std::chrono::high_resolution_clock::now();
    Trajectory tr = param.parameterize(path);
    r.paramTime_ms = std::chrono::duration<double,std::milli>(std::chrono::high_resolution_clock::now()-t0).count();
    if (tr.empty()) return r;
    r.totalTime_s = tr.totalTime; r.totalPoints = (int)tr.size();
    auto t2 = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < tr.size(); i++) {
        const auto& p = tr.points[i];
        for (int j=0;j<6;j++){double v=std::fabs(p.velocity[j])*180/M_PI; if(v>r.peakVelocity_dps)r.peakVelocity_dps=v;}
        if (i%10==0||i==tr.size()-1) {
            auto rp = checker.getCollisionReport(p.config, true);
            if (rp.selfCollision) r.collisionCount++;
            if (rp.envCollision) r.envCollisionCount++;
            if (rp.selfMinDist_mm < r.minSelfDist_mm) r.minSelfDist_mm = rp.selfMinDist_mm;
            if (csv) { auto q=p.config.toDegrees();
                // FK2 v1.0.0 返回mm, 直接输出
                fprintf(csv,"%d,%d,%d,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%d,%d,%.1f,%.1f,%.1f\n",
                    ti,si,(int)i,p.time,q[0],q[1],q[2],q[3],q[4],q[5],
                    rp.selfMinDist_mm,rp.selfCollision?1:0,rp.envCollision?1:0,
                    rp.hasTcpPose?rp.tcpPose.X:0,rp.hasTcpPose?rp.tcpPose.Y:0,rp.hasTcpPose?rp.tcpPose.Z:0);
            }
        }
        if (traj && (i%20==0||i==tr.size()-1)) {
            auto q=p.config.toDegrees(); SO_COORD_REF tc; checker.forwardKinematics(p.config,tc);
            fprintf(traj,"%d %d %.4f",ti,si,p.time);
            for(int j=0;j<6;j++)fprintf(traj," %.4f",q[j]);
            for(int j=0;j<6;j++)fprintf(traj," %.3f",p.velocity[j]*180/M_PI);
            // FK2 v1.0.0 返回mm, 直接输出
            fprintf(traj," %.2f %.1f %.1f %.1f\n",r.minSelfDist_mm,tc.X,tc.Y,tc.Z);
        }
    }
    r.collCheckTime_ms = std::chrono::duration<double,std::milli>(std::chrono::high_resolution_clock::now()-t2).count();
    r.totalSegmentTime_ms = std::chrono::duration<double,std::milli>(std::chrono::high_resolution_clock::now()-tS).count();
    r.success = true; return r;
}

// ============================================================================
// 辅助: 检查直线路径是否穿越TCP排除区 (FK采样)
// ============================================================================
static bool isDirectPathInWorkspace(
    const JointConfig& start, const JointConfig& target,
    CollisionCheckerSO& checker,
    const std::vector<TCPPlannerConfig::TCPExclusionBox>& exclusionBoxes,
    int nSamples = 40)
{
    if (exclusionBoxes.empty()) return true;
    for (int i = 0; i <= nSamples; i++) {
        double t = (double)i / nSamples;
        JointConfig q = start.interpolate(target, t);
        SO_COORD_REF tcp;
        if (!checker.forwardKinematics(q, tcp)) return false;
        for (const auto& box : exclusionBoxes) {
            if (box.contains(tcp.X, tcp.Y, tcp.Z)) return false;
        }
    }
    return true;
}

// ============================================================================
// 辅助: 递归路径分割 — 当RRT失败时自动将长段拆成短段
// ============================================================================
static Path planWithSplitting(
    const JointConfig& start, const JointConfig& target,
    CollisionCheckerSO& checker, PathPlannerSO& planner,
    const std::vector<TCPPlannerConfig::TCPExclusionBox>& exclusionBoxes,
    int depth, int maxDepth,
    double& totalPlanMs, double& totalOptMs, int& totalIter, int& totalNodes)
{
    double dist = start.distanceTo(target);

    // 1) 直线路径: 自碰撞 + TCP排除区 + 箱子碰撞 三重检查
    bool selfOk = checker.isPathCollisionFree(start, target, 0.03);
    bool wsOk = isDirectPathInWorkspace(start, target, checker, exclusionBoxes);
    bool boxOk = true;
    const auto& boxCfg = planner.getConfig().boxCollision;
    if (boxCfg.enabled) {
        boxOk = checker.isPathBoxCollisionFree(start, target, boxCfg, 0.05);
    }
    if (selfOk && wsOk && boxOk) {
        Path p;
        Waypoint w0(start); w0.pathParam=0; Waypoint w1(target); w1.pathParam=1;
        p.waypoints.push_back(w0); p.waypoints.push_back(w1);
        printf("        %*s直线ok (dist=%.2f)\n", depth*2, "", dist);
        return p;
    }
    if (!boxOk) {
        printf("        %*s直线箱子碰撞! (dist=%.2f) → RRT*\n", depth*2, "", dist);
    }

    // 2) RRT* 规划
    auto pr = planner.plan(start, target);
    auto pt = planner.getTimingReport();
    totalPlanMs += pt.planningTotal_ms;
    totalOptMs += pt.optimizationTotal_ms;
    totalIter += pt.planIterations;
    totalNodes += pt.nodesExplored;
    printf("        %*sRRT* depth=%d dist=%.2f: plan=%.1fms nodes=%d\n",
           depth*2, "", depth, dist, pt.planningTotal_ms, pt.nodesExplored);

    if (pr.isSuccess() && !pr.optimizedPath.empty()) {
        // 验证RRT路径不穿过排除区
        if (!exclusionBoxes.empty() && pr.optimizedPath.size() >= 2) {
            bool wallOk = true;
            for (size_t i = 0; i+1 < pr.optimizedPath.waypoints.size() && wallOk; i++)
                wallOk = isDirectPathInWorkspace(pr.optimizedPath.waypoints[i].config,
                         pr.optimizedPath.waypoints[i+1].config, checker, exclusionBoxes);
            if (wallOk) return pr.optimizedPath;
            printf("        %*sRRT路径穿墙, 继续分割\n", depth*2, "");
        } else {
            return pr.optimizedPath;
        }
    }
    if (pr.isSuccess() && !pr.rawPath.empty()) {
        if (!exclusionBoxes.empty() && pr.rawPath.size() >= 2) {
            bool wallOk = true;
            for (size_t i = 0; i+1 < pr.rawPath.waypoints.size() && wallOk; i++)
                wallOk = isDirectPathInWorkspace(pr.rawPath.waypoints[i].config,
                         pr.rawPath.waypoints[i+1].config, checker, exclusionBoxes);
            if (wallOk) return pr.rawPath;
            printf("        %*sraw路径穿墙, 继续分割\n", depth*2, "");
        } else {
            return pr.rawPath;
        }
    }

    // 3) RRT失败 — 在中点分割, 递归规划两半段
    if (depth < maxDepth) {
        // 尝试多种分割比例, 选择第一个有效中间点
        double splitTs[] = {0.5, 0.4, 0.6, 0.33, 0.67};
        for (double t : splitTs) {
            JointConfig mid = start.interpolate(target, t);
            if (!checker.isCollisionFree(mid)) continue;

            // 检查中间点TCP是否水平
            SO_COORD_REF tcp;
            checker.forwardKinematics(mid, tcp);
            double Br = tcp.B * M_PI / 180, Cr = tcp.C * M_PI / 180;
            double zz = std::cos(Br) * std::cos(Cr);
            double downDev = std::acos(std::clamp(-zz, -1.0, 1.0)) * 180.0 / M_PI;
            if (downDev > 30.0) continue;

            printf("        %*s分割 t=%.2f (%.2f→%.2f+%.2f) TCP:Z↓dev=%.1f°\n",
                   depth*2, "", t, dist,
                   start.distanceTo(mid), mid.distanceTo(target), downDev);

            Path p1 = planWithSplitting(start, mid, checker, planner, exclusionBoxes,
                                        depth+1, maxDepth, totalPlanMs, totalOptMs, totalIter, totalNodes);
            if (p1.empty()) continue;

            Path p2 = planWithSplitting(mid, target, checker, planner, exclusionBoxes,
                                        depth+1, maxDepth, totalPlanMs, totalOptMs, totalIter, totalNodes);
            if (p2.empty()) continue;

            // 拼接: p1 + p2(跳过首点避免重复)
            Path combined;
            combined.waypoints = p1.waypoints;
            for (size_t i = 1; i < p2.waypoints.size(); i++)
                combined.waypoints.push_back(p2.waypoints[i]);
            combined.updatePathParameters();
            printf("        %*s拼接成功 (%zu航点)\n", depth*2, "", combined.size());
            return combined;
        }
        printf("        %*s所有分割点均无效\n", depth*2, "");
    }

    // 完全失败
    return Path{};
}

// ============================================================================
// 辅助: RRT*规划段 (含自动分割)
// ============================================================================
SegmentResult executeRRTStar(const char* name,
    const JointConfig& start, const JointConfig& target,
    RobotModel& robot, CollisionCheckerSO& checker,
    PathPlannerSO& planner, TimeParameterizer& param,
    FILE* csv, int ti, int si, FILE* traj,
    const std::vector<TCPPlannerConfig::TCPExclusionBox>& exclusionBoxes = {})
{
    SegmentResult r; r.name = name;
    auto tS = std::chrono::high_resolution_clock::now();

    double totalPlanMs = 0, totalOptMs = 0;
    int totalIter = 0, totalNodes = 0;

    Path path = planWithSplitting(start, target, checker, planner, exclusionBoxes,
                                  0, 2, totalPlanMs, totalOptMs, totalIter, totalNodes);

    r.planningTime_ms = totalPlanMs;
    r.optimizationTime_ms = totalOptMs;
    r.planIterations = totalIter;
    r.planNodes = totalNodes;

    if (path.empty()) {
        printf("        ⚠️  分割规划完全失败, 使用直线回退\n");
        Waypoint w0(start); w0.pathParam=0; Waypoint w1(target); w1.pathParam=1;
        path.waypoints.push_back(w0); path.waypoints.push_back(w1);
    } else {
        printf("        ✅ 规划成功 (%zu航点, %.2frad)\n",
               path.size(), path.totalLength());
    }

    // v6.1: 组合路径后验优化 — 对拼接的路径做捷径+简化
    if (path.waypoints.size() > 3) {
        auto tPostOpt = std::chrono::high_resolution_clock::now();
        int preN = (int)path.waypoints.size();
        double preLen = path.totalLength();
        // 捷径优化: 随机选两点, 如果直线无碰撞则删除中间点
        std::mt19937 rng(42);
        for (int iter = 0; iter < 300; iter++) {
            int n = (int)path.waypoints.size();
            if (n < 3) break;
            int i = rng() % n, j = rng() % n;
            if (i > j) std::swap(i, j);
            if (j - i < 2) continue;
            auto& qi = path.waypoints[i].config;
            auto& qj = path.waypoints[j].config;
            if (!checker.isPathCollisionFree(qi, qj, 0.02)) continue;
            // 排除区验证
            if (!exclusionBoxes.empty() &&
                !isDirectPathInWorkspace(qi, qj, checker, exclusionBoxes)) continue;
            // 删除中间点
            path.waypoints.erase(path.waypoints.begin() + i + 1,
                                  path.waypoints.begin() + j);
        }
        path.updatePathParameters();
        double postOptMs = std::chrono::duration<double,std::milli>(
            std::chrono::high_resolution_clock::now()-tPostOpt).count();
        double postLen = path.totalLength();
        printf("        🔧 路径优化: %d→%zu航点 %.2f→%.2frad (%.1fms)\n",
               preN, path.size(), preLen, postLen, postOptMs);
        totalOptMs += postOptMs;
        r.optimizationTime_ms = totalOptMs;
    }
    r.pathWaypoints = (int)path.waypoints.size();
    r.pathLength_rad = path.totalLength();
    auto t0 = std::chrono::high_resolution_clock::now();
    Trajectory tr = param.parameterize(path);
    r.paramTime_ms = std::chrono::duration<double,std::milli>(std::chrono::high_resolution_clock::now()-t0).count();
    if (tr.empty()) return r;
    r.totalTime_s = tr.totalTime; r.totalPoints = (int)tr.size();
    auto t2 = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < tr.size(); i++) {
        const auto& p = tr.points[i];
        for(int j=0;j<6;j++){double v=std::fabs(p.velocity[j])*180/M_PI;if(v>r.peakVelocity_dps)r.peakVelocity_dps=v;}
        if (i%10==0||i==tr.size()-1) {
            auto rp = checker.getCollisionReport(p.config, true);
            if (rp.selfCollision) r.collisionCount++;
            if (rp.envCollision) r.envCollisionCount++;
            if (rp.selfMinDist_mm < r.minSelfDist_mm) r.minSelfDist_mm = rp.selfMinDist_mm;
            // TCP排除区后验证 (穿墙检测)
            if (rp.hasTcpPose && !exclusionBoxes.empty()) {
                for (const auto& box : exclusionBoxes) {
                    if (box.contains(rp.tcpPose.X, rp.tcpPose.Y, rp.tcpPose.Z)) {
                        r.tcpExclusionViolations++;
                        break;
                    }
                }
            }
            if (csv) { auto q=p.config.toDegrees();
                // FK2 v1.0.0 返回mm, 直接输出
                fprintf(csv,"%d,%d,%d,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%d,%d,%.1f,%.1f,%.1f\n",
                    ti,si,(int)i,p.time,q[0],q[1],q[2],q[3],q[4],q[5],
                    rp.selfMinDist_mm,rp.selfCollision?1:0,rp.envCollision?1:0,
                    rp.hasTcpPose?rp.tcpPose.X:0,rp.hasTcpPose?rp.tcpPose.Y:0,rp.hasTcpPose?rp.tcpPose.Z:0);
            }
        }
        if (traj && (i%20==0||i==tr.size()-1)) {
            auto q=p.config.toDegrees(); SO_COORD_REF tc; checker.forwardKinematics(p.config,tc);
            fprintf(traj,"%d %d %.4f",ti,si,p.time);
            for(int j=0;j<6;j++)fprintf(traj," %.4f",q[j]);
            for(int j=0;j<6;j++)fprintf(traj," %.3f",p.velocity[j]*180/M_PI);
            // FK2 v1.0.0 返回mm, 直接输出
            fprintf(traj," %.2f %.1f %.1f %.1f\n",r.minSelfDist_mm,tc.X,tc.Y,tc.Z);
        }
    }
    r.collCheckTime_ms = std::chrono::duration<double,std::milli>(std::chrono::high_resolution_clock::now()-t2).count();
    r.totalSegmentTime_ms = std::chrono::duration<double,std::milli>(std::chrono::high_resolution_clock::now()-tS).count();
    r.success = true; return r;
}

// ============================================================================
// 主程序
// ============================================================================
int main() {
    printf("╔═══════════════════════════════════════════════════════════════════╗\n");
    printf("║   HR_S50-2000 码垛仿真 v8.0 — v3.1安全优先 + 收缩通过策略        ║\n");
    printf("║   侧面进入框架 + RRT*搬运 + 箱子OBB旋转避障 + v3.1布局           ║\n");
    printf("╚═══════════════════════════════════════════════════════════════════╝\n\n");

    auto t_global = std::chrono::high_resolution_clock::now();

    printf("━━━━━━ 阶段1: 系统初始化 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n");

    RobotModel robot;
    CollisionCheckerSO checker(robot);
    auto tInit = std::chrono::high_resolution_clock::now();
    if (!checker.initialize()) { fprintf(stderr,"❌ 碰撞检测初始化失败!\n"); return 1; }
    double initMs = std::chrono::duration<double,std::milli>(std::chrono::high_resolution_clock::now()-tInit).count();
    printf("  碰撞检测: ✅ libHRCInterface.so (%.2f ms)\n", initMs);

    TCPPlannerConfig planConfig;
    planConfig.maxIterations = 80000; planConfig.maxPlanningTime = 20.0;
    planConfig.stepSize = 0.25; planConfig.goalBias = 0.20;
    planConfig.shortcutIterations = 200;
    planConfig.splineResolution = 50; planConfig.collisionResolution = 0.02;
    planConfig.rewireRadius = 0.4;
    // 码垛场景: TCP必须保持水平 (吸盘朝下, 仅允许Z轴旋转)
    planConfig.constrainTcpHorizontal = true;

    // TCP工作空间排除区 (AABB盒子) — 阻止TCP穿过框架三面封闭墙
    // 框架位置: CY=680, HW=600, NY=355, FY=1005, ZB=-800, ZT=1200 (mm)
    // 仅在框架空间范围内约束, 不影响框架外自由运动
    {
        double _fcy = scene::frameCY();
        double _fHW = scene::FRM_W/2;
        double _fNY = _fcy - scene::FRM_D/2;
        double _fFY = _fcy + scene::FRM_D/2;
        double _fZB = -scene::baseZ;
        double _fZT = scene::FRM_H - scene::baseZ;
        using Box = TCPPlannerConfig::TCPExclusionBox;
        planConfig.tcpExclusionBoxes = {
            // 后墙: Y > fFY, X ∈ frame, Z ∈ frame → 拒绝
            Box{-_fHW, _fHW, _fFY, 1e6, _fZB, _fZT},
            // 左墙: X < -fHW, Y ∈ frame, Z ∈ frame → 拒绝
            Box{-1e6, -_fHW, _fNY, _fFY, _fZB, _fZT},
            // 右墙: X > fHW, Y ∈ frame, Z ∈ frame → 拒绝
            Box{_fHW, 1e6, _fNY, _fFY, _fZB, _fZT},
        };
        printf("  排除区: 后墙(Y>%.0f,|X|<%.0f,Z∈[%.0f,%.0f])"
               " 左墙(X<-%.0f) 右墙(X>%.0f) (mm)\n",
               _fFY, _fHW, _fZB, _fZT, _fHW, _fHW);
    }

    PathPlannerSO planner(robot, checker, planConfig);
    printf("  路径规划: %s Informed RRT*\n",
           planConfig.constrainTcpHorizontal ? "TCP-Horizontal" : "Free-TCP");

    // ── 箱子-机械臂碰撞检测配置 (搬运段启用) ──
    // SO库pair(6,4)限制: 工具球z=-400mm无法覆盖箱子体积(z=0~-250mm)
    // 独立检测: getUIInfo碰撞体 + 26点OBB采样 → 精确箱子-臂碰撞
    // 在搬运规划前设置, 搬运完成后关闭
    planConfig.boxCollision.enabled     = false;  // 初始关闭, 搬运段动态启用
    planConfig.boxCollision.boxLengthX  = scene::BOX_LX;
    planConfig.boxCollision.boxWidthY   = scene::BOX_WY;
    planConfig.boxCollision.boxHeightZ  = scene::BOX_HZ;
    planConfig.boxCollision.safetyMargin= 40.0;   // v6.4: 40mm安全裕度 (Base+LowerArm硬约束, Elbow诊断)
    printf("  📦 箱子碰撞: %.0f×%.0f×%.0fmm margin=%.0fmm (Base+LArm硬约束, 搬运段启用)\n",
           planConfig.boxCollision.boxLengthX,
           planConfig.boxCollision.boxWidthY,
           planConfig.boxCollision.boxHeightZ,
           planConfig.boxCollision.safetyMargin);

    // 无TCP水平约束的规划器 — 用于不搬运箱子的回程段
    // (回程不携带箱子, 无需保持TCP水平, 接受率~100% → 更少迭代)
    TCPPlannerConfig freeConfig = planConfig;
    freeConfig.constrainTcpHorizontal = false;
    freeConfig.boxCollision.enabled = false;  // 回程不携带箱子, 无需箱子碰撞检测
    freeConfig.maxIterations = 30000;
    freeConfig.maxPlanningTime = 10.0;
    PathPlannerSO plannerFree(robot, checker, freeConfig);
    printf("  回程规划: Free-TCP Informed RRT* (30K iter, 10s, 无箱子段)\n");

    auto tpConfig = TimeParameterizationConfig::fromRobotParams(robot.getParams());
    tpConfig.profileType = VelocityProfileType::SCurve;
    tpConfig.samplePeriod = 0.004; tpConfig.velocityScaling = 1.0;
    TimeParameterizer parameterizer(tpConfig);
    printf("  轨迹生成: 7段S曲线 (4ms, 250Hz)\n\n");

    // ---- 工作空间探测 ----
    printf("━━━━━━ 阶段1b: 工作空间探测 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n");
    {
        double pq[][6] = {
            {0,-50,30,0,20,0}, {0,-60,25,0,35,0}, {0,-70,20,0,50,0},
            {0,-75,15,0,60,0}, {0,-65,20,0,45,0}, {0,-60,30,0,30,0},
            {10,-65,25,0,40,10},{-10,-65,25,0,40,-10},{-90,-50,40,0,10,-90},
        };
        printf("  %-36s %8s %8s %8s %8s\n","关节配置(deg)","X_mm","Y_mm","Z_mm","r_mm");
        for (auto& p : pq) {
            auto q = JointConfig::fromDegrees({p[0],p[1],p[2],p[3],p[4],p[5]});
            SO_COORD_REF tc; checker.forwardKinematics(q, tc);
            double r = std::sqrt(tc.X*tc.X+tc.Y*tc.Y); // FK2 v1.0.0 返回mm
            printf("  [%4.0f,%4.0f,%4.0f,%4.0f,%4.0f,%4.0f]  %8.1f %8.1f %8.1f %8.0f\n",
                   p[0],p[1],p[2],p[3],p[4],p[5], tc.X,tc.Y,tc.Z, r);
        }
        printf("\n");
    }

    // ---- 环境碰撞配置 ----
    printf("━━━━━━ 阶段2: 环境碰撞配置 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n");
    { bool f[7]={1,1,1,1,1,1,1}; checker.setLinkEnvCollisionEnabled(f); }
    printf("  连杆-环境碰撞: ✅ 7/7\n\n");

    double fcy = scene::frameCY();
    double fHW = scene::FRM_W/2, fNY = fcy-scene::FRM_D/2, fFY = fcy+scene::FRM_D/2;
    double fZB = -scene::baseZ, fZT = scene::FRM_H-scene::baseZ, fR = scene::FRM_TUBE_R+20;

    printf("  === 框架立柱 (envId 1-4) Y=[%.0f,%.0f] ===\n", fNY, fFY);
    int frmIds[] = {1,2,3,4};
    double frmX[] = {-fHW, fHW, fHW, -fHW};
    double frmY[] = {fNY, fNY, fFY, fFY};
    for (int i=0;i<4;i++) {
        bool ok = checker.addEnvObstacleCapsule(frmIds[i],
            Eigen::Vector3d(frmX[i],frmY[i],fZB), Eigen::Vector3d(frmX[i],frmY[i],fZT), fR);
        printf("    envId=%d (%.0f,%.0f): %s\n", frmIds[i], frmX[i], frmY[i], ok?"✅":"❌");
    }

    // 2根顶梁 (平行于Y轴, 连接同侧近端→远端立柱顶部)
    printf("\n  === 框架顶梁 (envId 20-21) ===\n");
    {
        // 左侧顶梁: 左近端→左远端 (X=-fHW, Z=fZT)
        bool ok0 = checker.addEnvObstacleCapsule(20,
            Eigen::Vector3d(-fHW, fNY, fZT), Eigen::Vector3d(-fHW, fFY, fZT), fR);
        printf("    envId=20 左侧顶梁: %s\n", ok0?"✅":"❌");
        // 右侧顶梁: 右近端→右远端 (X=fHW, Z=fZT)
        bool ok1 = checker.addEnvObstacleCapsule(21,
            Eigen::Vector3d(fHW, fNY, fZT), Eigen::Vector3d(fHW, fFY, fZT), fR);
        printf("    envId=21 右侧顶梁: %s\n", ok1?"✅":"❌");
    }

    // v8.1: 仅后(远端)X顶梁 — 前(近端)面完全开放, 无顶横梁 (便于机械臂进出)
    // 参考: S50_Palletizing_Scene_Description.md §5.2
    printf("\n  === 框架X方向顶梁 (仅后端 envId=9) ===\n");
    {
        // v8.1: 前X顶梁(envId=8)已移除 — 框架近端面完全开放
        // 后(远端)X顶梁: 左远→右远 (Y=fFY, Z=fZT)
        bool ok1 = checker.addEnvObstacleCapsule(9,
            Eigen::Vector3d(-fHW, fFY, fZT), Eigen::Vector3d(fHW, fFY, fZT), fR);
        printf("    envId=9 后X顶梁 (Y=%.0f): %s\n", fFY, ok1?"✅":"❌");
        printf("    envId=8 前X顶梁: 已移除 (近端面完全开放)\n");
    }

    // ---- 框架面板碰撞 — Lozenge OBB 实体面板 (SO库棱体碰撞) ----
    // v6.1: 使用 addEnvObstacleLozenge (棱体=圆角OBB) 替代稀疏胶囊横杆
    // Lozenge 参数: ref2local[6]={rx,ry,rz(deg),tx,ty,tz(mm)}, offset[3](mm), xLen,yLen,zLen(mm), radius(mm)
    printf("\n  === 框架面板 (Lozenge OBB v6.1) ===\n");
    {
        const double wallThickness = 100.0;  // 面板厚度 100mm
        const double wallRadius = 50.0;      // 圆角半径 50mm
        double zMid = (fZB + fZT) / 2.0;    // Z中点
        double zLen = fZT - fZB;             // Z范围 (框架高度)
        double yMid = (fNY + fFY) / 2.0;    // Y中点

        // 后面 (envId 5): Y=fFY 平面, 沿X方向铺满
        {
            double ref2local[6] = {0, 0, 0, 0, fFY, zMid};  // 面板中心
            bool ok = checker.addEnvObstacleLozenge(5, ref2local,
                Eigen::Vector3d(0, 0, 0), scene::FRM_W, wallThickness, zLen, wallRadius);
            printf("    envId=5 后面 Lozenge(%.0fx%.0fx%.0f r=%.0f) @ Y=%.0f: %s\n",
                   scene::FRM_W, wallThickness, zLen, wallRadius, fFY, ok?"✅":"❌");
            // 如果Lozenge失败, 回退到密集胶囊体
            if (!ok) {
                printf("    ⚠️ Lozenge不可用, 回退胶囊体\n");
                const int nLevels = 6;
                const double wallR = 250.0;
                const double zSpan = fZT - fZB;
                for (int i = 0; i < nLevels; i++) {
                    double zi = fZB + zSpan * (2*i + 1) / (2*nLevels);
                    int eid = 62 + i;  // envId 62-67 (后面回退, 不与X顶梁8-9冲突)
                    checker.addEnvObstacleCapsule(eid,
                        Eigen::Vector3d(-fHW, fFY, zi), Eigen::Vector3d(fHW, fFY, zi), wallR);
                }
            }
        }

        // 左面 (envId 6): X=-fHW 平面, 沿Y方向铺满
        {
            double ref2local[6] = {0, 0, 0, -fHW, yMid, zMid};
            bool ok = checker.addEnvObstacleLozenge(6, ref2local,
                Eigen::Vector3d(0, 0, 0), wallThickness, scene::FRM_D, zLen, wallRadius);
            printf("    envId=6 左面 Lozenge(%.0fx%.0fx%.0f) @ X=-%.0f: %s\n",
                   wallThickness, scene::FRM_D, zLen, fHW, ok?"✅":"❌");
            if (!ok) {
                printf("    ⚠️ Lozenge不可用, 回退胶囊体\n");
                const int nLevels = 6;
                const double wallR = 250.0;
                const double zSpan = fZT - fZB;
                for (int i = 0; i < nLevels; i++) {
                    double zi = fZB + zSpan * (2*i + 1) / (2*nLevels);
                    int eid = 68 + i;  // envId 68-73 (左面回退, 不与其他面板冲突)
                    checker.addEnvObstacleCapsule(eid,
                        Eigen::Vector3d(-fHW, fNY, zi), Eigen::Vector3d(-fHW, fFY, zi), wallR);
                }
            }
        }

        // 右面 (envId 7): X=fHW 平面, 沿Y方向铺满
        {
            double ref2local[6] = {0, 0, 0, fHW, yMid, zMid};
            bool ok = checker.addEnvObstacleLozenge(7, ref2local,
                Eigen::Vector3d(0, 0, 0), wallThickness, scene::FRM_D, zLen, wallRadius);
            printf("    envId=7 右面 Lozenge(%.0fx%.0fx%.0f) @ X=+%.0f: %s\n",
                   wallThickness, scene::FRM_D, zLen, fHW, ok?"✅":"❌");
            if (!ok) {
                printf("    ⚠️ Lozenge不可用, 回退胶囊体\n");
                const int nLevels = 6;
                const double wallR = 250.0;
                const double zSpan = fZT - fZB;
                for (int i = 0; i < nLevels; i++) {
                    double zi = fZB + zSpan * (2*i + 1) / (2*nLevels);
                    int eid = 74 + i;  // envId 74-79 (右面回退, 不与其他面板冲突)
                    checker.addEnvObstacleCapsule(eid,
                        Eigen::Vector3d(fHW, fNY, zi), Eigen::Vector3d(fHW, fFY, zi), wallR);
                }
            }
        }

        printf("    面板策略: Lozenge OBB优先 → 胶囊体回退 (6层 r=250mm)\n");
    }

    printf("\n  === 电箱 (envId 10-13) ===\n");
    double cHW=scene::CAB_W/2, cHD=scene::CAB_D/2, cZB=fZB, cZT=-80, cR=80;
    struct CO { int id; double x1,y1,x2,y2; };
    CO cabObs[] = {
        {10,-cHW,-cHD,-cHW, cHD}, {11, cHW,-cHD, cHW, cHD},
        {12,-cHW,-cHD, cHW,-cHD}, {13,-cHW, cHD, cHW, cHD},
    };
    for (auto& c : cabObs) {
        bool ok = checker.addEnvObstacleCapsule(c.id,
            Eigen::Vector3d(c.x1,c.y1,cZB), Eigen::Vector3d(c.x2,c.y2,cZT), cR);
        printf("    envId=%d: %s\n", c.id, ok?"✅":"❌");
    }

    printf("\n  === 传送带 (envId 15-17) ===\n");
    double cvX=scene::convCX(), cvY=scene::CONV_CY, cvSZ=scene::convSurfBase();
    double cvHW=scene::CONV_W/2, cvHL=scene::CONV_LEN/2;
    struct CV { int id; double x1,y1,z1,x2,y2,z2,r; };
    CV convObs[] = {
        {15, cvX-cvHW, cvY-cvHL, -200, cvX-cvHW, cvY+cvHL, -200, 80},
        {16, cvX+cvHW, cvY-cvHL, -200, cvX+cvHW, cvY+cvHL, -200, 80},
        {17, cvX, cvY-cvHL, cvSZ-20, cvX, cvY+cvHL, cvSZ-20, cvHW},
    };
    for (auto& v : convObs) {
        bool ok = checker.addEnvObstacleCapsule(v.id,
            Eigen::Vector3d(v.x1,v.y1,v.z1), Eigen::Vector3d(v.x2,v.y2,v.z2), v.r);
        printf("    envId=%d: %s\n", v.id, ok?"✅":"❌");
    }

    const double boxEnvR = 250.0;
    printf("\n  环境障碍: 电箱(4) + 传送带(3) + 框架立柱(4) + Y顶梁(2) + X顶梁(2) + 面板Lozenge(3) + 已放箱子(动态)\n");
    printf("  面板封锁: 后面+左面+右面 Lozenge OBB (前面开放, 机械臂入口)\n");
    printf("  布局间距: FRAME_GAP=%.0f CONV_GAP=%.0f CONV_LEN=%.0f (mm)\n",
           scene::FRAME_GAP, scene::CONV_GAP, scene::CONV_LEN);
    printf("  框架: X=[%.0f,%.0f] Y=[%.0f,%.0f]  传送带: X=[%.0f,%.0f] Y=[%.0f,%.0f]\n",
           -fHW, fHW, fNY, fFY, cvX-cvHW, cvX+cvHW, cvY-cvHL, cvY+cvHL);
    printf("  X间距: %.0f  Y间距: %.0f (无重叠✓)\n\n",
           (cvX-cvHW) - fHW, fNY - (cvY+cvHL));

    // ---- 码垛布局 + IK求解 ----
    printf("━━━━━━ 阶段3: 单箱布局 + IK求解 ━━━━━━━━━━━━━━━━━━━━━━━━━\n\n");

    double palSurf = scene::palletSurfBase();
    // v8.1: 箱子放置位置 — 后左角贴壁放置
    // 用户要求: "箱子要首先放在蓝框的最里面, 侧面要挨着两个蓝框的侧面"
    // 后墙内壁: fFY - WALL_COLL_R; 左墙内壁: -(fHW - WALL_COLL_R)
    // 箱子侧面距墙内壁 WALL_GAP=5mm
    constexpr double WALL_COLL_R = 50.0;  // 墙面/立柱碰撞半径 (mm)
    constexpr double WALL_GAP    = 5.0;   // 箱子与墙面间隙 (mm)
    double boxPlaceX = -(fHW - WALL_COLL_R - scene::BOX_LX/2.0 - WALL_GAP);  // 左墙贴壁 ≈ -370mm
    double boxPlaceY = fFY - WALL_COLL_R - scene::BOX_WY/2.0 - WALL_GAP;     // 后墙贴壁 ≈ 1184.4mm
    double boxPlaceZ = palSurf + scene::BOX_HZ;  // 第一层顶(基座) = -600+250 = -350mm (世界450mm)

    printf("  框架CY=%.1f  fNY=%.1f  fFY=%.1f  托盘面Z(base)=%.1f\n", fcy, fNY, fFY, palSurf);
    printf("  箱子放置(v8.1后左角贴壁): (%.1f, %.1f, %.1f) mm\n", boxPlaceX, boxPlaceY, boxPlaceZ);
    printf("  距左墙: %.1fmm  距右墙: %.1fmm  距后墙: %.1fmm  距前道: %.1fmm\n\n",
           fHW + boxPlaceX - scene::BOX_LX/2.0,
           fHW - boxPlaceX - scene::BOX_LX/2.0,
           fFY - boxPlaceY - scene::BOX_WY/2.0,
           boxPlaceY - scene::BOX_WY/2.0 - fNY);

    constexpr int numPositions = 1;
    std::vector<BoxTarget> boxTargets;
    {
        BoxTarget bt;
        bt.label = "BK-L";
        bt.pos_mm = Eigen::Vector3d(boxPlaceX, boxPlaceY, boxPlaceZ);
        bt.approach_mm = Eigen::Vector3d(boxPlaceX, boxPlaceY, boxPlaceZ + 300);
        boxTargets.push_back(bt);
    }

    printf("  %-10s  %8s %8s %8s\n","位置","X_mm","Y_mm","Z_mm");
    for (auto& b : boxTargets)
        printf("  %-10s  %8.1f %8.1f %8.1f\n", b.label.c_str(), b.pos_mm.x(), b.pos_mm.y(), b.pos_mm.z());

    printf("\n  === IK求解 (放料位) ===\n");
    // v8.2: 放料在后左角贴壁 (X≈-370, Y≈1184, Z≈-350 base = 450mm world)
    // TCP目标大幅低于基座 (Z_base=-350mm), 臂需要向前/下伸展
    // J2接近0°(臂水平)~-30°, J3=50°~90° (前臂折回使TCP朝下)
    // ★关键: 放料J1必须接近框架入口J1, 否则RRT*路径距离过大
    std::vector<JointConfig> placeSeeds;
    double sA[][6] = {
        // 低Z目标种子: J2=-10°~-40°, J3=50°~90° (臂伸展向下)
        {-95,-20,60,0,90,-95},{-100,-25,65,0,90,-100},{-105,-15,55,0,90,-105},
        {-110,-30,70,0,90,-110},{-90,-25,60,0,90,-90},{-100,-10,50,0,90,-100},
        {-95,-35,75,0,90,-95},{-108,-20,65,0,90,-108},{-100,-15,80,0,90,-100},
        {-92,-25,60,0,90,-92},{-98,-20,55,0,90,-98},{-115,-25,65,0,90,-115},
        // 中间范围种子
        {-100,-40,45,0,90,-100},{-95,-45,40,0,90,-95},{-105,-35,50,0,90,-105},
        // 备选: J1≈260-280° (等效, 避免±180°跳变)
        {265,-20,60,0,90,265},{270,-25,65,0,90,270},{260,-15,55,0,90,260},
        {275,-30,70,0,90,275},{268,-20,65,0,90,268},{272,-25,60,0,90,272},
    };
    for (auto& s : sA) placeSeeds.push_back(JointConfig::fromDegrees({s[0],s[1],s[2],s[3],s[4],s[5]}));

    struct PlaceConfig { JointConfig approach, place; double aErr, pErr; };
    std::vector<PlaceConfig> placeConfigs;

    auto tIK = std::chrono::high_resolution_clock::now();
    int ikFail = 0;
    for (int i=0; i<numPositions; i++) {
        auto& bt = boxTargets[i];
        // 优先使用SO IK (精确6D位姿, 保证TCP水平)
        // 回退到NumericalIK (仅位置+关节投影)
        auto ikP = soIKHorizontal(checker, robot, bt.pos_mm, placeSeeds, 3.0);
        if (!ikP.converged) ikP = multiStartIK(checker, robot, bt.pos_mm, placeSeeds, 3.0, true);
        std::vector<JointConfig> aSeeds = placeSeeds;
        if (ikP.converged) aSeeds.insert(aSeeds.begin(), ikP.config);
        auto ikA = soIKHorizontal(checker, robot, bt.approach_mm, aSeeds, 3.0);
        if (!ikA.converged) ikA = multiStartIK(checker, robot, bt.approach_mm, aSeeds, 3.0, true);
        PlaceConfig pc; pc.place=ikP.config; pc.approach=ikA.config;
        pc.pErr=ikP.posError_mm; pc.aErr=ikA.posError_mm;
        placeConfigs.push_back(pc);
        if (!ikP.converged||!ikA.converged) ikFail++;
        printf("    [%2d] %-10s place:%s(%.1fmm,%diter) approach:%s(%.1fmm,%diter)\n",
               i, bt.label.c_str(), ikP.converged?"✅":"❌", ikP.posError_mm, ikP.iterations,
               ikA.converged?"✅":"❌", ikA.posError_mm, ikA.iterations);
        if (ikP.converged) { placeSeeds.insert(placeSeeds.begin(), ikP.config);
            if (placeSeeds.size()>20) placeSeeds.pop_back(); }
    }
    double ikMs = std::chrono::duration<double,std::milli>(std::chrono::high_resolution_clock::now()-tIK).count();
    printf("\n  IK: %.1fms (%d/%d ok)\n", ikMs, numPositions-ikFail, numPositions);

    // FK验证 (位置 + TCP朝向 + 实际Z轴偏差)
    printf("\n  === FK验证 ===\n");
    for (int i=0;i<numPositions;i++) {
        SO_COORD_REF tc; checker.forwardKinematics(placeConfigs[i].place, tc);
        auto& t=boxTargets[i].pos_mm;
        double e=std::sqrt(std::pow(tc.X-t.x(),2)+std::pow(tc.Y-t.y(),2)+std::pow(tc.Z-t.z(),2));
        auto& pq = placeConfigs[i].place;
        double tcpH_deg = (-pq.q[1]+pq.q[2]+pq.q[4])*180.0/M_PI;
        // TCP水平偏差 = |B| (FK2的B=TCP切面倾斜角, B=0→TCP精确朝下)
        double downDev = std::fabs(tc.B);
        // 打印实际关节角 (用于调试约束)
        printf("    %-10s q=[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]deg\n",
               boxTargets[i].label.c_str(),
               pq.q[0]*180/M_PI, pq.q[1]*180/M_PI, pq.q[2]*180/M_PI,
               pq.q[3]*180/M_PI, pq.q[4]*180/M_PI, pq.q[5]*180/M_PI);
        printf("    %-10s err=%.1fmm tcpH=%.1f° A=%.1f B=%.1f C=%.1f |B|=%.1f°%s\n",
               boxTargets[i].label.c_str(), e, tcpH_deg, tc.A, tc.B, tc.C, downDev,
               (e<5 && downDev<5)?"":"  ⚠️");
    }

    // 取料位IK (TCP保持水平: q5=180+q2-q3, q6=0)
    printf("\n  === 取料位IK ===\n");
    // v8.0: 拾取位 = (CONV_CX, CONV_CY, convSurf+BOX_HZ)
    // v3.1: x_c=977.2, y_c=176.7 (传送带在+X方向, 向+Y偏移)
    double pkX=scene::CONV_CX, pkY=scene::CONV_CY, pkZ=scene::convSurfBase()+scene::BOX_HZ;
    Eigen::Vector3d pickTgt(pkX,pkY,pkZ), pickApprTgt(pkX,pkY,pkZ+300);
    std::vector<JointConfig> pkSeeds;
    // v8.0: ★关键 — 拾取种子必须与放料同一运动学分支 (J4≈-140, J5≈90)
    // 否则搬运段关节距离过大 (530°→ RRT*失败)
    // 方向: atan2(176.7, 977.2)≈10° from +X → J1≈170°~190° in FK2
    // 使用放料分支参数 (J4≈-140, J5=90) 确保运动学一致性
    {
        auto pd = placeConfigs[0].place.toDegrees();  // 放料IK结果 (J5≈90°分支)
        // 主种子: 使用放料分支的J4/J5, J1适配拾取方向
        for (double j1 : {170.0, 175.0, 180.0, 185.0, 165.0, -175.0, -170.0, 160.0}) {
            pkSeeds.push_back(JointConfig::fromDegrees(
                {j1, pd[1], pd[2], pd[3], pd[4], j1}));
            pkSeeds.push_back(JointConfig::fromDegrees(
                {j1, pd[1]+5, pd[2]-10, pd[3]+10, pd[4], j1+5}));
            pkSeeds.push_back(JointConfig::fromDegrees(
                {j1, pd[1]-5, pd[2]+5, pd[3]-10, pd[4], j1-5}));
        }
        // 备选: 通用J5=90种子 (不同J4/J2/J3组合)
        double bkS[][6] = {{175,-70,130,-150,90,175},{180,-65,125,-140,90,180},{170,-75,135,-130,90,170},
                           {185,-60,120,-145,90,185},{165,-80,140,-155,90,165},{-175,-70,130,-135,90,-175}};
        for (auto& s:bkS) pkSeeds.push_back(JointConfig::fromDegrees({s[0],s[1],s[2],s[3],s[4],s[5]}));
    }

    // v8.0: Pick IK — 先尝试SO IK (精确6D), 回退NumericalIK (DLS)
    auto ikPk = soIKHorizontal(checker, robot, pickTgt, pkSeeds, 3.0);
    if (!ikPk.converged) ikPk = multiStartIK(checker, robot, pickTgt, pkSeeds, 3.0, true);
    auto pkApSeeds = pkSeeds;
    if (ikPk.converged) pkApSeeds.insert(pkApSeeds.begin(), ikPk.config);
    auto ikPkA = soIKHorizontal(checker, robot, pickApprTgt, pkApSeeds, 3.0);
    if (!ikPkA.converged) ikPkA = multiStartIK(checker, robot, pickApprTgt, pkApSeeds, 3.0, true);

    auto PICK_POS = ikPk.config;
    auto PICK_APPROACH = ikPkA.config;
    {
        SO_COORD_REF pkTcp, paTcp;
        checker.forwardKinematics(PICK_POS, pkTcp);
        checker.forwardKinematics(PICK_APPROACH, paTcp);
        auto downDev = [](const SO_COORD_REF& tc) {
            double Br=tc.B*M_PI/180, Cr=tc.C*M_PI/180;
            double zz=std::cos(Br)*std::cos(Cr);
            return std::acos(std::clamp(-zz,-1.0,1.0))*180.0/M_PI;
        };
        printf("    取料: q=[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]deg\n",
               PICK_POS.q[0]*180/M_PI, PICK_POS.q[1]*180/M_PI, PICK_POS.q[2]*180/M_PI,
               PICK_POS.q[3]*180/M_PI, PICK_POS.q[4]*180/M_PI, PICK_POS.q[5]*180/M_PI);
        printf("    取料: %s(%.1fmm, B=%.1f C=%.1f Z↓dev=%.1f°)\n",
               ikPk.converged?"✅":"❌",ikPk.posError_mm, pkTcp.B, pkTcp.C, downDev(pkTcp));
        printf("    接近: %s(%.1fmm, B=%.1f C=%.1f Z↓dev=%.1f°)\n",
               ikPkA.converged?"✅":"❌",ikPkA.posError_mm, paTcp.B, paTcp.C, downDev(paTcp));
    }

    // === J1旋转方向优化: 选择最短弧 ===
    // FK2约定: J1=0°→-X方向. 取料J1≈144° (朝+X), 放料J1≈126° (-X,+Y方向)
    // 取→放旋转: 144°→126° = 18° (本身已是短弧, 无需360°偏移)
    // 若IK返回不同J1范围, 此优化器自动选择最短弧路径
    printf("\n  === J1旋转方向优化 (短弧选择) ===\n");
    {
        auto pickDeg_opt = PICK_APPROACH.toDegrees();
        double pickJ1 = pickDeg_opt[0];  // ≈144° (FK2: J1=0°→-X, 取料在+X方向)
        auto& pc0 = placeConfigs[0];
        auto placeDeg_opt = pc0.approach.toDegrees();
        double origJ1 = placeDeg_opt[0]; // ≈126° (FK2: 放料在-X,+Y方向)
        double origDist = std::fabs(pickJ1 - origJ1);

        double bestJ1 = origJ1, bestDist = origDist;
        for (double shift : {360.0, -360.0}) {
            double cand = origJ1 + shift;
            if (cand < -360.0 || cand > 360.0) continue;
            double d = std::fabs(pickJ1 - cand);
            if (d < bestDist) { bestDist = d; bestJ1 = cand; }
        }

        if (std::fabs(bestJ1 - origJ1) > 1.0) {
            double shift_rad = (bestJ1 - origJ1) * M_PI / 180.0;
            pc0.place.q[0] += shift_rad;
            pc0.approach.q[0] += shift_rad;
            printf("  放料J1: %.1f° → %.1f° (旋转: %.1f° → %.1f°, 节省%.0f%%)\n",
                   origJ1, bestJ1, origDist, bestDist,
                   100.0*(1.0 - bestDist/origDist));
        } else {
            printf("  J1已最优: %.1f° (旋转: %.1f°)\n", origJ1, bestDist);
        }
        printf("  路径方向: 取J1=%.1f° → 放J1=%.1f° (%s, 经过+X/+Y象限)\n",
               pickJ1, bestJ1,
               bestJ1 > pickJ1 ? "J1递增" : "J1递减");
    }

    // v8.0: v3.1安全优先布局 + 收缩通过策略
    printf("  📌 v8.0: v3.1安全优先布局 + 收缩通过策略 + TCP旋转避障\n");

    // ---- 单箱搬运执行 ----
    printf("\n━━━━━━ 阶段4: 单箱搬运 (v8.0 收缩通过+RRT*搬运) ━━━━━━━━━━━━━\n\n");

    FILE* fpCsv = fopen("data/so_palletizing_profile.csv","w");
    if(fpCsv) fprintf(fpCsv,"task,segment,step,time_s,q1,q2,q3,q4,q5,q6,"
                      "selfDist_mm,selfCollision,envCollision,tcpX_mm,tcpY_mm,tcpZ_mm\n");
    FILE* fpTraj = fopen("data/so_palletizing_trajectory.txt","w");
    if(fpTraj) { fprintf(fpTraj,"# HR_S50-2000 码垛v8.0 — 收缩通过 (7段RRT*+TCP旋转避障)\n");
                 fprintf(fpTraj,"# task seg time q1..q6 v1..v6 dist tcpX tcpY tcpZ\n"); }

    double totalMotionTime=0; int totalCollisions=0, totalEnvCollisions=0;
    double globalMinDist_mm=1e10; int totalSegments=0;
    double grandTotalPlanMs=0, grandTotalParamMs=0, grandTotalCollMs=0;

    auto& bt = boxTargets[0];
    auto& pc = placeConfigs[0];

    printf("  目标: %s TCP=(%.0f,%.0f,%.0f)mm\n", bt.label.c_str(),
           bt.pos_mm.x(), bt.pos_mm.y(), bt.pos_mm.z());
    printf("  取料: TCP=(%.0f,%.0f,%.0f)mm → 放料: TCP=(%.0f,%.0f,%.0f)mm\n\n",
           pickTgt.x(), pickTgt.y(), pickTgt.z(),
           bt.pos_mm.x(), bt.pos_mm.y(), bt.pos_mm.z());

    // v8.0: 收缩通过策略 — 搬运路径经由框架前方开口进入
    //
    // v3.1布局 (FRAME_GAP=404mm): 框架前开口宽1100mm, 足够通过工具球(Ø450)+臂链
    // 框架入口航路点: (x_place, fNY, z_place+Δz_app)
    // Δz_app=150mm (S50_Layout_Optimization_Model_v3.md §5.4)
    //
    // 搬运策略: PICK_APPROACH → FRAME_ENTRY → place_approach
    //   seg 2: 拾取→框架入口 (RRT*, TCP-Horizontal, box-env检测)
    //   seg 3: 框架入口→放料接近 (RRT*, TCP-Horizontal, box-env检测)

    // 取/放 接近配置
    auto paDeg = PICK_APPROACH.toDegrees();  // 取料接近
    auto plDeg = pc.approach.toDegrees();    // 放料接近

    // === v8.0: 框架入口航路点 (收缩通过策略, v3.md §5.4) ===
    // 航路点位于框架前开口处: (x_place, fNY, z_place+150mm)
    // 机械臂收缩手臂从框架前方开口进入, 而非从顶部翻越 (节省450mm垂直距离)
    printf("  === 框架入口航路点 (收缩通过策略 v8.0) ===\n");
    constexpr double DELTA_Z_APP = 150.0;  // 接近高度偏移 (v3.md §3.4)
    double wpX = boxPlaceX;                 // 与放置位X对齐
    double wpY = fNY;                       // 框架前道 (近端立柱连线)
    double wpZ = boxPlaceZ + DELTA_Z_APP;   // 放置位上方150mm
    double frameEntryWidth = scene::FRM_W - 2*fR;  // 净通道宽度 (mm)
    printf("  航路点TCP目标: (%.1f, %.1f, %.1f) mm\n", wpX, wpY, wpZ);
    printf("  框架前开口: Y=%.1f, 净宽=%.0fmm (需≥%.0fmm)\n",
           fNY, frameEntryWidth, 2*225.0 + 2*50.0);
    printf("  vs顶部翻越: Z_top=%.0f+r_tool=225→需Z≥%.0f (收缩通过Z=%.0f, 节省%.0fmm)\n",
           fZT, fZT + 225.0, wpZ, (fZT + 225.0) - wpZ);

    // IK求解框架入口航路点 (TCP保持水平)
    Eigen::Vector3d wpTarget(wpX, wpY, wpZ);
    // 航路点种子: 结合放料和取料种子, 并添加中间J1值
    std::vector<JointConfig> wpSeeds = placeSeeds;
    wpSeeds.insert(wpSeeds.end(), pkSeeds.begin(), pkSeeds.end());
    // 额外种子: J1在取/放中间, 低J2保持臂水平
    double midJ1_seed = (paDeg[0] + plDeg[0]) / 2.0;
    for (double j2s : {-55.0, -50.0, -45.0, -60.0}) {
        wpSeeds.push_back(JointConfig::fromDegrees(
            {midJ1_seed, j2s, 30, 0, 90, midJ1_seed}));
        wpSeeds.push_back(JointConfig::fromDegrees(
            {midJ1_seed+15, j2s, 25, 0, 90, midJ1_seed+15}));
        wpSeeds.push_back(JointConfig::fromDegrees(
            {midJ1_seed-15, j2s, 35, 0, 90, midJ1_seed-15}));
    }
    auto ikWP = soIKHorizontal(checker, robot, wpTarget, wpSeeds, 3.0);
    if (!ikWP.converged) ikWP = multiStartIK(checker, robot, wpTarget, wpSeeds, 3.0, true);

    if (!ikWP.converged) {
        printf("  ⚠️ 框架入口IK失败! 尝试降低精度 (5mm容差)\n");
        ikWP = soIKHorizontal(checker, robot, wpTarget, wpSeeds, 5.0);
        if (!ikWP.converged) ikWP = multiStartIK(checker, robot, wpTarget, wpSeeds, 5.0, true);
    }
    if (!ikWP.converged) {
        fprintf(stderr, "  ❌ 框架入口航路点IK完全失败! 使用中间插值回退\n");
        // 回退: 取料和放料接近点的中间配置 (不太理想但可用)
        ikWP.config = PICK_APPROACH.interpolate(pc.approach, 0.5);
        ikWP.converged = true;
        ikWP.posError_mm = 999.0;
    }
    JointConfig FRAME_ENTRY = ikWP.config;
    printf("  航路点IK: %s (err=%.1fmm)\n", ikWP.posError_mm < 10 ? "✅" : "⚠️回退", ikWP.posError_mm);

    // 碰撞验证航路点
    {
        auto rp = checker.getCollisionReport(FRAME_ENTRY, true);
        SO_COORD_REF tc; checker.forwardKinematics(FRAME_ENTRY, tc);
        auto qdeg = FRAME_ENTRY.toDegrees();
        printf("  航路点: q=[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]deg\n",
               qdeg[0], qdeg[1], qdeg[2], qdeg[3], qdeg[4], qdeg[5]);
        printf("    TCP=(%.0f,%.0f,%.0f)mm A=%.1f B=%.1f C=%.1f\n",
               tc.X, tc.Y, tc.Z, tc.A, tc.B, tc.C);
        printf("    self=%s(%.1fmm) env=%s |B|=%.1f°\n",
               rp.selfCollision?"⚠COLL":"ok", rp.selfMinDist_mm,
               rp.envCollision?"⚠ENV":"ok", std::fabs(tc.B));
        if (rp.selfCollision || rp.envCollision) {
            printf("    ⚠️ 航路点碰撞! 尝试微调...\n");
            // 微调: 在wpZ±50mm范围内搜索无碰撞位置
            for (double dz = -50; dz <= 50; dz += 25) {
                if (dz == 0) continue;
                Eigen::Vector3d wpAlt(wpX, wpY, wpZ + dz);
                auto ikAlt = soIKHorizontal(checker, robot, wpAlt, wpSeeds, 5.0);
                if (!ikAlt.converged) continue;
                auto rpAlt = checker.getCollisionReport(ikAlt.config, false);
                if (!rpAlt.selfCollision && !rpAlt.envCollision) {
                    FRAME_ENTRY = ikAlt.config;
                    printf("    ✅ 微调成功: dZ=%.0fmm → 无碰撞\n", dz);
                    break;
                }
            }
        }
    }

    // ★ v8.0: 框架入口J1/J6归一化 — 确保与place/pick同侧
    // 问题: IK返回J1=-96° vs pick J1=175°, 原始距离271° (实际最短89°)
    // 关节限位J1∈[-360,360], 需要wrap到与pick_approach最近的等效角
    {
        double refJ1 = PICK_APPROACH.q[0];  // 参考J1 (rad)
        // 对FRAME_ENTRY的J1和J6做wrapping
        for (int ji : {0, 5}) {  // J1(idx=0) 和 J6(idx=5) 有±360°限位
            double orig = FRAME_ENTRY.q[ji];
            double best = orig, bestDist = std::fabs(refJ1 - orig);
            for (double shift : {2*M_PI, -2*M_PI}) {
                double cand = orig + shift;
                if (cand < -2*M_PI || cand > 2*M_PI) continue;
                double d = std::fabs(refJ1 - cand);
                if (d < bestDist) { bestDist = d; best = cand; }
            }
            if (std::fabs(best - orig) > 0.01) {
                printf("  J%d归一化: %.1f° → %.1f° (距离: %.1f° → %.1f°)\n",
                       ji==0?1:6, orig*180/M_PI, best*180/M_PI,
                       std::fabs(refJ1 - orig)*180/M_PI, bestDist*180/M_PI);
                FRAME_ENTRY.q[ji] = best;
            }
        }
        // 同样对place approach的J6做wrapping (J1已在前面优化过)
        for (int ji : {5}) {
            double orig = pc.approach.q[ji];
            double ref = FRAME_ENTRY.q[ji];
            double best = orig, bestDist = std::fabs(ref - orig);
            for (double shift : {2*M_PI, -2*M_PI}) {
                double cand = orig + shift;
                if (cand < -2*M_PI || cand > 2*M_PI) continue;
                double d = std::fabs(ref - cand);
                if (d < bestDist) { bestDist = d; best = cand; }
            }
            if (std::fabs(best - orig) > 0.01) pc.approach.q[ji] = best;
        }
    }

    // 预填充box-env障碍物用于搬运段碰撞检测
    planConfig.boxCollision.enabled = true;
    planConfig.boxCollision.clearEnvObstacles();
    // 4根立柱
    planConfig.boxCollision.addEnvCapsule(-fHW,fNY,fZB, -fHW,fNY,fZT, fR);
    planConfig.boxCollision.addEnvCapsule( fHW,fNY,fZB,  fHW,fNY,fZT, fR);
    planConfig.boxCollision.addEnvCapsule( fHW,fFY,fZB,  fHW,fFY,fZT, fR);
    planConfig.boxCollision.addEnvCapsule(-fHW,fFY,fZB, -fHW,fFY,fZT, fR);
    // 2根Y顶梁 + 2根X顶梁
    planConfig.boxCollision.addEnvCapsule(-fHW,fNY,fZT, -fHW,fFY,fZT, fR);
    planConfig.boxCollision.addEnvCapsule( fHW,fNY,fZT,  fHW,fFY,fZT, fR);
    planConfig.boxCollision.addEnvCapsule(-fHW,fNY,fZT,  fHW,fNY,fZT, fR);
    planConfig.boxCollision.addEnvCapsule(-fHW,fFY,fZT,  fHW,fFY,fZT, fR);
    // 4条电箱边
    planConfig.boxCollision.addEnvCapsule(-cHW,-cHD,cZT, -cHW, cHD,cZT, cR);
    planConfig.boxCollision.addEnvCapsule( cHW,-cHD,cZT,  cHW, cHD,cZT, cR);
    planConfig.boxCollision.addEnvCapsule(-cHW,-cHD,cZT,  cHW,-cHD,cZT, cR);
    planConfig.boxCollision.addEnvCapsule(-cHW, cHD,cZT,  cHW, cHD,cZT, cR);
    // 重置boxCollision状态 (seg loop中toolAction==1会重新启用+填充)
    planConfig.boxCollision.enabled = false;
    planConfig.boxCollision.clearEnvObstacles();

    // 验证收缩通过路径
    printf("\n  搬运路径验证 (收缩通过策略):\n");
    {
        auto report_wp = [&](const char* label, const JointConfig& q) {
            auto rp = checker.getCollisionReport(q, true);
            SO_COORD_REF tc; checker.forwardKinematics(q, tc);
            auto qdeg = q.toDegrees();
            printf("    %s: q=[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]deg\n",
                   label, qdeg[0], qdeg[1], qdeg[2], qdeg[3], qdeg[4], qdeg[5]);
            printf("      TCP=(%.0f,%.0f,%.0f)mm A=%.1f B=%.1f C=%.1f\n",
                   tc.X, tc.Y, tc.Z, tc.A, tc.B, tc.C);
            printf("      self=%s(%.1fmm) env=%s |B|=%.1f°\n",
                   rp.selfCollision?"⚠COLL":"ok", rp.selfMinDist_mm,
                   rp.envCollision?"⚠ENV":"ok", std::fabs(tc.B));
        };
        report_wp("取料接近", PICK_APPROACH);
        report_wp("框架入口", FRAME_ENTRY);
        report_wp("放料接近", pc.approach);

        // TCP轨迹追踪 (收缩通过路径, 展示侧面进入)
        printf("\n  TCP轨迹追踪 (拾取→框架入口, 5点):\n");
        for (int i = 0; i <= 4; i++) {
            double t = (double)i / 4;
            JointConfig q = PICK_APPROACH.interpolate(FRAME_ENTRY, t);
            SO_COORD_REF tc; checker.forwardKinematics(q, tc);
            printf("    t=%.2f: TCP=(%.0f,%.0f,%.0f)mm |B|=%.1f° Z=%.0fmm %s\n",
                   t, tc.X, tc.Y, tc.Z, std::fabs(tc.B), tc.Z,
                   tc.Z < fZT ? "框架以下✓" : "框架以上⚠");
        }
        printf("  TCP轨迹追踪 (框架入口→放料接近, 5点):\n");
        for (int i = 0; i <= 4; i++) {
            double t = (double)i / 4;
            JointConfig q = FRAME_ENTRY.interpolate(pc.approach, t);
            SO_COORD_REF tc; checker.forwardKinematics(q, tc);
            bool inFrame = (tc.Y >= fNY && tc.Y <= fFY && tc.X >= -fHW && tc.X <= fHW);
            printf("    t=%.2f: TCP=(%.0f,%.0f,%.0f)mm |B|=%.1f° %s\n",
                   t, tc.X, tc.Y, tc.Z, std::fabs(tc.B),
                   inFrame ? "框架内✓" : "框架外");
        }
    }

    struct MotionSeg {
        const char* name;
        JointConfig start, target;
        bool useRRT;
        int toolAction;  // 0=无, 1=启用工具球, 2=禁用工具球+添加放置障碍
        bool freeTcp;    // true=使用Free-TCP规划器
    };
    // v8.0: 7段运动 (收缩通过策略 — 搬运段RRT*经框架前开口侧面进入)
    // 策略: 机械臂收缩手臂从框架前方开口进入, 而非手臂张开从框架顶部翻越
    // 参考: S50_Layout_Optimization_Model_v3.md §5.4, §6.3, §10.4
    MotionSeg motions[] = {
        {"取料接近→取料",         PICK_APPROACH,  PICK_POS,        false, 0, false},
        {"取料→取料抬升",         PICK_POS,       PICK_APPROACH,   false, 1, false},  // ★启用工具球+箱子碰撞
        {"搬运:拾取→框架入口",    PICK_APPROACH,  FRAME_ENTRY,     true,  0, false},  // ★RRT* 收缩通过
        {"搬运:框架入口→放料接近", FRAME_ENTRY,    pc.approach,     true,  0, false},  // ★RRT* 穿越前开口
        {"放料接近→放料",         pc.approach,    pc.place,        false, 0, false},
        {"放料→放料抬升",         pc.place,       pc.approach,     false, 2, false},  // ★禁用工具球
        {"回程:放料→取料接近",    pc.approach,    PICK_APPROACH,   true,  0, true},   // RRT* Free-TCP返回
    };
    const int NUM_SEGS = 7;

    for (int s = 0; s < NUM_SEGS; s++) {
        auto& m = motions[s];
        double segDist = m.start.distanceTo(m.target);
        printf("\n  --- seg %d: %s (dist=%.2f rad, %.1f°) ---\n", s, m.name, segDist, segDist*180/M_PI);

        // 动作前: 启用/禁用工具碰撞体
        if (m.toolAction == 1) {
            // ❗ SO库 toolIndex 只接受1或2 (Tool1/Tool2), 不是碰撞体索引(6/7)
            //
            // v8.2 箱子碰撞建模 — 低Z托盘策略:
            //   托盘在地面(Z_base=-600mm), 所有可行IK构型 J3>70°
            //   SO工具球 pair(6,3)=0: 高J3下工具球必然碰小臂(UpArm)
            //   无论z=-400/-300/-200/r=80~120, pair(6,3)始终碰撞
            //
            //   独立boxCollision系统: 26点OBB采样 + getUIInfo碰撞体
            //     → 精确检测箱子OBB vs 臂各连杆胶囊, 比SO单球体更准确
            //     → 同时检测箱子-环境碰撞 (立柱, 顶梁, 电箱边)
            //
            //   ★方案: 跳过SO工具球注册, 完全依赖boxCollision
            //   好处: 1) 避免pair(6,3)误判阻塞RRT*
            //         2) OBB检测比球体更精确反映真实箱子形状
            //         3) 环境碰撞同步检测 (SO只检自碰撞)
            printf("  📦 跳过SO工具球 (pair(6,3)低Z不兼容), 使用独立boxCollision\n");

            // ★ 启用独立箱子-机械臂碰撞检测 (绕过SO pair(6,4)限制)
            planConfig.boxCollision.enabled = true;

            // ★ v8.0: 填充箱子-环境碰撞障碍物 (框架立柱+顶梁)
            // RRT*搬运时, 26个OBB采样点检查与这些障碍物距离
            // TCP Z轴旋转 → 箱子OBB旋转 → 不同yaw下碰撞状态不同
            planConfig.boxCollision.clearEnvObstacles();
            // 4根立柱 (与SO环境碰撞envId 1-4一致, 但用于箱子OBB独立检测)
            planConfig.boxCollision.addEnvCapsule(-fHW,fNY,fZB, -fHW,fNY,fZT, fR);
            planConfig.boxCollision.addEnvCapsule( fHW,fNY,fZB,  fHW,fNY,fZT, fR);
            planConfig.boxCollision.addEnvCapsule( fHW,fFY,fZB,  fHW,fFY,fZT, fR);
            planConfig.boxCollision.addEnvCapsule(-fHW,fFY,fZB, -fHW,fFY,fZT, fR);
            // 2根Y顶梁 (envId 20-21)
            planConfig.boxCollision.addEnvCapsule(-fHW,fNY,fZT, -fHW,fFY,fZT, fR);
            planConfig.boxCollision.addEnvCapsule( fHW,fNY,fZT,  fHW,fFY,fZT, fR);
            // 2根X顶梁 (envId 8-9)
            planConfig.boxCollision.addEnvCapsule(-fHW,fNY,fZT,  fHW,fNY,fZT, fR);
            planConfig.boxCollision.addEnvCapsule(-fHW,fFY,fZT,  fHW,fFY,fZT, fR);
            // 电箱顶部4条边 (envId 10-13)
            planConfig.boxCollision.addEnvCapsule(-cHW,-cHD,cZT, -cHW, cHD,cZT, cR);
            planConfig.boxCollision.addEnvCapsule( cHW,-cHD,cZT,  cHW, cHD,cZT, cR);
            planConfig.boxCollision.addEnvCapsule(-cHW,-cHD,cZT,  cHW,-cHD,cZT, cR);
            planConfig.boxCollision.addEnvCapsule(-cHW, cHD,cZT,  cHW, cHD,cZT, cR);
            printf("  📦 箱子-环境障碍: %d个 (4柱+2Y梁+2X梁+4电箱边)\n",
                   planConfig.boxCollision.numEnvObstacles);

            planner.setConfig(planConfig);
            // v8.0: 搬运段使用plannerFree (Free-TCP)时也需要箱子碰撞
            freeConfig.boxCollision = planConfig.boxCollision;
            plannerFree.setConfig(freeConfig);
            printf("  📦 箱子碰撞检测: ✅ ON (%.0f×%.0f×%.0fmm, margin=%.0fmm, envMargin=%.0fmm)\n",
                   planConfig.boxCollision.boxLengthX,
                   planConfig.boxCollision.boxWidthY,
                   planConfig.boxCollision.boxHeightZ,
                   planConfig.boxCollision.safetyMargin,
                   planConfig.boxCollision.envSafetyMargin);

            // 诊断: 检查搬运段起止点碰撞状态 (含箱子碰撞)
            printf("  🔍 搬运段碰撞诊断 (seg %d-%d):\n", s, std::min(s+4, NUM_SEGS-1));
            for (int di = s; di <= std::min(s+4, NUM_SEGS-1); di++) {
                auto& dm = motions[di];
                auto rpS = checker.getCollisionReport(dm.start, true);
                auto rpT = checker.getCollisionReport(dm.target, true);
                printf("    seg%d start: self=%s dist=%.1fmm pair=(%d,%d)\n",
                       di, rpS.selfCollision?"⚠COLL":"ok", rpS.selfMinDist_mm,
                       rpS.selfPairA, rpS.selfPairB);
                printf("    seg%d goal:  self=%s dist=%.1fmm pair=(%d,%d)\n",
                       di, rpT.selfCollision?"⚠COLL":"ok", rpT.selfMinDist_mm,
                       rpT.selfPairA, rpT.selfPairB);
                // 箱子碰撞诊断
                auto bxS = checker.getBoxCollisionReport(dm.start, planConfig.boxCollision);
                auto bxT = checker.getBoxCollisionReport(dm.target, planConfig.boxCollision);
                printf("    seg%d start: box=%s critical=%.1fmm(%s) overall=%.1fmm(%s)\n",
                       di, bxS.collision?"⚠BOX-COLL":"ok",
                       bxS.criticalMinDist_mm, BoxCollisionReport::colliderName(bxS.criticalClosest),
                       bxS.minDistance_mm, BoxCollisionReport::colliderName(bxS.closestCollider));
                printf("    seg%d goal:  box=%s critical=%.1fmm(%s) overall=%.1fmm(%s)\n",
                       di, bxT.collision?"⚠BOX-COLL":"ok",
                       bxT.criticalMinDist_mm, BoxCollisionReport::colliderName(bxT.criticalClosest),
                       bxT.minDistance_mm, BoxCollisionReport::colliderName(bxT.closestCollider));
            }
        }

        SegmentResult seg;
        if (m.useRRT) {
            // 选择规划器: 搬运箱子时用TCP-Horizontal, 无箱子时用Free-TCP
            auto& activePlanner = m.freeTcp ? plannerFree : planner;
            // 回程(freeTcp)从框架内出发 → 不需要排除区 (TCP在框架内合法移动)
            // 搬运段 → 使用排除区 (防止TCP从外部穿过墙壁)
            static const std::vector<TCPPlannerConfig::TCPExclusionBox> emptyBoxes;
            auto& activeExclBoxes = m.freeTcp ? emptyBoxes : planConfig.tcpExclusionBoxes;
            seg = executeRRTStar(m.name, m.start, m.target,
                                 robot, checker, activePlanner, parameterizer, fpCsv, 1, s, fpTraj,
                                 activeExclBoxes);
        } else
            seg = executeP2P(m.name, m.start, m.target,
                             robot, checker, parameterizer, fpCsv, 1, s, fpTraj);

        printf("  [%d] %-24s %s %.3fs plan:%.1fms param:%.1fms dist:%.0fmm env:%d wall:%d\n",
               s, seg.name.c_str(), seg.success?"✅":"❌", seg.totalTime_s,
               seg.planningTime_ms, seg.paramTime_ms, seg.minSelfDist_mm, seg.envCollisionCount,
               seg.tcpExclusionViolations);

        // ★ v8.0: 搬运段 (seg 2-3) 箱子-环境碰撞诊断
        // P2P搬运后检测箱子与框架立柱/梁的碰撞状态, 评估是否需要yaw旋转
        if (s >= 2 && s <= 3 && planConfig.boxCollision.enabled) {
            int boxEnvColl = 0;
            double boxEnvMinD = 1e10;
            int nSample = 20;
            for (int i = 0; i <= nSample; i++) {
                double t = (double)i / nSample;
                JointConfig qm = m.start.interpolate(m.target, t);
                auto bxRp = checker.getBoxCollisionReport(qm, planConfig.boxCollision);
                if (bxRp.envCollision) boxEnvColl++;
                if (bxRp.envMinDistance_mm < boxEnvMinD) boxEnvMinD = bxRp.envMinDistance_mm;
            }
            printf("  📦 搬运段box-env诊断: %d/%d碰撞点 minDist=%.1fmm %s\n",
                   boxEnvColl, nSample+1, boxEnvMinD,
                   boxEnvColl==0?"✅":"⚠️需要TCP旋转避障");

            // J6旋转优化: 对碰撞点微调J6(yaw)使箱子旋转避开立柱
            if (boxEnvColl > 0) {
                printf("  🔄 J6旋转优化 (箱子yaw避障):\n");
                int fixed = 0, checked = 0;
                for (int i = 0; i <= nSample; i++) {
                    double t = (double)i / nSample;
                    JointConfig qm = m.start.interpolate(m.target, t);
                    auto bxRp = checker.getBoxCollisionReport(qm, planConfig.boxCollision);
                    if (!bxRp.envCollision) continue;
                    checked++;
                    // sweep J6 ±180° 寻找最佳yaw (完整旋转范围)
                    double bestJ6 = qm.q[5];
                    double bestDist = bxRp.envMinDistance_mm;
                    for (double dj6 = -180; dj6 <= 180; dj6 += 5) {
                        JointConfig qt = qm;
                        qt.q[5] = qm.q[5] + dj6 * M_PI / 180.0;
                        if (!robot.isWithinLimits(qt)) continue;
                        auto bx2 = checker.getBoxCollisionReport(qt, planConfig.boxCollision);
                        if (bx2.envMinDistance_mm > bestDist) {
                            bestDist = bx2.envMinDistance_mm;
                            bestJ6 = qt.q[5];
                        }
                    }
                    if (bestDist > planConfig.boxCollision.envSafetyMargin) fixed++;
                    if (checked <= 3) {
                        printf("    t=%.2f: envDist %.1f→%.1f mm (ΔJ6=%.1f°) %s\n",
                               t, bxRp.envMinDistance_mm, bestDist,
                               (bestJ6-qm.q[5])*180/M_PI,
                               bestDist>planConfig.boxCollision.envSafetyMargin?"✅fixed":"⚠️");
                    }
                }
                printf("    修复: %d/%d碰撞点 (J6±180° sweep)\n", fixed, checked);
            }
        }

        // TCP Z↓偏差后验 (搬运段 + RRT段)
        // ⚠️ FK2 Euler (A,B,C) 不是标准ZYX — C随J1变化, 不代表物理倾斜
        // 正确度量: |B| = TCP切面偏离水平的角度 (B=0时TCP精确朝下)
        // 与PathPlannerSO的TCP-Horizontal检查一致 (仅检查B)
        bool isCarryP2P = (!m.useRRT && s >= 2 && s <= 3);  // 搬运P2P子段 (2段对角)
        if ((m.useRRT || isCarryP2P) && seg.success) {
            auto calcDev = [&](const JointConfig& q) {
                SO_COORD_REF tc; checker.forwardKinematics(q, tc);
                return std::fabs(tc.B);  // |B|° = TCP倾斜角
            };
            double devStart = calcDev(m.start), devEnd = calcDev(m.target);
            // 沿路径采样检测中途TCP偏差 (关节空间直线插值)
            const int N_SAMPLES = 20;
            double maxDevMid = 0, sumDev = 0;
            int worstIdx = 0;
            for (int i = 1; i < N_SAMPLES; i++) {
                double t = (double)i / N_SAMPLES;
                JointConfig qMid = m.start.interpolate(m.target, t);
                double dev = calcDev(qMid);
                sumDev += dev;
                if (dev > maxDevMid) { maxDevMid = dev; worstIdx = i; }
            }
            double avgDev = sumDev / (N_SAMPLES - 1);
            printf("  📐 TCP Z↓偏差: start=%.1f° end=%.1f° mid-max=%.1f°(@%.0f%%) avg=%.1f°",
                   devStart, devEnd, maxDevMid, 100.0*worstIdx/N_SAMPLES, avgDev);
            if (!m.freeTcp)
                printf(" (TCP-Horizontal约束)\n");
            else
                printf(" (Free-TCP, 无朝向约束)\n");
            if (maxDevMid > 30.0 && isCarryP2P)
                printf("  ⚠️ 搬运中途TCP偏离水平 %.1f° > 30° (吸盘安全风险)\n", maxDevMid);
        }

        // 动作后: 禁用工具碰撞体 + 添加已放置箱子
        if (m.toolAction == 2) {
            // v8.2: SO工具球未注册, 无需removeTool
            printf("  📦 搬运结束 (boxCollision OFF)\n");

            // ★ 关闭箱子碰撞检测 (箱子已放下) + 清除环境障碍物列表
            planConfig.boxCollision.enabled = false;
            planConfig.boxCollision.clearEnvObstacles();
            planner.setConfig(planConfig);
            freeConfig.boxCollision.enabled = false;
            freeConfig.boxCollision.clearEnvObstacles();
            plannerFree.setConfig(freeConfig);
            printf("  📦 箱子碰撞检测: OFF (箱子已放置, envObstacles已清除)\n");
            SO_COORD_REF tc; checker.forwardKinematics(pc.place, tc);
            int eid = 46;
            bool ok = checker.addEnvObstacleBall(eid,
                Eigen::Vector3d(tc.X, tc.Y, tc.Z - scene::BOX_HZ/2), boxEnvR);
            printf("  📦 放置→envId=%d (%.0f,%.0f,%.0f) r=%.0f: %s\n",
                   eid, tc.X, tc.Y, tc.Z - scene::BOX_HZ/2, boxEnvR, ok?"✅":"❌");
        }

        // 统计累积
        totalMotionTime += seg.totalTime_s;
        totalCollisions += seg.collisionCount;
        totalEnvCollisions += seg.envCollisionCount;
        if (seg.minSelfDist_mm < globalMinDist_mm) globalMinDist_mm = seg.minSelfDist_mm;
        grandTotalPlanMs += seg.planningTime_ms + seg.optimizationTime_ms;
        grandTotalParamMs += seg.paramTime_ms;
        grandTotalCollMs += seg.collCheckTime_ms;
        totalSegments++;
    }

    if(fpCsv) fclose(fpCsv);
    if(fpTraj) fclose(fpTraj);

    // ---- 统计 ----
    double totalElapsed = std::chrono::duration<double>(std::chrono::high_resolution_clock::now()-t_global).count();

    printf("\n━━━━━━ 阶段5: 统计 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n");
    printf("  箱子: 1  运动段: %d  总运动: %.3fs\n", totalSegments, totalMotionTime);
    printf("  自碰撞: %d  环境碰撞: %d\n", totalCollisions, totalEnvCollisions);
    printf("  最小距离: %.1fmm\n", globalMinDist_mm);
    printf("  IK: %.1fms  规划: %.1fms  参数化: %.1fms  碰撞检测: %.1fms\n",
           ikMs, grandTotalPlanMs, grandTotalParamMs, grandTotalCollMs);
    printf("  总耗时: %.2fs\n", totalElapsed);

    auto collStats = checker.getTimingStats();
    printf("\n%s", collStats.toString().c_str());

    // 写摘要
    FILE* fpSum = fopen("data/so_palletizing_summary.txt","w");
    if (fpSum) {
        fprintf(fpSum,"# HR_S50-2000 码垛v8.0 — v3.1安全优先+收缩通过\n\nversion: 8.0\n");
        fprintf(fpSum,"positions: 1\nsegments: %d\ntotal_motion_s: %.4f\n",totalSegments,totalMotionTime);
        fprintf(fpSum,"self_collisions: %d\nenv_collisions: %d\nmin_dist_mm: %.2f\n",
                totalCollisions,totalEnvCollisions,globalMinDist_mm);
        fprintf(fpSum,"order: single_box_conveyor_to_frame\n");
        fprintf(fpSum,"env_obstacles: 4_cabinet+3_conveyor+4_pillar+2_topbar+3_lozengeWall+1_placed\n");
        fprintf(fpSum,"wall_panels: back+left+right_lozengeOBB_100mm_thick\n");
        fprintf(fpSum,"tool_collision: boxCollision_OBB_noSOtool_lowZpallet\n");
        fprintf(fpSum,"carry_strategy: rrt_star_side_pass_shrink_through\n");
        fprintf(fpSum,"box_collision: base_lowerarm_hard_elbow_diag_margin40mm\n");
        fprintf(fpSum,"frame_gap_mm: %.0f\nconv_gap_mm: %.0f\nframe_cy_mm: %.0f\n",
                scene::FRAME_GAP, scene::CONV_GAP, fcy);

        fprintf(fpSum,"\n# Box TCP (base frame, mm):\n");
        {
            SO_COORD_REF tc; checker.forwardKinematics(placeConfigs[0].place,tc);
            fprintf(fpSum,"box_0_tcp_mm: %.1f %.1f %.1f\n",tc.X,tc.Y,tc.Z);
        }
        fprintf(fpSum,"\n# IK place (deg):\n");
        {
            auto q=placeConfigs[0].place.toDegrees();
            fprintf(fpSum,"ik_place_0: %.2f %.2f %.2f %.2f %.2f %.2f\n",q[0],q[1],q[2],q[3],q[4],q[5]);
        }
        fprintf(fpSum,"\n# IK approach (deg):\n");
        {
            auto q=placeConfigs[0].approach.toDegrees();
            fprintf(fpSum,"ik_appr_0: %.2f %.2f %.2f %.2f %.2f %.2f\n",q[0],q[1],q[2],q[3],q[4],q[5]);
        }
        {auto q=PICK_POS.toDegrees(); auto a=PICK_APPROACH.toDegrees();
         fprintf(fpSum,"\n# Pick (deg):\nik_pick: %.2f %.2f %.2f %.2f %.2f %.2f\nik_pick_appr: %.2f %.2f %.2f %.2f %.2f %.2f\n",
                 q[0],q[1],q[2],q[3],q[4],q[5],a[0],a[1],a[2],a[3],a[4],a[5]);}
        {SO_COORD_REF ptc; checker.forwardKinematics(PICK_POS,ptc);
         fprintf(fpSum,"pick_tcp_mm: %.1f %.1f %.1f\n",ptc.X,ptc.Y,ptc.Z);}

        fprintf(fpSum,"\n# Timing:\ninit_ms: %.3f\nik_ms: %.3f\nplanning_total_ms: %.3f\n",initMs,ikMs,grandTotalPlanMs);
        fprintf(fpSum,"param_total_ms: %.3f\ncollision_runtime_ms: %.3f\nelapsed_s: %.3f\n",
                grandTotalParamMs,grandTotalCollMs,totalElapsed);
        auto cs=checker.getTimingStats();
        fprintf(fpSum,"\ncoll_init_ms: %.3f\ncoll_total_avg_us: %.3f\ncoll_calls: %d\n",
                cs.initTime_ms,cs.totalCheckTime_us,cs.callCount);
        fclose(fpSum);
    }

    printf("\n══════════════════════════════════════════════════════════════════\n");
    printf("  HR_S50-2000 码垛仿真 v8.0 — v3.1安全优先+收缩通过策略\n");
    printf("══════════════════════════════════════════════════════════════════\n");
    return 0;
}
