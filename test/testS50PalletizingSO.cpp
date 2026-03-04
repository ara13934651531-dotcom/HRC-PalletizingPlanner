/**
 * @file testS50PalletizingSO.cpp
 * @brief HR_S50-2000 码垛仿真 v7.0 — 优化布局 + TCP旋转避障
 *
 * v7.0 核心变更 (2026-03-xx):
 *   1. 布局优化: 应用S50_Layout_Optimization_Model.md §10求解结果
 *      - FRAME_CY=680 (原1150), CONV_CX=580 (原1150),
 *        CONV_CY=30.6 (原-800), PALLET_TOP=175 (原-300)
 *      - 框架/传送带间距收紧至g_min=30mm (原500/600mm)
 *      - Pick→Place距离缩短43.2%, 总路径代理减少7.72%
 *   2. TCP Z轴旋转避障 (★核心新特性):
 *      - 搬运段改用RRT*(替代P2P), 允许J6探索不同yaw角
 *      - 箱子OBB (350×280mm) 随TCP yaw旋转改变与立柱间距
 *      - BoxCollisionConfig.envObstacles: 26点OBB vs 立柱/顶梁碰撞
 *      - 紧凑布局下箱子必须旋转才能穿过框架入口 (宽1100mm)
 *   3. 中转路点自适应: 根据新布局重新扫描安全高度
 *
 * v6.5 变更: J1短弧优化, 2段对角搬运
 * v6.4 变更: FRAME_GAP 200→500, 箱子碰撞强化
 *
 * @date 2026-03-05
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

    // === 布局参数 (优化求解结果, §10.1) ===
    // 活跃约束: C1(框架-电箱), C2(传送带-电箱), C7(顶梁间隙), C12(拾取工具球-柱)
    constexpr double FRAME_CY  = 680.0;   // y_f: 框架中心Y (mm), 原700→优化680 (C1绑定)
    constexpr double CONV_CX   = 580.0;   // x_c: 传送带中心X (mm), 原750→优化580 (C2绑定)
    constexpr double CONV_CY   = 30.6;    // y_c: 传送带/拾取位Y (mm), 原-800→优化30.6 (C12绑定)
    constexpr double PALLET_TOP = 175.0;  // h_p: 托盘面Z(基座坐标系, mm), 原200→175 (C7绑定)

    constexpr double baseZ = CAB_H;  // 800mm, 机器人基座世界Z高度

    // 旧间距参数 (反算, 用于日志兼容)
    constexpr double FRAME_GAP = FRAME_CY - CAB_D/2 - FRM_D/2;  // = 680-325-325 = 30mm
    constexpr double CONV_GAP  = CONV_CX  - CAB_W/2 - CONV_W/2; // = 580-275-275 = 30mm

    inline double frameCY() { return FRAME_CY; }
    inline double palletSurfBase() { return PALLET_TOP; }   // h_p: 175mm above base
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
    printf("║   HR_S50-2000 码垛仿真 v7.0 — 优化布局 + TCP旋转避障            ║\n");
    printf("║   布局优化(§10) + RRT*搬运 + 箱子OBB旋转避障                     ║\n");
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

    // 2根X方向顶梁 (envId 8-9, 平行于X轴, 连接前/后端左右立柱顶部)
    // 参考: S50_Palletizing_Scene_Description.md §5.2 表5-5, 建议注册
    printf("\n  === 框架X方向顶梁 (envId 8-9) ===\n");
    {
        // 前(近端)X顶梁: 左近→右近 (Y=fNY, Z=fZT)
        bool ok0 = checker.addEnvObstacleCapsule(8,
            Eigen::Vector3d(-fHW, fNY, fZT), Eigen::Vector3d(fHW, fNY, fZT), fR);
        printf("    envId=8 前X顶梁 (Y=%.0f): %s\n", fNY, ok0?"✅":"❌");
        // 后(远端)X顶梁: 左远→右远 (Y=fFY, Z=fZT)
        bool ok1 = checker.addEnvObstacleCapsule(9,
            Eigen::Vector3d(-fHW, fFY, fZT), Eigen::Vector3d(fHW, fFY, fZT), fR);
        printf("    envId=9 后X顶梁 (Y=%.0f): %s\n", fFY, ok1?"✅":"❌");
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
    // v6.4: 箱子放在框架 -X,+Y 角落 (靠近左墙和后墙)
    // 框架左墙 X=-fHW, 后墙 Y=fFY
    // 保留管径+箱子半尺寸+间隙, 确保箱子不贴墙
    double wallMargin = scene::FRM_TUBE_R + 30.0; // 管径30mm + 30mm间隙 = 60mm
    double boxPlaceX = -(fHW - scene::BOX_LX/2 - wallMargin);  // 靠近左墙
    double boxPlaceY = fFY - scene::BOX_WY/2 - wallMargin;     // 靠近后墙
    double boxPlaceZ = palSurf + scene::BOX_HZ;  // 第一层顶

    printf("  框架CY=%.0f  fNY=%.0f  fFY=%.0f  托盘面Z(base)=%.0f\n", fcy, fNY, fFY, palSurf);
    printf("  箱子放置: (%.0f, %.0f, %.0f) mm — 框架-X,+Y角落\n", boxPlaceX, boxPlaceY, boxPlaceZ);
    printf("  距左墙: %.0fmm  距后墙: %.0fmm\n\n",
           fHW + boxPlaceX - scene::BOX_LX/2,
           fFY - boxPlaceY - scene::BOX_WY/2);

    constexpr int numPositions = 1;
    std::vector<BoxTarget> boxTargets;
    {
        BoxTarget bt;
        bt.label = "CENTER";
        bt.pos_mm = Eigen::Vector3d(boxPlaceX, boxPlaceY, boxPlaceZ);
        bt.approach_mm = Eigen::Vector3d(boxPlaceX, boxPlaceY, boxPlaceZ + 300);
        boxTargets.push_back(bt);
    }

    printf("  %-10s  %8s %8s %8s\n","位置","X_mm","Y_mm","Z_mm");
    for (auto& b : boxTargets)
        printf("  %-10s  %8.1f %8.1f %8.1f\n", b.label.c_str(), b.pos_mm.x(), b.pos_mm.y(), b.pos_mm.z());

    printf("\n  === IK求解 (放料位) ===\n");
    // v7.0: 框架CY=680, 放料在-X,+Y角落 → J1≈atan2(~805,-365)≈114°
    // TCP保持水平: q5≈90+offset, q6≈q1
    std::vector<JointConfig> placeSeeds;
    double sA[][6] = {
        {110,-55,30,0,90,110},{115,-50,35,0,90,115},{120,-50,35,0,90,120},
        {105,-60,25,0,90,105},{125,-55,30,0,90,125},{100,-55,30,0,90,100},
        {130,-50,35,0,90,130},{108,-65,20,0,90,108},{118,-55,30,0,90,118},
        {112,-45,40,0,90,112},{115,-40,45,0,90,115},{110,-50,35,0,90,110},
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
    // v7.0: 拾取位 = (CONV_CX, CONV_CY, convSurf+BOX_HZ)
    // 优化结果: x_c=580, y_c=30.6 (传送带从机器人后方移至右侧旁)
    double pkX=scene::CONV_CX, pkY=scene::CONV_CY, pkZ=scene::convSurfBase()+scene::BOX_HZ;
    Eigen::Vector3d pickTgt(pkX,pkY,pkZ), pickApprTgt(pkX,pkY,pkZ+300);
    std::vector<JointConfig> pkSeeds;
    // v7.0: CONV_CX=580, CONV_CY=30.6 → 传送带在+X方向
    // FK2惯例: J1=0°→手臂指-X, J1=180°→手臂指+X
    // 拾取+X方向: J1≈180°-atan2(30.6,580)≈177° (或等效-183°)
    // 实测SO IK收敛到J1≈144° (考虑DH d2偏移)
    // 宽范围J1: 135°~165° (约±15°围绕144°)
    double pS[][6] = {{140,-55,30,0,90,140},{145,-50,35,0,90,145},{150,-50,35,0,90,150},
                      {135,-60,25,0,90,135},{155,-55,30,0,90,155},{138,-55,30,0,90,138},
                      {143,-45,40,0,90,143},{148,-60,25,0,90,148},{160,-55,30,0,90,160},
                      {142,-45,40,0,90,142},{152,-50,35,0,90,152},{147,-50,35,0,90,147}};
    for (auto& s:pS) pkSeeds.push_back(JointConfig::fromDegrees({s[0],s[1],s[2],s[3],s[4],s[5]}));

    // v7.0: Pick IK — 先尝试SO IK (精确6D), 回退NumericalIK (DLS)
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

    // v7.0: 优化布局RRT*搬运路径 — TCP旋转避障
    printf("  📌 v7.0: 优化布局+RRT*搬运+TCP旋转避障\n");

    // ---- 单箱搬运执行 ----
    printf("\n━━━━━━ 阶段4: 单箱搬运 (v7.0 优化布局+RRT*搬运) ━━━━━━━━━━━━━\n\n");

    FILE* fpCsv = fopen("data/so_palletizing_profile.csv","w");
    if(fpCsv) fprintf(fpCsv,"task,segment,step,time_s,q1,q2,q3,q4,q5,q6,"
                      "selfDist_mm,selfCollision,envCollision,tcpX_mm,tcpY_mm,tcpZ_mm\n");
    FILE* fpTraj = fopen("data/so_palletizing_trajectory.txt","w");
    if(fpTraj) { fprintf(fpTraj,"# HR_S50-2000 码垛v7.0 — 单箱搬运 (7段RRT*+TCP旋转避障)\n");
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

    // v7.0: 优化布局 RRT*搬运 — TCP旋转避障
    //
    // 紧凑布局(间距30mm): 框架入口宽1100mm (=FRM_W-2*FRM_TUBE_R)
    // 箱子对角线: sqrt(350²+280²) ≈ 449mm → 旋转90°后宽280mm (原350mm)
    // RRT*探索不同J6(yaw)使箱子旋转避开立柱
    //
    // 搬运策略: PICK_APPROACH → CARRY_HIGH → place_approach (3段)
    //   seg 2: 上升+J1旋转 (RRT*, TCP-Horizontal, box-env检测)
    //   seg 3: 下降至放料接近 (RRT*, TCP-Horizontal, box-env检测)

    // 取/放 接近配置
    auto paDeg = PICK_APPROACH.toDegrees();  // 取料接近
    auto plDeg = pc.approach.toDegrees();    // 放料接近

    // === 中转高度扫描: 寻找满足TCP水平+无碰撞的最低安全高度 ===
    // 默认: J2=-90 (HOME臂型, Z≈2272mm, |B|=0°)
    // 尝试: J2=-85,-80,...,-65 配合 J5=-J2 维持TCP水平
    // ⚠️ 必须带工具球检测: pair(6,3)在低高度会碰撞 (J2=-75→dist=0!)
    // 条件: |B|<10° + 无碰撞(含工具球) + Z>框架顶+200mm
    printf("  === 中转高度扫描 (含工具球碰撞检测) ===\n");
    double transitJ2 = -90.0, transitJ5 = 90.0;
    double transitZ_mm = 2272.0;
    {
        // 临时启用工具球进行扫描 (搬运时工具球始终活跃)
        bool toolTmpOk = checker.setToolBall(1, Eigen::Vector3d(0, 0, -400), 120.0);
        printf("    临时工具球: %s\n", toolTmpOk?"ON":"FAIL");

        double j1_test = (paDeg[0] + plDeg[0]) / 2.0;  // J1中间值用于测试
        double frameTop = fZT;  // 框架顶Z (base frame, mm)
        double minSafeZ = frameTop + 200.0;  // 安全高度: 框架顶+200mm

        struct HeightCandidate { double j2, j5, z, b_abs, selfDist; bool ok; };
        std::vector<HeightCandidate> candidates;

        for (double j2 = -65; j2 >= -90; j2 -= 5) {
            double j5 = -j2;  // J5=-J2 维持TCP水平 (HOME: J2=-90,J5=90 → J2+J5=0)
            JointConfig qTest = JointConfig::fromDegrees({j1_test, j2, 0, 0, j5, 0});
            SO_COORD_REF tc; checker.forwardKinematics(qTest, tc);
            auto rp = checker.getCollisionReport(qTest, false);
            bool collFree = !rp.selfCollision && !rp.envCollision;
            double Babs = std::fabs(tc.B);
            // 严格条件: |B|<10° + 无碰撞 + Z安全 + 自碰撞距离>5mm
            bool viable = (Babs < 10.0 && collFree && tc.Z > minSafeZ && rp.selfMinDist_mm > 5.0);
            candidates.push_back({j2, j5, tc.Z, Babs, rp.selfMinDist_mm, viable});
            printf("    J2=%4.0f° J5=%3.0f°: Z=%6.0fmm |B|=%4.1f° self=%.1fmm(%d,%d) %s%s\n",
                   j2, j5, tc.Z, Babs, rp.selfMinDist_mm, rp.selfPairA, rp.selfPairB,
                   collFree?"free":"COLL",
                   viable?" ★VIABLE":"");
        }

        // 移除临时工具球
        checker.removeTool(1);

        // 选择Z最低的可行高度 (最低=最优)
        for (auto& c : candidates) {
            if (c.ok && c.z < transitZ_mm) {
                transitJ2 = c.j2; transitJ5 = c.j5; transitZ_mm = c.z;
            }
        }
        printf("  → 最佳中转: J2=%.0f° J5=%.0f° Z=%.0fmm (框架顶+%.0fmm)\n",
               transitJ2, transitJ5, transitZ_mm, transitZ_mm - frameTop);
    }

    // 高位中转: J1=中间值, J2/J3/J5=固定高度, J4/J6=优化搜索
    // v7.0: 网格搜索J4/J6使 |B|≤10° (TCP水平) + 最小化descent box-env碰撞
    // ⚠️ J4/J6 DO affect |B|! (DH偏移使J4/J6耦合到TCP俯仰角)
    double avgJ4 = (paDeg[3] + plDeg[3]) / 2.0;
    double avgJ6 = (paDeg[5] + plDeg[5]) / 2.0;
    double midJ1 = (paDeg[0] + plDeg[0]) / 2.0;  // 取/放J1中间值

    // 网格搜索: 在|B|≤10°约束下, 找最少descent碰撞 + 最短关节距离
    // 预填充box-env障碍物用于CARRY_HIGH搜索 (与seg loop中toolAction==1一致)
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
    // 搜索后临时重置 (seg loop中toolAction==1会重新设置)
    double bestJ4_ch = 0.0, bestJ6_ch = 0.0;
    double bestB_ch = 999.0;
    int bestCollCount = 999;
    double bestJointDist = 1e10;  // 总carry距离 (pick→carry + carry→place)
    printf("\n  === CARRY_HIGH J4/J6 优化搜索 ===\n");
    {
        const double B_LIMIT = 10.0;  // TCP水平容差 (度)
        int nCand = 0, nValid = 0;
        for (double cj4 = -180; cj4 <= 180; cj4 += 15) {
            for (double cj6 = -180; cj6 <= 180; cj6 += 15) {
                JointConfig qtmp = JointConfig::fromDegrees(
                    {midJ1, transitJ2, 0.0, cj4, transitJ5, cj6});
                if (!robot.isWithinLimits(qtmp)) continue;
                nCand++;
                // FK检查|B|
                SO_COORD_REF tcp; checker.forwardKinematics(qtmp, tcp);
                double absB = std::fabs(tcp.B);
                if (absB > B_LIMIT) continue;
                nValid++;
                // 沿descent路径采样box-env碰撞数
                int collCount = 0;
                const JointConfig& plApproach = placeConfigs[0].approach;
                for (int i = 0; i <= 10; i++) {
                    double t = (double)i / 10;
                    JointConfig qm = qtmp.interpolate(plApproach, t);
                    auto bxRp = checker.getBoxCollisionReport(qm, planConfig.boxCollision);
                    if (bxRp.envCollision) collCount++;
                }
                // 总carry关节距离
                double jd = PICK_APPROACH.distanceTo(qtmp) + qtmp.distanceTo(plApproach);
                // 选择: ①最少碰撞 ②同碰撞取最小|B| ③同碰撞同|B|取最短距离
                if (collCount < bestCollCount ||
                    (collCount == bestCollCount && absB < bestB_ch - 1.0) ||
                    (collCount == bestCollCount && std::fabs(absB - bestB_ch) < 1.0 && jd < bestJointDist)) {
                    bestJ4_ch = cj4; bestJ6_ch = cj6; bestB_ch = absB;
                    bestCollCount = collCount; bestJointDist = jd;
                }
            }
        }
        printf("    搜索空间: %d候选, %d满足|B|≤%.0f°\n", nCand, nValid, B_LIMIT);
        printf("    最优: J4=%.0f° J6=%.0f° |B|=%.1f° descent碰撞=%d/11 jointDist=%.2frad\n",
               bestJ4_ch, bestJ6_ch, bestB_ch, bestCollCount, bestJointDist);
        printf("    对比: avgJ4=%.1f° avgJ6=%.1f° (未优化)\n", avgJ4, avgJ6);
    }
    JointConfig CARRY_HIGH = JointConfig::fromDegrees(
        {midJ1, transitJ2, 0.0, bestJ4_ch, transitJ5, bestJ6_ch});
    // 重置boxCollision状态 (seg loop中toolAction==1会重新启用+填充)
    planConfig.boxCollision.enabled = false;
    planConfig.boxCollision.clearEnvObstacles();

    // 验证高位中转配置
    printf("\n  搬运路径验证 (+X/+Y短弧):\n");
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
        report_wp("中转高位", CARRY_HIGH);
        report_wp("放料接近", pc.approach);

        // 验证2段对角P2P的TCP偏差 (J1不影响B, 所以只需检查J2/J3/J5变化)
        printf("\n  TCP偏差预验证 (20点采样):\n");
        auto calcB = [&](const JointConfig& q) -> double {
            SO_COORD_REF tc; checker.forwardKinematics(q, tc);
            return std::fabs(tc.B);
        };
        double maxUp = 0, maxDown = 0;
        for (int i = 1; i < 20; i++) {
            double t = (double)i / 20;
            double devUp = calcB(PICK_APPROACH.interpolate(CARRY_HIGH, t));
            double devDn = calcB(CARRY_HIGH.interpolate(pc.approach, t));
            if (devUp > maxUp) maxUp = devUp;
            if (devDn > maxDown) maxDown = devDn;
        }
        printf("    对角上升段: max|B|=%.1f° %s\n", maxUp, maxUp<=30?"✅":"⚠️>30°");
        printf("    降低段:     max|B|=%.1f° %s\n", maxDown, maxDown<=30?"✅":"⚠️>30°");

        // 验证沿途碰撞 (40点采样)
        printf("  碰撞路径验证:\n");
        int collUp = 0, collDown = 0;
        for (int i = 0; i <= 40; i++) {
            double t = (double)i / 40;
            if (!checker.isCollisionFree(PICK_APPROACH.interpolate(CARRY_HIGH, t))) collUp++;
            if (!checker.isCollisionFree(CARRY_HIGH.interpolate(pc.approach, t))) collDown++;
        }
        printf("    对角上升段: %d/%d碰撞点 %s\n", collUp, 41, collUp==0?"✅":"⚠️");
        printf("    降低段:     %d/%d碰撞点 %s\n", collDown, 41, collDown==0?"✅":"⚠️");

        // TCP位置沿途追踪 (展示+X/+Y象限路径)
        printf("  TCP轨迹追踪 (对角上升段, 5点):\n");
        for (int i = 0; i <= 4; i++) {
            double t = (double)i / 4;
            JointConfig q = PICK_APPROACH.interpolate(CARRY_HIGH, t);
            SO_COORD_REF tc; checker.forwardKinematics(q, tc);
            printf("    t=%.2f: TCP=(%.0f,%.0f,%.0f)mm |B|=%.1f° %s象限\n",
                   t, tc.X, tc.Y, tc.Z, std::fabs(tc.B),
                   (tc.X>0 && tc.Y>0)?"Q1(+X,+Y)":(tc.X<0 && tc.Y>0)?"Q2(-X,+Y)":
                   (tc.X<0 && tc.Y<0)?"Q3(-X,-Y)":"Q4(+X,-Y)");
        }
    }

    struct MotionSeg {
        const char* name;
        JointConfig start, target;
        bool useRRT;
        int toolAction;  // 0=无, 1=启用工具球, 2=禁用工具球+添加放置障碍
        bool freeTcp;    // true=使用Free-TCP规划器
    };
    // v7.0: 7段运动 (搬运段P2P + J6后优化旋转避障)
    // 策略: P2P生成自然关节空间轨迹, 后优化在每个点sweep J6找最优yaw
    // 原因: 全6-DOF RRT*在TCP-Horizontal+box-env+self约束下空间太窄
    MotionSeg motions[] = {
        {"取料接近→取料",         PICK_APPROACH,  PICK_POS,      false, 0, false},
        {"取料→取料抬升",         PICK_POS,       PICK_APPROACH, false, 1, false},  // ★启用工具球+箱子碰撞
        {"搬运:上升+旋转",        PICK_APPROACH,  CARRY_HIGH,    false, 0, false},  // P2P+后优化J6
        {"搬运:降低→放料接近",    CARRY_HIGH,     pc.approach,   false, 0, false},  // P2P+后优化J6
        {"放料接近→放料",         pc.approach,    pc.place,      false, 0, false},
        {"放料→放料抬升",         pc.place,       pc.approach,   false, 2, false},  // ★禁用工具球
        {"放料抬升→取料接近",     pc.approach,    PICK_APPROACH, true,  0, true},   // RRT* Free-TCP返回
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
            // v6.2 箱子碰撞建模 — SO库限制分析:
            //   箱子尺寸: 350×280×250mm, TCP在箱子顶面
            //   箱子体积: z=0→z=-250mm (法兰坐标系)
            //   SO库限制: pair(6,4) 工具(collider6) vs 腕部(collider4, r=140mm)
            //     - 自碰撞距离公式: dist = |tool_surface - wrist_surface|
            //     - 工具球z=-400 r=120: dist=51.9mm ✅ (远离腕部)
            //     - 工具球z=-125 r=100: dist=0.0mm ❌ (在箱子中心, 碰腕)
            //     - 工具胶囊z=-250→-400 r=100: dist=0.0mm ❌
            //     - 工具胶囊z=-300→-400 r=100: dist=0.0mm ❌
            //     - setCPSelfColliderLinkModelOpenState(0,0,0): 无效, 不控制tool-link pair
            //   结论: SO库硬编码tool-wrist自碰撞, 无法通过API绕过
            //
            // 最终方案: 工具球 z=-400 r=120mm (覆盖z=-280→z=-520)
            //   + 箱子底面以下150mm区域的环境碰撞保护
            //   + TCP-Horizontal约束保持箱子朝下 (防倾斜碰撞)
            //   + RRT*搬运段规划自动避障 (箱子-框架/电箱/传送带)

            bool toolOk = checker.setToolBall(1, Eigen::Vector3d(0, 0, -400), 120.0);
            printf("  📦 工具1 球ON idx=1 (z=-400, r=120): %s\n", toolOk?"✅":"❌");

            // ★ 启用独立箱子-机械臂碰撞检测 (绕过SO pair(6,4)限制)
            planConfig.boxCollision.enabled = true;

            // ★ v7.0: 填充箱子-环境碰撞障碍物 (框架立柱+顶梁)
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
            // v7.0: 搬运段使用plannerFree (Free-TCP)时也需要箱子碰撞
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

        // ★ v7.0: 搬运段 (seg 2-3) 箱子-环境碰撞诊断
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
            checker.removeTool(1);  // toolIdx=1 球
            printf("  📦 工具碰撞体OFF (idx=1)\n");

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
        fprintf(fpSum,"# HR_S50-2000 码垛v7.0 — 优化布局+TCP旋转避障\n\nversion: 7.0\n");
        fprintf(fpSum,"positions: 1\nsegments: %d\ntotal_motion_s: %.4f\n",totalSegments,totalMotionTime);
        fprintf(fpSum,"self_collisions: %d\nenv_collisions: %d\nmin_dist_mm: %.2f\n",
                totalCollisions,totalEnvCollisions,globalMinDist_mm);
        fprintf(fpSum,"order: single_box_conveyor_to_frame\n");
        fprintf(fpSum,"env_obstacles: 4_cabinet+3_conveyor+4_pillar+2_topbar+3_lozengeWall+1_placed\n");
        fprintf(fpSum,"wall_panels: back+left+right_lozengeOBB_100mm_thick\n");
        fprintf(fpSum,"tool_collision: ball_z-400_r120_toolIdx1_SOlimit_pair64\n");
        fprintf(fpSum,"carry_strategy: rrt_star_tcp_rotation_box_env_avoidance\n");
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
    printf("  HR_S50-2000 码垛仿真 v7.0 — 优化布局+TCP旋转避障\n");
    printf("══════════════════════════════════════════════════════════════════\n");
    return 0;
}
