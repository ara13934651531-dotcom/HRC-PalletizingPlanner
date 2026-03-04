/**
 * @file testS50PalletizingSO.cpp
 * @brief HR_S50-2000 码垛仿真 v8.0 — 4箱码垛 + TCP旋转避障 + |B|修正
 *
 * v8.0 核心变更 (2026-03-xx):
 *   1. 4箱第一层码垛: BK-L, BK-R, FR-L, FR-R (托盘2×2布局)
 *   2. TCP Z轴旋转实际修正 (★核心新特性):
 *      - optimizeCarryPath(): 多航点路径 + J6 yaw sweep实际修正碰撞点
 *      - J5 sweep实际修正TCP水平偏差 (|B|>25° → sweep J5±30°)
 *      - 替代v7.0仅诊断不修正的方案
 *   3. computeCarryHigh(): 提取CARRY_HIGH优化器为独立函数, 每箱独立计算
 *   4. 多箱动态环境: 每放一箱添加envObstacle, 后续箱子规划考虑已放箱子
 *
 * v7.0 变更: 布局优化, TCP旋转避障诊断, CARRY_HIGH网格搜索
 * v6.5 变更: J1短弧优化, 2段对角搬运
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
// 辅助: 优化搬运路径 — J6 yaw旋转避障 + J5 TCP水平修正
// ============================================================================
// 生成N+1点路径, 每个中间点:
//   1) J5调整: 如果|B|>25°, sweep J5±30°找最小|B|
//   2) J6调整: 如果box-env碰撞或距离不足, sweep J6±180°找最优yaw
// 返回多航点Path, 供TimeParameterizer生成平滑轨迹
Path optimizeCarryPath(
    const JointConfig& start, const JointConfig& end,
    CollisionCheckerSO& checker, RobotModel& robot,
    const BoxCollisionConfig& boxCfg,
    int numWaypoints = 12)
{
    Path path;
    int j6Fixed = 0, bFixed = 0;

    for (int i = 0; i <= numWaypoints; i++) {
        double t = (double)i / numWaypoints;
        JointConfig q = start.interpolate(end, t);

        // 端点不修改 (保持IK精确解)
        if (i == 0 || i == numWaypoints) {
            Waypoint wp(q); wp.pathParam = t;
            path.waypoints.push_back(wp);
            continue;
        }

        // Step 1: TCP水平修正 — sweep J5 减小 |B|
        SO_COORD_REF tcp;
        checker.forwardKinematics(q, tcp);
        double absB = std::fabs(tcp.B);

        if (absB > 25.0) {
            double bestJ5 = q.q[4];
            double bestAbsB = absB;
            for (double dj5 = -30; dj5 <= 30; dj5 += 1) {
                JointConfig qt = q;
                qt.q[4] += dj5 * M_PI / 180.0;
                if (!robot.isWithinLimits(qt)) continue;
                if (!checker.isCollisionFree(qt)) continue;
                SO_COORD_REF tc2;
                checker.forwardKinematics(qt, tc2);
                double ab2 = std::fabs(tc2.B);
                if (ab2 < bestAbsB) {
                    bestAbsB = ab2;
                    bestJ5 = qt.q[4];
                }
            }
            if (bestAbsB < absB - 2.0) {
                q.q[4] = bestJ5;
                bFixed++;
                checker.forwardKinematics(q, tcp);
            }
        }

        // Step 2: J6 yaw旋转避障 — sweep J6 最大化box-env距离
        if (boxCfg.enabled) {
            auto bxRp = checker.getBoxCollisionReport(q, boxCfg);
            if (bxRp.envCollision || bxRp.envMinDistance_mm < boxCfg.envSafetyMargin + 10) {
                double bestJ6 = q.q[5];
                double bestDist = bxRp.envMinDistance_mm;
                for (double dj6 = -180; dj6 <= 180; dj6 += 3) {
                    JointConfig qt = q;
                    qt.q[5] += dj6 * M_PI / 180.0;
                    if (!robot.isWithinLimits(qt)) continue;
                    if (!checker.isCollisionFree(qt)) continue;
                    // J6改变后检查|B|是否仍满足
                    SO_COORD_REF tc2;
                    checker.forwardKinematics(qt, tc2);
                    if (std::fabs(tc2.B) > 30.0) continue;
                    auto bx2 = checker.getBoxCollisionReport(qt, boxCfg);
                    if (bx2.envMinDistance_mm > bestDist) {
                        bestDist = bx2.envMinDistance_mm;
                        bestJ6 = qt.q[5];
                    }
                }
                if (bestDist > bxRp.envMinDistance_mm + 5) {
                    q.q[5] = bestJ6;
                    j6Fixed++;
                }
            }
        }

        Waypoint wp(q); wp.pathParam = t;
        path.waypoints.push_back(wp);
    }

    printf("        优化结果: %d航点, J6修正%d/%d, |B|修正%d/%d\n",
           numWaypoints+1, j6Fixed, numWaypoints-1, bFixed, numWaypoints-1);
    return path;
}

// ============================================================================
// 辅助: 执行优化搬运段 (多航点路径 + J6/J5修正 + 时间参数化)
// ============================================================================
SegmentResult executeOptimizedCarry(const char* name,
    const JointConfig& start, const JointConfig& target,
    RobotModel& robot, CollisionCheckerSO& checker,
    const BoxCollisionConfig& boxCfg,
    TimeParameterizer& param, FILE* csv, int ti, int si, FILE* traj,
    int numWaypoints = 12)
{
    SegmentResult r; r.name = name;
    auto tS = std::chrono::high_resolution_clock::now();

    // 生成优化搬运路径
    Path path = optimizeCarryPath(start, target, checker, robot, boxCfg, numWaypoints);
    r.pathWaypoints = (int)path.waypoints.size();
    r.pathLength_rad = path.totalLength();

    // 时间参数化
    auto t0 = std::chrono::high_resolution_clock::now();
    Trajectory tr = param.parameterize(path);
    r.paramTime_ms = std::chrono::duration<double,std::milli>(
        std::chrono::high_resolution_clock::now()-t0).count();
    if (tr.empty()) return r;

    r.totalTime_s = tr.totalTime;
    r.totalPoints = (int)tr.size();

    // 碰撞检测与记录
    auto t2 = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < tr.size(); i++) {
        const auto& p = tr.points[i];
        for (int j=0;j<6;j++){
            double v=std::fabs(p.velocity[j])*180/M_PI;
            if(v>r.peakVelocity_dps) r.peakVelocity_dps=v;
        }
        if (i%10==0||i==tr.size()-1) {
            auto rp = checker.getCollisionReport(p.config, true);
            if (rp.selfCollision) r.collisionCount++;
            if (rp.envCollision) r.envCollisionCount++;
            if (rp.selfMinDist_mm < r.minSelfDist_mm) r.minSelfDist_mm = rp.selfMinDist_mm;
            if (csv) { auto q=p.config.toDegrees();
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
            fprintf(traj," %.2f %.1f %.1f %.1f\n",r.minSelfDist_mm,tc.X,tc.Y,tc.Z);
        }
    }
    r.collCheckTime_ms = std::chrono::duration<double,std::milli>(
        std::chrono::high_resolution_clock::now()-t2).count();
    r.totalSegmentTime_ms = std::chrono::duration<double,std::milli>(
        std::chrono::high_resolution_clock::now()-tS).count();
    r.success = true;
    return r;
}

// ============================================================================
// 辅助: CARRY_HIGH J4/J6 网格搜索优化
// ============================================================================
// 在|B|≤10°约束下搜索最优J4/J6: ①最少descent碰撞 ②最小|B| ③最短关节距离
JointConfig computeCarryHigh(
    double midJ1, double transitJ2, double transitJ5,
    const JointConfig& pickApproach, const JointConfig& placeApproach,
    CollisionCheckerSO& checker, RobotModel& robot,
    const BoxCollisionConfig& boxCfg)
{
    const double B_LIMIT = 10.0;
    double bestJ4 = 0.0, bestJ6 = 0.0;
    double bestB = 999.0;
    int bestCollCount = 999;
    double bestJointDist = 1e10;
    int nCand = 0, nValid = 0;

    for (double cj4 = -180; cj4 <= 180; cj4 += 15) {
        for (double cj6 = -180; cj6 <= 180; cj6 += 15) {
            JointConfig qtmp = JointConfig::fromDegrees(
                {midJ1, transitJ2, 0.0, cj4, transitJ5, cj6});
            if (!robot.isWithinLimits(qtmp)) continue;
            nCand++;
            SO_COORD_REF tcp; checker.forwardKinematics(qtmp, tcp);
            double absB = std::fabs(tcp.B);
            if (absB > B_LIMIT) continue;
            nValid++;
            // descent路径box-env碰撞数
            int collCount = 0;
            for (int i = 0; i <= 10; i++) {
                double t = (double)i / 10;
                JointConfig qm = qtmp.interpolate(placeApproach, t);
                auto bxRp = checker.getBoxCollisionReport(qm, boxCfg);
                if (bxRp.envCollision) collCount++;
            }
            double jd = pickApproach.distanceTo(qtmp) + qtmp.distanceTo(placeApproach);
            if (collCount < bestCollCount ||
                (collCount == bestCollCount && absB < bestB - 1.0) ||
                (collCount == bestCollCount && std::fabs(absB - bestB) < 1.0 && jd < bestJointDist)) {
                bestJ4 = cj4; bestJ6 = cj6; bestB = absB;
                bestCollCount = collCount; bestJointDist = jd;
            }
        }
    }
    printf("    搜索: %d候选, %d满足|B|≤%.0f°, 最优J4=%.0f° J6=%.0f° |B|=%.1f° collDescent=%d/11\n",
           nCand, nValid, B_LIMIT, bestJ4, bestJ6, bestB, bestCollCount);

    return JointConfig::fromDegrees({midJ1, transitJ2, 0.0, bestJ4, transitJ5, bestJ6});
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
    printf("║   HR_S50-2000 码垛仿真 v8.0 — 4箱码垛 + TCP旋转避障 + |B|修正   ║\n");
    printf("║   布局优化(§10) + 多航点搬运 + J6/J5实际修正                      ║\n");
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

    printf("\n  环境障碍: 电箱(4) + 传送带(3) + 框架立柱(4) + Y顶梁(2) + X顶梁(2) + 面板Lozenge(3) + 已放箱子(动态)\n");
    printf("  面板封锁: 后面+左面+右面 Lozenge OBB (前面开放, 机械臂入口)\n");
    printf("  布局间距: FRAME_GAP=%.0f CONV_GAP=%.0f CONV_LEN=%.0f (mm)\n",
           scene::FRAME_GAP, scene::CONV_GAP, scene::CONV_LEN);
    printf("  框架: X=[%.0f,%.0f] Y=[%.0f,%.0f]  传送带: X=[%.0f,%.0f] Y=[%.0f,%.0f]\n",
           -fHW, fHW, fNY, fFY, cvX-cvHW, cvX+cvHW, cvY-cvHL, cvY+cvHL);
    printf("  X间距: %.0f  Y间距: %.0f (无重叠✓)\n\n",
           (cvX-cvHW) - fHW, fNY - (cvY+cvHL));

    // ---- 码垛布局 + IK求解 ----
    printf("━━━━━━ 阶段3: 4箱码垛布局 + IK求解 ━━━━━━━━━━━━━━━━━━━━━━━\n\n");

    double palSurf = scene::palletSurfBase();
    // v8.0: 4箱第一层码垛 (2×2, 基于托盘尺寸)
    // 托盘: PAL_W=1000 × PAL_D=600, 中心=(0, fcy)
    // 每行2箱(X方向), 每列2箱(Y方向)
    double palCX = 0.0, palCY = fcy;
    double palHW = scene::PAL_W/2, palHD = scene::PAL_D/2;
    double xMargin = (scene::PAL_W - 2*scene::BOX_LX - scene::BOX_GAP) / 2;  // =145mm
    double yMargin = (scene::PAL_D - 2*scene::BOX_WY - scene::BOX_GAP) / 2;  // =15mm
    double xLeft  = palCX - palHW + xMargin + scene::BOX_LX/2;   // -180
    double xRight = palCX + palHW - xMargin - scene::BOX_LX/2;   //  180
    double yBack  = palCY + palHD - yMargin - scene::BOX_WY/2;   // 825
    double yFront = palCY - palHD + yMargin + scene::BOX_WY/2;   // 535
    double zL1    = palSurf + scene::BOX_HZ;                      // 425

    printf("  框架CY=%.0f  fNY=%.0f  fFY=%.0f  托盘面Z(base)=%.0f\n", fcy, fNY, fFY, palSurf);
    printf("  托盘: (%.0f,%.0f) %.0f×%.0fmm  箱子布局: 2×2 (X间距%.0f Y间距%.0fmm)\n",
           palCX, palCY, scene::PAL_W, scene::PAL_D,
           xRight-xLeft-scene::BOX_LX, yBack-yFront-scene::BOX_WY);

    constexpr int numPositions = 4;
    std::vector<BoxTarget> boxTargets;
    // 码垛顺序: BK-L → BK-R → FR-L → FR-R (列优先, 远端→近端)
    boxTargets.push_back({"BK-L", {xLeft,  yBack,  zL1}, {xLeft,  yBack,  zL1+300}});
    boxTargets.push_back({"BK-R", {xRight, yBack,  zL1}, {xRight, yBack,  zL1+300}});
    boxTargets.push_back({"FR-L", {xLeft,  yFront, zL1}, {xLeft,  yFront, zL1+300}});
    boxTargets.push_back({"FR-R", {xRight, yFront, zL1}, {xRight, yFront, zL1+300}});

    printf("\n  %-6s  %8s %8s %8s\n","位置","X_mm","Y_mm","Z_mm");
    for (auto& b : boxTargets)
        printf("  %-6s  %8.1f %8.1f %8.1f\n", b.label.c_str(), b.pos_mm.x(), b.pos_mm.y(), b.pos_mm.z());

    printf("\n  === IK求解 (放料位, %d位置) ===\n", numPositions);
    // v8.0: 4箱码垛, 每个位置根据目标XY估算J1生成种子
    // FK2约定: J1=0°→-X方向, J1=θ时TCP大致朝向(-cos(θ),-sin(θ))
    // 给定目标(tx,ty), J1 ≈ atan2(-ty, -tx) + 偏移(DH d2影响)
    auto makeSeeds = [&](const Eigen::Vector3d& target) -> std::vector<JointConfig> {
        double j1Est = std::atan2(-target.y(), -target.x()) * 180.0 / M_PI;
        // 从估算J1出发, 生成±20°范围的种子
        std::vector<JointConfig> seeds;
        for (double dj1 : {0.0, -5.0, 5.0, -10.0, 10.0, -15.0, 15.0, -20.0, 20.0}) {
            for (double j2 : {-55.0, -50.0, -60.0, -65.0, -45.0}) {
                double j3 = 90 + j2;  // 大约保持q2+q3≈35附近
                seeds.push_back(JointConfig::fromDegrees(
                    {j1Est+dj1, j2, j3, 0, 90, j1Est+dj1}));
            }
        }
        return seeds;
    };

    struct PlaceConfig { JointConfig approach, place; double aErr, pErr; };
    std::vector<PlaceConfig> placeConfigs;

    auto tIK = std::chrono::high_resolution_clock::now();
    int ikFail = 0;
    for (int i=0; i<numPositions; i++) {
        auto& bt = boxTargets[i];
        auto placeSeeds = makeSeeds(bt.pos_mm);
        auto ikP = soIKHorizontal(checker, robot, bt.pos_mm, placeSeeds, 3.0);
        if (!ikP.converged) ikP = multiStartIK(checker, robot, bt.pos_mm, placeSeeds, 3.0, true);
        auto aSeeds = placeSeeds;
        if (ikP.converged) aSeeds.insert(aSeeds.begin(), ikP.config);
        auto ikA = soIKHorizontal(checker, robot, bt.approach_mm, aSeeds, 3.0);
        if (!ikA.converged) ikA = multiStartIK(checker, robot, bt.approach_mm, aSeeds, 3.0, true);
        PlaceConfig pc; pc.place=ikP.config; pc.approach=ikA.config;
        pc.pErr=ikP.posError_mm; pc.aErr=ikA.posError_mm;
        placeConfigs.push_back(pc);
        if (!ikP.converged||!ikA.converged) ikFail++;
        printf("    [%2d] %-6s place:%s(%.1fmm,%diter) approach:%s(%.1fmm,%diter)\n",
               i, bt.label.c_str(), ikP.converged?"✅":"❌", ikP.posError_mm, ikP.iterations,
               ikA.converged?"✅":"❌", ikA.posError_mm, ikA.iterations);
    }
    double ikMs = std::chrono::duration<double,std::milli>(std::chrono::high_resolution_clock::now()-tIK).count();
    printf("\n  IK: %.1fms (%d/%d ok)\n", ikMs, numPositions-ikFail, numPositions);

    // FK验证 (位置 + TCP朝向 + 实际Z轴偏差)
    printf("\n  === FK验证 (%d位置) ===\n", numPositions);
    for (int i=0;i<numPositions;i++) {
        SO_COORD_REF tc; checker.forwardKinematics(placeConfigs[i].place, tc);
        auto& t=boxTargets[i].pos_mm;
        double e=std::sqrt(std::pow(tc.X-t.x(),2)+std::pow(tc.Y-t.y(),2)+std::pow(tc.Z-t.z(),2));
        auto& pq = placeConfigs[i].place;
        double downDev = std::fabs(tc.B);
        printf("    %-6s q=[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]deg err=%.1fmm |B|=%.1f°%s\n",
               boxTargets[i].label.c_str(),
               pq.q[0]*180/M_PI, pq.q[1]*180/M_PI, pq.q[2]*180/M_PI,
               pq.q[3]*180/M_PI, pq.q[4]*180/M_PI, pq.q[5]*180/M_PI,
               e, downDev, (e<5 && downDev<5)?"":"  ⚠️");
    }


    // 取料位IK
    printf("\n  === 取料位IK ===\n");
    double pkX=scene::CONV_CX, pkY=scene::CONV_CY, pkZ=scene::convSurfBase()+scene::BOX_HZ;
    Eigen::Vector3d pickTgt(pkX,pkY,pkZ), pickApprTgt(pkX,pkY,pkZ+300);
    auto pkSeeds = makeSeeds(pickTgt);

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
        printf("    取料: q=[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]deg %s(%.1fmm |B|=%.1f°)\n",
               PICK_POS.q[0]*180/M_PI, PICK_POS.q[1]*180/M_PI, PICK_POS.q[2]*180/M_PI,
               PICK_POS.q[3]*180/M_PI, PICK_POS.q[4]*180/M_PI, PICK_POS.q[5]*180/M_PI,
               ikPk.converged?"OK":"FAIL",ikPk.posError_mm, std::fabs(pkTcp.B));
        printf("    接近: %s(%.1fmm |B|=%.1f°)\n",
               ikPkA.converged?"OK":"FAIL",ikPkA.posError_mm, std::fabs(paTcp.B));
    }

    printf("  v8.0: 4箱码垛+多航点搬运+J6/J5实际修正\n");

    // ==== 中转高度扫描 (全局一次) ====
    printf("\n  === 中转高度扫描 (含工具球碰撞检测) ===\n");
    double transitJ2 = -90.0, transitJ5 = 90.0, transitZ_mm = 2272.0;
    {
        bool toolTmpOk = checker.setToolBall(1, Eigen::Vector3d(0, 0, -400), 120.0);
        printf("    临时工具球: %s\n", toolTmpOk?"ON":"FAIL");
        double frameTop = fZT;
        double minSafeZ = frameTop + 200.0;
        for (double j2 = -65; j2 >= -90; j2 -= 5) {
            double j5 = -j2;
            JointConfig qTest = JointConfig::fromDegrees({135, j2, 0, 0, j5, 0});
            SO_COORD_REF tc; checker.forwardKinematics(qTest, tc);
            auto rp = checker.getCollisionReport(qTest, false);
            bool collFree = !rp.selfCollision && !rp.envCollision;
            double Babs = std::fabs(tc.B);
            bool viable = (Babs < 10.0 && collFree && tc.Z > minSafeZ && rp.selfMinDist_mm > 5.0);
            printf("    J2=%4.0f J5=%3.0f: Z=%6.0fmm |B|=%4.1f self=%.1fmm %s%s\n",
                   j2, j5, tc.Z, Babs, rp.selfMinDist_mm,
                   collFree?"free":"COLL", viable?" *VIABLE":"");
            if (viable && tc.Z < transitZ_mm) {
                transitJ2 = j2; transitJ5 = j5; transitZ_mm = tc.Z;
            }
        }
        checker.removeTool(1);
        printf("  -> best transit: J2=%.0f J5=%.0f Z=%.0fmm\n", transitJ2, transitJ5, transitZ_mm);
    }

    // ==== Box-env障碍物模板 (搬运段共用) ====
    auto populateBoxEnv = [&](BoxCollisionConfig& boxCfg) {
        boxCfg.clearEnvObstacles();
        // 4根立柱
        boxCfg.addEnvCapsule(-fHW,fNY,fZB, -fHW,fNY,fZT, fR);
        boxCfg.addEnvCapsule( fHW,fNY,fZB,  fHW,fNY,fZT, fR);
        boxCfg.addEnvCapsule( fHW,fFY,fZB,  fHW,fFY,fZT, fR);
        boxCfg.addEnvCapsule(-fHW,fFY,fZB, -fHW,fFY,fZT, fR);
        // 4根顶梁 (2Y+2X)
        boxCfg.addEnvCapsule(-fHW,fNY,fZT, -fHW,fFY,fZT, fR);
        boxCfg.addEnvCapsule( fHW,fNY,fZT,  fHW,fFY,fZT, fR);
        boxCfg.addEnvCapsule(-fHW,fNY,fZT,  fHW,fNY,fZT, fR);
        boxCfg.addEnvCapsule(-fHW,fFY,fZT,  fHW,fFY,fZT, fR);
        // 4条电箱边
        double cHW_=scene::CAB_W/2, cHD_=scene::CAB_D/2, cZT_=-80;
        boxCfg.addEnvCapsule(-cHW_,-cHD_,cZT_, -cHW_, cHD_,cZT_, 80);
        boxCfg.addEnvCapsule( cHW_,-cHD_,cZT_,  cHW_, cHD_,cZT_, 80);
        boxCfg.addEnvCapsule(-cHW_,-cHD_,cZT_,  cHW_,-cHD_,cZT_, 80);
        boxCfg.addEnvCapsule(-cHW_, cHD_,cZT_,  cHW_, cHD_,cZT_, 80);
    };

    // ============================================================================
    // 阶段4: 4箱码垛执行
    // ============================================================================
    printf("\n====== Phase 4: 4-box palletizing (v8.0 multiWP carry + J6/J5 fix) ======\n");

    FILE* fpCsv = fopen("data/so_palletizing_profile.csv","w");
    if(fpCsv) fprintf(fpCsv,"task,segment,step,time_s,q1,q2,q3,q4,q5,q6,"
                      "selfDist_mm,selfCollision,envCollision,tcpX_mm,tcpY_mm,tcpZ_mm\n");
    FILE* fpTraj = fopen("data/so_palletizing_trajectory.txt","w");
    if(fpTraj) { fprintf(fpTraj,"# HR_S50-2000 palletizing v8.0 -- 4-box (multiWP carry + J6/J5 fix)\n");
                 fprintf(fpTraj,"# task seg time q1..q6 v1..v6 dist tcpX tcpY tcpZ\n"); }

    double totalMotionTime=0; int totalCollisions=0, totalEnvCollisions=0;
    double globalMinDist_mm=1e10; int totalSegments=0;
    double grandTotalPlanMs=0, grandTotalParamMs=0, grandTotalCollMs=0;
    const double boxEnvR = 250.0;

    for (int boxIdx = 0; boxIdx < numPositions; boxIdx++) {
        auto& bt = boxTargets[boxIdx];
        auto& pc = placeConfigs[boxIdx];
        int taskId = boxIdx + 1;

        printf("\n  +-------- box %d/%d: %s TCP=(%.0f,%.0f,%.0f)mm --------+\n",
               taskId, numPositions, bt.label.c_str(),
               bt.pos_mm.x(), bt.pos_mm.y(), bt.pos_mm.z());

        // J1旋转方向优化 (短弧选择)
        auto paDeg = PICK_APPROACH.toDegrees();
        auto plDeg = pc.approach.toDegrees();
        double pickJ1 = paDeg[0];
        double origJ1 = plDeg[0];
        double bestJ1 = origJ1;
        for (double shift : {360.0, -360.0}) {
            double cand = origJ1 + shift;
            if (cand < -360.0 || cand > 360.0) continue;
            if (std::fabs(pickJ1 - cand) < std::fabs(pickJ1 - bestJ1))
                bestJ1 = cand;
        }
        if (std::fabs(bestJ1 - origJ1) > 1.0) {
            double shift_rad = (bestJ1 - origJ1) * M_PI / 180.0;
            pc.place.q[0] += shift_rad;
            pc.approach.q[0] += shift_rad;
        }
        printf("    pick J1=%.1f -> place J1=%.1f (rotate %.1f deg)\n",
               pickJ1, bestJ1, std::fabs(pickJ1 - bestJ1));

        // CARRY_HIGH计算 (每箱独立J4/J6)
        double midJ1 = (paDeg[0] + bestJ1) / 2.0;
        planConfig.boxCollision.enabled = true;
        populateBoxEnv(planConfig.boxCollision);
        JointConfig CARRY_HIGH = computeCarryHigh(
            midJ1, transitJ2, transitJ5,
            PICK_APPROACH, pc.approach,
            checker, robot, planConfig.boxCollision);
        planConfig.boxCollision.enabled = false;
        planConfig.boxCollision.clearEnvObstacles();

        // verify CARRY_HIGH
        {
            SO_COORD_REF tc; checker.forwardKinematics(CARRY_HIGH, tc);
            auto rp = checker.getCollisionReport(CARRY_HIGH, false);
            auto cd = CARRY_HIGH.toDegrees();
            printf("    CARRY_HIGH: q=[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f] TCP=(%.0f,%.0f,%.0f) |B|=%.1f self=%.1f %s\n",
                   cd[0],cd[1],cd[2],cd[3],cd[4],cd[5],
                   tc.X,tc.Y,tc.Z, std::fabs(tc.B), rp.selfMinDist_mm,
                   (!rp.selfCollision && !rp.envCollision)?"ok":"COLL");
        }

        // 7段运动
        struct MotionSeg {
            const char* name;
            JointConfig start, target;
            bool useRRT;
            int toolAction;  // 0=none, 1=enable tool, 2=disable tool+add placed box
            bool freeTcp;
            bool optimizedCarry;  // v8.0: use multiWP carry
        };
        MotionSeg motions[] = {
            {"pickAppr->pick",           PICK_APPROACH,  PICK_POS,      false, 0, false, false},
            {"pick->pickAppr",           PICK_POS,       PICK_APPROACH, false, 1, false, false},
            {"carry:rise+rotate",        PICK_APPROACH,  CARRY_HIGH,    false, 0, false, true},
            {"carry:descend->plAppr",    CARRY_HIGH,     pc.approach,   false, 0, false, true},
            {"plAppr->place",            pc.approach,    pc.place,      false, 0, false, false},
            {"place->plAppr",            pc.place,       pc.approach,   false, 2, false, false},
            {"plAppr->pickAppr",         pc.approach,    PICK_APPROACH, true,  0, true,  false},
        };
        const int NUM_SEGS = 7;

        for (int s = 0; s < NUM_SEGS; s++) {
            auto& m = motions[s];
            double segDist = m.start.distanceTo(m.target);
            printf("\n    --- seg %d: %s (%.2f rad) ---\n", s, m.name, segDist);

            // pre-action: enable/disable tool collision
            if (m.toolAction == 1) {
                bool toolOk = checker.setToolBall(1, Eigen::Vector3d(0, 0, -400), 120.0);
                printf("    tool ball ON (z=-400, r=120): %s\n", toolOk?"ok":"FAIL");
                planConfig.boxCollision.enabled = true;
                populateBoxEnv(planConfig.boxCollision);
                planner.setConfig(planConfig);
                freeConfig.boxCollision = planConfig.boxCollision;
                freeConfig.boxCollision.enabled = true;
                plannerFree.setConfig(freeConfig);
            }

            SegmentResult seg;
            if (m.optimizedCarry) {
                // v8.0: multi-waypoint optimized carry (J6 yaw + J5 |B| correction)
                seg = executeOptimizedCarry(m.name, m.start, m.target,
                    robot, checker, planConfig.boxCollision,
                    parameterizer, fpCsv, taskId, s, fpTraj, 12);

                // post-check: box-env collision on optimized result
                int boxEnvColl = 0; double boxEnvMinD = 1e10; double maxB = 0;
                for (int i = 0; i <= 20; i++) {
                    double t = (double)i / 20;
                    JointConfig qm = m.start.interpolate(m.target, t);
                    if (planConfig.boxCollision.enabled) {
                        auto bxRp = checker.getBoxCollisionReport(qm, planConfig.boxCollision);
                        if (bxRp.envCollision) boxEnvColl++;
                        if (bxRp.envMinDistance_mm < boxEnvMinD) boxEnvMinD = bxRp.envMinDistance_mm;
                    }
                    SO_COORD_REF tc; checker.forwardKinematics(qm, tc);
                    double b = std::fabs(tc.B);
                    if (b > maxB) maxB = b;
                }
                printf("    box-env(linear ref): %d/21 coll minDist=%.1fmm |B|max=%.1f\n",
                       boxEnvColl, boxEnvMinD, maxB);
            }
            else if (m.useRRT) {
                auto& activePlanner = m.freeTcp ? plannerFree : planner;
                static const std::vector<TCPPlannerConfig::TCPExclusionBox> emptyBoxes;
                auto& activeExclBoxes = m.freeTcp ? emptyBoxes : planConfig.tcpExclusionBoxes;
                seg = executeRRTStar(m.name, m.start, m.target,
                                     robot, checker, activePlanner, parameterizer,
                                     fpCsv, taskId, s, fpTraj, activeExclBoxes);
            }
            else {
                seg = executeP2P(m.name, m.start, m.target,
                                 robot, checker, parameterizer, fpCsv, taskId, s, fpTraj);
            }

            printf("    [%d] %-24s %s %.3fs plan:%.1fms param:%.1fms dist:%.0fmm env:%d\n",
                   s, seg.name.c_str(), seg.success?"ok":"FAIL", seg.totalTime_s,
                   seg.planningTime_ms, seg.paramTime_ms, seg.minSelfDist_mm, seg.envCollisionCount);

            // post-action: disable tool + add placed box
            if (m.toolAction == 2) {
                checker.removeTool(1);
                planConfig.boxCollision.enabled = false;
                planConfig.boxCollision.clearEnvObstacles();
                planner.setConfig(planConfig);
                freeConfig.boxCollision.enabled = false;
                freeConfig.boxCollision.clearEnvObstacles();
                plannerFree.setConfig(freeConfig);
                printf("    tool OFF, box collision OFF\n");

                // add placed box as env obstacle
                SO_COORD_REF tc; checker.forwardKinematics(pc.place, tc);
                int eid = 46 + boxIdx;
                bool ok = checker.addEnvObstacleBall(eid,
                    Eigen::Vector3d(tc.X, tc.Y, tc.Z - scene::BOX_HZ/2), boxEnvR);
                printf("    placed -> envId=%d (%.0f,%.0f,%.0f) r=%.0f: %s\n",
                       eid, tc.X, tc.Y, tc.Z - scene::BOX_HZ/2, boxEnvR, ok?"ok":"FAIL");
            }

            totalMotionTime += seg.totalTime_s;
            totalCollisions += seg.collisionCount;
            totalEnvCollisions += seg.envCollisionCount;
            if (seg.minSelfDist_mm < globalMinDist_mm) globalMinDist_mm = seg.minSelfDist_mm;
            grandTotalPlanMs += seg.planningTime_ms + seg.optimizationTime_ms;
            grandTotalParamMs += seg.paramTime_ms;
            grandTotalCollMs += seg.collCheckTime_ms;
            totalSegments++;
        }

        printf("  +-------- box %d done --------+\n", taskId);
    }

    if(fpCsv) fclose(fpCsv);
    if(fpTraj) fclose(fpTraj);

    // ---- statistics ----
    double totalElapsed = std::chrono::duration<double>(std::chrono::high_resolution_clock::now()-t_global).count();

    printf("\n====== Phase 5: Statistics ======\n\n");
    printf("  boxes: %d  segments: %d  total motion: %.3fs\n", numPositions, totalSegments, totalMotionTime);
    printf("  self collisions: %d  env collisions: %d\n", totalCollisions, totalEnvCollisions);
    printf("  min distance: %.1fmm\n", globalMinDist_mm);
    printf("  IK: %.1fms  planning: %.1fms  param: %.1fms  coll check: %.1fms\n",
           ikMs, grandTotalPlanMs, grandTotalParamMs, grandTotalCollMs);
    printf("  total elapsed: %.2fs\n", totalElapsed);

    auto collStats = checker.getTimingStats();
    printf("\n%s", collStats.toString().c_str());

    // write summary
    FILE* fpSum = fopen("data/so_palletizing_summary.txt","w");
    if (fpSum) {
        fprintf(fpSum,"# HR_S50-2000 palletizing v8.0 -- 4-box + TCP rotation avoidance\n\nversion: 8.0\n");
        fprintf(fpSum,"positions: %d\nsegments: %d\ntotal_motion_s: %.4f\n",numPositions,totalSegments,totalMotionTime);
        fprintf(fpSum,"self_collisions: %d\nenv_collisions: %d\nmin_dist_mm: %.2f\n",
                totalCollisions,totalEnvCollisions,globalMinDist_mm);
        fprintf(fpSum,"order: BK-L_BK-R_FR-L_FR-R_column_first\n");
        fprintf(fpSum,"env_obstacles: 4_cabinet+3_conveyor+4_pillar+2_topbar+3_lozengeWall+dynamic\n");
        fprintf(fpSum,"carry_strategy: optimized_multiWP_J6yaw_J5tilt_correction\n");
        fprintf(fpSum,"box_collision: 26pt_OBB_envCapsule_margin40mm\n");
        fprintf(fpSum,"frame_gap_mm: %.0f\nconv_gap_mm: %.0f\nframe_cy_mm: %.0f\n",
                scene::FRAME_GAP, scene::CONV_GAP, fcy);

        for (int bi = 0; bi < numPositions; bi++) {
            SO_COORD_REF tc; checker.forwardKinematics(placeConfigs[bi].place,tc);
            fprintf(fpSum,"\nbox_%d_label: %s\nbox_%d_tcp_mm: %.1f %.1f %.1f\n",
                    bi, boxTargets[bi].label.c_str(), bi, tc.X,tc.Y,tc.Z);
            auto q=placeConfigs[bi].place.toDegrees();
            fprintf(fpSum,"ik_place_%d: %.2f %.2f %.2f %.2f %.2f %.2f\n",
                    bi, q[0],q[1],q[2],q[3],q[4],q[5]);
        }
        {auto q=PICK_POS.toDegrees(); auto a=PICK_APPROACH.toDegrees();
         fprintf(fpSum,"\nik_pick: %.2f %.2f %.2f %.2f %.2f %.2f\nik_pick_appr: %.2f %.2f %.2f %.2f %.2f %.2f\n",
                 q[0],q[1],q[2],q[3],q[4],q[5],a[0],a[1],a[2],a[3],a[4],a[5]);}
        {SO_COORD_REF ptc; checker.forwardKinematics(PICK_POS,ptc);
         fprintf(fpSum,"pick_tcp_mm: %.1f %.1f %.1f\n",ptc.X,ptc.Y,ptc.Z);}

        fprintf(fpSum,"\ninit_ms: %.3f\nik_ms: %.3f\nplanning_total_ms: %.3f\n",initMs,ikMs,grandTotalPlanMs);
        fprintf(fpSum,"param_total_ms: %.3f\ncollision_runtime_ms: %.3f\nelapsed_s: %.3f\n",
                grandTotalParamMs,grandTotalCollMs,totalElapsed);
        auto cs=checker.getTimingStats();
        fprintf(fpSum,"coll_init_ms: %.3f\ncoll_total_avg_us: %.3f\ncoll_calls: %d\n",
                cs.initTime_ms,cs.totalCheckTime_us,cs.callCount);
        fclose(fpSum);
    }

    printf("\n================================================================\n");
    printf("  HR_S50-2000 palletizing simulation v8.0 -- 4-box + TCP rotation avoidance\n");
    printf("================================================================\n");
    return 0;
}
