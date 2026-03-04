/**
 * @file testS50PalletizingSO.cpp
 * @brief HR_S50-2000 码垛仿真 v6.5 — +X/+Y短弧搬运路径优化
 *
 * v6.5 核心变更:
 *   1. J1短弧优化: 放料J1=-85°→275° (mod 360°), J1旋转116.5°替代243.5°
 *      - 搬运路径经过+X/+Y象限, 不再从顶部越过
 *   2. 2段对角搬运: 抬升与J1旋转并行, 3段→2段 (40%+时间节省)
 *      - KEY INSIGHT: J1不影响|B|(TCP pitch), 对角运动不增加TCP偏差
 *   3. 中转高度扫描: FK验证多种J2/J5, 选择满足|B|<15°的最低高度
 *
 * v6.4 变更:
 *   1. 场景布局优化: FRAME_GAP 200→500, CONV_GAP 400→600
 *   2. 箱子放置位置: 框架-X,+Y角落 (左后角)
 *   3. 运动简化: 起点/终点=取料接近位 (传送带上方)
 *   4. 箱子碰撞强化: Base+LowerArm硬约束, Elbow诊断, margin=40mm
 *
 * v6.3 变更: 独立箱子-机械臂碰撞检测 (BoxCollisionChecker)
 *
 * v6.2 变更:
 *   1. 工具碰撞体: 球 z=-400 r=120 (toolIdx=1)
 *      - SO库限制: pair(6,4) 工具vs腕部(r=140mm) 自碰撞距离硬编码
 *      - 箱子顶部(z=0~-250)无法通过SO API建模为工具碰撞体
 *      - 已测试: 胶囊z=-250/-300均碰撞, 通道控制(0,0,0)无效
 *      - 覆盖范围: z=-280→z=-520mm (箱子底面下方150mm保护)
 *   2. 取料TCP写入摘要 (pick_tcp_mm) 供MATLAB读取箱子位置
 *   3. 传送带箱位修正: 与C++ IK取料位一致
 *
 * v6.1 变更:
 *   1. 布局修正: FRAME_GAP 50→200, CONV_GAP 200→400, CONV_LEN 3500→2000
 *   2. 面板碰撞: 3个Lozenge OBB实体面板
 *   3. 工具球修正: toolIndex 6→1
 *   4. 路径后优化: 300次shortcut
 *
 * @date 2026-03-03
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
// 场景几何参数 (mm, 与MATLAB一致)
// ============================================================================
namespace scene {
    constexpr double CAB_W = 550, CAB_D = 650, CAB_H = 800;
    // 框架深度缩小至700mm, 确保后排箱子在机器人工作空间内 (~935mm max Y)
    constexpr double FRM_W = 1200, FRM_D = 650, FRM_H = 2000;
    constexpr double FRM_TUBE_R = 30;
    constexpr double PAL_W = 1000, PAL_D = 600, PAL_H = 500;
    constexpr double CONV_LEN = 2000, CONV_W = 550, CONV_H = 750;
    constexpr double CONV_GAP = 600;   // v6.4: 增大传送带间距 (显示机械臂完整工作范围)
    constexpr double CONV_OFF_Y = -800;
    constexpr double BOX_LX = 350, BOX_WY = 280, BOX_HZ = 250;
    constexpr double FRAME_GAP = 500;  // v6.4: 增大框架间距 (电箱与框架距离更大, 显示臂展可达范围)
    constexpr double BOX_GAP = 20;
    constexpr double baseZ = CAB_H;
    inline double frameCY() { return CAB_D/2 + FRAME_GAP + FRM_D/2; }
    inline double palletSurfBase() { return PAL_H - baseZ; }
    inline double convCX() { return CAB_W/2 + CONV_GAP + CONV_W/2; }
    inline double convSurfBase() { return CONV_H + 30 + 35 - baseZ; }
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
    printf("║   HR_S50-2000 码垛仿真 v6.5 — +X/+Y短弧搬运路径优化            ║\n");
    printf("║   J1短弧(116°) + 2段对角搬运 + 中转高度优化                     ║\n");
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
    // 框架位置: CY=700, HW=600, NY=375, FY=1025, ZB=-800, ZT=1200 (mm)
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
                int ids[] = {5, 6, 7, 8, 46+12, 46+13};
                for (int i = 0; i < nLevels; i++) {
                    double zi = fZB + zSpan * (2*i + 1) / (2*nLevels);
                    int eid = (i < 4) ? ids[i] : (46+12+i-4);
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
                    int eid = 22 + i;  // envId 22-27
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
                    int eid = (i < 4) ? (26 + i) : (46+14+i-4);  // envId 26-29, 60-61
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
    double cvX=scene::convCX(), cvY=scene::CONV_OFF_Y, cvSZ=scene::convSurfBase();
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
    printf("\n  环境障碍: 电箱(4) + 传送带(3) + 框架立柱(4) + 框架顶梁(2) + 框架面板Lozenge/胶囊(3) + 已放箱子(动态)\n");
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
    // IK种子: 箱子在-X,+Y角落, J1≈-70°~-110° (臂指向-X,+Y方向)
    // TCP保持水平: q5≈90+offset, q6≈q1
    std::vector<JointConfig> placeSeeds;
    double sA[][6] = {
        {-70,-60,25,0,90,-70},{-80,-55,30,0,90,-80},{-90,-50,35,0,90,-90},
        {-100,-65,20,0,90,-100},{-75,-55,30,0,90,-75},{-85,-60,25,0,90,-85},
        {-95,-50,35,0,90,-95},{-65,-70,20,0,90,-65},{-105,-55,30,0,90,-105},
        {-80,-45,40,0,90,-80},{-90,-40,45,0,90,-90},{-70,-50,35,0,90,-70},
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
    double pkX=scene::convCX(), pkY=scene::CONV_OFF_Y+500, pkZ=scene::convSurfBase()+scene::BOX_HZ;
    Eigen::Vector3d pickTgt(pkX,pkY,pkZ), pickApprTgt(pkX,pkY,pkZ+300);
    std::vector<JointConfig> pkSeeds;
    // 取料种子: convCX=1150mm(v6.4), 需要J1接近180°(arm→+X方向)
    // 提供宽范围J1覆盖新传送带位置
    double pS[][6] = {{170,-55,30,0,90,170},{175,-50,35,0,90,175},{180,-50,35,0,90,180},
                      {165,-60,25,0,90,165},{-175,-55,30,0,90,-175},{185,-50,35,0,90,185},
                      {175,-45,40,0,90,175},{180,-60,25,0,90,180},{-170,-55,30,0,90,-170},
                      {170,-45,40,0,90,170},{190,-50,35,0,90,190},{175,-65,20,0,90,175}};
    for (auto& s:pS) pkSeeds.push_back(JointConfig::fromDegrees({s[0],s[1],s[2],s[3],s[4],s[5]}));

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
    // 取料J1≈158° → 放料J1≈-85° = 243.5° (长弧, 经过-X/-Y方向)
    // 放料J1+360=275° → 158→275 = 116.5° (短弧, 经过+X/+Y方向!) 
    // J1=-85° ≡ J1=275° (mod 360°): FK完全相同, 仅关节空间路径不同
    printf("\n  === J1旋转方向优化 (短弧选择) ===\n");
    {
        auto pickDeg_opt = PICK_APPROACH.toDegrees();
        double pickJ1 = pickDeg_opt[0];  // ≈158.5°
        auto& pc0 = placeConfigs[0];
        auto placeDeg_opt = pc0.approach.toDegrees();
        double origJ1 = placeDeg_opt[0]; // ≈-85°
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

    // v6.5: J1短弧搬运路径 — 经过+X/+Y象限, 2段对角P2P
    printf("  📌 v6.5: J1短弧+2段对角搬运, 经过+X/+Y象限\n");

    // ---- 单箱搬运执行 ----
    printf("\n━━━━━━ 阶段4: 单箱搬运 (v6.5 +X/+Y短弧搬运) ━━━━━━━━━━━━━\n\n");

    FILE* fpCsv = fopen("data/so_palletizing_profile.csv","w");
    if(fpCsv) fprintf(fpCsv,"task,segment,step,time_s,q1,q2,q3,q4,q5,q6,"
                      "selfDist_mm,selfCollision,envCollision,tcpX_mm,tcpY_mm,tcpZ_mm\n");
    FILE* fpTraj = fopen("data/so_palletizing_trajectory.txt","w");
    if(fpTraj) { fprintf(fpTraj,"# HR_S50-2000 码垛v6.5 — 单箱搬运 (7段+X/+Y短弧)\n");
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

    // v6.5: +X/+Y短弧搬运 — 2段对角P2P (J1旋转与升降并行)
    //
    // v6.4问题: 搬运3段P2P经过顶部 (J1旋转243.5°, 绕过-X/-Y)
    //   - 总搬运时间12.2s (抬升4.3s + 旋转3.8s + 降低4.1s)
    //   - 路径从机械臂正上方越过, 距离长
    //
    // v6.5优化:
    //   1. J1短弧: 放料J1+360° → J1旋转仅116.5° (经过+X/+Y象限)
    //   2. 2段对角: J1旋转与升降并行 (S曲线各关节同时运动)
    //      seg_up:   PICK_APPROACH → CARRY_HIGH (J1转到放料方向 + 升高)
    //      seg_down: CARRY_HIGH → place_approach (仅降低, J1已到位)
    //   3. KEY INSIGHT: J1是基座旋转, 不影响|B|(TCP pitch)
    //      → 对角运动的TCP偏差 = 纯升降的TCP偏差 (已验证<30°)
    //
    // 为什么不用直接传输 (PICK→PLACE单段P2P):
    //   工作高度臂展1200mm, J1旋转116°时臂端扫过框架附近 (Y=825mm)
    //   → 必须先升高到框架上方, 再降低到放料位

    // 取/放 接近配置 (J1已通过短弧优化)
    auto paDeg = PICK_APPROACH.toDegrees();  // 取料接近
    auto plDeg = pc.approach.toDegrees();    // 放料接近 (J1已优化为+275°)

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

    // 高位中转: 仅配置CARRY_HIGH (放料J1方向, 选定高度)
    // v6.5: 只需一个HIGH点 (在放料J1方向), 不再需要HIGH_PICK
    // 2段对角: PICK_APPROACH → CARRY_HIGH → place_approach
    //   seg_up:   J1从取料→放料 + 升高 (并行, S曲线)
    //   seg_down: J1不变 + 降低 (仅纵向运动)
    // J4=0,J6=0: 确保B=0 (TCP精确朝下)
    JointConfig CARRY_HIGH = JointConfig::fromDegrees(
        {(double)plDeg[0], transitJ2, 0.0, 0.0, transitJ5, 0.0});

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
    // v6.5: 7段运动 (2段对角搬运替代3段顺序搬运)
    MotionSeg motions[] = {
        {"取料接近→取料",         PICK_APPROACH,  PICK_POS,      false, 0, false},
        {"取料→取料抬升",         PICK_POS,       PICK_APPROACH, false, 1, false},  // ★启用工具球+箱子碰撞
        {"搬运:对角上升+旋转",    PICK_APPROACH,  CARRY_HIGH,    false, 0, false},  // ★对角: J1旋转+升高并行
        {"搬运:降低→放料接近",    CARRY_HIGH,     pc.approach,   false, 0, false},  // 降低到放料位
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
            planner.setConfig(planConfig);
            // v6.4: 搬运段使用plannerFree (Free-TCP), 也需要箱子碰撞
            freeConfig.boxCollision = planConfig.boxCollision;
            plannerFree.setConfig(freeConfig);
            printf("  📦 箱子-机械臂碰撞检测: ✅ ON (%.0f×%.0f×%.0fmm, margin=%.0fmm)\n",
                   planConfig.boxCollision.boxLengthX,
                   planConfig.boxCollision.boxWidthY,
                   planConfig.boxCollision.boxHeightZ,
                   planConfig.boxCollision.safetyMargin);

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

            // ★ 关闭箱子-机械臂碰撞检测 (箱子已放下)
            planConfig.boxCollision.enabled = false;
            planner.setConfig(planConfig);
            freeConfig.boxCollision.enabled = false;
            plannerFree.setConfig(freeConfig);
            printf("  📦 箱子-机械臂碰撞检测: OFF (箱子已放置)\n");
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
        fprintf(fpSum,"# HR_S50-2000 码垛v6.5 — +X/+Y短弧搬运路径优化\n\nversion: 6.5\n");
        fprintf(fpSum,"positions: 1\nsegments: %d\ntotal_motion_s: %.4f\n",totalSegments,totalMotionTime);
        fprintf(fpSum,"self_collisions: %d\nenv_collisions: %d\nmin_dist_mm: %.2f\n",
                totalCollisions,totalEnvCollisions,globalMinDist_mm);
        fprintf(fpSum,"order: single_box_conveyor_to_frame\n");
        fprintf(fpSum,"env_obstacles: 4_cabinet+3_conveyor+4_pillar+2_topbar+3_lozengeWall+1_placed\n");
        fprintf(fpSum,"wall_panels: back+left+right_lozengeOBB_100mm_thick\n");
        fprintf(fpSum,"tool_collision: ball_z-400_r120_toolIdx1_SOlimit_pair64\n");
        fprintf(fpSum,"carry_strategy: short_arc_2diag_J1opt_XpYp_transit\n");
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
    printf("  HR_S50-2000 码垛仿真 v6.5 — +X/+Y短弧搬运路径优化\n");
    printf("══════════════════════════════════════════════════════════════════\n");
    return 0;
}
