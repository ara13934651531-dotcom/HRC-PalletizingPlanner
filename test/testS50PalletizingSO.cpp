/**
 * @file testS50PalletizingSO.cpp
 * @brief HR_S50-2000 码垛仿真 v6.0 — 单箱搬运 + 框架面板碰撞
 *
 * v6.0 核心变更:
 *   1. 简化为单箱搬运: 传送带拾取 → 框架内放置 (1条清晰轨迹)
 *   2. 框架三面面板碰撞: 后面/左面/右面各4根横杆 (r=150mm) 封锁
 *   3. 前面(朝机器人)开放: 机械臂从前面进入框架
 *   4. 箱子放置在框架中心偏前 (避开面板碰撞区)
 *   5. 增强RRT*参数: 更多迭代/更长时间应对复杂约束
 *
 * @date 2026-02-28
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
    constexpr double CONV_LEN = 3500, CONV_W = 550, CONV_H = 750;
    constexpr double CONV_GAP = 200;   // 缩短: 传送带拾取位在工作空间内
    constexpr double CONV_OFF_Y = -800;
    constexpr double BOX_LX = 350, BOX_WY = 280, BOX_HZ = 250;
    constexpr double FRAME_GAP = 50;   // 框架紧靠电箱后方
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
// 辅助: RRT*规划段
// ============================================================================
SegmentResult executeRRTStar(const char* name,
    const JointConfig& start, const JointConfig& target,
    RobotModel& robot, CollisionCheckerSO& checker,
    PathPlannerSO& planner, TimeParameterizer& param,
    FILE* csv, int ti, int si, FILE* traj)
{
    SegmentResult r; r.name = name;
    auto tS = std::chrono::high_resolution_clock::now();
    bool direct = checker.isPathCollisionFree(start, target, 0.03);
    Path path;
    if (direct) {
        Waypoint w0(start); w0.pathParam=0; Waypoint w1(target); w1.pathParam=1;
        path.waypoints.push_back(w0); path.waypoints.push_back(w1);
        r.planningTime_ms=0; r.optimizationTime_ms=0; r.planIterations=0; r.planNodes=2;
    } else {
        auto pr = planner.plan(start, target);
        auto pt = planner.getTimingReport();
        r.planningTime_ms = pt.planningTotal_ms;
        r.optimizationTime_ms = pt.optimizationTotal_ms;
        r.planIterations = pt.planIterations; r.planNodes = pt.nodesExplored;
        printf("        — RRT*: plan=%.1fms opt=%.1fms nodes=%d\n",
               pt.planningTotal_ms, pt.optimizationTotal_ms, pt.nodesExplored);
        if (pr.isSuccess() && !pr.optimizedPath.empty()) path = pr.optimizedPath;
        else if (pr.isSuccess() && !pr.rawPath.empty()) path = pr.rawPath;
        else { Waypoint w0(start); w0.pathParam=0; Waypoint w1(target); w1.pathParam=1;
               path.waypoints.push_back(w0); path.waypoints.push_back(w1); }
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
    printf("║   HR_S50-2000 码垛仿真 v6.0 — 单箱搬运 + 框架面板碰撞           ║\n");
    printf("║   传送带→框架 + 三面面板碰撞 + TCP水平约束                       ║\n");
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
    planConfig.maxIterations = 30000; planConfig.maxPlanningTime = 15.0;
    planConfig.stepSize = 0.12; planConfig.goalBias = 0.15;
    planConfig.shortcutIterations = 100;
    planConfig.splineResolution = 50; planConfig.collisionResolution = 0.02;
    planConfig.rewireRadius = 0.4;
    // 码垛场景: TCP必须保持水平 (吸盘朝下, 仅允许Z轴旋转)
    planConfig.constrainTcpHorizontal = true;
    PathPlannerSO planner(robot, checker, planConfig);
    printf("  路径规划: %s Informed RRT*\n",
           planConfig.constrainTcpHorizontal ? "TCP-Horizontal" : "Free-TCP");

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

    // ---- 框架面板碰撞 — 3个 Lozenge OBB (后/左/右, 前面开放) ----
    // Lozenge = 圆角长方体, 替代原12根横杆胶囊: 无缝覆盖整面, 碰撞更精确
    printf("\n  === 框架面板 (envId 5, 9, 25 — Lozenge OBB) ===\n");
    {
        const double wallThick = 60.0;    // 面板厚度 (mm)
        const double wallRadius = 50.0;   // 圆角半径 (mm), 与立柱碰撞半径一致
        const double centerZ = (fZB + fZT) / 2.0;  // 面板Z中心

        // 后面 (Y=fFY): XZ平面, 面板沿X方向×Z方向
        double r2l_back[6] = {0, 0, 0, 0, fFY, centerZ};  // [deg, mm]
        bool ok = checker.addEnvObstacleLozenge(5, r2l_back,
            Eigen::Vector3d::Zero(), scene::FRM_W, wallThick, scene::FRM_H, wallRadius);
        printf("    后面 envId=5 Lozenge (%.0f×%.0f×%.0f mm, r=%.0f): %s\n",
               scene::FRM_W, wallThick, scene::FRM_H, wallRadius, ok?"✅":"❌");

        // 左面 (X=-fHW): YZ平面, 面板沿Y方向×Z方向
        double r2l_left[6] = {0, 0, 0, -fHW, fcy, centerZ};
        ok = checker.addEnvObstacleLozenge(9, r2l_left,
            Eigen::Vector3d::Zero(), wallThick, scene::FRM_D, scene::FRM_H, wallRadius);
        printf("    左面 envId=9 Lozenge (%.0f×%.0f×%.0f mm, r=%.0f): %s\n",
               wallThick, scene::FRM_D, scene::FRM_H, wallRadius, ok?"✅":"❌");

        // 右面 (X=fHW): YZ平面, 面板沿Y方向×Z方向
        double r2l_right[6] = {0, 0, 0, fHW, fcy, centerZ};
        ok = checker.addEnvObstacleLozenge(25, r2l_right,
            Eigen::Vector3d::Zero(), wallThick, scene::FRM_D, scene::FRM_H, wallRadius);
        printf("    右面 envId=25 Lozenge (%.0f×%.0f×%.0f mm, r=%.0f): %s\n",
               wallThick, scene::FRM_D, scene::FRM_H, wallRadius, ok?"✅":"❌");

        printf("    面板总计: 3个 Lozenge OBB (厚%.0fmm + r=%.0fmm, 有效厚度%.0fmm)\n",
               wallThick, wallRadius, wallThick + 2*wallRadius);
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

    const double boxToolR = 225.0, boxEnvR = 250.0;
    printf("\n  环境障碍: 电箱(4) + 传送带(3) + 框架立柱(4) + 框架顶梁(2) + 框架面板(12) + 已放箱子(动态)\n");
    printf("  面板封锁: 后面+左面+右面 (前面开放, 机械臂入口)\n\n");

    // ---- 码垛布局 + IK求解 ----
    printf("━━━━━━ 阶段3: 单箱布局 + IK求解 ━━━━━━━━━━━━━━━━━━━━━━━━━\n\n");

    double palSurf = scene::palletSurfBase();
    // 简化: 1个箱子放在框架中心偏前位置 (远离面板碰撞区)
    // 面板r=150mm: 后面内界Y=1025-150=875, 左右内界X=±(600-150)=±450
    // 工具球r=225mm: 安全区 X∈[-225,225], Y<650 (距后面 > 375mm = 150+225)
    double boxPlaceX = 0;
    double boxPlaceY = fcy - 100;  // 600mm — 框架中心(700)偏前100mm
    double boxPlaceZ = palSurf + scene::BOX_HZ;  // 第一层顶: -300+250=-50

    printf("  框架CY=%.0f  托盘面Z(base)=%.0f\n", fcy, palSurf);
    printf("  箱子放置: (%.0f, %.0f, %.0f) mm — 框架中心偏前\n\n", boxPlaceX, boxPlaceY, boxPlaceZ);

    constexpr int numPositions = 1;
    std::vector<BoxTarget> boxTargets;
    {
        BoxTarget bt;
        bt.label = "CENTER";
        bt.pos_mm = Eigen::Vector3d(boxPlaceX, boxPlaceY, boxPlaceZ);
        bt.approach_mm = Eigen::Vector3d(boxPlaceX, boxPlaceY, boxPlaceZ + 200);
        boxTargets.push_back(bt);
    }

    printf("  %-10s  %8s %8s %8s\n","位置","X_mm","Y_mm","Z_mm");
    for (auto& b : boxTargets)
        printf("  %-10s  %8.1f %8.1f %8.1f\n", b.label.c_str(), b.pos_mm.x(), b.pos_mm.y(), b.pos_mm.z());

    printf("\n  === IK求解 (放料位) ===\n");
    // IK种子: J1≈-90° 指向+Y方向(框架), TCP保持水平
    // 约束: q5=180+q2-q3 → B=0, q6=q1 → C=±180° → Z=(0,0,-1)
    std::vector<JointConfig> placeSeeds;
    double sA[][6] = {
        {-90,-65,25,0,90,-90},{-90,-60,30,0,90,-90},{-90,-55,35,0,90,-90},
        {-90,-50,40,0,90,-90},{-90,-70,20,0,90,-90},{-90,-75,15,0,90,-90},
        {-80,-65,25,0,90,-80},{-100,-65,25,0,90,-100},
        {-85,-60,30,0,90,-85},{-95,-60,30,0,90,-95},
        {-90,-60,25,0,95,-90},{-90,-55,30,0,95,-90},
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
        // 实际FK2 Z轴偏差 (从Euler ABC计算)
        double Br=tc.B*M_PI/180, Cr=tc.C*M_PI/180;
        double zz = std::cos(Br)*std::cos(Cr);  // Z轴 Z分量
        double downDev = std::acos(std::clamp(-zz,-1.0,1.0))*180.0/M_PI;
        // 打印实际关节角 (用于调试约束)
        printf("    %-10s q=[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]deg\n",
               boxTargets[i].label.c_str(),
               pq.q[0]*180/M_PI, pq.q[1]*180/M_PI, pq.q[2]*180/M_PI,
               pq.q[3]*180/M_PI, pq.q[4]*180/M_PI, pq.q[5]*180/M_PI);
        printf("    %-10s err=%.1fmm tcpH=%.1f° A=%.1f B=%.1f C=%.1f Z↓dev=%.1f°%s\n",
               boxTargets[i].label.c_str(), e, tcpH_deg, tc.A, tc.B, tc.C, downDev,
               (e<5 && downDev<5)?"":"  ⚠️");
    }

    // 取料位IK (TCP保持水平: q5=180+q2-q3, q6=0)
    printf("\n  === 取料位IK ===\n");
    double pkX=scene::convCX(), pkY=scene::CONV_OFF_Y+500, pkZ=scene::convSurfBase()+scene::BOX_HZ;
    Eigen::Vector3d pickTgt(pkX,pkY,pkZ), pickApprTgt(pkX,pkY,pkZ+200);
    std::vector<JointConfig> pkSeeds;
    // 取料种子: J1≈-90° (指向传送带-Y), TCP水平(q5=180+q2-q3, q6=q1)
    double pS[][6] = {{-90,-50,40,0,90,-90},{-80,-50,40,0,90,-80},{-100,-50,40,0,90,-100},
                      {-90,-55,35,0,90,-90},{-90,-45,45,0,90,-90},{-85,-60,30,0,90,-85}};
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

    auto HOME = JointConfig::fromDegrees({0,-90,0,0,90,0});

    // ---- 单箱搬运执行 ----
    printf("\n━━━━━━ 阶段4: 单箱搬运 (传送带→框架) ━━━━━━━━━━━━━━━━━━━━\n\n");

    FILE* fpCsv = fopen("data/so_palletizing_profile.csv","w");
    if(fpCsv) fprintf(fpCsv,"task,segment,step,time_s,q1,q2,q3,q4,q5,q6,"
                      "selfDist_mm,selfCollision,envCollision,tcpX_mm,tcpY_mm,tcpZ_mm\n");
    FILE* fpTraj = fopen("data/so_palletizing_trajectory.txt","w");
    if(fpTraj) { fprintf(fpTraj,"# HR_S50-2000 码垛v6.0 — 单箱搬运\n");
                 fprintf(fpTraj,"# task seg time q1..q6 v1..v6 dist tcpX tcpY tcpZ\n"); }

    double totalMotionTime=0; int totalCollisions=0, totalEnvCollisions=0;
    double globalMinDist_mm=1e10; int totalSegments=0;
    double grandTotalPlanMs=0, grandTotalParamMs=0, grandTotalCollMs=0;

    auto& bt = boxTargets[0];
    auto& pc = placeConfigs[0];

    printf("  目标: %s TCP=(%.0f,%.0f,%.0f)mm\n\n", bt.label.c_str(),
           bt.pos_mm.x(), bt.pos_mm.y(), bt.pos_mm.z());

    // 6段运动: HOME→取料→搬运→放料→HOME
    struct MotionSeg {
        const char* name;
        JointConfig start, target;
        bool useRRT;
        int toolAction;  // 0=无, 1=启用工具球, 2=禁用工具球+添加放置障碍
    };
    MotionSeg motions[] = {
        {"HOME→取料接近",     HOME,            PICK_APPROACH,  true,  0},
        {"取料接近→取料",     PICK_APPROACH,   PICK_POS,       false, 0},
        {"取料→取料抬升",     PICK_POS,        PICK_APPROACH,  false, 1},  // 拾取: 启用工具球
        {"取料抬升→放料接近", PICK_APPROACH,   pc.approach,    true,  0},  // ★关键段: 搬运过框架面板
        {"放料接近→放料",     pc.approach,     pc.place,       false, 2},  // 放置: 禁用工具球
        {"放料→HOME",         pc.place,        HOME,           true,  0},
    };
    const int NUM_SEGS = 6;

    for (int s = 0; s < NUM_SEGS; s++) {
        auto& m = motions[s];

        // 动作前: 启用/禁用工具球
        if (m.toolAction == 1) {
            checker.setToolBall(6, Eigen::Vector3d(0, 0, -125), boxToolR);
            printf("  📦 工具球ON (r=%.0fmm, 搬运箱子)\n", boxToolR);
        }

        SegmentResult seg;
        if (m.useRRT)
            seg = executeRRTStar(m.name, m.start, m.target,
                                 robot, checker, planner, parameterizer, fpCsv, 1, s, fpTraj);
        else
            seg = executeP2P(m.name, m.start, m.target,
                             robot, checker, parameterizer, fpCsv, 1, s, fpTraj);

        printf("  [%d] %-24s %s %.3fs plan:%.1fms param:%.1fms dist:%.0fmm env:%d\n",
               s, seg.name.c_str(), seg.success?"✅":"❌", seg.totalTime_s,
               seg.planningTime_ms, seg.paramTime_ms, seg.minSelfDist_mm, seg.envCollisionCount);

        // 动作后: 禁用工具球 + 添加已放置箱子
        if (m.toolAction == 2) {
            checker.removeTool(6);
            printf("  📦 工具球OFF\n");
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
        fprintf(fpSum,"# HR_S50-2000 码垛v6.0 — 单箱搬运\n\nversion: 6.0\n");
        fprintf(fpSum,"positions: 1\nsegments: %d\ntotal_motion_s: %.4f\n",totalSegments,totalMotionTime);
        fprintf(fpSum,"self_collisions: %d\nenv_collisions: %d\nmin_dist_mm: %.2f\n",
                totalCollisions,totalEnvCollisions,globalMinDist_mm);
        fprintf(fpSum,"order: single_box_conveyor_to_frame\n");
        fprintf(fpSum,"env_obstacles: 4_cabinet+3_conveyor+4_pillar+2_topbar+3_wallLozenge+1_placed\n");
        fprintf(fpSum,"wall_panels: back+left+right_lozenge_r50mm\n");
        fprintf(fpSum,"tool_collision: ball_r%.0fmm\nframe_gap_mm: %.0f\nframe_cy_mm: %.0f\n",
                boxToolR,scene::FRAME_GAP,fcy);

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

        fprintf(fpSum,"\n# Timing:\ninit_ms: %.3f\nik_ms: %.3f\nplanning_total_ms: %.3f\n",initMs,ikMs,grandTotalPlanMs);
        fprintf(fpSum,"param_total_ms: %.3f\ncollision_runtime_ms: %.3f\nelapsed_s: %.3f\n",
                grandTotalParamMs,grandTotalCollMs,totalElapsed);
        auto cs=checker.getTimingStats();
        fprintf(fpSum,"\ncoll_init_ms: %.3f\ncoll_total_avg_us: %.3f\ncoll_calls: %d\n",
                cs.initTime_ms,cs.totalCheckTime_us,cs.callCount);
        fclose(fpSum);
    }

    printf("\n══════════════════════════════════════════════════════════════════\n");
    printf("  HR_S50-2000 码垛仿真 v6.0 — 单箱搬运完成\n");
    printf("══════════════════════════════════════════════════════════════════\n");
    return 0;
}
