/**
 * @file testS50PalletizingSO.cpp
 * @brief HR_S50-2000 码垛仿真 v5.0 — IK求解 + 正确布局 + 完整环境碰撞
 *
 * v5.0 核心升级:
 *   1. 数值逆运动学(IK): 基于HRC FK的阻尼最小二乘法, 自动求解放料关节角
 *   2. 正确码垛布局: 箱子紧靠框架内侧, 里→外/左→右/下→上 顺序
 *   3. 完整环境碰撞: 框架立柱 + 电箱(基座) + 传送带支架 + 已放箱子
 *   4. IK验证取料位: 传送带拾取位也由IK求解
 *   5. STL精确可视化 + 碰撞体快速计算架构
 *
 * @date 2026-02-24
 * @copyright Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.
 */

#include "PalletizingPlanner/RobotModel.hpp"
#include "PalletizingPlanner/CollisionCheckerSO.hpp"
#include "PalletizingPlanner/PathPlannerSO.hpp"
#include "PalletizingPlanner/TimeParameterization.hpp"
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
// 数值逆运动学 (Damped Least-Squares)
// ============================================================================
struct IKResult {
    JointConfig config;
    double posError_mm = 1e10;
    int iterations = 0;
    bool converged = false;
};

IKResult numericalIK(CollisionCheckerSO& checker,
    const Eigen::Vector3d& target_mm, const JointConfig& seed,
    double tol_mm = 2.0, int maxIter = 500, double lambda = 5.0)
{
    IKResult result;
    JointConfig q = seed;
    const double eps = 1e-4;
    const double jMin[6] = {-2*M_PI, -190*M_PI/180, -165*M_PI/180, -2*M_PI, -2*M_PI, -2*M_PI};
    const double jMax[6] = { 2*M_PI,   10*M_PI/180,  165*M_PI/180,  2*M_PI,  2*M_PI,  2*M_PI};

    for (int iter = 0; iter < maxIter; iter++) {
        result.iterations = iter + 1;
        SO_COORD_REF tcp;
        if (!checker.forwardKinematics(q, tcp)) { result.posError_mm=1e10; return result; }
        Eigen::Vector3d pos(tcp.X*1000, tcp.Y*1000, tcp.Z*1000);
        Eigen::Vector3d err = target_mm - pos;
        result.posError_mm = err.norm();
        if (result.posError_mm < tol_mm) { result.config = q; result.converged = true; return result; }

        Eigen::Matrix<double,3,6> J;
        for (int j = 0; j < 6; j++) {
            JointConfig qp = q; qp.q[j] += eps;
            SO_COORD_REF tp;
            if (!checker.forwardKinematics(qp, tp)) {
                // FK失败, 使用零列避免NaN注入
                J.col(j).setZero();
                continue;
            }
            J.col(j) = (Eigen::Vector3d(tp.X*1000,tp.Y*1000,tp.Z*1000) - pos) / eps;
        }
        Eigen::Matrix3d JJT = J * J.transpose() + lambda*lambda*Eigen::Matrix3d::Identity();
        Eigen::Matrix<double,6,1> dq = J.transpose() * JJT.ldlt().solve(err);

        double maxStep = 0.08;
        double dqN = dq.norm();
        if (dqN > maxStep) dq *= maxStep / dqN;
        for (int j = 0; j < 6; j++) {
            q.q[j] = std::clamp(q.q[j] + dq(j), jMin[j], jMax[j]);
        }
    }
    result.config = q;
    return result;
}

IKResult multiStartIK(CollisionCheckerSO& checker,
    const Eigen::Vector3d& target_mm, const std::vector<JointConfig>& seeds,
    double tol_mm = 2.0)
{
    IKResult best; best.posError_mm = 1e10;
    for (const auto& seed : seeds) {
        auto r = numericalIK(checker, target_mm, seed, tol_mm);
        if (r.converged && r.posError_mm < best.posError_mm) best = r;
        if (best.converged && best.posError_mm < tol_mm*0.5) break;
    }
    if (!best.converged) {
        for (const auto& seed : seeds) {
            auto r = numericalIK(checker, target_mm, seed, tol_mm, 1000, 2.0);
            if (r.posError_mm < best.posError_mm) best = r;
            if (r.converged) break;
        }
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
    r.collCheckTime_ms = std::chrono::duration<double,std::milli>(std::chrono::high_resolution_clock::now()-t2).count();
    r.totalSegmentTime_ms = std::chrono::duration<double,std::milli>(std::chrono::high_resolution_clock::now()-tS).count();
    r.success = true; return r;
}

// ============================================================================
// 主程序
// ============================================================================
int main() {
    printf("╔═══════════════════════════════════════════════════════════════════╗\n");
    printf("║   HR_S50-2000 码垛仿真 v5.0 — IK求解 + 正确布局 + 完整环境碰撞   ║\n");
    printf("║   数值IK + 里→外/左→右/下→上 + 电箱/传送带碰撞体               ║\n");
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
    planConfig.maxIterations = 5000; planConfig.maxPlanningTime = 2.0;
    planConfig.stepSize = 0.15; planConfig.goalBias = 0.2;
    planConfig.shortcutIterations = 60;
    planConfig.splineResolution = 35; planConfig.collisionResolution = 0.03;
    // 自由TCP模式: 运动中TCP可变, 纯关节空间最优 (吸盘吸力足够)
    planConfig.freeTcpDuringTransit = true;
    PathPlannerSO planner(robot, checker, planConfig);
    printf("  路径规划: Free-TCP Informed RRT* (纯关节空间代价)\n");

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
            double r = std::sqrt(tc.X*tc.X+tc.Y*tc.Y)*1000;
            printf("  [%4.0f,%4.0f,%4.0f,%4.0f,%4.0f,%4.0f]  %8.1f %8.1f %8.1f %8.0f\n",
                   p[0],p[1],p[2],p[3],p[4],p[5], tc.X*1000,tc.Y*1000,tc.Z*1000, r);
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

    printf("  === 框架立柱 (envId 30-33) Y=[%.0f,%.0f] ===\n", fNY, fFY);
    int frmIds[] = {30,31,32,33};
    double frmX[] = {-fHW, fHW, fHW, -fHW};
    double frmY[] = {fNY, fNY, fFY, fFY};
    for (int i=0;i<4;i++) {
        bool ok = checker.addEnvObstacleCapsule(frmIds[i],
            Eigen::Vector3d(frmX[i],frmY[i],fZB), Eigen::Vector3d(frmX[i],frmY[i],fZT), fR);
        printf("    envId=%d (%.0f,%.0f): %s\n", frmIds[i], frmX[i], frmY[i], ok?"✅":"❌");
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
    printf("\n  环境障碍: 电箱(4) + 传送带(3) + 框架(4) + 已放箱子(动态)\n\n");

    // ---- 码垛布局 + IK求解 ----
    printf("━━━━━━ 阶段3: 码垛布局 + IK求解 ━━━━━━━━━━━━━━━━━━━━━━━━━\n\n");

    double palSurf = scene::palletSurfBase();
    // 箱子紧靠框架内侧: 物理管径 + 5mm间隙 + 半箱宽
    double boxMargin = scene::FRM_TUBE_R + scene::BOX_WY/2 + 5;
    double boxBackY  = fFY - boxMargin;
    double boxFrontY = boxBackY - scene::BOX_WY - scene::BOX_GAP;
    double boxLeftX  = -(scene::BOX_LX/2 + scene::BOX_GAP/2);
    double boxRightX =  (scene::BOX_LX/2 + scene::BOX_GAP/2);

    printf("  框架CY=%.0f  托盘面Z(base)=%.0f\n", fcy, palSurf);
    printf("  箱子Y: [%.0f, %.0f]  X: [%.0f, %.0f]\n\n", boxFrontY, boxBackY, boxLeftX, boxRightX);

    // 码垛顺序: 里→外(BK→FR), 左→右(L→R), 下→上(L1→L3)
    // 每个XY位置先堆满3层再移到下一个位置
    std::vector<BoxTarget> boxTargets;
    const char* lN[] = {"L1","L2","L3"};
    const char* cN[] = {"BK","FR"};
    const char* rN[] = {"L","R"};

    for (int col=0; col<2; col++) {          // 里→外: BK(0) → FR(1)
        double y = (col==0) ? boxBackY : boxFrontY;
        for (int row=0; row<2; row++) {      // 左→右: L(0) → R(1)
            double x = (row==0) ? boxLeftX : boxRightX;
            for (int layer=0; layer<3; layer++) {  // 下→上: L1(0) → L3(2)
                double tcpZ = palSurf + (layer+1)*scene::BOX_HZ;
                char lb[32]; snprintf(lb,32,"%s-%s-%s",lN[layer],cN[col],rN[row]);
                BoxTarget bt; bt.label = lb;
                bt.pos_mm = Eigen::Vector3d(x, y, tcpZ);
                bt.approach_mm = Eigen::Vector3d(x, y, tcpZ + 150);
                boxTargets.push_back(bt);
            }
        }
    }
    int numPositions = (int)boxTargets.size();

    printf("  %-10s  %8s %8s %8s\n","位置","X_mm","Y_mm","Z_mm");
    for (auto& b : boxTargets)
        printf("  %-10s  %8.1f %8.1f %8.1f\n", b.label.c_str(), b.pos_mm.x(), b.pos_mm.y(), b.pos_mm.z());

    printf("\n  === IK求解 ===\n");
    std::vector<JointConfig> placeSeeds;
    double sA[][6] = {
        {0,-65,25,0,40,0},{5,-60,30,0,30,5},{-5,-60,30,0,30,-5},
        {0,-70,20,0,50,0},{0,-55,35,0,20,0},{10,-65,20,0,45,10},
        {-10,-65,20,0,45,-10},{0,-75,15,0,60,0},{0,-60,25,10,35,0},
        {15,-60,25,0,35,15},{-15,-60,25,0,35,-15},{0,-68,22,0,46,0},
    };
    for (auto& s : sA) placeSeeds.push_back(JointConfig::fromDegrees({s[0],s[1],s[2],s[3],s[4],s[5]}));

    struct PlaceConfig { JointConfig approach, place; double aErr, pErr; };
    std::vector<PlaceConfig> placeConfigs;

    auto tIK = std::chrono::high_resolution_clock::now();
    int ikFail = 0;
    for (int i=0; i<numPositions; i++) {
        auto& bt = boxTargets[i];
        auto ikP = multiStartIK(checker, bt.pos_mm, placeSeeds, 3.0);
        std::vector<JointConfig> aSeeds = placeSeeds;
        if (ikP.converged) aSeeds.insert(aSeeds.begin(), ikP.config);
        auto ikA = multiStartIK(checker, bt.approach_mm, aSeeds, 3.0);
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

    // FK验证
    printf("\n  === FK验证 ===\n");
    for (int i=0;i<numPositions;i++) {
        SO_COORD_REF tc; checker.forwardKinematics(placeConfigs[i].place, tc);
        auto& t=boxTargets[i].pos_mm;
        double e=std::sqrt(std::pow(tc.X*1000-t.x(),2)+std::pow(tc.Y*1000-t.y(),2)+std::pow(tc.Z*1000-t.z(),2));
        printf("    %-10s target=(%.0f,%.0f,%.0f) actual=(%.0f,%.0f,%.0f) err=%.1f%s\n",
               boxTargets[i].label.c_str(), t.x(),t.y(),t.z(), tc.X*1000,tc.Y*1000,tc.Z*1000,
               e, e<5?"":"  ⚠️");
    }

    // 取料位IK
    printf("\n  === 取料位IK ===\n");
    double pkX=scene::convCX(), pkY=scene::CONV_OFF_Y+500, pkZ=scene::convSurfBase()+scene::BOX_HZ;
    Eigen::Vector3d pickTgt(pkX,pkY,pkZ), pickApprTgt(pkX,pkY,pkZ+200);
    std::vector<JointConfig> pkSeeds;
    double pS[][6] = {{-90,-50,40,0,10,-90},{-80,-50,40,0,10,-80},{-100,-50,40,0,10,-100},
                      {-90,-55,35,0,20,-90},{-90,-45,45,0,0,-90},{-85,-60,30,0,30,-85}};
    for (auto& s:pS) pkSeeds.push_back(JointConfig::fromDegrees({s[0],s[1],s[2],s[3],s[4],s[5]}));

    auto ikPk = multiStartIK(checker, pickTgt, pkSeeds, 3.0);
    auto pkApSeeds = pkSeeds;
    if (ikPk.converged) pkApSeeds.insert(pkApSeeds.begin(), ikPk.config);
    auto ikPkA = multiStartIK(checker, pickApprTgt, pkApSeeds, 3.0);

    auto PICK_POS = ikPk.config;
    auto PICK_APPROACH = ikPkA.config;
    printf("    取料: %s(%.1fmm)  接近: %s(%.1fmm)\n",
           ikPk.converged?"✅":"❌",ikPk.posError_mm, ikPkA.converged?"✅":"❌",ikPkA.posError_mm);

    auto HOME = JointConfig::fromDegrees({0,-90,0,0,90,0});
    auto SAFE_TRANSIT = JointConfig::fromDegrees({0,-70,40,0,30,0});

    // ---- 码垛执行 ----
    printf("\n━━━━━━ 阶段4: 码垛仿真 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n");

    FILE* fpCsv = fopen("data/so_palletizing_profile.csv","w");
    if(fpCsv) fprintf(fpCsv,"task,segment,step,time_s,q1,q2,q3,q4,q5,q6,"
                      "selfDist_mm,selfCollision,envCollision,tcpX_mm,tcpY_mm,tcpZ_mm\n");
    FILE* fpTraj = fopen("data/so_palletizing_trajectory.txt","w");
    if(fpTraj) { fprintf(fpTraj,"# HR_S50-2000 码垛v5.0\n");
                 fprintf(fpTraj,"# task seg time q1..q6 v1..v6 dist tcpX tcpY tcpZ\n"); }

    std::vector<TaskResult> taskResults;
    double totalMotionTime=0; int totalCollisions=0, totalEnvCollisions=0;
    double globalMinDist_mm=1e10; int totalSegments=0;
    double grandTotalPlanMs=0, grandTotalParamMs=0, grandTotalCollMs=0;

    printf("  ── HOME→安全 ──\n");
    { auto s=executeP2P("HOME→安全",HOME,SAFE_TRANSIT,robot,checker,parameterizer,fpCsv,0,0,fpTraj);
      printf("    %s %.3fs dist:%.0fmm\n",s.name.c_str(),s.totalTime_s,s.minSelfDist_mm);
      totalMotionTime+=s.totalTime_s; if(s.minSelfDist_mm<globalMinDist_mm)globalMinDist_mm=s.minSelfDist_mm;
      grandTotalParamMs+=s.paramTime_ms; grandTotalCollMs+=s.collCheckTime_ms; totalSegments++; }

    auto currentPos = SAFE_TRANSIT;
    std::vector<int> order; for(int i=0;i<numPositions;i++) order.push_back(i);

    for (int t=0; t<(int)order.size(); t++) {
        int pi = order[t];
        auto& bt = boxTargets[pi];
        auto& pc = placeConfigs[pi];
        TaskResult tr; tr.taskIndex = t+1;
        char d[128]; snprintf(d,128,"T%d→%s",t+1,bt.label.c_str()); tr.description=d;

        printf("\n  ── 任务%d/%d: %s TCP=(%.0f,%.0f,%.0f) ──\n",
               t+1,numPositions,bt.label.c_str(),bt.pos_mm.x(),bt.pos_mm.y(),bt.pos_mm.z());

        struct M{const char*n;JointConfig s,t;bool rrt;};
        M motions[]={
            {"安全→取料接近",currentPos,PICK_APPROACH,true},
            {"取料接近→取料",PICK_APPROACH,PICK_POS,false},
            {"取料→取料抬升",PICK_POS,PICK_APPROACH,false},
            {"取料抬升→安全",PICK_APPROACH,SAFE_TRANSIT,true},
            {"安全→放料接近",SAFE_TRANSIT,pc.approach,true},
            {"放料接近→放料",pc.approach,pc.place,false},
            {"放料→放料抬升",pc.place,pc.approach,false},
        };

        for (int s=0;s<7;s++) {
            if (s==2) { checker.setToolBall(6,Eigen::Vector3d(0,0,-125),boxToolR);
                        printf("    📦 工具球ON\n"); }

            SegmentResult seg;
            if (motions[s].rrt)
                seg=executeRRTStar(motions[s].n,motions[s].s,motions[s].t,robot,checker,planner,parameterizer,fpCsv,t+1,s,fpTraj);
            else
                seg=executeP2P(motions[s].n,motions[s].s,motions[s].t,robot,checker,parameterizer,fpCsv,t+1,s,fpTraj);

            printf("    %-20s %s %.3fs plan:%.1fms param:%.1fms dist:%.0fmm env:%d\n",
                   seg.name.c_str(),seg.success?"✅":"❌",seg.totalTime_s,
                   seg.planningTime_ms,seg.paramTime_ms,seg.minSelfDist_mm,seg.envCollisionCount);

            if (s==5) {
                checker.removeTool(6); printf("    📦 工具球OFF\n");
                SO_COORD_REF tc; checker.forwardKinematics(pc.place,tc);
                double tx=tc.X*1000, ty=tc.Y*1000, tz=tc.Z*1000;
                int eid=34+t;
                bool ok=checker.addEnvObstacleBall(eid,Eigen::Vector3d(tx,ty,tz-scene::BOX_HZ/2),boxEnvR);
                printf("    📦 放置→envId=%d (%.0f,%.0f,%.0f) r=%.0f: %s\n",eid,tx,ty,tz-scene::BOX_HZ/2,boxEnvR,ok?"✅":"❌");
            }

            tr.segments.push_back(seg);
            tr.totalTime_s+=seg.totalTime_s; tr.totalCollisions+=seg.collisionCount;
            tr.totalEnvCollisions+=seg.envCollisionCount;
            if(seg.minSelfDist_mm<tr.minSelfDist_mm)tr.minSelfDist_mm=seg.minSelfDist_mm;
            tr.totalPlanningMs+=seg.planningTime_ms+seg.optimizationTime_ms;
            tr.totalParamMs+=seg.paramTime_ms; tr.totalCollCheckMs+=seg.collCheckTime_ms;
            totalMotionTime+=seg.totalTime_s; totalCollisions+=seg.collisionCount;
            totalEnvCollisions+=seg.envCollisionCount;
            if(seg.minSelfDist_mm<globalMinDist_mm)globalMinDist_mm=seg.minSelfDist_mm;
            grandTotalPlanMs+=seg.planningTime_ms+seg.optimizationTime_ms;
            grandTotalParamMs+=seg.paramTime_ms; grandTotalCollMs+=seg.collCheckTime_ms;
            totalSegments++;
        }
        currentPos = pc.approach;
        taskResults.push_back(tr);
    }

    printf("\n  ── 返回HOME ──\n");
    { auto s=executeRRTStar("返回HOME",currentPos,HOME,robot,checker,planner,parameterizer,fpCsv,numPositions+1,0,fpTraj);
      printf("    %s %.3fs plan:%.1fms dist:%.0fmm\n",s.name.c_str(),s.totalTime_s,s.planningTime_ms,s.minSelfDist_mm);
      totalMotionTime+=s.totalTime_s; if(s.minSelfDist_mm<globalMinDist_mm)globalMinDist_mm=s.minSelfDist_mm;
      grandTotalPlanMs+=s.planningTime_ms+s.optimizationTime_ms;
      grandTotalParamMs+=s.paramTime_ms; grandTotalCollMs+=s.collCheckTime_ms; totalSegments++; }

    if(fpCsv) fclose(fpCsv);
    if(fpTraj) fclose(fpTraj);

    // ---- 统计 ----
    double totalElapsed = std::chrono::duration<double>(std::chrono::high_resolution_clock::now()-t_global).count();

    printf("\n━━━━━━ 阶段5: 统计 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n");
    printf("  码垛位: %d  运动段: %d  总运动: %.3fs\n", numPositions, totalSegments, totalMotionTime);
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
        fprintf(fpSum,"# HR_S50-2000 码垛v5.0\n\nversion: 5.0\n");
        fprintf(fpSum,"positions: %d\nsegments: %d\ntotal_motion_s: %.4f\n",numPositions,totalSegments,totalMotionTime);
        fprintf(fpSum,"self_collisions: %d\nenv_collisions: %d\nmin_dist_mm: %.2f\n",
                totalCollisions,totalEnvCollisions,globalMinDist_mm);
        fprintf(fpSum,"order: FIFO_in2out_left2right_bottom2top_columnwise\n");
        fprintf(fpSum,"env_obstacles: 4_cabinet+3_conveyor+4_frame+%d_placed\n",(int)taskResults.size());
        fprintf(fpSum,"tool_collision: ball_r%.0fmm\nframe_gap_mm: %.0f\nframe_cy_mm: %.0f\n",
                boxToolR,scene::FRAME_GAP,fcy);

        fprintf(fpSum,"\n# Box TCP (base frame, mm):\n");
        for (int i=0;i<numPositions;i++) {
            SO_COORD_REF tc; checker.forwardKinematics(placeConfigs[i].place,tc);
            fprintf(fpSum,"box_%d_tcp_mm: %.1f %.1f %.1f\n",i,tc.X*1000,tc.Y*1000,tc.Z*1000);
        }
        fprintf(fpSum,"\n# IK place (deg):\n");
        for (int i=0;i<numPositions;i++) {
            auto q=placeConfigs[i].place.toDegrees();
            fprintf(fpSum,"ik_place_%d: %.2f %.2f %.2f %.2f %.2f %.2f\n",i,q[0],q[1],q[2],q[3],q[4],q[5]);
        }
        fprintf(fpSum,"\n# IK approach (deg):\n");
        for (int i=0;i<numPositions;i++) {
            auto q=placeConfigs[i].approach.toDegrees();
            fprintf(fpSum,"ik_appr_%d: %.2f %.2f %.2f %.2f %.2f %.2f\n",i,q[0],q[1],q[2],q[3],q[4],q[5]);
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
    printf("  HR_S50-2000 码垛仿真 v5.0 完成\n");
    printf("══════════════════════════════════════════════════════════════════\n");
    return 0;
}
