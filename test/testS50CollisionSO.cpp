/**
 * @file testS50CollisionSO.cpp
 * @brief HR_S50-2000 碰撞仿真 v3.0 — 独立.so碰撞库 + S曲线 + 分层计时
 *
 * 核心升级:
 *   1. 碰撞检测使用独立开发的 libHRCInterface.so (dlopen动态加载)
 *   2. 支持环境障碍物碰撞检测 (示例: 地面平面 + 箱体)
 *   3. 分层时间分析 (初始化 / 轨迹生成 / 碰撞检测 / FK / 总计)
 *   4. TCP位姿由.so内置FK计算 (更高精度)
 *
 * 输出:
 *   data/so_collision_trajectory.txt  — 完整轨迹
 *   data/so_collision_profile.csv     — 碰撞距离曲线
 *   data/so_collision_summary.txt     — 统计摘要 + 分层计时
 *   data/so_collision_poses.txt       — 关键姿态快照
 *
 * @date 2026-02-23
 * @copyright Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.
 */

#include "PalletizingPlanner/RobotModel.hpp"
#include "PalletizingPlanner/CollisionCheckerSO.hpp"
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

using namespace palletizing;

// ============================================================================
// 数据结构
// ============================================================================
struct CollisionSample {
    double time;
    double position_deg[6];
    double velocity_dps[6];
    double acceleration_dpss[6];
    double selfMinDist_mm;     // mm (注意单位变化: 旧版是m)
    bool   selfCollision;
    int    pairA, pairB;
    double tcpPos_mm[3];       // 由.so FK直接输出
    double tcpOrient_deg[3];   // A,B,C (deg)
};

struct ScenarioResult {
    std::string name;
    double startDeg[6];
    double targetDeg[6];
    std::vector<CollisionSample> samples;
    double totalTime_s;
    int    totalPoints;
    double minSelfDist_mm;
    int    collisionCount;
    double peakVelocity_dps;
    double peakAcceleration_dpss;
    bool   success;
    
    // 分层计时 (ms)
    double paramTime_ms;
    double collisionCheckTime_ms;
    double fkTime_ms;
    double perSampleAvgUs;
};

// ============================================================================
// 辅助函数
// ============================================================================
Path makeP2PPath(const JointConfig& start, const JointConfig& end) {
    Path p;
    Waypoint wp0(start); wp0.pathParam = 0.0;
    Waypoint wp1(end);   wp1.pathParam = 1.0;
    p.waypoints.push_back(wp0);
    p.waypoints.push_back(wp1);
    return p;
}

ScenarioResult runScenario(
    const char* name,
    const double startDeg[6],
    const double targetDeg[6],
    RobotModel& robot,
    CollisionCheckerSO& checker,
    double velScaling = 1.0,
    int collisionInterval = 5)
{
    ScenarioResult result;
    result.name = name;
    memcpy(result.startDeg, startDeg, sizeof(double)*6);
    memcpy(result.targetDeg, targetDeg, sizeof(double)*6);
    result.minSelfDist_mm = 1e10;
    result.collisionCount = 0;
    result.peakVelocity_dps = 0;
    result.peakAcceleration_dpss = 0;
    result.success = false;

    auto startCfg = JointConfig::fromDegrees({startDeg[0],startDeg[1],startDeg[2],
                                               startDeg[3],startDeg[4],startDeg[5]});
    auto targetCfg = JointConfig::fromDegrees({targetDeg[0],targetDeg[1],targetDeg[2],
                                                targetDeg[3],targetDeg[4],targetDeg[5]});

    Path path = makeP2PPath(startCfg, targetCfg);

    auto tpConfig = TimeParameterizationConfig::fromRobotParams(robot.getParams());
    tpConfig.profileType = VelocityProfileType::SCurve;
    tpConfig.velocityScaling = velScaling;
    tpConfig.samplePeriod = 0.004;

    TimeParameterizer parameterizer(tpConfig);

    // ── S曲线轨迹生成 ──
    auto t0 = std::chrono::high_resolution_clock::now();
    Trajectory traj = parameterizer.parameterize(path);
    auto t1 = std::chrono::high_resolution_clock::now();
    result.paramTime_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    if (traj.empty()) {
        result.totalTime_s = 0;
        result.totalPoints = 0;
        return result;
    }

    result.totalTime_s = traj.totalTime;
    result.totalPoints = (int)traj.size();

    // 重置碰撞检测器计时
    checker.resetTimingStats();

    // ── 沿轨迹碰撞检测 ──
    auto t2 = std::chrono::high_resolution_clock::now();
    
    for (size_t i = 0; i < traj.size(); i++) {
        const auto& pt = traj.points[i];

        for (int j = 0; j < 6; j++) {
            double v_dps = std::fabs(pt.velocity[j]) * 180.0 / M_PI;
            double a_dpss = std::fabs(pt.acceleration[j]) * 180.0 / M_PI;
            if (v_dps > result.peakVelocity_dps) result.peakVelocity_dps = v_dps;
            if (a_dpss > result.peakAcceleration_dpss) result.peakAcceleration_dpss = a_dpss;
        }

        if ((int)i % collisionInterval == 0 || i == traj.size() - 1) {
            CollisionSample cs;
            cs.time = pt.time;
            auto q_deg = pt.config.toDegrees();
            for (int j = 0; j < 6; j++) {
                cs.position_deg[j] = q_deg[j];
                cs.velocity_dps[j] = pt.velocity[j] * 180.0 / M_PI;
                cs.acceleration_dpss[j] = pt.acceleration[j] * 180.0 / M_PI;
            }

            // 使用新.so碰撞检测 (含TCP位姿)
            auto report = checker.getCollisionReport(pt.config, true);
            cs.selfMinDist_mm = report.selfMinDist_mm;
            cs.selfCollision = report.selfCollision;
            cs.pairA = report.selfPairA;
            cs.pairB = report.selfPairB;

            if (report.selfCollision) result.collisionCount++;
            if (report.selfMinDist_mm < result.minSelfDist_mm)
                result.minSelfDist_mm = report.selfMinDist_mm;

            // TCP位姿直接来自.so FK
            if (report.hasTcpPose) {
                cs.tcpPos_mm[0] = report.tcpPose.X;
                cs.tcpPos_mm[1] = report.tcpPose.Y;
                cs.tcpPos_mm[2] = report.tcpPose.Z;
                cs.tcpOrient_deg[0] = report.tcpPose.A;
                cs.tcpOrient_deg[1] = report.tcpPose.B;
                cs.tcpOrient_deg[2] = report.tcpPose.C;
            } else {
                // 备用: RobotModel FK
                auto pose = robot.forwardKinematics(pt.config);
                cs.tcpPos_mm[0] = pose.position.x() * 1000;
                cs.tcpPos_mm[1] = pose.position.y() * 1000;
                cs.tcpPos_mm[2] = pose.position.z() * 1000;
                cs.tcpOrient_deg[0] = cs.tcpOrient_deg[1] = cs.tcpOrient_deg[2] = 0;
            }

            result.samples.push_back(cs);
        }
    }
    
    auto t3 = std::chrono::high_resolution_clock::now();
    result.collisionCheckTime_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();

    // 从碰撞检测器获取FK耗时
    auto collStats = checker.getTimingStats();
    result.fkTime_ms = collStats.fkTime_us / 1000.0 * collStats.callCount;
    result.perSampleAvgUs = collStats.totalCheckTime_us;

    result.success = true;
    return result;
}

void printScenarioResult(const ScenarioResult& r, int idx) {
    printf("  ┌─────────────────────────────────────────────────────────────────┐\n");
    printf("  │ 场景 %d: %-54s │\n", idx, r.name.c_str());
    printf("  ├─────────────────────────────────────────────────────────────────┤\n");
    printf("  │ 状态: %s  |  轨迹点: %-6d  |  时长: %.3f s                  │\n",
           r.success ? "✅ 完成" : "❌ 失败", r.totalPoints, r.totalTime_s);
    printf("  │ S曲线: %.2f ms  |  碰撞检测: %.2f ms  |  单次: %.1f μs      │\n",
           r.paramTime_ms, r.collisionCheckTime_ms, r.perSampleAvgUs);
    printf("  │ 峰值速度: %.1f °/s  |  峰值加速度: %.1f °/s²               │\n",
           r.peakVelocity_dps, r.peakAcceleration_dpss);
    if (r.collisionCount > 0) {
        printf("  │ ⚠️  碰撞: %d 次  |  最小距离: %.1f mm                       │\n",
               r.collisionCount, r.minSelfDist_mm);
    } else {
        printf("  │ ✅ 安全: 无碰撞  |  最小距离: %.1f mm                       │\n",
               r.minSelfDist_mm);
    }
    printf("  └─────────────────────────────────────────────────────────────────┘\n\n");
}

// ============================================================================
// 主程序
// ============================================================================
int main() {
    printf("╔═══════════════════════════════════════════════════════════════════╗\n");
    printf("║   HR_S50-2000 碰撞仿真 v3.0 — 独立.so碰撞库 + 分层计时          ║\n");
    printf("║   libHRCInterface.so + 7段S曲线 + TCP位姿 + 环境碰撞             ║\n");
    printf("╚═══════════════════════════════════════════════════════════════════╝\n\n");

    auto t_global = std::chrono::high_resolution_clock::now();

    // ---- 系统初始化 ----
    printf("━━━━━━ 阶段1: 系统初始化 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n");

    RobotModel robot;
    printf("  [1] 机器人: HR_S50-2000 (6-DOF, 标准DH)\n");
    printf("      DH: d=[296.5, 336.2, 239.0, 158.5, 158.5, 134.5] mm\n");
    printf("          a=[0, 900.0, 941.5, 0, 0, 0] mm\n\n");

    CollisionCheckerSO checker(robot);
    bool collisionOk = checker.initialize();
    if (!collisionOk) {
        fprintf(stderr, "❌ 碰撞检测初始化失败!\n");
        return 1;
    }
    printf("  [2] 碰撞检测: ✅ libHRCInterface.so 初始化成功\n");
    printf("      碰撞模型: 胶囊体+球体 (新版几何参数)\n");
    printf("      安全余量: %.0f mm\n\n", checker.getSafetyMarginMm());

    // 添加环境障碍物示例
    printf("  [3] 环境配置:\n");
    // 地面平面 (z=0)
    Eigen::Vector3d groundCenter(0, 0, -100); // 地下100mm处
    checker.addEnvObstacleBall(1, groundCenter, 100);  // 用球体模拟地面
    printf("      地面障碍物: 球体 @ (0,0,-100) R=100mm\n");

    printf("\n  [4] 轨迹生成器: 7段S曲线时间参数化\n");
    printf("      插补周期: 4ms (250 Hz)\n\n");

    // ---- 7个测试场景 ----
    printf("━━━━━━ 阶段2: 碰撞仿真 (7个场景) ━━━━━━━━━━━━━━━━━━━━━━━━\n\n");

    struct Scenario {
        const char* name;
        double start[6];
        double target[6];
        double velScale;
    };

    Scenario scenarios[] = {
        {"HOME → 常规工作姿态",
         {0, -90, 0, 0, 90, 0},
         {45, -60, 30, 0, 60, 0}, 1.0},

        {"常规工作 → 大角度运动",
         {45, -60, 30, 0, 60, 0},
         {-120, -30, 120, 90, -45, 180}, 1.0},

        {"大角度 → 折叠近碰撞区",
         {-120, -30, 120, 90, -45, 180},
         {0, -50, 130, 0, -60, 0}, 0.8},

        {"碰撞区 → 安全区恢复",
         {0, -50, 130, 0, -60, 0},
         {30, -120, 60, 45, 30, -90}, 1.0},

        {"全轴大范围运动",
         {0, -90, 0, 0, 90, 0},
         {-150, -10, -120, 180, -90, 270}, 1.0},

        {"极限距离伸展",
         {-150, -10, -120, 180, -90, 270},
         {0, -170, 150, 0, -160, 0}, 0.6},

        {"返回HOME",
         {0, -170, 150, 0, -160, 0},
         {0, -90, 0, 0, 90, 0}, 1.0},
    };
    int numScenarios = sizeof(scenarios) / sizeof(scenarios[0]);

    std::vector<ScenarioResult> results;

    // 打开输出文件
    FILE* fpCsv = fopen("data/so_collision_profile.csv", "w");
    if (fpCsv) {
        fprintf(fpCsv, "scenario,time_s,q1_deg,q2_deg,q3_deg,q4_deg,q5_deg,q6_deg,"
                "v_peak_dps,selfMinDist_mm,selfCollision,"
                "tcpX_mm,tcpY_mm,tcpZ_mm,tcpA_deg,tcpB_deg,tcpC_deg\n");
    }

    FILE* fpTraj = fopen("data/so_collision_trajectory.txt", "w");
    if (fpTraj) {
        fprintf(fpTraj, "# HR_S50-2000 碰撞仿真v3.0 (libHRCInterface.so)\n");
        fprintf(fpTraj, "# scenario time_s q1 q2 q3 q4 q5 q6 v1 v2 v3 v4 v5 v6 "
                "a1 a2 a3 a4 a5 a6 selfDist_mm tcpX tcpY tcpZ tcpA tcpB tcpC\n");
    }

    FILE* fpPoses = fopen("data/so_collision_poses.txt", "w");
    if (fpPoses) {
        fprintf(fpPoses, "# 关键姿态快照 (用于STL可视化)\n");
        fprintf(fpPoses, "# scenario time_s q1_deg q2_deg q3_deg q4_deg q5_deg q6_deg "
                "selfDist_mm tcpX tcpY tcpZ label\n");
    }

    double totalTime = 0;
    int totalCollisions = 0;
    double globalMinDist_mm = 1e10;
    double totalParamTime = 0, totalCollisionTime = 0;

    for (int s = 0; s < numScenarios; s++) {
        auto& sc = scenarios[s];
        auto res = runScenario(sc.name, sc.start, sc.target,
                               robot, checker, sc.velScale, 5);
        printScenarioResult(res, s + 1);

        // 写CSV
        for (auto& cs : res.samples) {
            if (fpCsv) {
                double v_peak = 0;
                for (int j = 0; j < 6; j++)
                    v_peak = std::max(v_peak, std::fabs(cs.velocity_dps[j]));
                fprintf(fpCsv, "%d,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                        "%.2f,%.2f,%d,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f\n",
                        s+1, cs.time,
                        cs.position_deg[0],cs.position_deg[1],cs.position_deg[2],
                        cs.position_deg[3],cs.position_deg[4],cs.position_deg[5],
                        v_peak, cs.selfMinDist_mm, cs.selfCollision ? 1 : 0,
                        cs.tcpPos_mm[0], cs.tcpPos_mm[1], cs.tcpPos_mm[2],
                        cs.tcpOrient_deg[0], cs.tcpOrient_deg[1], cs.tcpOrient_deg[2]);
            }
            if (fpTraj) {
                fprintf(fpTraj, "%d %.4f", s+1, cs.time);
                for (int j = 0; j < 6; j++) fprintf(fpTraj, " %.4f", cs.position_deg[j]);
                for (int j = 0; j < 6; j++) fprintf(fpTraj, " %.3f", cs.velocity_dps[j]);
                for (int j = 0; j < 6; j++) fprintf(fpTraj, " %.3f", cs.acceleration_dpss[j]);
                fprintf(fpTraj, " %.2f %.1f %.1f %.1f %.2f %.2f %.2f\n",
                        cs.selfMinDist_mm,
                        cs.tcpPos_mm[0], cs.tcpPos_mm[1], cs.tcpPos_mm[2],
                        cs.tcpOrient_deg[0], cs.tcpOrient_deg[1], cs.tcpOrient_deg[2]);
            }
        }

        // 关键姿态快照
        if (fpPoses && !res.samples.empty()) {
            auto& first = res.samples.front();
            fprintf(fpPoses, "%d %.4f %.3f %.3f %.3f %.3f %.3f %.3f %.2f %.1f %.1f %.1f start\n",
                    s+1, first.time, first.position_deg[0],first.position_deg[1],
                    first.position_deg[2],first.position_deg[3],
                    first.position_deg[4],first.position_deg[5],
                    first.selfMinDist_mm, first.tcpPos_mm[0],first.tcpPos_mm[1],first.tcpPos_mm[2]);

            size_t minIdx = 0;
            for (size_t i = 1; i < res.samples.size(); i++)
                if (res.samples[i].selfMinDist_mm < res.samples[minIdx].selfMinDist_mm)
                    minIdx = i;
            auto& minPt = res.samples[minIdx];
            fprintf(fpPoses, "%d %.4f %.3f %.3f %.3f %.3f %.3f %.3f %.2f %.1f %.1f %.1f min_dist\n",
                    s+1, minPt.time, minPt.position_deg[0],minPt.position_deg[1],
                    minPt.position_deg[2],minPt.position_deg[3],
                    minPt.position_deg[4],minPt.position_deg[5],
                    minPt.selfMinDist_mm, minPt.tcpPos_mm[0],minPt.tcpPos_mm[1],minPt.tcpPos_mm[2]);

            auto& last = res.samples.back();
            fprintf(fpPoses, "%d %.4f %.3f %.3f %.3f %.3f %.3f %.3f %.2f %.1f %.1f %.1f end\n",
                    s+1, last.time, last.position_deg[0],last.position_deg[1],
                    last.position_deg[2],last.position_deg[3],
                    last.position_deg[4],last.position_deg[5],
                    last.selfMinDist_mm, last.tcpPos_mm[0],last.tcpPos_mm[1],last.tcpPos_mm[2]);
        }

        totalTime += res.totalTime_s;
        totalCollisions += res.collisionCount;
        if (res.minSelfDist_mm < globalMinDist_mm)
            globalMinDist_mm = res.minSelfDist_mm;
        totalParamTime += res.paramTime_ms;
        totalCollisionTime += res.collisionCheckTime_ms;

        results.push_back(res);
    }

    if (fpCsv)   fclose(fpCsv);
    if (fpTraj)  fclose(fpTraj);
    if (fpPoses) fclose(fpPoses);

    // ---- 碰撞检测器分层计时 ----
    auto collStats = checker.getTimingStats();
    
    // ---- 统计报告 ----
    printf("━━━━━━ 阶段3: 仿真统计分析 + 分层计时 ━━━━━━━━━━━━━━━━━━━━\n\n");

    int successCount = 0;
    double maxPeakVel = 0, maxPeakAcc = 0;
    for (auto& r : results) {
        if (r.success) successCount++;
        if (r.peakVelocity_dps > maxPeakVel) maxPeakVel = r.peakVelocity_dps;
        if (r.peakAcceleration_dpss > maxPeakAcc) maxPeakAcc = r.peakAcceleration_dpss;
    }

    auto t_end = std::chrono::high_resolution_clock::now();
    double totalElapsed = std::chrono::duration<double>(t_end - t_global).count();

    printf("  ╔════════════════════════════════════════════════════════════╗\n");
    printf("  ║       碰撞仿真 v3.0 统计报告 (libHRCInterface.so)          ║\n");
    printf("  ╠════════════════════════════════════════════════════════════╣\n");
    printf("  ║ 碰撞检测:  独立 .so (dlopen动态加载)                       ║\n");
    printf("  ║ 轨迹生成:  7段S曲线 (4ms, 250Hz)                          ║\n");
    printf("  ║ TCP位姿:   .so内置FK (forwardKinematics2)                 ║\n");
    printf("  ╠════════════════════════════════════════════════════════════╣\n");
    printf("  ║ 场景: %d  完成: %d/%d                                     ║\n",
           numScenarios, successCount, numScenarios);
    printf("  ║ 总运动: %.3f s  碰撞: %d  最小距离: %.1f mm               ║\n",
           totalTime, totalCollisions, globalMinDist_mm);
    printf("  ║ 峰值速度: %.1f °/s  峰值加速度: %.1f °/s²                 ║\n",
           maxPeakVel, maxPeakAcc);
    printf("  ╠════════════════════════════════════════════════════════════╣\n");
    
    // 分层计时
    printf("  ║ ──── 分层计时分析 ────                                     ║\n");
    printf("  ║ S曲线参数化总计: %8.2f ms (%.2f ms/场景)                  ║\n",
           totalParamTime, totalParamTime / numScenarios);
    printf("  ║ 碰撞检测总计:   %8.2f ms (%.2f ms/场景)                  ║\n",
           totalCollisionTime, totalCollisionTime / numScenarios);
    printf("  ╠════════════════════════════════════════════════════════════╣\n");
    
    // 碰撞检测器内部分层
    printf("%s", collStats.toString().c_str());
    
    printf("  ║ 总耗时(含IO):  %.2f s                                     ║\n", totalElapsed);
    printf("  ╚════════════════════════════════════════════════════════════╝\n\n");

    // 详细场景表
    printf("  详细场景对比:\n");
    printf("  序号 场景                          时间(s)  步数 最小距离mm S曲线ms 碰撞ms  单次μs 状态\n");
    printf("  ──── ────────────────────────────── ─────── ───── ───────── ─────── ─────── ────── ──────\n");
    for (int i = 0; i < (int)results.size(); i++) {
        auto& r = results[i];
        printf("  %-4d %-30s %6.3f %5d  %7.1f   %6.2f  %6.2f  %5.1f %s\n",
               i+1, r.name.c_str(), r.totalTime_s, r.totalPoints,
               r.minSelfDist_mm, r.paramTime_ms, r.collisionCheckTime_ms,
               r.perSampleAvgUs,
               r.success ? "✅完成" : "❌失败");
    }

    // 写统计摘要
    FILE* fpSum = fopen("data/so_collision_summary.txt", "w");
    if (fpSum) {
        fprintf(fpSum, "# HR_S50-2000 碰撞仿真 v3.0 (libHRCInterface.so)\n");
        fprintf(fpSum, "# 碰撞检测: 独立.so (dlopen)\n");
        fprintf(fpSum, "# TCP FK: .so内置 forwardKinematics2\n\n");
        fprintf(fpSum, "scenarios: %d\n", numScenarios);
        fprintf(fpSum, "success: %d\n", successCount);
        fprintf(fpSum, "total_time_s: %.4f\n", totalTime);
        fprintf(fpSum, "total_collisions: %d\n", totalCollisions);
        fprintf(fpSum, "global_min_dist_mm: %.2f\n", globalMinDist_mm);
        fprintf(fpSum, "max_peak_vel_dps: %.2f\n", maxPeakVel);
        fprintf(fpSum, "max_peak_acc_dpss: %.2f\n", maxPeakAcc);
        fprintf(fpSum, "\n# Timing (ms):\n");
        fprintf(fpSum, "scurve_total_ms: %.3f\n", totalParamTime);
        fprintf(fpSum, "collision_total_ms: %.3f\n", totalCollisionTime);
        fprintf(fpSum, "collision_init_ms: %.3f\n", collStats.initTime_ms);
        fprintf(fpSum, "collision_update_avg_us: %.3f\n", collStats.updateTime_us);
        fprintf(fpSum, "collision_self_avg_us: %.3f\n", collStats.selfCheckTime_us);
        fprintf(fpSum, "collision_fk_avg_us: %.3f\n", collStats.fkTime_us);
        fprintf(fpSum, "collision_total_avg_us: %.3f\n", collStats.totalCheckTime_us);
        fprintf(fpSum, "collision_calls: %d\n", collStats.callCount);
        fprintf(fpSum, "total_elapsed_s: %.3f\n", totalElapsed);
        
        fprintf(fpSum, "\n# Per-scenario:\n");
        for (int i = 0; i < (int)results.size(); i++) {
            auto& r = results[i];
            fprintf(fpSum, "%d \"%s\" time=%.4f points=%d minDist=%.2f "
                    "paramMs=%.3f collMs=%.3f singleUs=%.2f\n",
                    i+1, r.name.c_str(), r.totalTime_s, r.totalPoints,
                    r.minSelfDist_mm, r.paramTime_ms, r.collisionCheckTime_ms,
                    r.perSampleAvgUs);
        }
        fclose(fpSum);
    }

    printf("\n  输出文件:\n");
    printf("    data/so_collision_trajectory.txt  — 完整轨迹\n");
    printf("    data/so_collision_profile.csv     — 碰撞距离曲线\n");
    printf("    data/so_collision_summary.txt     — 统计摘要+分层计时\n");
    printf("    data/so_collision_poses.txt       — 关键姿态快照\n");

    printf("\n  总耗时: %.2f s\n", totalElapsed);

    printf("\n══════════════════════════════════════════════════════════════\n");
    printf("  HR_S50-2000 碰撞仿真 v3.0 完成 (libHRCInterface.so)\n");
    printf("══════════════════════════════════════════════════════════════\n");

    return 0;
}
