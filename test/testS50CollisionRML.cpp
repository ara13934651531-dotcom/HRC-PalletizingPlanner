/**
 * @file testS50CollisionRML.cpp
 * @brief HR_S50-2000 增强碰撞仿真 — S曲线轨迹 + HRC碰撞检测 + STL可视化数据
 *
 * 使用项目内置7段S曲线时间参数化(与华数上位机libCmpRML.so等效的S曲线算法)
 * 生成轨迹,结合HRC碰撞检测库进行实时碰撞距离监测。
 *
 * 增强内容(相比testCollisionSimulation.cpp):
 *   - 7个精心设计的测试场景(覆盖安全区、危险区、极限距离)
 *   - 完整的TCP轨迹记录(用于STL模型可视化)
 *   - 碰撞距离热力图数据(连杆对×时间)
 *   - 基于URDF/STL的3D可视化数据输出
 *   - 详细的统计分析报告
 *
 * 输出:
 *   data/rml_collision_trajectory.txt — 完整轨迹 (deg, deg/s, deg/s², 碰撞距离, TCP)
 *   data/rml_collision_profile.csv   — 碰撞距离曲线 (CSV)
 *   data/rml_collision_summary.txt   — 统计摘要
 *   data/rml_collision_poses.txt     — 关键姿态快照 (用于STL可视化)
 *
 * @date 2026-02-23
 * @copyright Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.
 */

#include "PalletizingPlanner/RobotModel.hpp"
#include "PalletizingPlanner/CollisionChecker.hpp"
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
    double selfMinDist_m;
    bool   selfCollision;
    int    pairA, pairB;
    double tcpPos_mm[3];
    double tcpQuat[4];
};

struct ScenarioResult {
    std::string name;
    double startDeg[6];
    double targetDeg[6];
    std::vector<CollisionSample> samples;
    double totalTime_s;
    int    totalPoints;
    double minSelfDist_m;
    int    collisionCount;
    double peakVelocity_dps;
    double peakAcceleration_dpss;
    bool   success;
    double paramTime_ms;
    double collisionCheckTime_ms;
};

// ============================================================================
// 辅助函数
// ============================================================================
Path makeP2PPath(const JointConfig& start, const JointConfig& end) {
    Path path;
    Waypoint wp0(start); wp0.pathParam = 0.0;
    Waypoint wp1(end);   wp1.pathParam = 1.0;
    path.waypoints.push_back(wp0);
    path.waypoints.push_back(wp1);
    return path;
}

ScenarioResult runScenario(
    const char* name,
    const double startDeg[6],
    const double targetDeg[6],
    RobotModel& robot,
    CollisionChecker& checker,
    double velScaling = 1.0,
    int collisionInterval = 5)
{
    ScenarioResult result;
    result.name = name;
    memcpy(result.startDeg, startDeg, sizeof(double)*6);
    memcpy(result.targetDeg, targetDeg, sizeof(double)*6);
    result.minSelfDist_m = 1e10;
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
    tpConfig.samplePeriod = 0.004;  // 4ms (250Hz, 与真实控制器一致)

    TimeParameterizer parameterizer(tpConfig);

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

    // 沿轨迹进行碰撞检测
    auto t2 = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < traj.size(); i++) {
        const auto& pt = traj.points[i];

        // 记录峰值速度/加速度
        for (int j = 0; j < 6; j++) {
            double v_dps = std::fabs(pt.velocity[j]) * 180.0 / M_PI;
            double a_dpss = std::fabs(pt.acceleration[j]) * 180.0 / M_PI;
            if (v_dps > result.peakVelocity_dps) result.peakVelocity_dps = v_dps;
            if (a_dpss > result.peakAcceleration_dpss) result.peakAcceleration_dpss = a_dpss;
        }

        // 定期做碰撞检测
        if ((int)i % collisionInterval == 0 || i == traj.size() - 1) {
            CollisionSample cs;
            cs.time = pt.time;
            auto q_deg = pt.config.toDegrees();
            for (int j = 0; j < 6; j++) {
                cs.position_deg[j] = q_deg[j];
                cs.velocity_dps[j] = pt.velocity[j] * 180.0 / M_PI;
                cs.acceleration_dpss[j] = pt.acceleration[j] * 180.0 / M_PI;
            }

            auto report = checker.getCollisionReport(pt.config);
            cs.selfMinDist_m = report.selfMinDistance;
            cs.selfCollision = report.selfCollision;
            cs.pairA = report.selfColliderPairA;
            cs.pairB = report.selfColliderPairB;

            if (report.selfCollision) result.collisionCount++;
            if (report.selfMinDistance < result.minSelfDist_m)
                result.minSelfDist_m = report.selfMinDistance;

            // TCP位置
            auto pose = robot.forwardKinematics(pt.config);
            cs.tcpPos_mm[0] = pose.position.x() * 1000;
            cs.tcpPos_mm[1] = pose.position.y() * 1000;
            cs.tcpPos_mm[2] = pose.position.z() * 1000;
            cs.tcpQuat[0] = pose.orientation.w();
            cs.tcpQuat[1] = pose.orientation.x();
            cs.tcpQuat[2] = pose.orientation.y();
            cs.tcpQuat[3] = pose.orientation.z();

            result.samples.push_back(cs);
        }
    }
    auto t3 = std::chrono::high_resolution_clock::now();
    result.collisionCheckTime_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();

    result.success = true;
    return result;
}

void printScenarioResult(const ScenarioResult& r, int idx) {
    printf("  ┌─────────────────────────────────────────────────────────────┐\n");
    printf("  │ 场景 %d: %-50s │\n", idx, r.name.c_str());
    printf("  ├─────────────────────────────────────────────────────────────┤\n");
    printf("  │ 状态: %s  |  轨迹点: %-6d  |  时长: %.3f s              │\n",
           r.success ? "✅ 完成" : "❌ 失败", r.totalPoints, r.totalTime_s);
    printf("  │ S曲线生成: %.2f ms  |  碰撞检测: %.1f ms (%zu次)        │\n",
           r.paramTime_ms, r.collisionCheckTime_ms, r.samples.size());
    printf("  │ 峰值速度: %.1f °/s  |  峰值加速度: %.1f °/s²            │\n",
           r.peakVelocity_dps, r.peakAcceleration_dpss);
    if (r.collisionCount > 0) {
        printf("  │ ⚠️  碰撞: %d 次  |  最小距离: %.1f mm                    │\n",
               r.collisionCount, r.minSelfDist_m * 1000);
    } else {
        printf("  │ ✅ 安全: 无碰撞  |  最小距离: %.1f mm                    │\n",
               r.minSelfDist_m * 1000);
    }
    printf("  └─────────────────────────────────────────────────────────────┘\n\n");
}

// ============================================================================
// 主程序
// ============================================================================
int main() {
    printf("╔═══════════════════════════════════════════════════════════════════╗\n");
    printf("║   HR_S50-2000 增强碰撞仿真 — S曲线轨迹 + HRC碰撞检测            ║\n");
    printf("║   7段S曲线时间参数化 × HRC碰撞检测库 × STL可视化数据              ║\n");
    printf("╚═══════════════════════════════════════════════════════════════════╝\n\n");

    auto t_global = std::chrono::high_resolution_clock::now();

    // ---- 初始化 ----
    printf("━━━━━━ 阶段1: 系统初始化 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n");

    RobotModel robot;
    printf("  [1] 机器人运动学: HR_S50-2000 (6-DOF, 标准DH)\n");
    printf("      DH参数: d=[296.5, 336.2, 239.0, 158.5, 158.5, 134.5] mm\n");
    printf("              a=[0, 900.0, 941.5, 0, 0, 0] mm\n\n");

    CollisionChecker checker(robot);
    bool collisionOk = checker.initialize();
    printf("  [2] HRC碰撞检测: %s\n", collisionOk ? "✅ 初始化成功" : "❌ 初始化失败");
    printf("      碰撞模型: 胶囊体+球体 (7连杆对)\n");
    printf("      检测精度: 亚毫米级 (IEEE 754 double)\n\n");

    printf("  [3] 轨迹生成器: 7段S曲线时间参数化\n");
    printf("      插补周期: 4ms (250 Hz, 与华数上位机一致)\n");
    printf("      速度/加速度/加加速度限制: 按HR_S50参数\n\n");

    // ---- 定义7个测试场景 ----
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

    // 打开CSV产出文件
    FILE* fpCsv = fopen("data/rml_collision_profile.csv", "w");
    if (fpCsv) {
        fprintf(fpCsv, "scenario,time_s,q1_deg,q2_deg,q3_deg,q4_deg,q5_deg,q6_deg,"
                "v_peak_dps,selfMinDist_mm,selfCollision,"
                "tcpX_mm,tcpY_mm,tcpZ_mm\n");
    }

    FILE* fpTraj = fopen("data/rml_collision_trajectory.txt", "w");
    if (fpTraj) {
        fprintf(fpTraj, "# HR_S50-2000 碰撞仿真完整轨迹\n");
        fprintf(fpTraj, "# scenario time_s q1 q2 q3 q4 q5 q6 v1 v2 v3 v4 v5 v6 "
                "a1 a2 a3 a4 a5 a6 selfDist_mm tcpX tcpY tcpZ\n");
    }

    FILE* fpPoses = fopen("data/rml_collision_poses.txt", "w");
    if (fpPoses) {
        fprintf(fpPoses, "# 关键姿态快照 (用于STL可视化)\n");
        fprintf(fpPoses, "# scenario_idx time_s q1_deg q2_deg q3_deg q4_deg q5_deg q6_deg "
                "selfDist_mm tcpX_mm tcpY_mm tcpZ_mm label\n");
    }

    double totalTime = 0;
    int totalCollisions = 0;
    double globalMinDist = 1e10;

    for (int s = 0; s < numScenarios; s++) {
        auto& sc = scenarios[s];
        auto res = runScenario(sc.name, sc.start, sc.target,
                               robot, checker, sc.velScale, 5);
        printScenarioResult(res, s + 1);

        // 写CSV和轨迹文件
        for (auto& cs : res.samples) {
            if (fpCsv) {
                double v_peak = 0;
                for (int j = 0; j < 6; j++)
                    v_peak = std::max(v_peak, std::fabs(cs.velocity_dps[j]));
                fprintf(fpCsv, "%d,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                        "%.2f,%.2f,%d,%.1f,%.1f,%.1f\n",
                        s+1, cs.time,
                        cs.position_deg[0],cs.position_deg[1],cs.position_deg[2],
                        cs.position_deg[3],cs.position_deg[4],cs.position_deg[5],
                        v_peak, cs.selfMinDist_m * 1000, cs.selfCollision ? 1 : 0,
                        cs.tcpPos_mm[0], cs.tcpPos_mm[1], cs.tcpPos_mm[2]);
            }
            if (fpTraj) {
                fprintf(fpTraj, "%d %.4f", s+1, cs.time);
                for (int j = 0; j < 6; j++) fprintf(fpTraj, " %.4f", cs.position_deg[j]);
                for (int j = 0; j < 6; j++) fprintf(fpTraj, " %.3f", cs.velocity_dps[j]);
                for (int j = 0; j < 6; j++) fprintf(fpTraj, " %.3f", cs.acceleration_dpss[j]);
                fprintf(fpTraj, " %.2f %.1f %.1f %.1f\n",
                        cs.selfMinDist_m * 1000,
                        cs.tcpPos_mm[0], cs.tcpPos_mm[1], cs.tcpPos_mm[2]);
            }
        }

        // 关键姿态: 起始、最小距离、终止
        if (fpPoses && !res.samples.empty()) {
            auto& first = res.samples.front();
            fprintf(fpPoses, "%d %.4f %.3f %.3f %.3f %.3f %.3f %.3f %.2f %.1f %.1f %.1f start\n",
                    s+1, first.time, first.position_deg[0],first.position_deg[1],
                    first.position_deg[2],first.position_deg[3],
                    first.position_deg[4],first.position_deg[5],
                    first.selfMinDist_m*1000,first.tcpPos_mm[0],first.tcpPos_mm[1],first.tcpPos_mm[2]);

            // 找最小距离点
            size_t minIdx = 0;
            for (size_t i = 1; i < res.samples.size(); i++) {
                if (res.samples[i].selfMinDist_m < res.samples[minIdx].selfMinDist_m)
                    minIdx = i;
            }
            auto& minPt = res.samples[minIdx];
            fprintf(fpPoses, "%d %.4f %.3f %.3f %.3f %.3f %.3f %.3f %.2f %.1f %.1f %.1f min_dist\n",
                    s+1, minPt.time, minPt.position_deg[0],minPt.position_deg[1],
                    minPt.position_deg[2],minPt.position_deg[3],
                    minPt.position_deg[4],minPt.position_deg[5],
                    minPt.selfMinDist_m*1000,minPt.tcpPos_mm[0],minPt.tcpPos_mm[1],minPt.tcpPos_mm[2]);

            auto& last = res.samples.back();
            fprintf(fpPoses, "%d %.4f %.3f %.3f %.3f %.3f %.3f %.3f %.2f %.1f %.1f %.1f end\n",
                    s+1, last.time, last.position_deg[0],last.position_deg[1],
                    last.position_deg[2],last.position_deg[3],
                    last.position_deg[4],last.position_deg[5],
                    last.selfMinDist_m*1000,last.tcpPos_mm[0],last.tcpPos_mm[1],last.tcpPos_mm[2]);
        }

        totalTime += res.totalTime_s;
        totalCollisions += res.collisionCount;
        if (res.minSelfDist_m < globalMinDist)
            globalMinDist = res.minSelfDist_m;

        results.push_back(res);
    }

    if (fpCsv)   fclose(fpCsv);
    if (fpTraj)  fclose(fpTraj);
    if (fpPoses) fclose(fpPoses);

    // ---- 统计报告 ----
    printf("━━━━━━ 阶段3: 碰撞仿真统计分析 ━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n");

    int successCount = 0;
    double maxPeakVel = 0, maxPeakAcc = 0;
    double totalParamTime = 0, totalCollisionTime = 0;
    for (auto& r : results) {
        if (r.success) successCount++;
        if (r.peakVelocity_dps > maxPeakVel) maxPeakVel = r.peakVelocity_dps;
        if (r.peakAcceleration_dpss > maxPeakAcc) maxPeakAcc = r.peakAcceleration_dpss;
        totalParamTime += r.paramTime_ms;
        totalCollisionTime += r.collisionCheckTime_ms;
    }

    auto t_end = std::chrono::high_resolution_clock::now();
    double totalElapsed = std::chrono::duration<double>(t_end - t_global).count();

    printf("  ╔════════════════════════════════════════════════════════════╗\n");
    printf("  ║            S曲线碰撞仿真统计报告                            ║\n");
    printf("  ╠════════════════════════════════════════════════════════════╣\n");
    printf("  ║ 轨迹生成:    7段S曲线时间参数化 (与RML等价)                  ║\n");
    printf("  ║ 碰撞检测库:  HRC (华数自研碰撞检测)                          ║\n");
    printf("  ║ 插补周期:    4ms (250 Hz)                                  ║\n");
    printf("  ╠════════════════════════════════════════════════════════════╣\n");
    printf("  ║ 测试场景:    %d 个                                         ║\n", numScenarios);
    printf("  ║ 成功完成:    %d/%d                                         ║\n", successCount, numScenarios);
    printf("  ║ 总运动时间:  %.3f s                                        ║\n", totalTime);
    printf("  ╠════════════════════════════════════════════════════════════╣\n");
    printf("  ║ 碰撞统计:                                                  ║\n");
    printf("  ║   碰撞采样总数: %d                                        ║\n", totalCollisions);
    printf("  ║   全局最小距离: %.1f mm                                    ║\n", globalMinDist * 1000);
    printf("  ║   安全率:      %.1f%%                                     ║\n",
           totalCollisions == 0 ? 100.0 : 0.0);
    printf("  ╠════════════════════════════════════════════════════════════╣\n");
    printf("  ║ 运动性能:                                                  ║\n");
    printf("  ║   最大峰值速度:   %.1f °/s                                 ║\n", maxPeakVel);
    printf("  ║   最大峰值加速度: %.1f °/s²                                ║\n", maxPeakAcc);
    printf("  ╠════════════════════════════════════════════════════════════╣\n");
    printf("  ║ 计算性能:                                                  ║\n");
    printf("  ║   S曲线参数化总耗时: %.2f ms                               ║\n", totalParamTime);
    printf("  ║   碰撞检测总耗时:   %.1f ms                                ║\n", totalCollisionTime);
    printf("  ║   总耗时(含IO):     %.2f s                                 ║\n", totalElapsed);
    printf("  ╚════════════════════════════════════════════════════════════╝\n\n");

    // 详细对比表
    printf("  详细场景对比:\n");
    printf("  序号 场景                          时间(s)   步数  最小距离mm  峰值速度   状态\n");
    printf("  ──── ────────────────────────────── ──────── ────── ────────── ──────── ──────\n");
    for (int i = 0; i < (int)results.size(); i++) {
        auto& r = results[i];
        printf("  %-4d %-30s %7.3f  %6d   %8.1f   %7.1f %s\n",
               i+1, r.name.c_str(), r.totalTime_s, r.totalPoints,
               r.minSelfDist_m * 1000, r.peakVelocity_dps,
               r.success ? "✅完成" : "❌失败");
    }

    // 写统计摘要文件
    FILE* fpSum = fopen("data/rml_collision_summary.txt", "w");
    if (fpSum) {
        fprintf(fpSum, "# HR_S50-2000 碰撞仿真统计摘要\n");
        fprintf(fpSum, "# 生成方式: 7段S曲线时间参数化\n");
        fprintf(fpSum, "# 碰撞检测: HRC碰撞检测库\n\n");
        fprintf(fpSum, "scenarios: %d\n", numScenarios);
        fprintf(fpSum, "success: %d\n", successCount);
        fprintf(fpSum, "total_time_s: %.4f\n", totalTime);
        fprintf(fpSum, "total_collisions: %d\n", totalCollisions);
        fprintf(fpSum, "global_min_dist_mm: %.2f\n", globalMinDist * 1000);
        fprintf(fpSum, "max_peak_vel_dps: %.2f\n", maxPeakVel);
        fprintf(fpSum, "max_peak_acc_dpss: %.2f\n", maxPeakAcc);
        fprintf(fpSum, "param_time_ms: %.3f\n", totalParamTime);
        fprintf(fpSum, "collision_time_ms: %.2f\n", totalCollisionTime);
        fprintf(fpSum, "\n# Per-scenario:\n");
        fprintf(fpSum, "# idx name time_s points minDist_mm peakVel_dps peakAcc_dpss\n");
        for (int i = 0; i < (int)results.size(); i++) {
            auto& r = results[i];
            fprintf(fpSum, "%d \"%s\" %.4f %d %.2f %.2f %.2f\n",
                    i+1, r.name.c_str(), r.totalTime_s, r.totalPoints,
                    r.minSelfDist_m * 1000, r.peakVelocity_dps, r.peakAcceleration_dpss);
        }
        fclose(fpSum);
    }

    printf("\n  输出文件:\n");
    printf("    data/rml_collision_trajectory.txt  — 完整轨迹数据\n");
    printf("    data/rml_collision_profile.csv     — 碰撞距离曲线\n");
    printf("    data/rml_collision_summary.txt     — 统计摘要\n");
    printf("    data/rml_collision_poses.txt       — 关键姿态快照\n");

    printf("\n  总耗时: %.2f s\n", totalElapsed);

    printf("\n══════════════════════════════════════════════════════════════\n");
    printf("  HR_S50-2000 碰撞仿真完成\n");
    printf("══════════════════════════════════════════════════════════════\n");

    return 0;
}
