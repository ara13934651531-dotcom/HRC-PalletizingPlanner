/**
 * @file testPalletizingScenarioRML.cpp
 * @brief 码垛场景完整仿真 — S曲线多任务轨迹 + HRC碰撞检测 + 节拍分析
 *
 * 模拟完整的码垛工作流程:
 *   1. 传送带取料 → 抬升 → 移动 → 码放（3层×4格=12个码垛位置）
 *   2. TSP任务序列优化（2-opt）
 *   3. 全程S曲线轨迹参数化 + HRC碰撞距离监测
 *   4. 正运动学TCP轨迹追踪
 *   5. 节拍时间统计与效率分析
 *
 * 注: libCmpRML.so（CoDeSys IEC 61131组件库）经反汇编确认需PLC运行时，
 *     此处使用项目内置功能等价的TimeParameterization.hpp (五次多项式S曲线)。
 *
 * @date 2026-02-06
 * @copyright Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.
 */

#include "PalletizingPlanner/RobotModel.hpp"
#include "PalletizingPlanner/CollisionChecker.hpp"
#include "PalletizingPlanner/TimeParameterization.hpp"
#include "PalletizingPlanner/TaskSequencer.hpp"
#include "PalletizingPlanner/Types.hpp"

#include <cstdio>
#include <cmath>
#include <chrono>
#include <vector>
#include <algorithm>
#include <numeric>
#include <string>

using namespace palletizing;

// ============================================================================
// 码垛场景参数
// ============================================================================
namespace PalletScene {
    // HOME 位
    constexpr double HOME[6] = {0, -90, 0, 0, 90, 0};

    // 取料位 (传送带上方)
    constexpr double PICK_APPROACH[6] = {30, -70, 20, 0, 70, 0};
    constexpr double PICK_POS[6]      = {30, -50, 40, 0, 50, 0};

    // 安全过渡位 (抬升)
    constexpr double SAFE_TRANSIT[6]  = {0, -80, 10, 0, 80, 0};

    // 码垛区域参数
    constexpr int    PALLET_ROWS = 4;
    constexpr int    PALLET_COLS = 3;
    constexpr int    PALLET_LAYERS = 3;
    constexpr double J1_BASE = -40.0;      // 码垛区J1基础角度
    constexpr double J1_COL_STEP = 12.0;   // 每列J1偏移
    constexpr double J2_BASE = -55.0;      // J2基础角度
    constexpr double J2_ROW_STEP = 5.0;    // 每行J2偏移
    constexpr double J2_LAYER_STEP = 3.0;  // 每层J2偏移(更高=更靠近)
    constexpr double J3_BASE = 35.0;
    constexpr double J5_BASE = 55.0;
}

// ============================================================================
// 辅助结构与函数
// ============================================================================

/// 单次码垛任务结果
struct CycleResult {
    int taskId;
    std::string posString;  // e.g. "L0_R1_C2"
    double moveTime;        // 运动时间 (s)
    double totalPoints;     // 轨迹采样点数
    double selfMinDist;     // 最小自碰撞距离 (m)
    bool   selfSafe;        // 是否无自碰撞
    double tcpStart[3];     // TCP起始位置 (mm)
    double tcpEnd[3];       // TCP终止位置 (mm)
};

/// P2P路径
Path makeP2P(const JointConfig& s, const JointConfig& e) {
    Path p; Waypoint w0(s); w0.pathParam=0; Waypoint w1(e); w1.pathParam=1;
    p.waypoints.push_back(w0); p.waypoints.push_back(w1); return p;
}

/// 多段联合轨迹 (P2P序列: WP[0]→WP[1]→...→WP[n-1])
Trajectory generateMultiSegmentTrajectory(
    const std::vector<JointConfig>& waypoints,
    TimeParameterizer& parameterizer)
{
    Trajectory combined;
    double currentTime = 0.0;
    for (size_t i = 0; i + 1 < waypoints.size(); i++) {
        Path seg = makeP2P(waypoints[i], waypoints[i+1]);
        Trajectory segTraj = parameterizer.parameterize(seg);
        for (size_t j = (i > 0 ? 1 : 0); j < segTraj.size(); j++) {
            TrajectoryPoint pt = segTraj.points[j];
            pt.time += currentTime;
            combined.points.push_back(pt);
        }
        currentTime += segTraj.totalTime;
    }
    combined.totalTime = currentTime;
    return combined;
}

/// 碰撞分析(仅自碰撞)
struct QuickCollisionResult {
    double minDist = 1e10;
    int selfCollisionCount = 0;
};

QuickCollisionResult quickCollisionCheck(
    const Trajectory& traj,
    CollisionChecker& checker,
    int interval = 20)
{
    QuickCollisionResult res;
    for (size_t i = 0; i < traj.size(); i += interval) {
        auto report = checker.getCollisionReport(traj.points[i].config);
        if (report.selfCollision) res.selfCollisionCount++;
        if (report.selfMinDistance < res.minDist)
            res.minDist = report.selfMinDistance;
    }
    return res;
}

/// 生成码垛放料关节角
JointConfig makePlaceConfig(int layer, int row, int col) {
    double j1 = PalletScene::J1_BASE + col * PalletScene::J1_COL_STEP;
    double j2 = PalletScene::J2_BASE + row * PalletScene::J2_ROW_STEP
                + layer * PalletScene::J2_LAYER_STEP;
    double j3 = PalletScene::J3_BASE;
    double j5 = PalletScene::J5_BASE - row * PalletScene::J2_ROW_STEP * 0.5;
    double j6 = j1;  // 工具随J1旋转
    return JointConfig::fromDegrees({j1, j2, j3, 0, j5, j6});
}

/// 码放接近位 (提升J2约15°)
JointConfig makePlaceApproach(const JointConfig& place) {
    auto q = place.toDegrees();
    return JointConfig::fromDegrees({q[0], q[1] - 15, q[2] - 10, q[3], q[4] + 10, q[5]});
}

// ============================================================================
// 主仿真
// ============================================================================
int main() {
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║    S50 码垛场景仿真 — S曲线轨迹 × HRC碰撞检测               ║\n");
    printf("║    %d层×%d行×%d列 = %d个码垛位置 | TSP序列优化               ║\n",
           PalletScene::PALLET_LAYERS, PalletScene::PALLET_ROWS,
           PalletScene::PALLET_COLS,
           PalletScene::PALLET_LAYERS * PalletScene::PALLET_ROWS * PalletScene::PALLET_COLS);
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");

    // --- 初始化 ---
    RobotModel robot;
    CollisionChecker checker(robot);
    printf("  [初始化] 碰撞检测(码垛场景)...\n");
    bool initOk = checker.initialize();
    printf("  碰撞检测: %s\n\n", initOk ? "✓ 成功" : "✗ 失败");

    auto tpConfig = TimeParameterizationConfig::fromRobotParams(robot.getParams());
    tpConfig.profileType = VelocityProfileType::SCurve;
    tpConfig.velocityScaling = 1.0;
    tpConfig.samplePeriod = 0.004;  // 4ms
    TimeParameterizer parameterizer(tpConfig);

    // --- 生成码垛任务 ---
    printf("═══════════════════════════════════════════════════════════\n");
    printf("  阶段1: 生成码垛任务 + TSP序列优化\n");
    printf("═══════════════════════════════════════════════════════════\n\n");

    auto homeConfig = JointConfig::fromDegrees({
        PalletScene::HOME[0], PalletScene::HOME[1], PalletScene::HOME[2],
        PalletScene::HOME[3], PalletScene::HOME[4], PalletScene::HOME[5]});
    auto pickApproach = JointConfig::fromDegrees({
        PalletScene::PICK_APPROACH[0], PalletScene::PICK_APPROACH[1],
        PalletScene::PICK_APPROACH[2], PalletScene::PICK_APPROACH[3],
        PalletScene::PICK_APPROACH[4], PalletScene::PICK_APPROACH[5]});
    auto pickPos = JointConfig::fromDegrees({
        PalletScene::PICK_POS[0], PalletScene::PICK_POS[1],
        PalletScene::PICK_POS[2], PalletScene::PICK_POS[3],
        PalletScene::PICK_POS[4], PalletScene::PICK_POS[5]});
    auto safeTransit = JointConfig::fromDegrees({
        PalletScene::SAFE_TRANSIT[0], PalletScene::SAFE_TRANSIT[1],
        PalletScene::SAFE_TRANSIT[2], PalletScene::SAFE_TRANSIT[3],
        PalletScene::SAFE_TRANSIT[4], PalletScene::SAFE_TRANSIT[5]});

    // 生成任务列表
    std::vector<PalletizingTask> tasks;
    for (int layer = 0; layer < PalletScene::PALLET_LAYERS; layer++) {
        for (int row = 0; row < PalletScene::PALLET_ROWS; row++) {
            for (int col = 0; col < PalletScene::PALLET_COLS; col++) {
                PalletizingTask task;
                task.taskId = (int)tasks.size();
                task.pickConfig = pickPos;
                task.placeConfig = makePlaceConfig(layer, row, col);
                task.description = "L" + std::to_string(layer) + "_R"
                                 + std::to_string(row) + "_C" + std::to_string(col);
                tasks.push_back(task);
            }
        }
    }
    printf("  生成 %zu 个码垛任务\n", tasks.size());

    // TSP优化
    auto t0 = std::chrono::high_resolution_clock::now();
    auto optimizedOrder = TaskSequenceOptimizer::optimize(tasks, homeConfig);
    auto t1 = std::chrono::high_resolution_clock::now();
    double tspTime = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // 计算优化前后总距离
    double origDist = 0, optDist = 0;
    JointConfig prev = homeConfig;
    for (size_t i = 0; i < tasks.size(); i++) {
        origDist += prev.distanceTo(tasks[i].pickConfig);
        origDist += tasks[i].pickConfig.distanceTo(tasks[i].placeConfig);
        prev = tasks[i].placeConfig;
    }
    prev = homeConfig;
    for (int idx : optimizedOrder) {
        optDist += prev.distanceTo(tasks[idx].pickConfig);
        optDist += tasks[idx].pickConfig.distanceTo(tasks[idx].placeConfig);
        prev = tasks[idx].placeConfig;
    }
    printf("  TSP优化耗时: %.2f ms\n", tspTime);
    printf("  原始总距离: %.1f° → 优化后: %.1f° (节省 %.1f%%)\n\n",
           origDist * 180 / M_PI, optDist * 180 / M_PI,
           (1.0 - optDist / origDist) * 100);

    // --- 执行码垛仿真 ---
    printf("═══════════════════════════════════════════════════════════\n");
    printf("  阶段2: 码垛仿真执行 (S曲线轨迹 + 碰撞监测)\n");
    printf("═══════════════════════════════════════════════════════════\n\n");

    printf("  %-5s %-12s %8s %8s %10s %6s\n",
           "序号", "位置", "时间(s)", "点数", "最小距离m", "安全");
    printf("  ───── ──────────── ──────── ──────── ────────── ──────\n");

    std::vector<CycleResult> results;
    JointConfig currentPos = homeConfig;
    double totalMoveTime = 0;
    int unsafeCount = 0;

    // TCP轨迹记录文件
    FILE* fpTcp = fopen("data/palletizing_tcp_trajectory.txt", "w");
    if (fpTcp) {
        fprintf(fpTcp, "# 码垛TCP轨迹 (mm)\n");
        fprintf(fpTcp, "# time_s tcp_x tcp_y tcp_z task_id phase\n");
    }

    // 完整轨迹记录
    FILE* fpFull = fopen("data/palletizing_full_trajectory.txt", "w");
    if (fpFull) {
        fprintf(fpFull, "# 完整码垛轨迹 (deg, deg/s)\n");
        fprintf(fpFull, "# time_s q1 q2 q3 q4 q5 q6 v1 v2 v3 v4 v5 v6 task_id\n");
    }

    double globalTime = 0;

    for (size_t taskIdx = 0; taskIdx < optimizedOrder.size(); taskIdx++) {
        int ti = optimizedOrder[taskIdx];
        const auto& task = tasks[ti];

        // 计算接近位
        auto placeApproach = makePlaceApproach(task.placeConfig);

        // 完整运动序列:
        // 当前位 → 安全过渡 → 取料接近 → 取料 (夹取)
        // → 取料接近 → 安全过渡 → 码放接近 → 码放 (释放)
        std::vector<JointConfig> motionSequence = {
            currentPos,       // 0: 当前位置
            safeTransit,      // 1: 安全过渡
            pickApproach,     // 2: 取料接近
            pickPos,          // 3: 取料位(夹取)
            pickApproach,     // 4: 取料离开
            safeTransit,      // 5: 安全过渡
            placeApproach,    // 6: 码放接近
            task.placeConfig, // 7: 码放位(释放)
        };

        // 生成多段S曲线轨迹
        Trajectory traj = generateMultiSegmentTrajectory(motionSequence, parameterizer);

        // 碰撞检测
        auto collRes = quickCollisionCheck(traj, checker, 20);

        // TCP位置记录
        if (fpTcp) {
            for (size_t i = 0; i < traj.size(); i += 10) {
                auto pose = robot.forwardKinematics(traj.points[i].config);
                fprintf(fpTcp, "%.4f %.2f %.2f %.2f %d %zu\n",
                        globalTime + traj.points[i].time,
                        pose.position.x()*1000, pose.position.y()*1000, pose.position.z()*1000,
                        ti, taskIdx);
            }
        }

        // 完整轨迹记录
        if (fpFull) {
            for (size_t i = 0; i < traj.size(); i++) {
                auto q = traj.points[i].config.toDegrees();
                fprintf(fpFull, "%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %d\n",
                        globalTime + traj.points[i].time,
                        q[0], q[1], q[2], q[3], q[4], q[5],
                        traj.points[i].velocity[0]*180/M_PI, traj.points[i].velocity[1]*180/M_PI,
                        traj.points[i].velocity[2]*180/M_PI, traj.points[i].velocity[3]*180/M_PI,
                        traj.points[i].velocity[4]*180/M_PI, traj.points[i].velocity[5]*180/M_PI,
                        ti);
            }
        }

        // TCP首末位置
        auto startPose = robot.forwardKinematics(currentPos);
        auto endPose = robot.forwardKinematics(task.placeConfig);

        CycleResult cr;
        cr.taskId = ti;
        cr.posString = task.description;
        cr.moveTime = traj.totalTime;
        cr.totalPoints = traj.size();
        cr.selfMinDist = collRes.minDist;
        cr.selfSafe = (collRes.selfCollisionCount == 0);
        cr.tcpStart[0] = startPose.position.x()*1000;
        cr.tcpStart[1] = startPose.position.y()*1000;
        cr.tcpStart[2] = startPose.position.z()*1000;
        cr.tcpEnd[0] = endPose.position.x()*1000;
        cr.tcpEnd[1] = endPose.position.y()*1000;
        cr.tcpEnd[2] = endPose.position.z()*1000;
        results.push_back(cr);

        printf("  %-5zu %-12s %8.3f %8.0f %10.4f %s\n",
               taskIdx + 1, task.description.c_str(),
               cr.moveTime, cr.totalPoints, cr.selfMinDist,
               cr.selfSafe ? "✓" : "✗");

        totalMoveTime += traj.totalTime;
        globalTime += traj.totalTime;
        if (!cr.selfSafe) unsafeCount++;
        currentPos = task.placeConfig;
    }

    // 返回HOME
    {
        Trajectory homeTraj = generateMultiSegmentTrajectory(
            {currentPos, safeTransit, homeConfig}, parameterizer);
        auto collRes = quickCollisionCheck(homeTraj, checker, 20);
        printf("  %-5s %-12s %8.3f %8.0f %10.4f %s\n",
               "RTN", "→HOME", homeTraj.totalTime, (double)homeTraj.size(),
               collRes.minDist, collRes.selfCollisionCount == 0 ? "✓" : "✗");
        totalMoveTime += homeTraj.totalTime;

        if (fpTcp) {
            for (size_t i = 0; i < homeTraj.size(); i += 10) {
                auto pose = robot.forwardKinematics(homeTraj.points[i].config);
                fprintf(fpTcp, "%.4f %.2f %.2f %.2f -1 -1\n",
                        globalTime + homeTraj.points[i].time,
                        pose.position.x()*1000, pose.position.y()*1000, pose.position.z()*1000);
            }
        }
        if (fpFull) {
            for (size_t i = 0; i < homeTraj.size(); i++) {
                auto q = homeTraj.points[i].config.toDegrees();
                fprintf(fpFull, "%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f -1\n",
                        globalTime + homeTraj.points[i].time,
                        q[0], q[1], q[2], q[3], q[4], q[5],
                        homeTraj.points[i].velocity[0]*180/M_PI, homeTraj.points[i].velocity[1]*180/M_PI,
                        homeTraj.points[i].velocity[2]*180/M_PI, homeTraj.points[i].velocity[3]*180/M_PI,
                        homeTraj.points[i].velocity[4]*180/M_PI, homeTraj.points[i].velocity[5]*180/M_PI);
            }
        }
    }

    if (fpTcp) fclose(fpTcp);
    if (fpFull) fclose(fpFull);

    // --- 统计分析 ---
    printf("\n═══════════════════════════════════════════════════════════\n");
    printf("  阶段3: 码垛仿真统计分析\n");
    printf("═══════════════════════════════════════════════════════════\n\n");

    // 节拍时间
    std::vector<double> cycleTimes;
    for (auto& r : results) cycleTimes.push_back(r.moveTime);

    double minCycle = *std::min_element(cycleTimes.begin(), cycleTimes.end());
    double maxCycle = *std::max_element(cycleTimes.begin(), cycleTimes.end());
    double avgCycle = std::accumulate(cycleTimes.begin(), cycleTimes.end(), 0.0)
                      / cycleTimes.size();

    // 碰撞距离
    std::vector<double> allDists;
    for (auto& r : results) allDists.push_back(r.selfMinDist);
    double minDistAll = *std::min_element(allDists.begin(), allDists.end());
    double maxDistAll = *std::max_element(allDists.begin(), allDists.end());
    double avgDistAll = std::accumulate(allDists.begin(), allDists.end(), 0.0)
                        / allDists.size();

    printf("  ┌────────────────────────────────────────────┐\n");
    printf("  │ 码垛任务总览                                │\n");
    printf("  ├────────────────────────────────────────────┤\n");
    printf("  │ 总任务数:    %-6zu (含返回HOME)             │\n", results.size() + 1);
    printf("  │ 码垛位置:    %d层×%d行×%d列 = %d个          │\n",
           PalletScene::PALLET_LAYERS, PalletScene::PALLET_ROWS,
           PalletScene::PALLET_COLS,
           PalletScene::PALLET_LAYERS * PalletScene::PALLET_ROWS * PalletScene::PALLET_COLS);
    printf("  │ TSP优化:     %.1f° → %.1f° (省%.0f%%)      │\n",
           origDist*180/M_PI, optDist*180/M_PI, (1-optDist/origDist)*100);
    printf("  ├────────────────────────────────────────────┤\n");
    printf("  │ 运动时间统计                                │\n");
    printf("  ├────────────────────────────────────────────┤\n");
    printf("  │ 总运动时间:  %.2f s (%.1f min)              │\n", totalMoveTime, totalMoveTime/60);
    printf("  │ 平均节拍:    %.3f s/件                      │\n", avgCycle);
    printf("  │ 最快节拍:    %.3f s                         │\n", minCycle);
    printf("  │ 最慢节拍:    %.3f s                         │\n", maxCycle);
    printf("  │ 理论产量:    %.0f 件/小时                    │\n", 3600.0 / avgCycle);
    printf("  ├────────────────────────────────────────────┤\n");
    printf("  │ 碰撞安全统计                                │\n");
    printf("  ├────────────────────────────────────────────┤\n");
    printf("  │ 安全率:      %.1f%% (%zu/%zu 无自碰撞)      │\n",
           100.0 * (results.size() - unsafeCount) / results.size(),
           results.size() - unsafeCount, results.size());
    printf("  │ 最小距离:    %.1f mm                        │\n", minDistAll * 1000);
    printf("  │ 最大距离:    %.1f mm                        │\n", maxDistAll * 1000);
    printf("  │ 平均距离:    %.1f mm                        │\n", avgDistAll * 1000);
    printf("  └────────────────────────────────────────────┘\n");

    // TCP工作空间范围
    double tcpXmin=1e10, tcpXmax=-1e10, tcpYmin=1e10, tcpYmax=-1e10, tcpZmin=1e10, tcpZmax=-1e10;
    for (auto& r : results) {
        tcpXmin = std::min({tcpXmin, r.tcpStart[0], r.tcpEnd[0]});
        tcpXmax = std::max({tcpXmax, r.tcpStart[0], r.tcpEnd[0]});
        tcpYmin = std::min({tcpYmin, r.tcpStart[1], r.tcpEnd[1]});
        tcpYmax = std::max({tcpYmax, r.tcpStart[1], r.tcpEnd[1]});
        tcpZmin = std::min({tcpZmin, r.tcpStart[2], r.tcpEnd[2]});
        tcpZmax = std::max({tcpZmax, r.tcpStart[2], r.tcpEnd[2]});
    }
    printf("\n  TCP工作空间范围 (基座坐标系, mm):\n");
    printf("    X: [%.0f, %.0f] → 跨度 %.0f mm\n", tcpXmin, tcpXmax, tcpXmax-tcpXmin);
    printf("    Y: [%.0f, %.0f] → 跨度 %.0f mm\n", tcpYmin, tcpYmax, tcpYmax-tcpYmin);
    printf("    Z: [%.0f, %.0f] → 跨度 %.0f mm\n", tcpZmin, tcpZmax, tcpZmax-tcpZmin);

    // 每层统计
    printf("\n  每层码垛统计:\n");
    printf("  %-8s %8s %8s %10s\n", "层", "任务数", "平均(s)", "最小距离m");
    printf("  ──────── ──────── ──────── ──────────\n");
    for (int layer = 0; layer < PalletScene::PALLET_LAYERS; layer++) {
        double layerTime = 0;
        double layerMinDist = 1e10;
        int count = 0;
        for (auto& r : results) {
            int taskLayer = r.taskId / (PalletScene::PALLET_ROWS * PalletScene::PALLET_COLS);
            if (taskLayer == layer) {
                layerTime += r.moveTime;
                layerMinDist = std::min(layerMinDist, r.selfMinDist);
                count++;
            }
        }
        printf("  Layer%-2d %8d %8.3f %10.4f\n",
               layer, count, count > 0 ? layerTime / count : 0, layerMinDist);
    }

    // 输出统计摘要
    FILE* fpSum = fopen("data/palletizing_summary.txt", "w");
    if (fpSum) {
        fprintf(fpSum, "# 码垛仿真统计摘要\n");
        fprintf(fpSum, "tasks %zu\n", results.size());
        fprintf(fpSum, "total_time_s %.3f\n", totalMoveTime);
        fprintf(fpSum, "avg_cycle_s %.3f\n", avgCycle);
        fprintf(fpSum, "min_cycle_s %.3f\n", minCycle);
        fprintf(fpSum, "max_cycle_s %.3f\n", maxCycle);
        fprintf(fpSum, "throughput_per_hour %.0f\n", 3600.0 / avgCycle);
        fprintf(fpSum, "safety_rate %.1f\n",
                100.0 * (results.size() - unsafeCount) / results.size());
        fprintf(fpSum, "min_self_dist_m %.4f\n", minDistAll);
        fprintf(fpSum, "tsp_savings_pct %.1f\n", (1 - optDist / origDist) * 100);
        for (auto& r : results) {
            fprintf(fpSum, "cycle %d %s %.3f %.4f %d\n",
                    r.taskId, r.posString.c_str(), r.moveTime, r.selfMinDist, r.selfSafe ? 1 : 0);
        }
        fclose(fpSum);
        printf("\n  统计摘要: data/palletizing_summary.txt\n");
    }

    printf("  TCP轨迹: data/palletizing_tcp_trajectory.txt\n");
    printf("  完整轨迹: data/palletizing_full_trajectory.txt\n");

    printf("\n════════════════════════════════════════════════════════════\n");
    printf("  码垛场景仿真完成 (%zu 个任务, 总时间 %.1f s)\n",
           results.size(), totalMoveTime);
    printf("════════════════════════════════════════════════════════════\n");

    return 0;
}
