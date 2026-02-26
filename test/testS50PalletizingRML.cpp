/**
 * @file testS50PalletizingRML.cpp
 * @brief HR_S50-2000 码垛场景仿真 — S曲线轨迹 + 碰撞检测 + TSP优化
 *
 * 完整的码垛作业仿真:
 *   - 12个码垛位置 (3层×2行×2列)
 *   - TSP 2-opt任务序列优化
 *   - 7段S曲线轨迹生成 (4ms周期)
 *   - HRC 碰撞检测 (全程监测)
 *   - TCP/关节轨迹记录 (用于STL可视化)
 *
 * 码垛流程:
 *   HOME → (循环: 安全高度 → 取料接近 → 抓取 → 取料抬升 →
 *            安全高度 → 放料接近 → 放置 → 放料抬升) → HOME
 *
 * 输出:
 *   data/rml_palletizing_trajectory.txt — 完整轨迹
 *   data/rml_palletizing_profile.csv    — 碰撞距离曲线
 *   data/rml_palletizing_summary.txt    — 统计摘要
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
#include <limits>

using namespace palletizing;

// ============================================================================
// 数据结构
// ============================================================================
struct SegmentResult {
    std::string name;
    double totalTime_s;
    int    totalPoints;
    double minSelfDist_m;
    int    collisionCount;
    double peakVelocity_dps;
    bool   success;
};

struct TaskResult {
    int taskIndex;
    std::string description;
    std::vector<SegmentResult> segments;
    double totalTime_s;
    double totalDistance_deg;
    double minSelfDist_m;
    int    totalCollisions;
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

SegmentResult executeSegment(
    const char* name,
    const JointConfig& start,
    const JointConfig& target,
    RobotModel& robot,
    CollisionChecker& checker,
    TimeParameterizer& parameterizer,
    FILE* fpCsv, int taskIdx, int segIdx,
    FILE* fpTraj)
{
    SegmentResult result;
    result.name = name;
    result.minSelfDist_m = 1e10;
    result.collisionCount = 0;
    result.peakVelocity_dps = 0;
    result.success = false;

    Path path = makeP2PPath(start, target);
    Trajectory traj = parameterizer.parameterize(path);

    if (traj.empty()) {
        result.totalTime_s = 0;
        result.totalPoints = 0;
        return result;
    }

    result.totalTime_s = traj.totalTime;
    result.totalPoints = (int)traj.size();

    for (size_t i = 0; i < traj.size(); i++) {
        const auto& pt = traj.points[i];

        for (int j = 0; j < 6; j++) {
            double v = std::fabs(pt.velocity[j]) * 180.0 / M_PI;
            if (v > result.peakVelocity_dps) result.peakVelocity_dps = v;
        }

        // 碰撞检测 (每10步检测一次)
        if (i % 10 == 0 || i == traj.size() - 1) {
            auto report = checker.getCollisionReport(pt.config);
            if (report.selfCollision) result.collisionCount++;
            if (report.selfMinDistance < result.minSelfDist_m)
                result.minSelfDist_m = report.selfMinDistance;

            if (fpCsv) {
                auto q = pt.config.toDegrees();
                auto pose = robot.forwardKinematics(pt.config);
                fprintf(fpCsv, "%d,%d,%d,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                        "%.2f,%d,%.1f,%.1f,%.1f\n",
                        taskIdx, segIdx, (int)i, pt.time,
                        q[0],q[1],q[2],q[3],q[4],q[5],
                        report.selfMinDistance * 1000, report.selfCollision ? 1 : 0,
                        pose.position.x()*1000, pose.position.y()*1000, pose.position.z()*1000);
            }
        }

        // 每20步写轨迹
        if (fpTraj && (i % 20 == 0 || i == traj.size() - 1)) {
            auto q = pt.config.toDegrees();
            auto pose = robot.forwardKinematics(pt.config);
            fprintf(fpTraj, "%d %d %.4f", taskIdx, segIdx, pt.time);
            for (int j = 0; j < 6; j++) fprintf(fpTraj, " %.4f", q[j]);
            for (int j = 0; j < 6; j++) fprintf(fpTraj, " %.3f", pt.velocity[j]*180/M_PI);
            fprintf(fpTraj, " %.2f %.1f %.1f %.1f\n",
                    result.minSelfDist_m*1000,
                    pose.position.x()*1000, pose.position.y()*1000, pose.position.z()*1000);
        }
    }

    result.success = true;
    return result;
}

// ============================================================================
// 主程序
// ============================================================================
int main() {
    printf("╔═══════════════════════════════════════════════════════════════════╗\n");
    printf("║   HR_S50-2000 码垛场景仿真 — S曲线 + 碰撞检测 + TSP优化         ║\n");
    printf("║   12位码垛 × 7段运动 × HRC碰撞检测 × STL可视化数据               ║\n");
    printf("╚═══════════════════════════════════════════════════════════════════╝\n\n");

    auto t_global = std::chrono::high_resolution_clock::now();

    // ---- 初始化 ----
    printf("━━━━━━ 阶段1: 系统初始化 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n");

    RobotModel robot;
    CollisionChecker checker(robot);
    bool collisionOk = checker.initialize();
    printf("  机器人: HR_S50-2000 (6-DOF, 50kg载荷)\n");
    printf("  碰撞检测: %s\n", collisionOk ? "✅ HRC初始化成功" : "❌ 初始化失败");

    auto tpConfig = TimeParameterizationConfig::fromRobotParams(robot.getParams());
    tpConfig.profileType = VelocityProfileType::SCurve;
    tpConfig.samplePeriod = 0.004;  // 4ms
    tpConfig.velocityScaling = 1.0;
    TimeParameterizer parameterizer(tpConfig);
    printf("  轨迹生成: 7段S曲线 (4ms周期, 250Hz)\n\n");

    // ---- 定义码垛参数 ----
    printf("━━━━━━ 阶段2: 码垛布局定义 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n");

    // HOME位置
    auto HOME = JointConfig::fromDegrees({0, -90, 0, 0, 90, 0});

    // 安全过渡高度位置
    auto SAFE_TRANSIT = JointConfig::fromDegrees({0, -70, 40, 0, 30, 0});

    // 取料工位 (固定位置)
    auto PICK_APPROACH = JointConfig::fromDegrees({-60, -40, 80, 0, -40, -60});
    auto PICK_POS = JointConfig::fromDegrees({-60, -30, 90, 0, -60, -60});

    // 12个码垛位置: 3层 × 2行 × 2列
    // 关节配置根据正运动学逆推,覆盖工作空间右侧区域
    struct PlacePosition {
        const char* label;
        double approach[6];  // 接近位置
        double place[6];     // 放置位置
    };

    PlacePosition placePositions[] = {
        // 第1层 (低)
        {"L1-R1-C1", {50,-50,70,0,-20,50},  {50,-40,80,0,-40,50}},
        {"L1-R1-C2", {70,-50,70,0,-20,70},  {70,-40,80,0,-40,70}},
        {"L1-R2-C1", {50,-50,70,30,-20,50}, {50,-40,80,30,-40,50}},
        {"L1-R2-C2", {70,-50,70,30,-20,70}, {70,-40,80,30,-40,70}},
        // 第2层 (中)
        {"L2-R1-C1", {50,-55,65,0,-10,50},  {50,-45,75,0,-30,50}},
        {"L2-R1-C2", {70,-55,65,0,-10,70},  {70,-45,75,0,-30,70}},
        {"L2-R2-C1", {50,-55,65,30,-10,50}, {50,-45,75,30,-30,50}},
        {"L2-R2-C2", {70,-55,65,30,-10,70}, {70,-45,75,30,-30,70}},
        // 第3层 (高)
        {"L3-R1-C1", {50,-60,60,0,0,50},    {50,-50,70,0,-20,50}},
        {"L3-R1-C2", {70,-60,60,0,0,70},    {70,-50,70,0,-20,70}},
        {"L3-R2-C1", {50,-60,60,30,0,50},   {50,-50,70,30,-20,50}},
        {"L3-R2-C2", {70,-60,60,30,0,70},   {70,-50,70,30,-20,70}},
    };
    int numPositions = 12;

    printf("  码垛布局: 3层 × 2行 × 2列 = 12 位\n");
    printf("  HOME:        [0, -90, 0, 0, 90, 0]°\n");
    printf("  安全过渡:    [0, -70, 40, 0, 30, 0]°\n");
    printf("  取料工位:    [-60, -30, 90, 0, -60, -60]°\n");
    printf("  放料区域:    J1∈[50°,70°], J2∈[-60°,-30°]\n\n");

    // 构建place配置向量 (TSP优化用)
    std::vector<JointConfig> placeConfigs;
    for (int i = 0; i < numPositions; i++) {
        placeConfigs.push_back(JointConfig::fromDegrees(
            {placePositions[i].place[0], placePositions[i].place[1],
             placePositions[i].place[2], placePositions[i].place[3],
             placePositions[i].place[4], placePositions[i].place[5]}));
    }

    // ---- TSP 2-opt优化 (内联实现) ----
    printf("━━━━━━ 阶段3: TSP任务序列优化 ━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n");

    // 贪心 + 2-opt
    auto tspDist = [&](int i, int j) -> double {
        return placeConfigs[i].distanceTo(placeConfigs[j]);
    };

    // Greedy nearest-neighbor from HOME
    std::vector<int> order;
    std::vector<bool> visited(numPositions, false);
    {
        JointConfig current = HOME;
        for (int step = 0; step < numPositions; step++) {
            double best = 1e30;
            int bestIdx = -1;
            for (int j = 0; j < numPositions; j++) {
                if (!visited[j]) {
                    double d = current.distanceTo(placeConfigs[j]);
                    if (d < best) { best = d; bestIdx = j; }
                }
            }
            order.push_back(bestIdx);
            visited[bestIdx] = true;
            current = placeConfigs[bestIdx];
        }
    }

    // 2-opt improvement
    bool improved = true;
    while (improved) {
        improved = false;
        for (int i = 0; i < (int)order.size() - 1; i++) {
            for (int j = i + 2; j < (int)order.size(); j++) {
                double d1 = tspDist(order[i], order[i+1]) + 
                            (j+1 < (int)order.size() ? tspDist(order[j], order[j+1]) : 0);
                double d2 = tspDist(order[i], order[j]) + 
                            (j+1 < (int)order.size() ? tspDist(order[i+1], order[j+1]) : 0);
                if (d2 < d1 - 1e-10) {
                    std::reverse(order.begin() + i + 1, order.begin() + j + 1);
                    improved = true;
                }
            }
        }
    }

    printf("  优化前顺序: 0,1,2,...,11\n");
    printf("  优化后顺序: ");
    for (int i = 0; i < (int)order.size(); i++) {
        printf("%d", order[i]);
        if (i < (int)order.size() - 1) printf(",");
    }
    printf("\n");

    // 计算优化前后总距离
    double distBefore = 0, distAfter = 0;
    for (int i = 0; i < numPositions; i++) {
        int next = (i + 1) % numPositions;
        distBefore += placeConfigs[i].distanceTo(placeConfigs[next]) * 180 / M_PI;
    }
    for (int i = 0; i < (int)order.size(); i++) {
        int next = (i + 1) % (int)order.size();
        distAfter += placeConfigs[order[i]].distanceTo(placeConfigs[order[next]]) * 180 / M_PI;
    }
    printf("  优化前总距离: %.1f°  |  优化后: %.1f°  |  节省: %.1f%%\n\n",
           distBefore, distAfter, (1.0 - distAfter/distBefore) * 100);

    // ---- 执行码垛仿真 ----
    printf("━━━━━━ 阶段4: 码垛仿真执行 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n");

    FILE* fpCsv = fopen("data/rml_palletizing_profile.csv", "w");
    if (fpCsv) {
        fprintf(fpCsv, "task,segment,step,time_s,q1,q2,q3,q4,q5,q6,"
                "selfDist_mm,selfCollision,tcpX_mm,tcpY_mm,tcpZ_mm\n");
    }

    FILE* fpTraj = fopen("data/rml_palletizing_trajectory.txt", "w");
    if (fpTraj) {
        fprintf(fpTraj, "# HR_S50-2000 码垛仿真轨迹\n");
        fprintf(fpTraj, "# task segment time_s q1 q2 q3 q4 q5 q6 v1 v2 v3 v4 v5 v6 "
                "selfDist_mm tcpX tcpY tcpZ\n");
    }

    std::vector<TaskResult> taskResults;
    double totalMotionTime = 0;
    int totalCollisions = 0;
    double globalMinDist = 1e10;
    int totalSegments = 0;

    // 开始: HOME → 安全过渡
    printf("  ── HOME → 安全过渡 ──\n");
    {
        auto seg = executeSegment("HOME→安全过渡", HOME, SAFE_TRANSIT,
                                   robot, checker, parameterizer, fpCsv, 0, 0, fpTraj);
        printf("    %s  %.3fs  %d点  最小距离%.1fmm\n",
               seg.success?"✅":"❌", seg.totalTime_s, seg.totalPoints, seg.minSelfDist_m*1000);
        totalMotionTime += seg.totalTime_s;
        if (seg.minSelfDist_m < globalMinDist) globalMinDist = seg.minSelfDist_m;
        totalSegments++;
    }

    auto currentPos = SAFE_TRANSIT;

    for (int t = 0; t < (int)order.size(); t++) {
        int placeIdx = order[t];
        auto& pp = placePositions[placeIdx];

        TaskResult taskRes;
        taskRes.taskIndex = t + 1;
        char desc[128];
        snprintf(desc, sizeof(desc), "任务%d: 取料→放置[%s]", t+1, pp.label);
        taskRes.description = desc;
        taskRes.totalTime_s = 0;
        taskRes.totalDistance_deg = 0;
        taskRes.minSelfDist_m = 1e10;
        taskRes.totalCollisions = 0;

        printf("\n  ── 任务 %d/%d: 放置位 %s ──\n", t+1, numPositions, pp.label);

        // 7-段运动: 安全位→取料接近→取料→取料抬升→安全位→放料接近→放料
        struct MotionSeg {
            const char* name;
            JointConfig start;
            JointConfig target;
        };

        auto pickApproach = JointConfig::fromDegrees({PICK_APPROACH.toDegrees()[0],
            PICK_APPROACH.toDegrees()[1],PICK_APPROACH.toDegrees()[2],
            PICK_APPROACH.toDegrees()[3],PICK_APPROACH.toDegrees()[4],PICK_APPROACH.toDegrees()[5]});
        auto pickPos = JointConfig::fromDegrees({PICK_POS.toDegrees()[0],
            PICK_POS.toDegrees()[1],PICK_POS.toDegrees()[2],
            PICK_POS.toDegrees()[3],PICK_POS.toDegrees()[4],PICK_POS.toDegrees()[5]});

        auto placeApproach = JointConfig::fromDegrees(
            {pp.approach[0],pp.approach[1],pp.approach[2],
             pp.approach[3],pp.approach[4],pp.approach[5]});
        auto placePos = JointConfig::fromDegrees(
            {pp.place[0],pp.place[1],pp.place[2],
             pp.place[3],pp.place[4],pp.place[5]});

        MotionSeg motions[] = {
            {"安全→取料接近", currentPos, pickApproach},
            {"取料接近→取料", pickApproach, pickPos},
            {"取料→取料抬升", pickPos, pickApproach},
            {"取料抬升→安全", pickApproach, SAFE_TRANSIT},
            {"安全→放料接近", SAFE_TRANSIT, placeApproach},
            {"放料接近→放料", placeApproach, placePos},
            {"放料→放料抬升", placePos, placeApproach},
        };

        for (int s = 0; s < 7; s++) {
            auto seg = executeSegment(motions[s].name, motions[s].start, motions[s].target,
                                       robot, checker, parameterizer,
                                       fpCsv, t+1, s, fpTraj);
            taskRes.segments.push_back(seg);
            taskRes.totalTime_s += seg.totalTime_s;
            taskRes.totalCollisions += seg.collisionCount;
            if (seg.minSelfDist_m < taskRes.minSelfDist_m)
                taskRes.minSelfDist_m = seg.minSelfDist_m;

            totalMotionTime += seg.totalTime_s;
            totalCollisions += seg.collisionCount;
            if (seg.minSelfDist_m < globalMinDist) globalMinDist = seg.minSelfDist_m;
            totalSegments++;
        }

        // 更新当前位置为放料抬升后的位置
        currentPos = placeApproach;

        printf("    总时间: %.3fs | 段数: 7 | 最小距离: %.1fmm | 碰撞: %d\n",
               taskRes.totalTime_s, taskRes.minSelfDist_m * 1000, taskRes.totalCollisions);

        taskResults.push_back(taskRes);
    }

    // 返回HOME
    printf("\n  ── 返回HOME ──\n");
    {
        auto seg = executeSegment("返回HOME", currentPos, HOME,
                                   robot, checker, parameterizer, fpCsv, numPositions+1, 0, fpTraj);
        printf("    %s  %.3fs  %d点  最小距离%.1fmm\n",
               seg.success?"✅":"❌", seg.totalTime_s, seg.totalPoints, seg.minSelfDist_m*1000);
        totalMotionTime += seg.totalTime_s;
        if (seg.minSelfDist_m < globalMinDist) globalMinDist = seg.minSelfDist_m;
        totalSegments++;
    }

    if (fpCsv)  fclose(fpCsv);
    if (fpTraj) fclose(fpTraj);

    // ---- 统计报告 ----
    auto t_end = std::chrono::high_resolution_clock::now();
    double totalElapsed = std::chrono::duration<double>(t_end - t_global).count();

    printf("\n━━━━━━ 阶段5: 码垛仿真统计分析 ━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n");

    printf("  ╔════════════════════════════════════════════════════════════════╗\n");
    printf("  ║              码垛场景仿真统计报告                                ║\n");
    printf("  ╠════════════════════════════════════════════════════════════════╣\n");
    printf("  ║ 机器人:      HR_S50-2000 (50kg载荷)                            ║\n");
    printf("  ║ 轨迹生成:    7段S曲线 (4ms周期, 与华数上位机等价)                 ║\n");
    printf("  ║ 碰撞检测:    HRC碰撞检测库                                      ║\n");
    printf("  ╠════════════════════════════════════════════════════════════════╣\n");
    printf("  ║ 码垛布局:    3层 × 2行 × 2列 = 12 位                          ║\n");
    printf("  ║ 取放任务:    12 组 (每组7段运动)                                ║\n");
    printf("  ║ 运动段总数:  %d (含HOME↔安全过渡)                              ║\n", totalSegments);
    printf("  ╠════════════════════════════════════════════════════════════════╣\n");
    printf("  ║ 总运动时间:  %.3f s (%.1f min)                                ║\n",
           totalMotionTime, totalMotionTime / 60.0);
    printf("  ║ 单任务平均:  %.3f s                                            ║\n",
           taskResults.empty() ? 0 : totalMotionTime / taskResults.size());
    printf("  ╠════════════════════════════════════════════════════════════════╣\n");
    printf("  ║ 碰撞统计:                                                      ║\n");
    printf("  ║   碰撞事件:   %d                                              ║\n", totalCollisions);
    printf("  ║   全局最小距离: %.1f mm                                        ║\n", globalMinDist * 1000);
    printf("  ║   安全状态:    %s                                        ║\n",
           totalCollisions == 0 ? "✅ 全程无碰撞" : "⚠️ 存在碰撞风险");
    printf("  ╠════════════════════════════════════════════════════════════════╣\n");
    printf("  ║ 执行效率:                                                      ║\n");
    printf("  ║   仿真总耗时: %.2f s                                           ║\n", totalElapsed);
    printf("  ║   实时比:     %.1f × 实时                                      ║\n",
           totalElapsed > 0 ? totalMotionTime / totalElapsed : 0);
    printf("  ╚════════════════════════════════════════════════════════════════╝\n\n");

    // 每任务详情
    printf("  每任务详情:\n");
    printf("  任务  放置位      时间(s)  段数  最小距离mm  碰撞\n");
    printf("  ────  ──────────  ───────  ────  ──────────  ────\n");
    for (auto& tr : taskResults) {
        printf("  %-4d  %-10s  %7.3f    7    %8.1f    %d\n",
               tr.taskIndex, tr.description.substr(tr.description.find('[')+1,
               tr.description.find(']')-tr.description.find('[')-1).c_str(),
               tr.totalTime_s, tr.minSelfDist_m * 1000, tr.totalCollisions);
    }

    // 写统计摘要
    FILE* fpSum = fopen("data/rml_palletizing_summary.txt", "w");
    if (fpSum) {
        fprintf(fpSum, "# HR_S50-2000 码垛场景仿真摘要\n");
        fprintf(fpSum, "# 轨迹: 7段S曲线 | 碰撞: HRC | 优化: TSP 2-opt\n\n");
        fprintf(fpSum, "palletizing_positions: %d\n", numPositions);
        fprintf(fpSum, "total_tasks: %d\n", numPositions);
        fprintf(fpSum, "total_segments: %d\n", totalSegments);
        fprintf(fpSum, "total_motion_time_s: %.4f\n", totalMotionTime);
        fprintf(fpSum, "total_collisions: %d\n", totalCollisions);
        fprintf(fpSum, "global_min_dist_mm: %.2f\n", globalMinDist * 1000);
        fprintf(fpSum, "simulation_time_s: %.3f\n", totalElapsed);
        fprintf(fpSum, "tsp_dist_before_deg: %.2f\n", distBefore);
        fprintf(fpSum, "tsp_dist_after_deg: %.2f\n", distAfter);
        fprintf(fpSum, "tsp_improvement_pct: %.1f\n", (1.0 - distAfter/distBefore) * 100);
        fprintf(fpSum, "\n# Per-task:\n");
        for (auto& tr : taskResults) {
            fprintf(fpSum, "task%d: time=%.4f minDist=%.2f collisions=%d\n",
                    tr.taskIndex, tr.totalTime_s, tr.minSelfDist_m*1000, tr.totalCollisions);
        }
        fclose(fpSum);
    }

    printf("\n  输出文件:\n");
    printf("    data/rml_palletizing_trajectory.txt — 完整轨迹数据\n");
    printf("    data/rml_palletizing_profile.csv    — 碰撞距离曲线\n");
    printf("    data/rml_palletizing_summary.txt    — 统计摘要\n");

    printf("\n  总耗时: %.2f s\n", totalElapsed);

    printf("\n══════════════════════════════════════════════════════════════════\n");
    printf("  HR_S50-2000 码垛场景仿真完成\n");
    printf("══════════════════════════════════════════════════════════════════\n");

    return 0;
}
