/**
 * @file testCollisionSimulation.cpp
 * @brief S50机器人碰撞仿真 — S曲线轨迹 + HRC实时碰撞检测
 *
 * 使用项目内置的7段S曲线时间参数化(TimeParameterization.hpp)生成轨迹，
 * 结合HRC碰撞检测库进行实时碰撞距离监测。
 *
 * 注意: libCmpRML.so经反汇编分析确认需要CoDeSys PLC运行时(ComponentEntry循环)
 * 才能生成轨迹,此处使用功能等价的内置S曲线实现（五次多项式平滑）。
 *
 * ★ 路径点数说明: TimeParameterization对每对相邻航点独立做S曲线参数化,
 *   因此 point-to-point 运动应只传2个航点(起点+终点),多航点用于折线运动。
 *
 * @date 2026-02-06
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
#include <algorithm>
#include <numeric>
#include <vector>

using namespace palletizing;

// ============================================================================
// 辅助: 两点直达路径 (S曲线 point-to-point)
// ============================================================================
Path makeP2PPath(const JointConfig& start, const JointConfig& end) {
    Path path;
    Waypoint wp0(start); wp0.pathParam = 0.0;
    Waypoint wp1(end);   wp1.pathParam = 1.0;
    path.waypoints.push_back(wp0);
    path.waypoints.push_back(wp1);
    return path;
}

// ============================================================================
// 辅助: 多航点折线路径 (经过中间目标点)
// ============================================================================
Path makeWaypointPath(const std::vector<JointConfig>& configs) {
    Path path;
    for (size_t i = 0; i < configs.size(); i++) {
        Waypoint wp(configs[i]);
        wp.pathParam = (configs.size() > 1) ?
            static_cast<double>(i) / (configs.size() - 1) : 0.0;
        path.waypoints.push_back(wp);
    }
    return path;
}

// ============================================================================
// 辅助: 打印轨迹摘要统计
// ============================================================================
void printTrajectoryStats(const Trajectory& traj, const char* label) {
    if (traj.empty()) {
        printf("    [%s] 轨迹为空!\n", label);
        return;
    }

    JointVector maxVel = JointVector::Zero();
    JointVector maxAcc = JointVector::Zero();

    for (size_t i = 0; i < traj.size(); i++) {
        for (int j = 0; j < 6; j++) {
            maxVel[j] = std::max(maxVel[j], std::fabs(traj.points[i].velocity[j]));
            maxAcc[j] = std::max(maxAcc[j], std::fabs(traj.points[i].acceleration[j]));
        }
    }

    printf("    [%s] 轨迹统计:\n", label);
    printf("      总时长: %.3f s | 采样点: %zu | 周期: %.1f ms\n",
           traj.totalTime, traj.size(),
           traj.size() > 1 ? (traj.totalTime / (traj.size()-1)) * 1000 : 0);
    printf("      峰值速度 (°/s): [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]\n",
           maxVel[0]*180/M_PI, maxVel[1]*180/M_PI, maxVel[2]*180/M_PI,
           maxVel[3]*180/M_PI, maxVel[4]*180/M_PI, maxVel[5]*180/M_PI);
    printf("      峰值加速度 (°/s²): [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]\n",
           maxAcc[0]*180/M_PI, maxAcc[1]*180/M_PI, maxAcc[2]*180/M_PI,
           maxAcc[3]*180/M_PI, maxAcc[4]*180/M_PI, maxAcc[5]*180/M_PI);
}

// ============================================================================
// 辅助: 沿轨迹碰撞距离分析
// ============================================================================
struct CollisionProfile {
    std::vector<double> times;
    std::vector<double> minDistances;
    std::vector<bool>   selfCollisions;
    std::vector<bool>   envCollisions;
    double overallMinDist = 1e10;
    double overallMinDistTime = 0.0;
    int selfCollisionCount = 0;
    int envCollisionCount = 0;
};

CollisionProfile analyzeCollisionProfile(
    const Trajectory& traj,
    CollisionChecker& checker,
    int sampleInterval = 10)
{
    CollisionProfile profile;
    for (size_t i = 0; i < traj.size(); i += sampleInterval) {
        const auto& pt = traj.points[i];
        auto report = checker.getCollisionReport(pt.config);

        profile.times.push_back(pt.time);
        profile.minDistances.push_back(report.selfMinDistance);
        profile.selfCollisions.push_back(report.selfCollision);
        profile.envCollisions.push_back(report.envCollision);

        if (report.selfCollision) profile.selfCollisionCount++;
        if (report.envCollision)  profile.envCollisionCount++;
        if (report.selfMinDistance < profile.overallMinDist) {
            profile.overallMinDist = report.selfMinDistance;
            profile.overallMinDistTime = pt.time;
        }
    }
    return profile;
}

void printCollisionProfile(const CollisionProfile& cp, const char* label) {
    printf("    [%s] 碰撞分析:\n", label);
    printf("      采样点: %zu\n", cp.times.size());
    printf("      最小自碰撞距离: %.4f m (t=%.3fs)\n", cp.overallMinDist, cp.overallMinDistTime);
    printf("      自碰撞事件: %d\n", cp.selfCollisionCount);
    printf("      TCP区域入侵(envCollision): %d (含地面安全平面)\n", cp.envCollisionCount);
    if (cp.selfCollisionCount == 0)
        printf("      ✓ 全程无自碰撞\n");
    else
        printf("      ✗ 检测到自碰撞!\n");
}

// ============================================================================
// 测试1: 安全运动 — S曲线 point-to-point + 碰撞监测
// ============================================================================
void testSafeMotion() {
    printf("\n═══════════════════════════════════════════════════════════\n");
    printf("  测试1: 安全 P2P 运动 — S曲线轨迹 + 碰撞距离监测\n");
    printf("═══════════════════════════════════════════════════════════\n\n");

    RobotModel robot;
    CollisionChecker checker(robot);
    printf("  初始化碰撞检测(默认码垛场景)...\n");
    bool initOk = checker.initialize();
    printf("  碰撞检测初始化: %s\n\n", initOk ? "✓ 成功" : "✗ 失败");

    auto start = JointConfig::fromDegrees({0, -90, 0, 0, 90, 0});
    auto goal  = JointConfig::fromDegrees({45, -60, 30, 0, 60, 0});

    printf("  起始: [0, -90, 0, 0, 90, 0]°\n");
    printf("  目标: [45, -60, 30, 0, 60, 0]°\n");
    printf("  关节空间距离: %.1f°\n\n",
           start.distanceTo(goal) * 180.0 / M_PI);

    auto startReport = checker.getCollisionReport(start);
    auto goalReport = checker.getCollisionReport(goal);
    printf("  起始碰撞: self=%d selfDist=%.4fm\n",
           startReport.selfCollision, startReport.selfMinDistance);
    printf("  目标碰撞: self=%d selfDist=%.4fm\n\n",
           goalReport.selfCollision, goalReport.selfMinDistance);

    // ★ P2P路径 (只有起终点2个航点 → 一段完整S曲线)
    Path path = makeP2PPath(start, goal);

    auto tpConfig = TimeParameterizationConfig::fromRobotParams(robot.getParams());
    tpConfig.profileType = VelocityProfileType::SCurve;
    tpConfig.velocityScaling = 1.0;
    tpConfig.samplePeriod = 0.004;   // 4ms (机器人控制周期)

    TimeParameterizer parameterizer(tpConfig);

    auto t0 = std::chrono::high_resolution_clock::now();
    Trajectory traj = parameterizer.parameterize(path);
    auto t1 = std::chrono::high_resolution_clock::now();
    double paramTime = std::chrono::duration<double, std::milli>(t1 - t0).count();

    printf("  S曲线参数化耗时: %.3f ms\n", paramTime);
    printTrajectoryStats(traj, "安全P2P运动");

    TrajectoryValidator validator;
    bool valid = validator.validate(traj);
    printf("  轨迹验证(关节限位): %s\n\n", valid ? "✓ 通过" : "✗ 超限");

    // 碰撞分析(每10个采样点检测一次)
    auto t2 = std::chrono::high_resolution_clock::now();
    auto cp = analyzeCollisionProfile(traj, checker, 5);
    auto t3 = std::chrono::high_resolution_clock::now();
    double collTime = std::chrono::duration<double, std::milli>(t3 - t2).count();

    printCollisionProfile(cp, "安全P2P");
    printf("  碰撞分析耗时: %.2f ms (%zu次检测)\n", collTime, cp.times.size());

    // 输出轨迹数据
    FILE* fp = fopen("data/collision_sim_safe.txt", "w");
    if (fp) {
        fprintf(fp, "# 安全P2P运动 S曲线轨迹 + 碰撞距离\n");
        fprintf(fp, "# time_s q1_deg q2_deg q3_deg q4_deg q5_deg q6_deg ");
        fprintf(fp, "v1_deg_s v2_deg_s v3_deg_s v4_deg_s v5_deg_s v6_deg_s ");
        fprintf(fp, "a1_deg_ss a2_deg_ss a3_deg_ss a4_deg_ss a5_deg_ss a6_deg_ss selfDist_m\n");

        size_t cpIdx = 0;
        for (size_t i = 0; i < traj.size(); i++) {
            const auto& pt = traj.points[i];
            auto q = pt.config.toDegrees();
            fprintf(fp, "%.4f", pt.time);
            for (int j = 0; j < 6; j++) fprintf(fp, " %.6f", q[j]);
            for (int j = 0; j < 6; j++) fprintf(fp, " %.6f", pt.velocity[j] * 180.0 / M_PI);
            for (int j = 0; j < 6; j++) fprintf(fp, " %.6f", pt.acceleration[j] * 180.0 / M_PI);
            double dist = -1.0;
            if (cpIdx < cp.times.size() && std::fabs(pt.time - cp.times[cpIdx]) < 0.002) {
                dist = cp.minDistances[cpIdx];
                cpIdx++;
            }
            fprintf(fp, " %.6f\n", dist);
        }
        fclose(fp);
        printf("  轨迹数据已输出: data/collision_sim_safe.txt\n");
    }

    // TCP位置
    auto startPose = robot.forwardKinematics(start);
    auto goalPose = robot.forwardKinematics(goal);
    printf("\n  TCP起始(基座坐标系): [%.1f, %.1f, %.1f] mm\n",
           startPose.position.x()*1000, startPose.position.y()*1000, startPose.position.z()*1000);
    printf("  TCP目标(基座坐标系): [%.1f, %.1f, %.1f] mm\n",
           goalPose.position.x()*1000, goalPose.position.y()*1000, goalPose.position.z()*1000);
}

// ============================================================================
// 测试2: 大范围运动 — 自碰撞距离分布分析
// ============================================================================
void testLargeMotion() {
    printf("\n═══════════════════════════════════════════════════════════\n");
    printf("  测试2: 大范围 P2P 运动 — 自碰撞距离变化分析\n");
    printf("═══════════════════════════════════════════════════════════\n\n");

    RobotModel robot;
    CollisionChecker checker(robot);
    checker.initialize();

    auto start = JointConfig::fromDegrees({0, -90, 0, 0, 90, 0});
    auto goal  = JointConfig::fromDegrees({-120, -30, 120, 90, -45, 180});

    printf("  起始: [0, -90, 0, 0, 90, 0]°\n");
    printf("  目标: [-120, -30, 120, 90, -45, 180]°\n");
    double jointDist = start.distanceTo(goal);
    printf("  关节空间距离: %.1f° (%.3f rad)\n\n", jointDist * 180 / M_PI, jointDist);

    Path path = makeP2PPath(start, goal);  // P2P 一段式S曲线
    auto tpConfig = TimeParameterizationConfig::fromRobotParams(robot.getParams());
    tpConfig.profileType = VelocityProfileType::SCurve;
    tpConfig.velocityScaling = 1.0;
    tpConfig.samplePeriod = 0.004;

    auto t0 = std::chrono::high_resolution_clock::now();
    TimeParameterizer parameterizer(tpConfig);
    Trajectory traj = parameterizer.parameterize(path);
    auto t1 = std::chrono::high_resolution_clock::now();
    printf("  参数化耗时: %.3f ms\n", std::chrono::duration<double, std::milli>(t1 - t0).count());
    printTrajectoryStats(traj, "大范围P2P");

    auto cp = analyzeCollisionProfile(traj, checker, 1);
    printCollisionProfile(cp, "大范围P2P");

    if (!cp.minDistances.empty()) {
        double minD = *std::min_element(cp.minDistances.begin(), cp.minDistances.end());
        double maxD = *std::max_element(cp.minDistances.begin(), cp.minDistances.end());
        double avgD = std::accumulate(cp.minDistances.begin(), cp.minDistances.end(), 0.0)
                      / cp.minDistances.size();

        printf("\n    碰撞距离分布:\n");
        printf("      最小: %.4f m (%.1f mm) | 最大: %.4f m (%.1f mm)\n",
               minD, minD*1000, maxD, maxD*1000);
        printf("      平均: %.4f m (%.1f mm)\n", avgD, avgD*1000);

        int danger30 = 0, danger50 = 0;
        for (double d : cp.minDistances) {
            if (d < 0.030) danger30++;
            if (d < 0.050) danger50++;
        }
        printf("      危险点(<30mm): %d | 警告点(<50mm): %d / %zu\n",
               danger30, danger50, cp.minDistances.size());
    }

    FILE* fp = fopen("data/collision_sim_large.txt", "w");
    if (fp) {
        fprintf(fp, "# 大范围运动碰撞距离剖面\n");
        fprintf(fp, "# time_s min_dist_m self_collision\n");
        for (size_t i = 0; i < cp.times.size(); i++) {
            fprintf(fp, "%.4f %.6f %d\n",
                    cp.times[i], cp.minDistances[i], cp.selfCollisions[i] ? 1 : 0);
        }
        fclose(fp);
        printf("\n  碰撞数据已输出: data/collision_sim_large.txt\n");
    }
}

// ============================================================================
// 测试3: 障碍物环境 — OBB + 环境碰撞检测
// ============================================================================
void testObstacleEnvironment() {
    printf("\n═══════════════════════════════════════════════════════════\n");
    printf("  测试3: 障碍物环境 — OBB障碍物 + 环境碰撞检测\n");
    printf("═══════════════════════════════════════════════════════════\n\n");

    RobotModel robot;
    CollisionChecker checker(robot);
    SceneConfig scene = SceneConfig::defaultPalletizing();

    OBBObstacle workTable;
    workTable.id = 8;
    workTable.pose = Pose6D::fromEulerZYX(0.6, 0.0, 0.4, 0, 0, 0);
    workTable.lwh = Eigen::Vector3d(0.8, 1.2, 0.05);
    workTable.name = "WorkTable";
    scene.obstacles.push_back(workTable);

    OBBObstacle pillar;
    pillar.id = 9;
    pillar.pose = Pose6D::fromEulerZYX(0.5, 0.5, 0.3, 0, 0, 0);
    pillar.lwh = Eigen::Vector3d(0.1, 0.1, 0.6);
    pillar.name = "Pillar";
    scene.obstacles.push_back(pillar);

    printf("  场景: 码垛默认 + 工作台(0.8×1.2m @z=0.4m) + 支撑柱(0.1×0.1×0.6m)\n");
    bool initOk = checker.initialize(scene);
    printf("  碰撞检测初始化: %s\n\n", initOk ? "✓ 成功" : "✗ 失败");

    auto start = JointConfig::fromDegrees({0, -90, 0, 0, 90, 0});
    auto goal  = JointConfig::fromDegrees({90, -30, 60, 0, 30, 90});

    printf("  起始: [0, -90, 0, 0, 90, 0]°\n");
    printf("  目标: [90, -30, 60, 0, 30, 90]°\n\n");

    Path path = makeP2PPath(start, goal);
    auto tpConfig = TimeParameterizationConfig::fromRobotParams(robot.getParams());
    tpConfig.profileType = VelocityProfileType::SCurve;
    tpConfig.velocityScaling = 1.0;
    tpConfig.samplePeriod = 0.004;

    TimeParameterizer parameterizer(tpConfig);
    Trajectory traj = parameterizer.parameterize(path);
    printTrajectoryStats(traj, "障碍物P2P");

    auto cp = analyzeCollisionProfile(traj, checker, 1);
    printCollisionProfile(cp, "障碍物P2P");

    FILE* fp = fopen("data/collision_sim_obstacle.txt", "w");
    if (fp) {
        fprintf(fp, "# 障碍物环境碰撞检测剖面\n");
        fprintf(fp, "# time_s min_dist_m self_col env_col q1 q2 q3 q4 q5 q6\n");
        for (size_t i = 0; i < cp.times.size(); i++) {
            size_t idx = i;  // cp is 1:1 with traj since sampleInterval=1
            auto q = traj.points[idx].config.toDegrees();
            fprintf(fp, "%.4f %.6f %d %d %.4f %.4f %.4f %.4f %.4f %.4f\n",
                    cp.times[i], cp.minDistances[i],
                    cp.selfCollisions[i] ? 1 : 0, cp.envCollisions[i] ? 1 : 0,
                    q[0], q[1], q[2], q[3], q[4], q[5]);
        }
        fclose(fp);
        printf("\n  碰撞数据已输出: data/collision_sim_obstacle.txt\n");
    }
}

// ============================================================================
// 测试4: S曲线性能基准 (5个运动场景)
// ============================================================================
void testSCurvePerformance() {
    printf("\n═══════════════════════════════════════════════════════════\n");
    printf("  测试4: S曲线 P2P 性能基准 (5个运动场景)\n");
    printf("═══════════════════════════════════════════════════════════\n\n");

    RobotModel robot;
    CollisionChecker checker(robot);
    checker.initialize();

    struct TestCase { const char* name; double start[6]; double goal[6]; };
    TestCase cases[] = {
        {"短距离",     {0,-90,0,0,90,0}, {10,-85,5,0,85,0}},
        {"中距离",     {0,-90,0,0,90,0}, {45,-60,30,0,60,0}},
        {"远距离",     {0,-90,0,0,90,0}, {-120,-30,120,90,-45,180}},
        {"单轴J1",     {0,-90,0,0,90,0}, {180,-90,0,0,90,0}},
        {"三轴J123",   {0,-90,0,0,90,0}, {90,-45,90,0,90,0}},
    };

    auto tpConfig = TimeParameterizationConfig::fromRobotParams(robot.getParams());
    tpConfig.profileType = VelocityProfileType::SCurve;
    tpConfig.velocityScaling = 1.0;
    tpConfig.samplePeriod = 0.004;
    TimeParameterizer parameterizer(tpConfig);

    printf("  %-12s %8s %8s %8s %10s %10s %6s\n",
           "场景", "距离(°)", "时间(s)", "点数", "参数化ms", "碰撞ms", "自碰撞");
    printf("  ─────────── ──────── ──────── ──────── ────────── ────────── ──────\n");

    FILE* fp = fopen("data/collision_sim_benchmark.txt", "w");
    if (fp) fprintf(fp, "# S曲线P2P性能基准\n# name dist_deg time_s points param_ms collision_ms self_safe\n");

    for (auto& tc : cases) {
        auto s = JointConfig::fromDegrees({tc.start[0], tc.start[1], tc.start[2],
                                           tc.start[3], tc.start[4], tc.start[5]});
        auto g = JointConfig::fromDegrees({tc.goal[0], tc.goal[1], tc.goal[2],
                                           tc.goal[3], tc.goal[4], tc.goal[5]});
        double dist = s.distanceTo(g) * 180 / M_PI;
        Path path = makeP2PPath(s, g);  // ★ P2P 路径

        auto t0 = std::chrono::high_resolution_clock::now();
        Trajectory traj = parameterizer.parameterize(path);
        auto t1 = std::chrono::high_resolution_clock::now();
        double paramTime = std::chrono::duration<double, std::milli>(t1 - t0).count();

        auto t2 = std::chrono::high_resolution_clock::now();
        auto cp = analyzeCollisionProfile(traj, checker, 10);
        auto t3 = std::chrono::high_resolution_clock::now();
        double collTime = std::chrono::duration<double, std::milli>(t3 - t2).count();

        bool selfSafe = (cp.selfCollisionCount == 0);
        printf("  %-12s %8.1f %8.3f %8zu %10.3f %10.2f %s\n",
               tc.name, dist, traj.totalTime, traj.size(), paramTime, collTime,
               selfSafe ? "✓" : "✗");
        if (fp) fprintf(fp, "%s %.1f %.3f %zu %.3f %.2f %d\n",
                        tc.name, dist, traj.totalTime, traj.size(), paramTime, collTime, selfSafe?1:0);
    }
    if (fp) { fclose(fp); printf("\n  基准数据: data/collision_sim_benchmark.txt\n"); }
}

// ============================================================================
// 测试5: S曲线七段剖面 — 高精度速度/加速度/Jerk分析
// ============================================================================
void testSCurveDetailedProfile() {
    printf("\n═══════════════════════════════════════════════════════════\n");
    printf("  测试5: S曲线七段详细剖面 (高精度1ms采样)\n");
    printf("═══════════════════════════════════════════════════════════\n\n");

    RobotModel robot;
    auto start = JointConfig::fromDegrees({0, -90, 0, 0, 90, 0});
    auto goal  = JointConfig::fromDegrees({45, -60, 30, 0, 60, 0});

    Path path = makeP2PPath(start, goal);
    auto tpConfig = TimeParameterizationConfig::fromRobotParams(robot.getParams());
    tpConfig.profileType = VelocityProfileType::SCurve;
    tpConfig.velocityScaling = 1.0;
    tpConfig.samplePeriod = 0.001;  // 1ms高分辨率

    TimeParameterizer parameterizer(tpConfig);
    Trajectory traj = parameterizer.parameterize(path);

    printf("  高分辨率轨迹 (1ms采样):\n");
    printTrajectoryStats(traj, "高分辨率P2P");

    // 输出完整剖面 (J1, J2 两轴)
    FILE* fp = fopen("data/collision_sim_scurve_detail.txt", "w");
    if (fp) {
        fprintf(fp, "# S曲线七段详细剖面 (P2P, 1ms采样, °单位)\n");
        fprintf(fp, "# time_s q1 q2 v1 v2 a1 a2 jerk1 jerk2\n");
        for (size_t i = 0; i < traj.size(); i++) {
            const auto& pt = traj.points[i];
            auto q = pt.config.toDegrees();
            double v1 = pt.velocity[0]*180/M_PI, v2 = pt.velocity[1]*180/M_PI;
            double a1 = pt.acceleration[0]*180/M_PI, a2 = pt.acceleration[1]*180/M_PI;
            double j1=0, j2=0;
            if (i > 0) {
                double dt = pt.time - traj.points[i-1].time;
                if (dt > 0) {
                    j1 = (pt.acceleration[0]-traj.points[i-1].acceleration[0])/dt*180/M_PI;
                    j2 = (pt.acceleration[1]-traj.points[i-1].acceleration[1])/dt*180/M_PI;
                }
            }
            fprintf(fp, "%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",
                    pt.time, q[0], q[1], v1, v2, a1, a2, j1, j2);
        }
        fclose(fp);
        printf("  详细剖面数据: data/collision_sim_scurve_detail.txt\n");
    }

    // S曲线加速度段分析
    printf("\n  S曲线加速度切换点检测 (J1轴 — 最大运动距离45°):\n");
    printf("  %-12s %10s %12s %12s\n", "事件", "时间(s)", "加速度(°/s²)", "速度(°/s)");
    printf("  ──────────── ────────── ──────────── ────────────\n");

    int phaseId = 0;
    double prevAcc = 0;
    const char* phaseNames[] = {"加速起始", "加速恒定", "加速结束", "匀速",
                                "减速起始", "减速恒定", "减速结束", "停止"};
    for (size_t i = 1; i < traj.size(); i++) {
        double acc = traj.points[i].acceleration[0] * 180 / M_PI;
        double vel = traj.points[i].velocity[0] * 180 / M_PI;
        double dt = traj.points[i].time - traj.points[i-1].time;

        // 检测加速度发生显著变化 (>= 2°/s² 跳变)
        if (dt > 0 && std::fabs(acc - prevAcc) > 2.0 && phaseId < 8) {
            const char* phase = (phaseId < 8) ? phaseNames[phaseId] : "其他";
            printf("  %-12s %10.4f %12.2f %12.2f\n",
                   phase, traj.points[i].time, acc, vel);
            phaseId++;
        }
        prevAcc = acc;
    }

    printf("\n  五次多项式插值(smootherStep)特性:\n");
    printf("    s(τ) = 6τ⁵ - 15τ⁴ + 10τ³\n");
    printf("    ds/dτ|max = 30/16 = 1.875 (τ=0.5)\n");
    printf("    d²s/dτ²|max = 10√3/9 ≈ 1.925 (τ=(5±√5)/10)\n");
    printf("    → 速度峰值出现在运动中点, 加速度经过零点\n");
}

// ============================================================================
// 测试6: 多航点折线运动 + 碰撞监测
// ============================================================================
void testMultiWaypointMotion() {
    printf("\n═══════════════════════════════════════════════════════════\n");
    printf("  测试6: 多航点折线运动 — 经过4个中间姿态\n");
    printf("═══════════════════════════════════════════════════════════\n\n");

    RobotModel robot;
    CollisionChecker checker(robot);
    checker.initialize();

    // 模拟码垛顺序: HOME → 抓取预备 → 抓取 → 提升 → 码放 → HOME
    std::vector<JointConfig> waypoints = {
        JointConfig::fromDegrees({0, -90, 0, 0, 90, 0}),        // HOME
        JointConfig::fromDegrees({30, -60, 30, 0, 60, 0}),      // 抓取预备
        JointConfig::fromDegrees({30, -45, 45, 0, 45, 0}),      // 抓取位置
        JointConfig::fromDegrees({30, -70, 20, 0, 70, 0}),      // 提升
        JointConfig::fromDegrees({-30, -50, 40, 0, 55, 90}),    // 码放位置
        JointConfig::fromDegrees({0, -90, 0, 0, 90, 0}),        // 回HOME
    };

    printf("  航点数: %zu (含 HOME → 抓取 → 码放 → HOME)\n", waypoints.size());
    for (size_t i = 0; i < waypoints.size(); i++) {
        auto q = waypoints[i].toDegrees();
        printf("    WP%zu: [%.0f, %.0f, %.0f, %.0f, %.0f, %.0f]°\n",
               i, q[0], q[1], q[2], q[3], q[4], q[5]);
    }

    Path path = makeWaypointPath(waypoints);
    auto tpConfig = TimeParameterizationConfig::fromRobotParams(robot.getParams());
    tpConfig.profileType = VelocityProfileType::SCurve;
    tpConfig.velocityScaling = 1.0;
    tpConfig.samplePeriod = 0.004;

    TimeParameterizer parameterizer(tpConfig);

    auto t0 = std::chrono::high_resolution_clock::now();
    Trajectory traj = parameterizer.parameterize(path);
    auto t1 = std::chrono::high_resolution_clock::now();
    double paramTime = std::chrono::duration<double, std::milli>(t1 - t0).count();

    printf("\n  S曲线参数化耗时: %.3f ms\n", paramTime);
    printTrajectoryStats(traj, "折线运动");

    auto cp = analyzeCollisionProfile(traj, checker, 5);
    printCollisionProfile(cp, "折线运动");

    // 段时间分析
    printf("\n    各段运动时间:\n");
    double prevSegEnd = 0;
    for (size_t s = 0; s < waypoints.size() - 1; s++) {
        // 找到对应段的时间范围(通过pathParam)
        double segStart = -1, segEnd = -1;
        double targetParamStart = static_cast<double>(s) / (waypoints.size() - 1);
        double targetParamEnd = static_cast<double>(s + 1) / (waypoints.size() - 1);

        for (size_t i = 0; i < traj.size(); i++) {
            if (segStart < 0 && traj.points[i].pathParam >= targetParamStart - 0.01)
                segStart = traj.points[i].time;
            if (traj.points[i].pathParam >= targetParamEnd - 0.01) {
                segEnd = traj.points[i].time;
                break;
            }
        }
        if (segStart >= 0 && segEnd >= 0) {
            auto qs = waypoints[s].toDegrees();
            auto qe = waypoints[s + 1].toDegrees();
            double dist = waypoints[s].distanceTo(waypoints[s + 1]) * 180 / M_PI;
            printf("      段%zu→%zu: %.3f s (距离 %.1f°)\n",
                   s, s + 1, segEnd - segStart, dist);
        }
    }

    FILE* fp = fopen("data/collision_sim_waypoints.txt", "w");
    if (fp) {
        fprintf(fp, "# 多航点折线运动轨迹\n");
        fprintf(fp, "# time_s q1 q2 q3 q4 q5 q6 v1 v2 v3 v4 v5 v6 pathParam\n");
        for (size_t i = 0; i < traj.size(); i++) {
            const auto& pt = traj.points[i];
            auto q = pt.config.toDegrees();
            fprintf(fp, "%.4f", pt.time);
            for (int j = 0; j < 6; j++) fprintf(fp, " %.4f", q[j]);
            for (int j = 0; j < 6; j++) fprintf(fp, " %.4f", pt.velocity[j]*180/M_PI);
            fprintf(fp, " %.6f\n", pt.pathParam);
        }
        fclose(fp);
        printf("\n  轨迹数据: data/collision_sim_waypoints.txt\n");
    }
}

// ============================================================================
int main() {
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║   S50机器人碰撞仿真 — S曲线轨迹 + HRC碰撞检测               ║\n");
    printf("║   TimeParameterization (五次多项式S曲线) + CollisionChecker   ║\n");
    printf("║   采样周期: 4ms (等效libCmpRML.so控制周期)                    ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n");

    try {
        testSafeMotion();
        testLargeMotion();
        testObstacleEnvironment();
        testSCurvePerformance();
        testSCurveDetailedProfile();
        testMultiWaypointMotion();
    } catch (const std::exception& e) {
        printf("\n  [异常] %s\n", e.what());
        return 1;
    }

    printf("\n════════════════════════════════════════════════════════════\n");
    printf("  全部6项碰撞仿真测试完成\n");
    printf("════════════════════════════════════════════════════════════\n");
    return 0;
}
