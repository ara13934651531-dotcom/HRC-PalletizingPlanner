/**
 * @file testTrajectoryOptimality.cpp
 * @brief 系统性轨迹最优性测试 — Free-TCP RRT* 全面验证
 *
 * 测试内容:
 *   A. 无障碍基准性能 (多组起止配置, 统计路径长度/规划时间)
 *   B. 障碍物避障 (框架/电箱, 验证无碰撞+路径合理性)
 *   C. TCP-to-TCP 工作流 (IK求解 + 规划 + 全链计时)
 *   D. 参数敏感性 (迭代数/步长/goalBias 影响)
 *   E. 路径质量指标 (总长度/最大关节跳变/平均段长)
 *   F. 重复性验证 (同一对多次规划, 统计分布)
 *
 * @date 2026-02-24
 * @copyright Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.
 */

#include "PalletizingPlanner/RobotModel.hpp"
#include "PalletizingPlanner/CollisionCheckerSO.hpp"
#include "PalletizingPlanner/PathPlannerSO.hpp"
#include "PalletizingPlanner/PathOptimizer.hpp"
#include "PalletizingPlanner/TimeParameterization.hpp"
#include "PalletizingPlanner/Types.hpp"

#include <cstdio>
#include <cmath>
#include <chrono>
#include <vector>
#include <algorithm>
#include <numeric>
#include <array>
#include <string>
#include <random>

using namespace palletizing;

// ============================================================================
// 统计工具
// ============================================================================
struct Stats {
    double mean = 0, stddev = 0, min = 1e30, max = -1e30;
    int count = 0;
    std::vector<double> data;

    void add(double v) {
        data.push_back(v);
        count++;
        if (v < min) min = v;
        if (v > max) max = v;
    }
    void compute() {
        if (data.empty()) return;
        double sum = 0;
        for (auto v : data) sum += v;
        mean = sum / data.size();
        double sq = 0;
        for (auto v : data) sq += (v - mean) * (v - mean);
        stddev = std::sqrt(sq / data.size());
    }
    double median() const {
        if (data.empty()) return 0;
        auto sorted = data;
        std::sort(sorted.begin(), sorted.end());
        return sorted[sorted.size() / 2];
    }
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
        if (!checker.forwardKinematics(q, tcp)) return result;
        Eigen::Vector3d pos(tcp.X*1000, tcp.Y*1000, tcp.Z*1000);
        Eigen::Vector3d err = target_mm - pos;
        result.posError_mm = err.norm();
        if (result.posError_mm < tol_mm) { result.config = q; result.converged = true; return result; }

        Eigen::Matrix<double,3,6> J;
        for (int j = 0; j < 6; j++) {
            JointConfig qp = q; qp.q[j] += eps;
            SO_COORD_REF tp; checker.forwardKinematics(qp, tp);
            J.col(j) = (Eigen::Vector3d(tp.X*1000,tp.Y*1000,tp.Z*1000) - pos) / eps;
        }
        Eigen::Matrix3d JJT = J * J.transpose() + lambda*lambda*Eigen::Matrix3d::Identity();
        Eigen::Matrix<double,6,1> dq = J.transpose() * JJT.ldlt().solve(err);

        double maxStep = 0.08;
        double dqN = dq.norm();
        if (dqN > maxStep) dq *= maxStep / dqN;
        for (int j = 0; j < 6; j++)
            q.q[j] = std::clamp(q.q[j] + dq(j), jMin[j], jMax[j]);
    }
    result.config = q;
    return result;
}

// ============================================================================
// 测试用例定义
// ============================================================================
struct TestCase {
    std::string name;
    JointConfig start;
    JointConfig goal;
};

struct PlanResult {
    bool success;
    double planTime_ms;
    double optTime_ms;
    double totalTime_ms;
    double pathLength_rad;
    int iterations;
    int nodesExplored;
    int waypoints;
    double maxJointJump_rad;  // 最大单步关节跳变
    bool collisionFree;
};

PlanResult runOnePlan(PathPlannerSO& planner, CollisionCheckerSO& checker,
                      const JointConfig& start, const JointConfig& goal,
                      double collRes = 0.03)
{
    PlanResult r = {};
    auto t0 = std::chrono::high_resolution_clock::now();
    auto pr = planner.plan(start, goal);
    auto t1 = std::chrono::high_resolution_clock::now();
    r.totalTime_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    auto timing = planner.getTimingReport();
    r.planTime_ms = timing.planningTotal_ms;
    r.optTime_ms = timing.optimizationTotal_ms;
    r.success = pr.isSuccess();
    r.iterations = timing.planIterations;
    r.nodesExplored = timing.nodesExplored;

    if (r.success && !pr.optimizedPath.empty()) {
        auto& path = pr.optimizedPath;
        r.waypoints = (int)path.waypoints.size();
        r.pathLength_rad = path.totalLength();

        // 最大关节跳变
        r.maxJointJump_rad = 0;
        for (size_t i = 1; i < path.waypoints.size(); i++) {
            double d = path.waypoints[i-1].config.distanceTo(path.waypoints[i].config);
            if (d > r.maxJointJump_rad) r.maxJointJump_rad = d;
        }

        // 碰撞验证
        r.collisionFree = true;
        for (size_t i = 0; i + 1 < path.waypoints.size(); i++) {
            if (!checker.isPathCollisionFree(path.waypoints[i].config,
                                              path.waypoints[i+1].config, collRes)) {
                r.collisionFree = false;
                break;
            }
        }
    }
    return r;
}

// ============================================================================
// 测试A: 无障碍基准性能
// ============================================================================
void testA_FreeSpaceBaseline(RobotModel& robot, CollisionCheckerSO& checker) {
    printf("\n╔══════════════════════════════════════════════════════════════════╗\n");
    printf("║  测试A: 无障碍基准性能 — 多组起止配置统计                       ║\n");
    printf("╚══════════════════════════════════════════════════════════════════╝\n\n");

    TCPPlannerConfig cfg;
    cfg.maxIterations = 3000;
    cfg.maxPlanningTime = 2.0;
    cfg.stepSize = 0.15;
    cfg.goalBias = 0.2;
    cfg.shortcutIterations = 50;
    cfg.splineResolution = 30;
    cfg.collisionResolution = 0.03;
    cfg.freeTcpDuringTransit = true;
    PathPlannerSO planner(robot, checker, cfg);

    // 典型码垛工作空间配置对 (deg)
    double cases[][12] = {
        // 起 (J1-J6),                         止 (J1-J6)
        { 0,-90,  0,0, 90,0,    0,-50, 30,0, 20,0},   // HOME → 工作
        { 0,-50, 30,0, 20,0,  -90,-50, 40,0, 10,-90},  // 工作 → 取料
        {-90,-50, 40,0, 10,-90,   0,-65, 25,0, 40, 0},  // 取料 → 放料
        { 0,-65, 25,0, 40,0,    0,-70, 20,0, 50, 0},   // 放料A → 放料B
        { 0,-70, 20,0, 50,0,    0,-90,  0,0, 90, 0},   // 工作 → HOME
        { 0,-60, 25,0, 35,0,   10,-65, 20,0, 45,10},   // 近距离
        {-10,-55, 35,0, 20,-10, -90,-50, 40,0, 10,-90},  // 大范围
        { 15,-60, 30,0, 30,15, -15,-75, 15,0, 60,-15},  // 对角
    };
    int numCases = sizeof(cases)/sizeof(cases[0]);

    Stats timeStat, lengthStat, iterStat;
    int successes = 0;
    int collFreeCount = 0;

    printf("  %-6s %-25s %8s %8s %8s %6s %6s %5s\n",
           "编号", "配置对", "规划ms", "优化ms", "路径rad", "迭代", "节点", "碰撞");

    for (int i = 0; i < numCases; i++) {
        auto start = JointConfig::fromDegrees({cases[i][0],cases[i][1],cases[i][2],
                                                cases[i][3],cases[i][4],cases[i][5]});
        auto goal  = JointConfig::fromDegrees({cases[i][6],cases[i][7],cases[i][8],
                                                cases[i][9],cases[i][10],cases[i][11]});

        auto r = runOnePlan(planner, checker, start, goal);
        char desc[64];
        snprintf(desc, sizeof(desc), "[%.0f,%.0f,..]→[%.0f,%.0f,..]",
                 cases[i][0],cases[i][1], cases[i][6],cases[i][7]);

        if (r.success) {
            successes++;
            if (r.collisionFree) collFreeCount++;
            timeStat.add(r.planTime_ms);
            lengthStat.add(r.pathLength_rad);
            iterStat.add(r.iterations);
        }

        printf("  [%2d]  %-25s %8.2f %8.2f %8.3f %6d %6d %s\n",
               i+1, desc, r.planTime_ms, r.optTime_ms, r.pathLength_rad,
               r.iterations, r.nodesExplored, r.collisionFree ? "✅" : "❌");
    }

    timeStat.compute();
    lengthStat.compute();
    iterStat.compute();

    printf("\n  ── 统计 ──\n");
    printf("  成功率: %d/%d (%.1f%%)\n", successes, numCases, 100.0*successes/numCases);
    printf("  碰撞安全: %d/%d\n", collFreeCount, successes);
    printf("  规划时间: 平均=%.2fms 中位=%.2fms σ=%.2fms [%.2f, %.2f]\n",
           timeStat.mean, timeStat.median(), timeStat.stddev, timeStat.min, timeStat.max);
    printf("  路径长度: 平均=%.3frad 中位=%.3frad σ=%.3frad\n",
           lengthStat.mean, lengthStat.median(), lengthStat.stddev);
    printf("  迭代次数: 平均=%.0f 中位=%.0f\n", iterStat.mean, iterStat.median());
}

// ============================================================================
// 测试B: 障碍物避障
// ============================================================================
void testB_ObstacleAvoidance(RobotModel& robot, CollisionCheckerSO& checker) {
    printf("\n╔══════════════════════════════════════════════════════════════════╗\n");
    printf("║  测试B: 障碍物避障 — 框架+电箱环境下路径质量                    ║\n");
    printf("╚══════════════════════════════════════════════════════════════════╝\n\n");

    // 添加码垛环境障碍物
    printf("  添加障碍物...\n");
    double baseZ = 800;  // 电箱高度

    // 框架立柱 (4 capsules)
    double fHW = 600, fNY = 425, fFY = 1075, fZB = -baseZ, fZT = 1200, fR = 50;
    int frmIds[] = {30,31,32,33};
    double frmX[] = {-fHW, fHW, fHW, -fHW};
    double frmY[] = {fNY, fNY, fFY, fFY};
    for (int i=0;i<4;i++) {
        checker.addEnvObstacleCapsule(frmIds[i],
            Eigen::Vector3d(frmX[i],frmY[i],fZB), Eigen::Vector3d(frmX[i],frmY[i],fZT), fR);
    }

    // 电箱 (base obstacle)
    checker.addEnvObstacleCapsule(10,
        Eigen::Vector3d(-275,-325,-baseZ), Eigen::Vector3d(-275,325,-80), 80);
    checker.addEnvObstacleCapsule(11,
        Eigen::Vector3d(275,-325,-baseZ), Eigen::Vector3d(275,325,-80), 80);
    printf("  框架(4) + 电箱(2) 已添加\n");

    // 启用环境碰撞
    bool f[7] = {true,true,true,true,true,true,true};
    checker.setLinkEnvCollisionEnabled(f);

    TCPPlannerConfig cfg;
    cfg.maxIterations = 5000;
    cfg.maxPlanningTime = 3.0;
    cfg.stepSize = 0.15;
    cfg.goalBias = 0.15;
    cfg.shortcutIterations = 80;
    cfg.splineResolution = 40;
    cfg.collisionResolution = 0.02;
    cfg.freeTcpDuringTransit = true;
    PathPlannerSO planner(robot, checker, cfg);

    // 测试: 需要绕过障碍物的配置对
    struct ObsCase {
        const char* name;
        double start[6], goal[6];
    };
    ObsCase cases[] = {
        {"安全→框架后方", {0,-70,40,0,30,0}, {0,-65,25,0,40,0}},
        {"框架前→框架后", {0,-55,35,0,20,0}, {0,-75,15,0,60,0}},
        {"取料→框架内放料", {-90,-50,40,0,10,-90}, {5,-60,30,0,30,5}},
        {"放料→取料(大范围)", {10,-65,20,0,45,10}, {-90,-50,40,0,10,-90}},
        {"框架左→框架右", {-10,-65,25,0,40,-10}, {10,-65,25,0,40,10}},
    };
    int numCases = sizeof(cases)/sizeof(cases[0]);

    Stats timeStat, lengthStat;
    int successes = 0, collFree = 0;

    printf("\n  %-6s %-22s %8s %8s %8s %6s %6s %5s\n",
           "编号", "场景", "规划ms", "优化ms", "路径rad", "节点", "路点", "安全");

    for (int i = 0; i < numCases; i++) {
        auto s = JointConfig::fromDegrees({cases[i].start[0],cases[i].start[1],cases[i].start[2],
                                           cases[i].start[3],cases[i].start[4],cases[i].start[5]});
        auto g = JointConfig::fromDegrees({cases[i].goal[0],cases[i].goal[1],cases[i].goal[2],
                                           cases[i].goal[3],cases[i].goal[4],cases[i].goal[5]});

        auto r = runOnePlan(planner, checker, s, g);
        if (r.success) {
            successes++;
            if (r.collisionFree) collFree++;
            timeStat.add(r.planTime_ms + r.optTime_ms);
            lengthStat.add(r.pathLength_rad);
        }

        printf("  [%2d]  %-22s %8.2f %8.2f %8.3f %6d %6d %s\n",
               i+1, cases[i].name, r.planTime_ms, r.optTime_ms,
               r.pathLength_rad, r.nodesExplored, r.waypoints,
               r.collisionFree ? "✅" : (r.success ? "⚠️" : "❌"));
    }

    timeStat.compute();
    lengthStat.compute();

    printf("\n  ── 统计 ──\n");
    printf("  成功率: %d/%d  碰撞安全: %d/%d\n", successes, numCases, collFree, successes);
    if (successes > 0) {
        printf("  总时间: 平均=%.2fms [%.2f, %.2f]\n", timeStat.mean, timeStat.min, timeStat.max);
        printf("  路径长: 平均=%.3frad [%.3f, %.3f]\n", lengthStat.mean, lengthStat.min, lengthStat.max);
    }

    // 清理环境障碍
    for (int id : {10,11,30,31,32,33})
        checker.removeEnvObstacle(id);
}

// ============================================================================
// 测试C: TCP-to-TCP 规划工作流
// ============================================================================
void testC_TcpToTcpWorkflow(RobotModel& robot, CollisionCheckerSO& checker) {
    printf("\n╔══════════════════════════════════════════════════════════════════╗\n");
    printf("║  测试C: TCP-to-TCP 规划 — IK求解 + 路径规划 + 全链计时          ║\n");
    printf("╚══════════════════════════════════════════════════════════════════╝\n\n");

    TCPPlannerConfig cfg;
    cfg.maxIterations = 5000;
    cfg.maxPlanningTime = 3.0;
    cfg.stepSize = 0.15;
    cfg.goalBias = 0.2;
    cfg.shortcutIterations = 60;
    cfg.collisionResolution = 0.03;
    cfg.freeTcpDuringTransit = true;
    PathPlannerSO planner(robot, checker, cfg);

    // TCP 目标位置 (mm, 基座坐标系)
    struct TcpTarget {
        const char* name;
        Eigen::Vector3d pos_mm;
    };
    TcpTarget targets[] = {
        {"传送带取料",   Eigen::Vector3d(575, -300, 115)},
        {"框架内放料L1", Eigen::Vector3d(-175, 772, -50)},
        {"框架内放料L2", Eigen::Vector3d(175, 772, 200)},
        {"框架内放料L3", Eigen::Vector3d(-175, 492, -50)},
    };

    // IK Seeds
    std::vector<JointConfig> seeds;
    double sA[][6] = {
        {0,-65,25,0,40,0}, {-90,-50,40,0,10,-90}, {0,-70,20,0,50,0},
        {5,-60,30,0,30,5}, {-5,-60,30,0,30,-5}, {10,-65,20,0,45,10},
    };
    for (auto& s : sA) seeds.push_back(JointConfig::fromDegrees({s[0],s[1],s[2],s[3],s[4],s[5]}));

    printf("  %-20s %8s %8s %8s %8s %8s\n",
           "TCP目标", "IK_ms", "IK误差mm", "规划ms", "路径rad", "状态");

    JointConfig prevConfig = JointConfig::fromDegrees({0,-90,0,0,90,0}); // HOME

    for (int i = 0; i < 4; i++) {
        auto& tgt = targets[i];

        // IK
        auto tIK = std::chrono::high_resolution_clock::now();
        IKResult bestIK;
        bestIK.posError_mm = 1e10;
        for (auto& seed : seeds) {
            auto r = numericalIK(checker, tgt.pos_mm, seed, 3.0);
            if (r.converged && r.posError_mm < bestIK.posError_mm) bestIK = r;
            if (bestIK.converged && bestIK.posError_mm < 1.0) break;
        }
        double ikMs = std::chrono::duration<double,std::milli>(
            std::chrono::high_resolution_clock::now() - tIK).count();

        if (!bestIK.converged) {
            printf("  %-20s %8.1f %8.1f %8s %8s ❌ IK失败\n",
                   tgt.name, ikMs, bestIK.posError_mm, "--", "--");
            continue;
        }

        // 规划
        auto r = runOnePlan(planner, checker, prevConfig, bestIK.config);

        printf("  %-20s %8.1f %8.2f %8.2f %8.3f %s\n",
               tgt.name, ikMs, bestIK.posError_mm, r.planTime_ms + r.optTime_ms,
               r.pathLength_rad, r.success ? (r.collisionFree ? "✅" : "⚠️碰撞") : "❌规划失败");

        if (r.success) prevConfig = bestIK.config;

        // 将成功IK解加入seeds池
        if (bestIK.converged) seeds.insert(seeds.begin(), bestIK.config);
        if (seeds.size() > 15) seeds.pop_back();
    }
}

// ============================================================================
// 测试D: 参数敏感性分析
// ============================================================================
void testD_ParameterSensitivity(RobotModel& robot, CollisionCheckerSO& checker) {
    printf("\n╔══════════════════════════════════════════════════════════════════╗\n");
    printf("║  测试D: 参数敏感性 — 迭代数/步长/goalBias 影响分析              ║\n");
    printf("╚══════════════════════════════════════════════════════════════════╝\n\n");

    // 固定测试配置对
    auto start = JointConfig::fromDegrees({-90,-50,40,0,10,-90});
    auto goal  = JointConfig::fromDegrees({5,-60,30,0,30,5});

    struct ParamSet {
        const char* label;
        int maxIter;
        double stepSize;
        double goalBias;
    };
    ParamSet params[] = {
        {"默认",       3000, 0.15, 0.20},
        {"多迭代",     8000, 0.15, 0.20},
        {"少迭代",     1000, 0.15, 0.20},
        {"大步长",     3000, 0.30, 0.20},
        {"小步长",     3000, 0.08, 0.20},
        {"高goalBias", 3000, 0.15, 0.40},
        {"低goalBias", 3000, 0.15, 0.05},
    };
    int numParams = sizeof(params)/sizeof(params[0]);

    printf("  %-14s %6s %7s %9s  |  %8s %8s %6s\n",
           "参数组", "迭代", "步长", "goalBias", "规划ms", "路径rad", "成功");

    for (int p = 0; p < numParams; p++) {
        TCPPlannerConfig cfg;
        cfg.maxIterations = params[p].maxIter;
        cfg.maxPlanningTime = 5.0;
        cfg.stepSize = params[p].stepSize;
        cfg.goalBias = params[p].goalBias;
        cfg.shortcutIterations = 50;
        cfg.collisionResolution = 0.03;
        cfg.freeTcpDuringTransit = true;
        PathPlannerSO planner(robot, checker, cfg);

        // 3次取最优
        double bestLen = 1e30, bestTime = 1e30;
        int succ = 0;
        for (int trial = 0; trial < 3; trial++) {
            auto r = runOnePlan(planner, checker, start, goal);
            if (r.success) {
                succ++;
                if (r.pathLength_rad < bestLen) bestLen = r.pathLength_rad;
                if (r.planTime_ms < bestTime) bestTime = r.planTime_ms;
            }
        }

        printf("  %-14s %6d %7.2f %9.2f  |  %8.2f %8.3f %d/3\n",
               params[p].label, params[p].maxIter, params[p].stepSize, params[p].goalBias,
               bestTime, bestLen < 1e10 ? bestLen : 0.0, succ);
    }
}

// ============================================================================
// 测试E: 路径质量详细分析
// ============================================================================
void testE_PathQualityAnalysis(RobotModel& robot, CollisionCheckerSO& checker) {
    printf("\n╔══════════════════════════════════════════════════════════════════╗\n");
    printf("║  测试E: 路径质量详细分析 — 长度/跳变/段长/碰撞距离              ║\n");
    printf("╚══════════════════════════════════════════════════════════════════╝\n\n");

    TCPPlannerConfig cfg;
    cfg.maxIterations = 5000;
    cfg.maxPlanningTime = 3.0;
    cfg.stepSize = 0.15;
    cfg.goalBias = 0.2;
    cfg.shortcutIterations = 80;
    cfg.splineResolution = 40;
    cfg.collisionResolution = 0.02;
    cfg.freeTcpDuringTransit = true;
    PathPlannerSO planner(robot, checker, cfg);

    // 代表性配置对
    auto start = JointConfig::fromDegrees({0,-90,0,0,90,0});
    auto goal  = JointConfig::fromDegrees({0,-65,25,0,40,0});

    auto pr = planner.plan(start, goal);
    auto timing = planner.getTimingReport();

    printf("  全流水线分层计时:\n");
    printf("%s\n", timing.toString().c_str());

    if (!pr.isSuccess()) {
        printf("  ❌ 规划失败: %s\n", pr.errorMessage.c_str());
        return;
    }

    auto& raw = pr.rawPath;
    auto& opt = pr.optimizedPath;

    printf("  ── 路径对比 ──\n");
    printf("  %-12s %8s %8s %10s\n", "", "原始路径", "优化路径", "改善");
    printf("  %-12s %8d %8d\n", "路点数", (int)raw.waypoints.size(), (int)opt.waypoints.size());
    double rawLen = raw.totalLength(), optLen = opt.totalLength();
    printf("  %-12s %8.3f %8.3f %9.1f%%\n", "总长度(rad)", rawLen, optLen,
           rawLen > 0 ? (1.0 - optLen/rawLen)*100 : 0);

    // 路径采样点TCP位姿分析
    printf("\n  ── TCP位姿变化 (优化路径, 每5路点采样) ──\n");
    printf("  %-5s %8s %8s %8s %8s %8s %8s\n",
           "序号", "X_mm", "Y_mm", "Z_mm", "A_deg", "B_deg", "C_deg");

    int step = std::max(1, (int)opt.waypoints.size() / 10);
    for (size_t i = 0; i < opt.waypoints.size(); i += step) {
        SO_COORD_REF tcp;
        checker.forwardKinematics(opt.waypoints[i].config, tcp);
        printf("  [%3zu] %8.1f %8.1f %8.1f %8.1f %8.1f %8.1f\n",
               i, tcp.X*1000, tcp.Y*1000, tcp.Z*1000, tcp.A, tcp.B, tcp.C);
    }
    // 最后一个路点
    if (opt.waypoints.size() > 1) {
        SO_COORD_REF tcp;
        checker.forwardKinematics(opt.waypoints.back().config, tcp);
        printf("  [END] %8.1f %8.1f %8.1f %8.1f %8.1f %8.1f\n",
               tcp.X*1000, tcp.Y*1000, tcp.Z*1000, tcp.A, tcp.B, tcp.C);
    }

    // 碰撞距离分析
    printf("\n  ── 碰撞距离分析 (优化路径) ──\n");
    double minSelf = 1e10;
    int collCount = 0;
    for (size_t i = 0; i < opt.waypoints.size(); i++) {
        auto rp = checker.getCollisionReport(opt.waypoints[i].config, true);
        if (rp.selfCollision) collCount++;
        if (rp.selfMinDist_mm < minSelf) minSelf = rp.selfMinDist_mm;
    }
    printf("  最小自碰撞距离: %.1fmm  碰撞路点: %d/%d\n",
           minSelf, collCount, (int)opt.waypoints.size());

    // 关节跳变分析
    printf("\n  ── 关节跳变分析 (优化路径) ──\n");
    double maxJump[6] = {0}, avgJump[6] = {0};
    for (size_t i = 1; i < opt.waypoints.size(); i++) {
        for (int j = 0; j < 6; j++) {
            double d = std::abs(opt.waypoints[i].config.q[j] - opt.waypoints[i-1].config.q[j]);
            if (d > maxJump[j]) maxJump[j] = d;
            avgJump[j] += d;
        }
    }
    int segs = std::max(1, (int)opt.waypoints.size() - 1);
    printf("  %-6s", "");
    for (int j=0;j<6;j++) printf(" %8s", (std::string("J")+std::to_string(j+1)).c_str());
    printf("\n  最大跳变(deg):");
    for (int j=0;j<6;j++) printf(" %8.2f", maxJump[j]*180/M_PI);
    printf("\n  平均跳变(deg):");
    for (int j=0;j<6;j++) printf(" %8.2f", avgJump[j]/segs*180/M_PI);
    printf("\n");
}

// ============================================================================
// 测试F: 重复性验证
// ============================================================================
void testF_Repeatability(RobotModel& robot, CollisionCheckerSO& checker) {
    printf("\n╔══════════════════════════════════════════════════════════════════╗\n");
    printf("║  测试F: 重复性验证 — 同配置多次规划, 统计一致性                 ║\n");
    printf("╚══════════════════════════════════════════════════════════════════╝\n\n");

    TCPPlannerConfig cfg;
    cfg.maxIterations = 3000;
    cfg.maxPlanningTime = 2.0;
    cfg.stepSize = 0.15;
    cfg.goalBias = 0.2;
    cfg.shortcutIterations = 50;
    cfg.collisionResolution = 0.03;
    cfg.freeTcpDuringTransit = true;
    PathPlannerSO planner(robot, checker, cfg);

    auto start = JointConfig::fromDegrees({-90,-50,40,0,10,-90});
    auto goal  = JointConfig::fromDegrees({0,-65,25,0,40,0});

    const int N = 20;
    Stats timeStat, lengthStat;
    int succ = 0, collFree = 0;

    printf("  配置: [取料]→[放料]  重复 %d 次\n\n", N);
    printf("  %-5s %8s %8s %8s %6s %5s\n", "次数", "规划ms", "优化ms", "路径rad", "节点", "安全");

    for (int i = 0; i < N; i++) {
        auto r = runOnePlan(planner, checker, start, goal);
        if (r.success) {
            succ++;
            if (r.collisionFree) collFree++;
            timeStat.add(r.planTime_ms + r.optTime_ms);
            lengthStat.add(r.pathLength_rad);
        }
        printf("  [%3d] %8.2f %8.2f %8.3f %6d %s\n",
               i+1, r.planTime_ms, r.optTime_ms, r.pathLength_rad, r.nodesExplored,
               r.collisionFree ? "✅" : (r.success ? "⚠️" : "❌"));
    }

    timeStat.compute();
    lengthStat.compute();

    printf("\n  ── 统计 (N=%d) ──\n", N);
    printf("  成功率: %d/%d (%.1f%%)  碰撞安全: %d/%d\n",
           succ, N, 100.0*succ/N, collFree, succ);
    printf("  规划时间: μ=%.2fms σ=%.2fms 中位=%.2fms [%.2f, %.2f]\n",
           timeStat.mean, timeStat.stddev, timeStat.median(), timeStat.min, timeStat.max);
    printf("  路径长度: μ=%.3frad σ=%.3frad 中位=%.3frad [%.3f, %.3f]\n",
           lengthStat.mean, lengthStat.stddev, lengthStat.median(), lengthStat.min, lengthStat.max);
    printf("  变异系数: 时间=%.1f%%  路径=%.1f%%\n",
           timeStat.mean > 0 ? timeStat.stddev/timeStat.mean*100 : 0,
           lengthStat.mean > 0 ? lengthStat.stddev/lengthStat.mean*100 : 0);
}

// ============================================================================
// 主函数
// ============================================================================
int main() {
    printf("╔═══════════════════════════════════════════════════════════════════╗\n");
    printf("║   HR_S50-2000 系统性轨迹最优性测试 — Free-TCP RRT              ║\n");
    printf("║   纯关节空间代价 + 运动中TCP自由变化 + 全面性能分析             ║\n");
    printf("╚═══════════════════════════════════════════════════════════════════╝\n\n");

    auto t_global = std::chrono::high_resolution_clock::now();

    // 初始化
    RobotModel robot;
    CollisionCheckerSO checker(robot);
    auto tInit = std::chrono::high_resolution_clock::now();
    if (!checker.initialize()) {
        fprintf(stderr, "❌ 碰撞检测初始化失败! 请设置 HRC_LIB_PATH 环境变量\n");
        return 1;
    }
    double initMs = std::chrono::duration<double,std::milli>(
        std::chrono::high_resolution_clock::now()-tInit).count();
    printf("  碰撞检测: ✅ libHRCInterface.so (%.2f ms)\n", initMs);
    printf("  模式: Free-TCP (运动中TCP可变, 纯关节空间代价)\n\n");

    // 运行所有测试
    testA_FreeSpaceBaseline(robot, checker);
    testB_ObstacleAvoidance(robot, checker);
    testC_TcpToTcpWorkflow(robot, checker);
    testD_ParameterSensitivity(robot, checker);
    testE_PathQualityAnalysis(robot, checker);
    testF_Repeatability(robot, checker);

    // 总结
    double elapsed = std::chrono::duration<double>(
        std::chrono::high_resolution_clock::now()-t_global).count();

    printf("\n══════════════════════════════════════════════════════════════════\n");
    printf("  全部测试完成  总耗时: %.2fs\n", elapsed);

    auto stats = checker.getTimingStats();
    printf("\n  碰撞检测统计:\n%s", stats.toString().c_str());
    printf("══════════════════════════════════════════════════════════════════\n");

    return 0;
}
