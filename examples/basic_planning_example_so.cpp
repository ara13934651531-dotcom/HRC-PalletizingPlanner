/**
 * @file basic_planning_example_so.cpp
 * @brief 基础路径规划示例 (SO栈) | Basic Path Planning Example (SO Stack)
 *
 * 基于 CollisionCheckerSO + PathPlannerSO 的路径规划示例。
 * 使用 dlopen 动态加载 libHRCInterface.so，无需链接静态库。
 *
 * 编译: g++ -std=c++17 -I../include -I/usr/include/eigen3 basic_planning_example_so.cpp -o example -lstdc++ -lm -lpthread -ldl
 * 运行: HRC_LIB_PATH=../lib/libHRCInterface.so ./example
 *
 * @copyright Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.
 * @date 2026-02-25
 */

#include "PalletizingPlanner/RobotModel.hpp"
#include "PalletizingPlanner/CollisionCheckerSO.hpp"
#include "PalletizingPlanner/PathPlannerSO.hpp"
#include "PalletizingPlanner/TimeParameterization.hpp"
#include "PalletizingPlanner/NumericalIK.hpp"
#include "PalletizingPlanner/Types.hpp"

#include <cstdio>
#include <chrono>

using namespace palletizing;

int main() {
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║   Huayan Robotics - SO Stack Planning Example            ║\n");
    printf("║   HR_S50-2000 Free-TCP RRT* Path Planning                ║\n");
    printf("╚══════════════════════════════════════════════════════════╝\n\n");

    // 1. 初始化
    RobotModel robot;
    CollisionCheckerSO checker(robot);
    
    printf("📦 初始化碰撞检测器...\n");
    if (!checker.initialize()) {
        fprintf(stderr, "❌ 初始化失败! 请设置 HRC_LIB_PATH 环境变量\n");
        fprintf(stderr, "   示例: export HRC_LIB_PATH=/path/to/libHRCInterface.so\n");
        return 1;
    }
    printf("✅ 初始化成功\n\n");

    // 2. 配置规划器
    TCPPlannerConfig cfg;
    cfg.maxIterations = 3000;
    cfg.maxPlanningTime = 3.0;   // 最大3秒
    cfg.stepSize = 0.15;
    cfg.goalBias = 0.2;
    cfg.shortcutIterations = 50;
    cfg.splineResolution = 30;
    cfg.collisionResolution = 0.03;
    cfg.constrainTcpHorizontal = true;  // 码埚模式: TCP保持水平
    
    PathPlannerSO planner(robot, checker, cfg);

    // 3. 定义起止配置 (度 → 内部自动转弧度)
    auto start = JointConfig::fromDegrees({0, -90, 0, 0, 90, 0});     // HOME
    auto goal  = JointConfig::fromDegrees({0, -65, 25, 0, 40, 0});    // 工作位

    printf("🔧 规划路径...\n");
    printf("  起点: [0, -90, 0, 0, 90, 0] deg\n");
    printf("  终点: [0, -65, 25, 0, 40, 0] deg\n\n");

    // 4. 规划
    auto t0 = std::chrono::high_resolution_clock::now();
    auto result = planner.plan(start, goal);
    auto t1 = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double, std::milli>(t1 - t0).count();

    if (!result.isSuccess()) {
        fprintf(stderr, "❌ 规划失败: %s\n", result.errorMessage.c_str());
        return 1;
    }

    printf("✅ 规划成功!\n");
    printf("  路径长度: %.3f rad\n", result.pathLength);
    printf("  路点数: %zu (原始) → %zu (优化)\n",
           result.rawPath.waypoints.size(), result.optimizedPath.waypoints.size());
    printf("  总耗时: %.2f ms\n", elapsed);

    // 5. 分层计时
    auto timing = planner.getTimingReport();
    printf("\n%s", timing.toString().c_str());

    // 6. 时间参数化
    TimeParameterizer parameterizer;
    auto trajectory = parameterizer.parameterize(result.optimizedPath);
    printf("  轨迹: %zu 采样点, %.2f 秒\n", trajectory.size(), trajectory.totalTime);

    // 7. TCP-to-TCP 示例 (IK求解)
    printf("\n🎯 TCP-to-TCP 规划示例:\n");
    Eigen::Vector3d target_mm(575, -300, 115);  // 传送带位置
    
    std::vector<JointConfig> seeds = {
        JointConfig::fromDegrees({0, -65, 25, 0, 40, 0}),
        JointConfig::fromDegrees({-90, -50, 40, 0, 10, -90})
    };
    
    auto ikResult = multiStartIK(checker, robot, target_mm, seeds, 3.0);
    if (ikResult.converged) {
        printf("  IK收敛: 误差=%.2f mm, 迭代=%d\n", ikResult.posError_mm, ikResult.iterations);
        
        auto tcpResult = planner.plan(goal, ikResult.config);
        if (tcpResult.isSuccess()) {
            printf("  TCP规划成功: 路径=%.3f rad\n", tcpResult.pathLength);
        }
    } else {
        printf("  IK未收敛: 误差=%.2f mm\n", ikResult.posError_mm);
    }

    printf("\n✅ 示例完成\n");
    return 0;
}
