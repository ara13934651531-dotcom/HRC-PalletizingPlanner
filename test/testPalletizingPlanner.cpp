/**
 * @file testPalletizingPlanner.cpp
 * @brief 码垛规划器测试程序
 * 
 * 测试HR_S50-2000协作机器人在码垛场景下的运动规划功能。
 * 
 * @author GitHub Copilot
 * @version 1.0.0
 * @date 2026-01-29
 */

#include <iostream>
#include <iomanip>
#include <chrono>
#include <fstream>
#include <cmath>

// 码垛规划器头文件
#include "PalletizingPlanner/Types.hpp"
#include "PalletizingPlanner/RobotModel.hpp"
#include "PalletizingPlanner/CollisionChecker.hpp"
#include "PalletizingPlanner/PathPlanner.hpp"
#include "PalletizingPlanner/PathOptimizer.hpp"
#include "PalletizingPlanner/PalletizingPlanner.hpp"
#include "PalletizingPlanner/TimeParameterization.hpp"
#include "PalletizingPlanner/TaskSequencer.hpp"

using namespace palletizing;

// ANSI颜色代码
#define COLOR_RESET   "\033[0m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_RED     "\033[31m"
#define COLOR_CYAN    "\033[36m"

/**
 * @brief 打印分隔线
 */
void printSeparator(const std::string& title = "") {
    std::cout << COLOR_CYAN;
    std::cout << "═══════════════════════════════════════════════════════════════\n";
    if (!title.empty()) {
        std::cout << "  " << title << "\n";
        std::cout << "═══════════════════════════════════════════════════════════════\n";
    }
    std::cout << COLOR_RESET;
}

/**
 * @brief 打印关节配置
 */
void printJointConfig(const std::string& name, const JointConfig& config) {
    std::cout << COLOR_BLUE << name << ": " << COLOR_RESET;
    std::cout << "[";
    for (int i = 0; i < 6; ++i) {
        std::cout << std::fixed << std::setprecision(2) 
                  << config.q[i] * 180.0 / M_PI << "°";
        if (i < 5) std::cout << ", ";
    }
    std::cout << "]\n";
}

/**
 * @brief 打印规划统计信息
 */
void printPlanningStats(const PlanningResult& result) {
    std::cout << COLOR_GREEN << "规划统计:" << COLOR_RESET << "\n";
    std::cout << "  状态: " << result.statusString() << "\n";
    std::cout << "  规划耗时: " << std::fixed << std::setprecision(3) 
              << result.planningTime << " s\n";
    std::cout << "  优化耗时: " << result.optimizationTime << " s\n";
    std::cout << "  迭代次数: " << result.iterations << "\n";
    std::cout << "  探索节点: " << result.nodesExplored << "\n";
    std::cout << "  路径长度: " << std::setprecision(4) << result.pathLength << " rad\n";
    std::cout << "  最大曲率: " << result.maxCurvature << "\n";
    std::cout << "  原始路径点数: " << result.rawPath.waypoints.size() << "\n";
    std::cout << "  优化路径点数: " << result.optimizedPath.waypoints.size() << "\n";
}

/**
 * @brief 保存路径到文件用于可视化
 */
void savePathForVisualization(const Path& path, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << COLOR_RED << "无法创建文件: " << filename << COLOR_RESET << "\n";
        return;
    }
    
    file << "# Palletizing Path Visualization Data\n";
    file << "# Format: index q1 q2 q3 q4 q5 q6 (degrees)\n";
    
    for (size_t i = 0; i < path.waypoints.size(); ++i) {
        file << i;
        for (int j = 0; j < 6; ++j) {
            file << " " << std::fixed << std::setprecision(4)
                 << path.waypoints[i].config.q[j] * 180.0 / M_PI;
        }
        file << "\n";
    }
    
    file.close();
    std::cout << COLOR_GREEN << "路径已保存到: " << filename << COLOR_RESET << "\n";
}

/**
 * @brief 保存B-Spline采样到文件
 */
void saveBSplineForVisualization(const BSpline& spline, const std::string& filename,
                                  int samples = 200) {
    std::ofstream file(filename);
    if (!file.is_open()) return;
    
    file << "# B-Spline Sampled Path\n";
    file << "# Format: t q1 q2 q3 q4 q5 q6 (degrees)\n";
    
    for (int i = 0; i < samples; ++i) {
        double t = static_cast<double>(i) / (samples - 1);
        JointConfig config = spline.evaluate(t);
        
        file << std::fixed << std::setprecision(4) << t;
        for (int j = 0; j < 6; ++j) {
            file << " " << config.q[j] * 180.0 / M_PI;
        }
        file << "\n";
    }
    
    file.close();
    std::cout << COLOR_GREEN << "B-Spline已保存到: " << filename << COLOR_RESET << "\n";
}

/**
 * @brief 测试1: 机器人模型验证
 */
void testRobotModel() {
    printSeparator("测试1: HR_S50-2000 机器人模型验证");
    
    RobotDHParams params = RobotDHParams::fromHRConfig();
    RobotModel robot(params);
    
    std::cout << "\n" << COLOR_YELLOW << "DH参数 (mm):" << COLOR_RESET << "\n";
    std::cout << "  d1=" << params.d1 << ", d2=" << params.d2 
              << ", d3=" << params.d3 << ", d4=" << params.d4
              << ", d5=" << params.d5 << ", d6=" << params.d6 << "\n";
    std::cout << "  a2=" << params.a2 << ", a3=" << params.a3 << "\n";
    std::cout << "  最大负载: " << params.maxPayload << " kg\n";
    
    // 测试正运动学
    JointConfig homeConfig({0, -M_PI/2, 0, 0, 0, 0});
    Pose6D homePose = robot.forwardKinematics(homeConfig);
    
    std::cout << "\n" << COLOR_YELLOW << "正运动学测试:" << COLOR_RESET << "\n";
    printJointConfig("Home配置", homeConfig);
    std::cout << "  TCP位置: (" << std::fixed << std::setprecision(3)
              << homePose.position.x() * 1000 << ", "
              << homePose.position.y() * 1000 << ", "
              << homePose.position.z() * 1000 << ") mm\n";
    
    // 测试关节限位
    std::cout << "\n" << COLOR_YELLOW << "关节限位 (degrees):" << COLOR_RESET << "\n";
    auto [minLim, maxLim] = params.getJointLimits();
    for (int i = 0; i < 6; ++i) {
        std::cout << "  J" << (i+1) << ": [" 
                  << minLim[i] * 180/M_PI << ", " 
                  << maxLim[i] * 180/M_PI << "]\n";
    }
    
    // 测试雅可比矩阵
    auto jacobian = robot.computeJacobian(homeConfig);
    double manip = robot.manipulability(homeConfig);
    std::cout << "\n可操作度: " << std::setprecision(6) << manip << "\n";
}

/**
 * @brief 测试2: 碰撞检测验证
 */
void testCollisionChecker() {
    printSeparator("测试2: 碰撞检测系统验证");
    
    RobotModel robot;
    CollisionChecker checker(robot);
    
    // 初始化默认码垛场景
    SceneConfig scene = SceneConfig::defaultPalletizing();
    std::cout << "\n" << COLOR_YELLOW << "场景配置:" << COLOR_RESET << "\n";
    std::cout << "  平台高度: " << scene.platformHeight * 1000 << " mm\n";
    std::cout << "  工作空间半径: " << scene.workspaceRadius * 1000 << " mm\n";
    std::cout << "  码垛区域数: " << scene.palletZones.size() << "\n";
    std::cout << "  安全平面数: " << scene.safePlanes.size() << "\n";
    
    bool initOk = checker.initialize(scene);
    std::cout << "\n碰撞检测初始化: " 
              << (initOk ? COLOR_GREEN "成功" : COLOR_RED "失败") 
              << COLOR_RESET << "\n";
    
    // 测试几个典型配置
    std::vector<std::pair<std::string, JointConfig>> testConfigs = {
        {"零位", JointConfig({0, 0, 0, 0, 0, 0})},
        {"Home", JointConfig({0, -M_PI/2, 0, 0, 0, 0})},
        {"伸展", JointConfig({0, -M_PI/4, -M_PI/4, 0, 0, 0})},
        {"折叠", JointConfig({0, -M_PI, M_PI/2, 0, 0, 0})}
    };
    
    std::cout << "\n" << COLOR_YELLOW << "配置碰撞测试:" << COLOR_RESET << "\n";
    for (const auto& [name, config] : testConfigs) {
        bool collisionFree = checker.isCollisionFree(config);
        double distance = checker.getCollisionDistance(config);
        
        std::cout << "  " << name << ": " 
                  << (collisionFree ? COLOR_GREEN "无碰撞" : COLOR_RED "碰撞!")
                  << COLOR_RESET 
                  << " (距离: " << std::fixed << std::setprecision(4) 
                  << distance * 1000 << " mm)\n";
    }
}

/**
 * @brief 测试3: 路径规划器
 */
void testPathPlanner() {
    printSeparator("测试3: Informed RRT* 路径规划器");
    
    RobotModel robot;
    CollisionChecker checker(robot);
    checker.initialize();
    
    PlannerConfig config;
    config.plannerType = PlannerType::InformedRRTStar;
    config.maxIterations = 10000;
    config.maxPlanningTime = 5.0;
    config.stepSize = 0.15;
    config.goalBias = 0.1;
    config.useInformedSampling = true;
    
    PathPlanner planner(robot, checker, config);
    
    // 定义测试用例
    JointConfig start = JointConfig::fromDegrees({0, -90, 45, 0, -45, 0});
    JointConfig goal = JointConfig::fromDegrees({90, -60, 30, 45, -30, 90});
    
    std::cout << "\n" << COLOR_YELLOW << "规划配置:" << COLOR_RESET << "\n";
    std::cout << "  算法: Informed RRT*\n";
    std::cout << "  最大迭代: " << config.maxIterations << "\n";
    std::cout << "  时间限制: " << config.maxPlanningTime << " s\n";
    std::cout << "  步长: " << config.stepSize << " rad\n";
    
    printJointConfig("\n起点", start);
    printJointConfig("终点", goal);
    
    std::cout << "\n" << COLOR_YELLOW << "开始规划..." << COLOR_RESET << "\n";
    
    auto startTime = std::chrono::high_resolution_clock::now();
    PlanningResult result = planner.plan(start, goal);
    auto endTime = std::chrono::high_resolution_clock::now();
    
    double elapsed = std::chrono::duration<double>(endTime - startTime).count();
    
    std::cout << "\n";
    if (result.isSuccess()) {
        std::cout << COLOR_GREEN << "✓ 规划成功!" << COLOR_RESET << "\n\n";
        printPlanningStats(result);
        
        // 保存原始路径
        savePathForVisualization(result.rawPath, "data/raw_path.txt");
    } else {
        std::cout << COLOR_RED << "✗ 规划失败: " << result.statusString() << COLOR_RESET << "\n";
    }
}

/**
 * @brief 测试4: 路径优化器
 */
void testPathOptimizer() {
    printSeparator("测试4: 路径优化与B-Spline平滑");
    
    RobotModel robot;
    CollisionChecker checker(robot);
    checker.initialize();
    
    PlannerConfig config;
    config.shortcutIterations = 300;
    config.smoothIterations = 50;
    config.splineDegree = 5;
    config.splineResolution = 150;
    
    PathPlanner planner(robot, checker, config);
    PathOptimizer optimizer(robot, checker, config);
    
    // 规划初始路径
    JointConfig start = JointConfig::fromDegrees({0, -90, 30, 0, -60, 0});
    JointConfig goal = JointConfig::fromDegrees({-60, -45, 60, 30, -30, 45});
    
    printJointConfig("起点", start);
    printJointConfig("终点", goal);
    
    std::cout << "\n" << COLOR_YELLOW << "步骤1: 规划原始路径..." << COLOR_RESET << "\n";
    PlanningResult planResult = planner.plan(start, goal);
    
    if (!planResult.isSuccess()) {
        std::cout << COLOR_RED << "规划失败!" << COLOR_RESET << "\n";
        return;
    }
    
    std::cout << "  原始路径点数: " << planResult.rawPath.waypoints.size() << "\n";
    std::cout << "  原始路径长度: " << std::fixed << std::setprecision(4) 
              << planResult.rawPath.totalLength() << " rad\n";
    
    std::cout << "\n" << COLOR_YELLOW << "步骤2: 优化路径..." << COLOR_RESET << "\n";
    
    auto startOpt = std::chrono::high_resolution_clock::now();
    PlanningResult optResult = optimizer.optimize(planResult.rawPath);
    auto endOpt = std::chrono::high_resolution_clock::now();
    
    double optTime = std::chrono::duration<double>(endOpt - startOpt).count();
    
    std::cout << "  优化耗时: " << std::setprecision(3) << optTime << " s\n";
    std::cout << "  优化后路径点数: " << optResult.optimizedPath.waypoints.size() << "\n";
    std::cout << "  优化后路径长度: " << std::setprecision(4) 
              << optResult.pathLength << " rad\n";
    
    double improvement = (1.0 - optResult.pathLength / planResult.rawPath.totalLength()) * 100;
    std::cout << "  路径长度改善: " << COLOR_GREEN << std::setprecision(1) 
              << improvement << "%" << COLOR_RESET << "\n";
    
    std::cout << "\n" << COLOR_YELLOW << "步骤3: B-Spline分析..." << COLOR_RESET << "\n";
    std::cout << "  样条阶数: " << optResult.smoothedSpline.degree << "\n";
    std::cout << "  控制点数: " << optResult.smoothedSpline.controlPoints.size() << "\n";
    std::cout << "  最大曲率: " << std::setprecision(6) << optResult.maxCurvature << "\n";
    
    // 保存可视化数据
    savePathForVisualization(planResult.rawPath, "data/path_raw.txt");
    savePathForVisualization(optResult.optimizedPath, "data/path_optimized.txt");
    saveBSplineForVisualization(optResult.smoothedSpline, "data/path_spline.txt");
}

/**
 * @brief 测试5: 完整码垛规划
 */
void testFullPalletizing() {
    printSeparator("测试5: 完整码垛任务规划");
    
    // 创建码垛规划器
    PalletizingPlanner planner;
    
    // 配置场景
    SceneConfig scene = SceneConfig::defaultPalletizing();
    planner.initialize(scene);
    
    // 配置规划参数
    PlannerConfig config;
    config.plannerType = PlannerType::InformedRRTStar;
    config.maxIterations = 15000;
    config.maxPlanningTime = 8.0;
    config.shortcutIterations = 400;
    config.splineDegree = 5;
    planner.setConfig(config);
    
    // 配置工具
    ToolConfig tool = ToolConfig::palletizingSuction();
    planner.setToolConfig(tool);
    
    std::cout << "\n" << COLOR_YELLOW << "场景已配置:" << COLOR_RESET << "\n";
    std::cout << "  机器人: HR_S50-2000\n";
    std::cout << "  工具: 吸盘末端执行器\n";
    std::cout << "  工作区: " << scene.workspaceRadius * 1000 << " mm 半径\n";
    
    // 创建码垛任务
    PalletizingTask task;
    task.pickConfig = JointConfig::fromDegrees({45, -60, 45, 0, -75, 0});
    task.placeConfig = JointConfig::fromDegrees({-45, -45, 30, 30, -60, 90});
    task.approachOffset = 0.1;
    task.retractOffset = 0.15;
    task.description = "Pick box from conveyor, place on pallet";
    
    printJointConfig("\n抓取位置", task.pickConfig);
    printJointConfig("放置位置", task.placeConfig);
    
    std::cout << "\n" << COLOR_YELLOW << "开始规划码垛任务..." << COLOR_RESET << "\n";
    
    auto startTime = std::chrono::high_resolution_clock::now();
    PalletizingResult result = planner.planPickAndPlace(task);
    auto endTime = std::chrono::high_resolution_clock::now();
    
    double totalTime = std::chrono::duration<double>(endTime - startTime).count();
    
    std::cout << "\n";
    if (result.success) {
        std::cout << COLOR_GREEN << "✓ 码垛规划成功!" << COLOR_RESET << "\n\n";
        std::cout << "统计信息:\n";
        std::cout << "  总规划时间: " << std::fixed << std::setprecision(3) 
                  << result.totalPlanningTime << " s\n";
        std::cout << "  完整路径点数: " << result.completePath.waypoints.size() << "\n";
        std::cout << "  路径总长度: " << std::setprecision(4) 
                  << result.totalPathLength << " rad\n";
        std::cout << "  最大曲率: " << result.maxCurvature << "\n";
        
        // 保存结果
        planner.savePathToFile(result.completePath, "data/palletizing_path.txt");
        planner.saveSplineToFile(result.smoothedSpline, "data/palletizing_spline.txt");
    } else {
        std::cout << COLOR_RED << "✗ 规划失败: " << result.errorMessage << COLOR_RESET << "\n";
    }
}

/**
 * @brief 测试6: 性能基准测试
 */
void testPerformanceBenchmark() {
    printSeparator("测试6: 性能基准测试");
    
    RobotModel robot;
    CollisionChecker checker(robot);
    checker.initialize();
    
    const int numTrials = 10;
    
    std::cout << "\n" << COLOR_YELLOW << "运行 " << numTrials << " 次随机规划测试..." << COLOR_RESET << "\n\n";
    
    std::vector<double> planTimes;
    std::vector<double> pathLengths;
    std::vector<int> iterations;
    int successCount = 0;
    
    PlannerConfig config;
    config.maxIterations = 5000;
    config.maxPlanningTime = 3.0;
    
    PathPlanner planner(robot, checker, config);
    PathOptimizer optimizer(robot, checker, config);
    
    for (int trial = 0; trial < numTrials; ++trial) {
        // 生成随机起点和终点
        JointConfig start, goal;
        int attempts = 0;
        
        do {
            start = robot.randomConfig();
            attempts++;
        } while (!checker.isCollisionFree(start) && attempts < 100);
        
        attempts = 0;
        do {
            goal = robot.randomConfig();
            attempts++;
        } while (!checker.isCollisionFree(goal) && attempts < 100);
        
        auto startTime = std::chrono::high_resolution_clock::now();
        PlanningResult result = planner.plan(start, goal);
        
        if (result.isSuccess()) {
            result = optimizer.optimize(result.rawPath);
        }
        
        auto endTime = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(endTime - startTime).count();
        
        std::cout << "  试验 " << std::setw(2) << (trial + 1) << ": ";
        
        if (result.isSuccess()) {
            successCount++;
            planTimes.push_back(elapsed);
            pathLengths.push_back(result.pathLength);
            iterations.push_back(result.iterations);
            
            std::cout << COLOR_GREEN << "成功" << COLOR_RESET
                      << " | 时间: " << std::fixed << std::setprecision(3) << elapsed << "s"
                      << " | 路径: " << std::setprecision(2) << result.pathLength << " rad\n";
        } else {
            std::cout << COLOR_RED << "失败" << COLOR_RESET 
                      << " | " << result.statusString() << "\n";
        }
    }
    
    // 统计
    std::cout << "\n" << COLOR_YELLOW << "性能统计:" << COLOR_RESET << "\n";
    std::cout << "  成功率: " << successCount << "/" << numTrials 
              << " (" << (100.0 * successCount / numTrials) << "%)\n";
    
    if (!planTimes.empty()) {
        double avgTime = std::accumulate(planTimes.begin(), planTimes.end(), 0.0) / planTimes.size();
        double minTime = *std::min_element(planTimes.begin(), planTimes.end());
        double maxTime = *std::max_element(planTimes.begin(), planTimes.end());
        
        std::cout << "  平均规划时间: " << std::fixed << std::setprecision(3) << avgTime << " s\n";
        std::cout << "  最短时间: " << minTime << " s\n";
        std::cout << "  最长时间: " << maxTime << " s\n";
        
        double avgLength = std::accumulate(pathLengths.begin(), pathLengths.end(), 0.0) / pathLengths.size();
        std::cout << "  平均路径长度: " << std::setprecision(4) << avgLength << " rad\n";
    }
}

/**
 * @brief 测试7: 时间参数化
 */
void testTimeParameterization() {
    printSeparator("测试7: S曲线时间参数化");
    
    RobotModel robot;
    CollisionChecker checker(robot);
    checker.initialize();
    
    PathPlanner planner(robot, checker);
    PathOptimizer optimizer(robot, checker);
    
    // 规划一条路径
    JointConfig start = JointConfig::fromDegrees({0, -90, 30, 0, -60, 0});
    JointConfig goal = JointConfig::fromDegrees({45, -60, 45, 30, -45, 45});
    
    PlanningResult planResult = planner.plan(start, goal);
    
    if (!planResult.isSuccess()) {
        std::cout << COLOR_RED << "路径规划失败" << COLOR_RESET << "\n";
        return;
    }
    
    PlanningResult optResult = optimizer.optimize(planResult.rawPath);
    
    std::cout << "\n" << COLOR_YELLOW << "路径信息:" << COLOR_RESET << "\n";
    std::cout << "  优化路径点数: " << optResult.optimizedPath.waypoints.size() << "\n";
    std::cout << "  路径长度: " << std::fixed << std::setprecision(4) 
              << optResult.pathLength << " rad\n";
    
    // 时间参数化
    TimeParameterizationConfig tpConfig = TimeParameterizationConfig::fromRobotParams(robot.getParams());
    tpConfig.profileType = VelocityProfileType::SCurve;
    tpConfig.velocityScaling = 0.7;
    tpConfig.samplePeriod = 0.004;  // 4ms
    
    TimeParameterizer parameterizer(tpConfig);
    
    std::cout << "\n" << COLOR_YELLOW << "时间参数化 (S曲线):" << COLOR_RESET << "\n";
    
    auto startTime = std::chrono::high_resolution_clock::now();
    Trajectory trajectory = parameterizer.parameterize(optResult.optimizedPath);
    auto endTime = std::chrono::high_resolution_clock::now();
    
    double elapsed = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    
    std::cout << "  参数化耗时: " << std::setprecision(2) << elapsed << " ms\n";
    std::cout << "  轨迹总时间: " << std::setprecision(3) << trajectory.totalTime << " s\n";
    std::cout << "  轨迹点数: " << trajectory.points.size() << "\n";
    
    // 验证轨迹
    TrajectoryValidator validator(robot.getParams());
    auto [maxVel, maxAcc] = validator.getMaxValues(trajectory);
    
    std::cout << "\n" << COLOR_YELLOW << "轨迹动力学:" << COLOR_RESET << "\n";
    std::cout << "  最大速度 (deg/s): [";
    for (int i = 0; i < 6; ++i) {
        std::cout << std::setprecision(1) << maxVel[i] * 180 / M_PI;
        if (i < 5) std::cout << ", ";
    }
    std::cout << "]\n";
    
    std::cout << "  最大加速度 (deg/s²): [";
    for (int i = 0; i < 6; ++i) {
        std::cout << std::setprecision(1) << maxAcc[i] * 180 / M_PI;
        if (i < 5) std::cout << ", ";
    }
    std::cout << "]\n";
    
    bool valid = validator.validate(trajectory);
    std::cout << "\n  约束验证: " << (valid ? COLOR_GREEN "通过" : COLOR_RED "失败") 
              << COLOR_RESET << "\n";
    
    // 保存轨迹
    std::ofstream file("data/trajectory.txt");
    if (file.is_open()) {
        file << "# Trajectory with time\n";
        file << "# Format: time q1 q2 q3 q4 q5 q6 v1 v2 v3 v4 v5 v6 (rad, rad/s)\n";
        
        for (const auto& pt : trajectory.points) {
            file << std::fixed << std::setprecision(4) << pt.time;
            for (int i = 0; i < 6; ++i) file << " " << pt.config.q[i];
            for (int i = 0; i < 6; ++i) file << " " << pt.velocity[i];
            file << "\n";
        }
        file.close();
        std::cout << COLOR_GREEN << "轨迹已保存到: trajectory.txt" << COLOR_RESET << "\n";
    }
}

/**
 * @brief 主函数
 */
int main(int argc, char** argv) {
    std::cout << "\n";
    printSeparator("HR_S50-2000 协作机器人码垛运动规划测试");
    std::cout << "\n";
    std::cout << "  版本: 1.0.0\n";
    std::cout << "  日期: 2026-01-29\n";
    std::cout << "  算法: Informed RRT* + B-Spline优化\n";
    std::cout << "\n";
    
    try {
        // 运行所有测试
        testRobotModel();
        std::cout << "\n";
        
        testCollisionChecker();
        std::cout << "\n";
        
        testPathPlanner();
        std::cout << "\n";
        
        testPathOptimizer();
        std::cout << "\n";
        
        testFullPalletizing();
        std::cout << "\n";
        
        testPerformanceBenchmark();
        std::cout << "\n";
        
        // 新增：时间参数化测试
        testTimeParameterization();
        
    } catch (const std::exception& e) {
        std::cerr << COLOR_RED << "\n异常: " << e.what() << COLOR_RESET << "\n";
        return 1;
    }
    
    printSeparator("所有测试完成");
    std::cout << "\n生成的文件:\n";
    std::cout << "  - data/raw_path.txt          原始规划路径\n";
    std::cout << "  - data/path_raw.txt          优化前路径\n";
    std::cout << "  - data/path_optimized.txt    优化后路径\n";
    std::cout << "  - data/path_spline.txt       B-Spline采样路径\n";
    std::cout << "  - data/palletizing_path.txt  码垛任务路径\n";
    std::cout << "  - data/palletizing_spline.txt 码垛B-Spline路径\n";
    std::cout << "\n";
    
    return 0;
}
