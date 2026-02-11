/**
 * @file testHighPerformance.cpp
 * @brief 高性能规划器性能测试
 * 
 * 对比优化前后的运动规划性能，验证世界顶尖水平。
 * 
 * @author GitHub Copilot
 * @version 2.0.0
 * @date 2026-01-29
 */

#include <iostream>
#include <iomanip>
#include <chrono>
#include <vector>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <fstream>

// 原始规划器
#include "PalletizingPlanner/Types.hpp"
#include "PalletizingPlanner/RobotModel.hpp"
#include "PalletizingPlanner/CollisionChecker.hpp"
#include "PalletizingPlanner/PathPlanner.hpp"
#include "PalletizingPlanner/PathOptimizer.hpp"

// 高性能规划器
#include "PalletizingPlanner/KDTree.hpp"
#include "PalletizingPlanner/CollisionCache.hpp"
#include "PalletizingPlanner/PathPlannerOptimized.hpp"
#include "PalletizingPlanner/PathOptimizerOptimized.hpp"
#include "PalletizingPlanner/TimeParameterizationOptimized.hpp"
#include "PalletizingPlanner/HighPerformancePlanner.hpp"

using namespace palletizing;

// ANSI颜色
#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define CYAN    "\033[36m"
#define BOLD    "\033[1m"

/**
 * @brief 打印漂亮的表头
 */
void printHeader(const std::string& title) {
    std::cout << "\n" << CYAN << BOLD;
    std::cout << "╔══════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  " << std::setw(64) << std::left << title << "║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════╝\n";
    std::cout << RESET;
}

/**
 * @brief 测试结果结构
 */
struct TestResult {
    std::string name;
    int trials;
    int successes;
    double minTime;
    double maxTime;
    double avgTime;
    double stdTime;
    double minLength;
    double maxLength;
    double avgLength;
    
    double successRate() const { return 100.0 * successes / trials; }
};

/**
 * @brief 打印测试结果对比
 */
void printComparison(const TestResult& original, const TestResult& optimized) {
    std::cout << "\n" << YELLOW << "┌────────────────────────────────────────────────────────────────┐\n";
    std::cout << "│                    PERFORMANCE COMPARISON                      │\n";
    std::cout << "├────────────────────┬──────────────────┬──────────────────┬──────┤\n";
    std::cout << "│ Metric             │ Original         │ Optimized        │ Gain │\n";
    std::cout << "├────────────────────┼──────────────────┼──────────────────┼──────┤\n" << RESET;
    
    // 成功率
    std::cout << "│ Success Rate       │ " 
              << std::setw(14) << std::fixed << std::setprecision(1) << original.successRate() << "% │ "
              << std::setw(14) << optimized.successRate() << "% │      │\n";
    
    // 平均时间
    double timeSpeedup = original.avgTime / std::max(optimized.avgTime, 0.001);
    std::cout << "│ Avg Time (ms)      │ " 
              << std::setw(15) << std::setprecision(2) << original.avgTime * 1000 << " │ "
              << std::setw(15) << optimized.avgTime * 1000 << " │ ";
    if (timeSpeedup > 1) {
        std::cout << GREEN << std::setw(4) << std::setprecision(1) << timeSpeedup << "x" << RESET;
    } else {
        std::cout << RED << std::setw(4) << timeSpeedup << "x" << RESET;
    }
    std::cout << " │\n";
    
    // 最小时间
    std::cout << "│ Min Time (ms)      │ " 
              << std::setw(15) << original.minTime * 1000 << " │ "
              << std::setw(15) << optimized.minTime * 1000 << " │      │\n";
    
    // 最大时间
    std::cout << "│ Max Time (ms)      │ " 
              << std::setw(15) << original.maxTime * 1000 << " │ "
              << std::setw(15) << optimized.maxTime * 1000 << " │      │\n";
    
    // 平均路径长度
    double lengthImprove = (original.avgLength - optimized.avgLength) / original.avgLength * 100;
    std::cout << "│ Avg Path Length    │ " 
              << std::setw(15) << std::setprecision(4) << original.avgLength << " │ "
              << std::setw(15) << optimized.avgLength << " │ ";
    if (lengthImprove > 0) {
        std::cout << GREEN << std::setw(3) << std::setprecision(0) << lengthImprove << "%" << RESET;
    } else {
        std::cout << "    ";
    }
    std::cout << " │\n";
    
    std::cout << YELLOW << "└────────────────────┴──────────────────┴──────────────────┴──────┘\n" << RESET;
}

/**
 * @brief 测试1: KD-Tree性能
 */
void testKDTreePerformance() {
    printHeader("TEST 1: KD-Tree Performance");
    
    std::cout << "\n生成10000个随机点进行测试...\n";
    
    // 生成随机点
    std::vector<JointConfig> points;
    points.reserve(10000);
    
    std::mt19937 gen(42);
    std::uniform_real_distribution<double> dist(-M_PI, M_PI);
    
    for (int i = 0; i < 10000; ++i) {
        JointVector q;
        for (int j = 0; j < 6; ++j) {
            q[j] = dist(gen);
        }
        points.emplace_back(q);
    }
    
    // 构建KD-Tree
    KDTree6D kdTree;
    
    auto buildStart = std::chrono::high_resolution_clock::now();
    kdTree.build(points);
    auto buildEnd = std::chrono::high_resolution_clock::now();
    double buildTime = std::chrono::duration<double>(buildEnd - buildStart).count();
    
    std::cout << "KD-Tree 构建时间: " << GREEN << std::fixed << std::setprecision(3) 
              << buildTime * 1000 << " ms" << RESET << "\n";
    
    // 测试最近邻查询
    int numQueries = 10000;
    std::vector<JointConfig> queries;
    for (int i = 0; i < numQueries; ++i) {
        JointVector q;
        for (int j = 0; j < 6; ++j) {
            q[j] = dist(gen);
        }
        queries.emplace_back(q);
    }
    
    // KD-Tree查询
    auto kdStart = std::chrono::high_resolution_clock::now();
    for (const auto& query : queries) {
        kdTree.findNearest(query);
    }
    auto kdEnd = std::chrono::high_resolution_clock::now();
    double kdTime = std::chrono::duration<double>(kdEnd - kdStart).count();
    
    // 线性搜索对比
    auto linearStart = std::chrono::high_resolution_clock::now();
    for (const auto& query : queries) {
        double minDist = std::numeric_limits<double>::infinity();
        int nearest = -1;
        for (size_t i = 0; i < points.size(); ++i) {
            double d = points[i].distanceTo(query);
            if (d < minDist) {
                minDist = d;
                nearest = static_cast<int>(i);
            }
        }
        (void)nearest;  // 避免优化掉
    }
    auto linearEnd = std::chrono::high_resolution_clock::now();
    double linearTime = std::chrono::duration<double>(linearEnd - linearStart).count();
    
    std::cout << "\n" << numQueries << " 次最近邻查询:\n";
    std::cout << "  线性搜索: " << YELLOW << std::setprecision(2) << linearTime * 1000 << " ms" << RESET << "\n";
    std::cout << "  KD-Tree:  " << GREEN << kdTime * 1000 << " ms" << RESET << "\n";
    std::cout << "  加速比:   " << BOLD << GREEN << std::setprecision(1) 
              << linearTime / kdTime << "x" << RESET << "\n";
}

/**
 * @brief 测试2: 碰撞检测缓存
 */
void testCollisionCache() {
    printHeader("TEST 2: Collision Cache Performance");
    
    RobotModel robot;
    CollisionChecker checker(robot);
    checker.initialize(SceneConfig::defaultPalletizing());
    
    std::mt19937 gen(123);
    
    // 生成测试配置
    std::vector<JointConfig> configs;
    for (int i = 0; i < 5000; ++i) {
        configs.push_back(robot.randomConfig());
    }
    
    // 无缓存测试
    auto noCacheStart = std::chrono::high_resolution_clock::now();
    int noCacheCollisions = 0;
    for (const auto& cfg : configs) {
        if (!checker.isCollisionFree(cfg)) {
            noCacheCollisions++;
        }
    }
    auto noCacheEnd = std::chrono::high_resolution_clock::now();
    double noCacheTime = std::chrono::duration<double>(noCacheEnd - noCacheStart).count();
    
    // 带缓存测试（模拟重复查询场景）
    CollisionCache cache(CollisionCacheConfig::defaultConfig());
    
    auto cacheStart = std::chrono::high_resolution_clock::now();
    int cacheCollisions = 0;
    
    // 第一遍填充缓存
    for (const auto& cfg : configs) {
        bool result;
        if (!cache.lookup(cfg, result)) {
            result = checker.isCollisionFree(cfg);
            cache.insert(cfg, result);
        }
        if (!result) cacheCollisions++;
    }
    
    // 第二遍利用缓存
    for (const auto& cfg : configs) {
        bool result;
        if (!cache.lookup(cfg, result)) {
            result = checker.isCollisionFree(cfg);
            cache.insert(cfg, result);
        }
    }
    auto cacheEnd = std::chrono::high_resolution_clock::now();
    double cacheTime = std::chrono::duration<double>(cacheEnd - cacheStart).count();
    
    auto stats = cache.getStats();
    
    std::cout << "\n5000 配置 x 2 = 10000 次查询:\n";
    std::cout << "  无缓存 (5000次): " << YELLOW << std::setprecision(2) << noCacheTime * 1000 << " ms" << RESET << "\n";
    std::cout << "  有缓存 (10000次): " << GREEN << cacheTime * 1000 << " ms" << RESET << "\n";
    std::cout << "  缓存命中率: " << BOLD << std::setprecision(1) << stats.hitRate() * 100 << "%" << RESET << "\n";
    std::cout << "  碰撞率: " << std::setprecision(1) << 100.0 * noCacheCollisions / configs.size() << "%\n";
}

/**
 * @brief 测试3: 原始规划器 vs 优化规划器
 */
void testPlannerComparison() {
    printHeader("TEST 3: Original vs Optimized Planner");
    
    RobotModel robot;
    CollisionChecker checker(robot);
    checker.initialize(SceneConfig::defaultPalletizing());
    
    // 原始规划器
    PlannerConfig originalConfig;
    originalConfig.maxIterations = 5000;
    originalConfig.maxPlanningTime = 5.0;
    originalConfig.stepSize = 0.3;
    originalConfig.goalBias = 0.05;
    originalConfig.useInformedSampling = true;
    
    PathPlanner originalPlanner(robot, checker, originalConfig);
    
    // 优化规划器
    OptimizedPlannerConfig optConfig = OptimizedPlannerConfig::defaultConfig();
    optConfig.maxIterations = 5000;
    optConfig.maxPlanningTime = 5.0;
    
    OptimizedPathPlanner optimizedPlanner(robot, checker, optConfig);
    
    // 生成测试用例
    std::vector<std::pair<JointConfig, JointConfig>> testCases;
    std::mt19937 gen(456);
    
    for (int i = 0; i < 20; ++i) {
        JointConfig start, goal;
        
        // 确保起点终点有效
        do { start = robot.randomConfig(); } while (!checker.isCollisionFree(start));
        do { goal = robot.randomConfig(); } while (!checker.isCollisionFree(goal));
        
        testCases.emplace_back(start, goal);
    }
    
    std::cout << "\n对 " << testCases.size() << " 个测试用例进行规划...\n\n";
    
    // 测试原始规划器
    TestResult originalResult;
    originalResult.name = "Original";
    originalResult.trials = testCases.size();
    originalResult.successes = 0;
    originalResult.minTime = std::numeric_limits<double>::infinity();
    originalResult.maxTime = 0;
    originalResult.avgTime = 0;
    originalResult.minLength = std::numeric_limits<double>::infinity();
    originalResult.maxLength = 0;
    originalResult.avgLength = 0;
    
    std::vector<double> originalTimes, originalLengths;
    
    std::cout << YELLOW << "测试原始规划器..." << RESET << "\n";
    for (size_t i = 0; i < testCases.size(); ++i) {
        auto& [start, goal] = testCases[i];
        
        auto t1 = std::chrono::high_resolution_clock::now();
        auto result = originalPlanner.plan(start, goal);
        auto t2 = std::chrono::high_resolution_clock::now();
        
        double time = std::chrono::duration<double>(t2 - t1).count();
        
        std::cout << "  [" << std::setw(2) << (i+1) << "/" << testCases.size() << "] ";
        
        if (result.status == PlanningStatus::Success) {
            originalResult.successes++;
            originalTimes.push_back(time);
            originalLengths.push_back(result.rawPath.totalLength());
            
            originalResult.minTime = std::min(originalResult.minTime, time);
            originalResult.maxTime = std::max(originalResult.maxTime, time);
            originalResult.minLength = std::min(originalResult.minLength, result.rawPath.totalLength());
            originalResult.maxLength = std::max(originalResult.maxLength, result.rawPath.totalLength());
            
            std::cout << GREEN << "✓ " << RESET << std::setprecision(0) << time * 1000 << " ms\n";
        } else {
            std::cout << RED << "✗ " << RESET << result.statusString() << "\n";
        }
    }
    
    if (!originalTimes.empty()) {
        originalResult.avgTime = std::accumulate(originalTimes.begin(), originalTimes.end(), 0.0) / originalTimes.size();
        originalResult.avgLength = std::accumulate(originalLengths.begin(), originalLengths.end(), 0.0) / originalLengths.size();
        
        double sumSq = 0;
        for (double t : originalTimes) {
            sumSq += (t - originalResult.avgTime) * (t - originalResult.avgTime);
        }
        originalResult.stdTime = std::sqrt(sumSq / originalTimes.size());
    }
    
    // 测试优化规划器
    TestResult optimizedResult;
    optimizedResult.name = "Optimized";
    optimizedResult.trials = testCases.size();
    optimizedResult.successes = 0;
    optimizedResult.minTime = std::numeric_limits<double>::infinity();
    optimizedResult.maxTime = 0;
    optimizedResult.avgTime = 0;
    optimizedResult.minLength = std::numeric_limits<double>::infinity();
    optimizedResult.maxLength = 0;
    optimizedResult.avgLength = 0;
    
    std::vector<double> optimizedTimes, optimizedLengths;
    
    std::cout << "\n" << GREEN << "测试优化规划器..." << RESET << "\n";
    for (size_t i = 0; i < testCases.size(); ++i) {
        auto& [start, goal] = testCases[i];
        
        auto t1 = std::chrono::high_resolution_clock::now();
        auto result = optimizedPlanner.plan(start, goal);
        auto t2 = std::chrono::high_resolution_clock::now();
        
        double time = std::chrono::duration<double>(t2 - t1).count();
        
        std::cout << "  [" << std::setw(2) << (i+1) << "/" << testCases.size() << "] ";
        
        if (result.status == PlanningStatus::Success) {
            optimizedResult.successes++;
            optimizedTimes.push_back(time);
            optimizedLengths.push_back(result.rawPath.totalLength());
            
            optimizedResult.minTime = std::min(optimizedResult.minTime, time);
            optimizedResult.maxTime = std::max(optimizedResult.maxTime, time);
            optimizedResult.minLength = std::min(optimizedResult.minLength, result.rawPath.totalLength());
            optimizedResult.maxLength = std::max(optimizedResult.maxLength, result.rawPath.totalLength());
            
            std::cout << GREEN << "✓ " << RESET << std::setprecision(0) << time * 1000 << " ms\n";
        } else {
            std::cout << RED << "✗ " << RESET << result.statusString() << "\n";
        }
    }
    
    if (!optimizedTimes.empty()) {
        optimizedResult.avgTime = std::accumulate(optimizedTimes.begin(), optimizedTimes.end(), 0.0) / optimizedTimes.size();
        optimizedResult.avgLength = std::accumulate(optimizedLengths.begin(), optimizedLengths.end(), 0.0) / optimizedLengths.size();
        
        double sumSq = 0;
        for (double t : optimizedTimes) {
            sumSq += (t - optimizedResult.avgTime) * (t - optimizedResult.avgTime);
        }
        optimizedResult.stdTime = std::sqrt(sumSq / optimizedTimes.size());
    }
    
    // 打印对比结果
    printComparison(originalResult, optimizedResult);
    
    // 打印缓存统计
    auto cacheStats = optimizedPlanner.getCacheStats();
    std::cout << "\n" << CYAN << "缓存统计:" << RESET << "\n";
    std::cout << "  命中: " << cacheStats.hits << "\n";
    std::cout << "  未命中: " << cacheStats.misses << "\n";
    std::cout << "  命中率: " << std::setprecision(1) << cacheStats.hitRate() * 100 << "%\n";
}

/**
 * @brief 测试4: 高性能规划器完整流程
 */
void testHighPerformancePlanner() {
    printHeader("TEST 4: High Performance Planner - Full Pipeline");
    
    RobotModel robot;
    CollisionChecker checker(robot);
    checker.initialize(SceneConfig::defaultPalletizing());
    
    HighPerformanceConfig config = HighPerformanceConfig::defaultConfig();
    config.verboseOutput = true;
    
    HighPerformancePlanner planner(robot, checker, config);
    
    // 定义测试用例
    JointConfig start = JointConfig::fromDegrees({0, -90, 30, 0, -60, 0});
    JointConfig goal = JointConfig::fromDegrees({45, -60, 45, 30, -45, 45});
    
    std::cout << "\n" << BLUE << "起点: " << RESET;
    for (int i = 0; i < 6; ++i) {
        std::cout << std::setprecision(1) << start.q[i] * 180 / M_PI << "° ";
    }
    std::cout << "\n" << BLUE << "终点: " << RESET;
    for (int i = 0; i < 6; ++i) {
        std::cout << std::setprecision(1) << goal.q[i] * 180 / M_PI << "° ";
    }
    std::cout << "\n";
    
    auto result = planner.plan(start, goal);
    
    if (result.isSuccess()) {
        result.performance.print();
        
        // 保存结果
        std::ofstream trajFile("data/high_perf_trajectory.txt");
        if (trajFile.is_open()) {
            trajFile << "# High Performance Trajectory\n";
            trajFile << "# time q1 q2 q3 q4 q5 q6 (rad)\n";
            for (const auto& pt : result.trajectory.points) {
                trajFile << std::fixed << std::setprecision(4) << pt.time;
                for (int i = 0; i < 6; ++i) {
                    trajFile << " " << pt.config.q[i];
                }
                trajFile << "\n";
            }
            trajFile.close();
            std::cout << GREEN << "\n轨迹已保存到 high_perf_trajectory.txt" << RESET << "\n";
        }
    } else {
        std::cout << RED << "规划失败: " << static_cast<int>(result.status) << RESET << "\n";
    }
}

/**
 * @brief 测试5: 快速模式 vs 质量模式
 */
void testFastVsQuality() {
    printHeader("TEST 5: Fast Mode vs Quality Mode");
    
    RobotModel robot;
    CollisionChecker checker(robot);
    checker.initialize(SceneConfig::defaultPalletizing());
    
    HighPerformancePlanner planner(robot, checker);
    
    // 生成测试用例
    std::vector<std::pair<JointConfig, JointConfig>> testCases;
    std::mt19937 gen(789);
    
    for (int i = 0; i < 10; ++i) {
        JointConfig start, goal;
        do { start = robot.randomConfig(); } while (!checker.isCollisionFree(start));
        do { goal = robot.randomConfig(); } while (!checker.isCollisionFree(goal));
        testCases.emplace_back(start, goal);
    }
    
    std::cout << "\n比较快速模式和质量模式...\n\n";
    
    double fastTotalTime = 0, qualityTotalTime = 0;
    double fastTotalLength = 0, qualityTotalLength = 0;
    int fastSuccess = 0, qualitySuccess = 0;
    
    for (size_t i = 0; i < testCases.size(); ++i) {
        auto& [start, goal] = testCases[i];
        
        std::cout << "用例 " << (i + 1) << "/" << testCases.size() << ":\n";
        
        // 快速模式
        auto fastResult = planner.planFast(start, goal);
        if (fastResult.isSuccess()) {
            fastSuccess++;
            fastTotalTime += fastResult.performance.totalTime;
            fastTotalLength += fastResult.performance.optimizedPathLength;
            std::cout << "  Fast:    " << GREEN << "✓" << RESET 
                      << " " << std::setprecision(0) << fastResult.performance.totalTime * 1000 << " ms, "
                      << std::setprecision(3) << fastResult.performance.optimizedPathLength << " rad\n";
        } else {
            std::cout << "  Fast:    " << RED << "✗" << RESET << "\n";
        }
        
        // 质量模式
        auto qualityResult = planner.planQuality(start, goal);
        if (qualityResult.isSuccess()) {
            qualitySuccess++;
            qualityTotalTime += qualityResult.performance.totalTime;
            qualityTotalLength += qualityResult.performance.optimizedPathLength;
            std::cout << "  Quality: " << GREEN << "✓" << RESET 
                      << " " << std::setprecision(0) << qualityResult.performance.totalTime * 1000 << " ms, "
                      << std::setprecision(3) << qualityResult.performance.optimizedPathLength << " rad\n";
        } else {
            std::cout << "  Quality: " << RED << "✗" << RESET << "\n";
        }
    }
    
    std::cout << "\n" << YELLOW << "Summary:" << RESET << "\n";
    std::cout << "  Fast Mode:    " << fastSuccess << "/" << testCases.size() 
              << " success, avg " << std::setprecision(0) << (fastTotalTime / std::max(1, fastSuccess)) * 1000 
              << " ms, avg length " << std::setprecision(3) << fastTotalLength / std::max(1, fastSuccess) << "\n";
    std::cout << "  Quality Mode: " << qualitySuccess << "/" << testCases.size() 
              << " success, avg " << std::setprecision(0) << (qualityTotalTime / std::max(1, qualitySuccess)) * 1000 
              << " ms, avg length " << std::setprecision(3) << qualityTotalLength / std::max(1, qualitySuccess) << "\n";
}

/**
 * @brief 测试6: 基准测试
 */
void runBenchmark() {
    printHeader("TEST 6: Comprehensive Benchmark (50 trials)");
    
    RobotModel robot;
    CollisionChecker checker(robot);
    checker.initialize(SceneConfig::defaultPalletizing());
    
    HighPerformanceConfig config = HighPerformanceConfig::defaultConfig();
    HighPerformancePlanner planner(robot, checker, config);
    
    std::cout << "\n运行基准测试...\n";
    
    auto benchResult = PlannerBenchmark::run(planner, robot, 50);
    benchResult.print();
}

/**
 * @brief 主函数
 */
int main() {
    std::cout << "\n";
    std::cout << BOLD << CYAN;
    std::cout << "╔════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║     HIGH PERFORMANCE MOTION PLANNER - PERFORMANCE TEST SUITE      ║\n";
    std::cout << "║                   World-Class Level Validation                    ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════════════╝\n";
    std::cout << RESET;
    
    try {
        // 测试1: KD-Tree性能
        testKDTreePerformance();
        
        // 测试2: 碰撞检测缓存
        testCollisionCache();
        
        // 测试3: 规划器对比
        testPlannerComparison();
        
        // 测试4: 高性能规划器完整流程
        testHighPerformancePlanner();
        
        // 测试5: 快速模式 vs 质量模式
        testFastVsQuality();
        
        // 测试6: 基准测试
        runBenchmark();
        
        std::cout << "\n" << GREEN << BOLD;
        std::cout << "╔════════════════════════════════════════════════════════════════════╗\n";
        std::cout << "║                    ALL TESTS COMPLETED                            ║\n";
        std::cout << "╚════════════════════════════════════════════════════════════════════╝\n";
        std::cout << RESET;
        
    } catch (const std::exception& e) {
        std::cerr << RED << "Error: " << e.what() << RESET << "\n";
        return 1;
    }
    
    return 0;
}
