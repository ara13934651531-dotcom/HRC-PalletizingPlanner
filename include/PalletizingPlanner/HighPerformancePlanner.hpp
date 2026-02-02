/**
 * @file HighPerformancePlanner.hpp
 * @brief 世界顶尖水平的集成运动规划器
 * 
 * 集成所有优化组件:
 * - KD-Tree加速最近邻 O(log n)
 * - 碰撞检测缓存
 * - 懒惰碰撞检测
 * - 并行采样
 * - 自适应捷径优化
 * - 高效B-Spline拟合
 * - S曲线时间参数化
 * 
 * 性能目标:
 * - 简单场景: < 100 ms
 * - 复杂场景: < 1 s
 * - 极端场景: < 5 s
 * 
 * @author GitHub Copilot
 * @version 2.0.0 - World-Class Performance
 * @date 2026-01-29
 */

#pragma once

#include "Types.hpp"
#include "RobotModel.hpp"
#include "CollisionChecker.hpp"
#include "PathPlannerOptimized.hpp"
#include "PathOptimizerOptimized.hpp"
#include "TimeParameterizationOptimized.hpp"

#include <chrono>
#include <iostream>
#include <iomanip>
#include <fstream>

namespace palletizing {

/**
 * @brief 完整性能报告
 */
struct PerformanceReport {
    // 时间分解
    double totalTime = 0;
    double validationTime = 0;
    double planningTime = 0;
    double optimizationTime = 0;
    double splineFittingTime = 0;
    double timeParameterizationTime = 0;
    
    // 规划统计
    int iterations = 0;
    int nodesExplored = 0;
    int collisionChecks = 0;
    int cacheHits = 0;
    int cacheMisses = 0;
    
    // 路径质量
    double rawPathLength = 0;
    double optimizedPathLength = 0;
    double lengthReduction = 0;  // 百分比
    double maxCurvature = 0;
    double meanCurvature = 0;
    
    // 轨迹质量
    double trajectoryDuration = 0;
    double maxVelocity = 0;
    double maxAcceleration = 0;
    
    void print() const {
        std::cout << "\n╔══════════════════════════════════════════════════════════╗" << std::endl;
        std::cout << "║           PERFORMANCE REPORT - World-Class Level         ║" << std::endl;
        std::cout << "╠══════════════════════════════════════════════════════════╣" << std::endl;
        
        std::cout << "║ TIMING BREAKDOWN                                         ║" << std::endl;
        std::cout << "║   Total Time:              " << std::setw(10) << std::fixed << std::setprecision(3) 
                  << totalTime * 1000 << " ms                ║" << std::endl;
        std::cout << "║   ├─ Validation:           " << std::setw(10) << validationTime * 1000 << " ms                ║" << std::endl;
        std::cout << "║   ├─ Path Planning:        " << std::setw(10) << planningTime * 1000 << " ms                ║" << std::endl;
        std::cout << "║   ├─ Path Optimization:    " << std::setw(10) << optimizationTime * 1000 << " ms                ║" << std::endl;
        std::cout << "║   ├─ Spline Fitting:       " << std::setw(10) << splineFittingTime * 1000 << " ms                ║" << std::endl;
        std::cout << "║   └─ Time Parameterization:" << std::setw(10) << timeParameterizationTime * 1000 << " ms                ║" << std::endl;
        
        std::cout << "╠══════════════════════════════════════════════════════════╣" << std::endl;
        std::cout << "║ PLANNING STATISTICS                                      ║" << std::endl;
        std::cout << "║   Iterations:              " << std::setw(10) << iterations << "                   ║" << std::endl;
        std::cout << "║   Nodes Explored:          " << std::setw(10) << nodesExplored << "                   ║" << std::endl;
        std::cout << "║   Collision Checks:        " << std::setw(10) << collisionChecks << "                   ║" << std::endl;
        
        double hitRate = (cacheHits + cacheMisses) > 0 ? 
            100.0 * cacheHits / (cacheHits + cacheMisses) : 0;
        std::cout << "║   Cache Hit Rate:          " << std::setw(9) << std::setprecision(1) << hitRate << "%                   ║" << std::endl;
        
        std::cout << "╠══════════════════════════════════════════════════════════╣" << std::endl;
        std::cout << "║ PATH QUALITY                                             ║" << std::endl;
        std::cout << "║   Raw Path Length:         " << std::setw(10) << std::setprecision(4) << rawPathLength << " rad              ║" << std::endl;
        std::cout << "║   Optimized Length:        " << std::setw(10) << optimizedPathLength << " rad              ║" << std::endl;
        std::cout << "║   Length Reduction:        " << std::setw(9) << std::setprecision(1) << lengthReduction << "%                   ║" << std::endl;
        std::cout << "║   Max Curvature:           " << std::setw(10) << std::setprecision(6) << maxCurvature << "                   ║" << std::endl;
        
        std::cout << "╠══════════════════════════════════════════════════════════╣" << std::endl;
        std::cout << "║ TRAJECTORY QUALITY                                       ║" << std::endl;
        std::cout << "║   Duration:                " << std::setw(10) << std::setprecision(3) << trajectoryDuration << " s                 ║" << std::endl;
        std::cout << "║   Max Velocity:            " << std::setw(10) << std::setprecision(4) << maxVelocity << " rad/s            ║" << std::endl;
        std::cout << "║   Max Acceleration:        " << std::setw(10) << maxAcceleration << " rad/s²           ║" << std::endl;
        
        std::cout << "╚══════════════════════════════════════════════════════════╝" << std::endl;
    }
    
    void saveToFile(const std::string& filename) const {
        std::ofstream file(filename);
        if (!file.is_open()) return;
        
        file << "# Performance Report\n";
        file << "total_time_ms," << totalTime * 1000 << "\n";
        file << "planning_time_ms," << planningTime * 1000 << "\n";
        file << "optimization_time_ms," << optimizationTime * 1000 << "\n";
        file << "iterations," << iterations << "\n";
        file << "nodes_explored," << nodesExplored << "\n";
        file << "collision_checks," << collisionChecks << "\n";
        file << "raw_path_length," << rawPathLength << "\n";
        file << "optimized_path_length," << optimizedPathLength << "\n";
        file << "trajectory_duration," << trajectoryDuration << "\n";
        file << "max_velocity," << maxVelocity << "\n";
        file << "max_acceleration," << maxAcceleration << "\n";
    }
};

/**
 * @brief 高性能规划器配置
 */
struct HighPerformanceConfig {
    // 规划配置
    OptimizedPlannerConfig plannerConfig = OptimizedPlannerConfig::defaultConfig();
    
    // 优化配置
    OptimizedOptimizerConfig optimizerConfig = OptimizedOptimizerConfig::defaultConfig();
    
    // 时间参数化配置
    OptimizedTimeConfig timeConfig;
    
    // B-Spline配置
    int splineControlPoints = 50;
    int splineDegree = 5;
    
    // 全局配置
    bool enableOptimization = true;
    bool enableSplineFitting = true;
    bool enableTimeParameterization = true;
    bool verboseOutput = false;
    
    static HighPerformanceConfig defaultConfig() {
        return HighPerformanceConfig();
    }
    
    static HighPerformanceConfig fastConfig() {
        HighPerformanceConfig config;
        config.plannerConfig = OptimizedPlannerConfig::fastConfig();
        config.optimizerConfig = OptimizedOptimizerConfig::fastConfig();
        config.splineControlPoints = 30;
        return config;
    }
    
    static HighPerformanceConfig qualityConfig() {
        HighPerformanceConfig config;
        config.plannerConfig = OptimizedPlannerConfig::qualityConfig();
        config.optimizerConfig = OptimizedOptimizerConfig::qualityConfig();
        config.splineControlPoints = 100;
        return config;
    }
};

/**
 * @brief 完整规划结果
 */
struct HighPerformanceResult {
    PlanningStatus status = PlanningStatus::NoPath;
    
    // 路径
    Path rawPath;
    Path optimizedPath;
    BSpline smoothedSpline;
    
    // 轨迹
    Trajectory trajectory;
    
    // 性能报告
    PerformanceReport performance;
    
    bool isSuccess() const {
        return status == PlanningStatus::Success;
    }
};

/**
 * @brief 世界顶尖水平的高性能运动规划器
 */
class HighPerformancePlanner {
public:
    HighPerformancePlanner(const RobotModel& robot,
                          CollisionChecker& checker,
                          const HighPerformanceConfig& config = HighPerformanceConfig::defaultConfig())
        : robot_(robot),
          checker_(checker),
          config_(config),
          planner_(robot, checker, config.plannerConfig),
          optimizer_(checker, config.optimizerConfig),
          timeParameterizer_(config.timeConfig) {
    }
    
    /**
     * @brief 完整规划流程
     */
    HighPerformanceResult plan(const JointConfig& start, const JointConfig& goal) {
        auto totalStart = std::chrono::high_resolution_clock::now();
        
        HighPerformanceResult result;
        PerformanceReport& perf = result.performance;
        
        if (config_.verboseOutput) {
            std::cout << "\n[HighPerformancePlanner] Starting planning..." << std::endl;
            std::cout << "  Start: [";
            for (int i = 0; i < 6; ++i) std::cout << start.q[i] * 180/M_PI << (i < 5 ? ", " : "");
            std::cout << "]" << std::endl;
            std::cout << "  Goal:  [";
            for (int i = 0; i < 6; ++i) std::cout << goal.q[i] * 180/M_PI << (i < 5 ? ", " : "");
            std::cout << "]" << std::endl;
        }
        
        // 1. 路径规划
        auto planStart = std::chrono::high_resolution_clock::now();
        PlanningResult planResult = planner_.plan(start, goal);
        auto planEnd = std::chrono::high_resolution_clock::now();
        perf.planningTime = std::chrono::duration<double>(planEnd - planStart).count();
        
        if (planResult.status != PlanningStatus::Success) {
            result.status = planResult.status;
            auto totalEnd = std::chrono::high_resolution_clock::now();
            perf.totalTime = std::chrono::duration<double>(totalEnd - totalStart).count();
            return result;
        }
        
        result.rawPath = planResult.rawPath;
        perf.iterations = planResult.iterations;
        perf.nodesExplored = planResult.nodesExplored;
        perf.rawPathLength = result.rawPath.totalLength();
        
        // 获取规划器统计
        auto plannerStats = planner_.getStats();
        perf.collisionChecks = plannerStats.collisionChecks;
        perf.cacheHits = plannerStats.cacheHits;
        perf.cacheMisses = plannerStats.cacheMisses;
        
        if (config_.verboseOutput) {
            std::cout << "  [Planning] Done in " << perf.planningTime * 1000 << " ms, "
                      << result.rawPath.waypoints.size() << " waypoints" << std::endl;
        }
        
        // 2. 路径优化
        if (config_.enableOptimization) {
            auto optStart = std::chrono::high_resolution_clock::now();
            result.optimizedPath = optimizer_.optimize(result.rawPath);
            auto optEnd = std::chrono::high_resolution_clock::now();
            perf.optimizationTime = std::chrono::duration<double>(optEnd - optStart).count();
            
            perf.optimizedPathLength = result.optimizedPath.totalLength();
            perf.lengthReduction = perf.rawPathLength > 0 ? 
                100.0 * (1.0 - perf.optimizedPathLength / perf.rawPathLength) : 0;
            
            if (config_.verboseOutput) {
                std::cout << "  [Optimization] Done in " << perf.optimizationTime * 1000 << " ms, "
                          << result.optimizedPath.waypoints.size() << " waypoints, "
                          << perf.lengthReduction << "% shorter" << std::endl;
            }
        } else {
            result.optimizedPath = result.rawPath;
            perf.optimizedPathLength = perf.rawPathLength;
        }
        
        // 3. B-Spline拟合
        if (config_.enableSplineFitting) {
            auto splineStart = std::chrono::high_resolution_clock::now();
            result.smoothedSpline = optimizer_.fitSpline(
                result.optimizedPath, config_.splineControlPoints);
            auto splineEnd = std::chrono::high_resolution_clock::now();
            perf.splineFittingTime = std::chrono::duration<double>(splineEnd - splineStart).count();
            
            // 计算曲率
            auto curvatures = CurvatureAnalyzer::computeSplineCurvature(result.smoothedSpline);
            perf.maxCurvature = CurvatureAnalyzer::getMaxCurvature(curvatures);
            perf.meanCurvature = CurvatureAnalyzer::getMeanCurvature(curvatures);
            
            if (config_.verboseOutput) {
                std::cout << "  [Spline] Done in " << perf.splineFittingTime * 1000 << " ms, "
                          << result.smoothedSpline.controlPoints.size() << " control points, "
                          << "max curvature: " << perf.maxCurvature << std::endl;
            }
        }
        
        // 4. 时间参数化
        if (config_.enableTimeParameterization) {
            auto timeStart = std::chrono::high_resolution_clock::now();
            
            if (config_.enableSplineFitting) {
                result.trajectory = timeParameterizer_.parameterizeSpline(result.smoothedSpline);
            } else {
                result.trajectory = timeParameterizer_.parameterize(result.optimizedPath);
            }
            
            auto timeEnd = std::chrono::high_resolution_clock::now();
            perf.timeParameterizationTime = std::chrono::duration<double>(timeEnd - timeStart).count();
            
            auto timeStats = timeParameterizer_.getStats();
            perf.trajectoryDuration = timeStats.totalDuration;
            perf.maxVelocity = timeStats.maxVelocity;
            perf.maxAcceleration = timeStats.maxAcceleration;
            
            if (config_.verboseOutput) {
                std::cout << "  [Time] Done in " << perf.timeParameterizationTime * 1000 << " ms, "
                          << "duration: " << perf.trajectoryDuration << " s" << std::endl;
            }
        }
        
        // 完成
        result.status = PlanningStatus::Success;
        
        auto totalEnd = std::chrono::high_resolution_clock::now();
        perf.totalTime = std::chrono::duration<double>(totalEnd - totalStart).count();
        
        if (config_.verboseOutput) {
            std::cout << "  [Total] Planning completed in " << perf.totalTime * 1000 << " ms" << std::endl;
        }
        
        return result;
    }
    
    /**
     * @brief 仅规划路径（不做时间参数化）
     */
    HighPerformanceResult planPath(const JointConfig& start, const JointConfig& goal) {
        HighPerformanceConfig tempConfig = config_;
        tempConfig.enableTimeParameterization = false;
        
        auto oldConfig = config_;
        config_ = tempConfig;
        
        auto result = plan(start, goal);
        
        config_ = oldConfig;
        return result;
    }
    
    /**
     * @brief 快速规划（最小化时间）
     */
    HighPerformanceResult planFast(const JointConfig& start, const JointConfig& goal) {
        HighPerformanceConfig fastConfig = HighPerformanceConfig::fastConfig();
        fastConfig.verboseOutput = config_.verboseOutput;
        
        auto oldConfig = config_;
        config_ = fastConfig;
        planner_.setConfig(fastConfig.plannerConfig);
        optimizer_.setConfig(fastConfig.optimizerConfig);
        
        auto result = plan(start, goal);
        
        config_ = oldConfig;
        planner_.setConfig(oldConfig.plannerConfig);
        optimizer_.setConfig(oldConfig.optimizerConfig);
        
        return result;
    }
    
    /**
     * @brief 高质量规划（最大化路径质量）
     */
    HighPerformanceResult planQuality(const JointConfig& start, const JointConfig& goal) {
        HighPerformanceConfig qualityConfig = HighPerformanceConfig::qualityConfig();
        qualityConfig.verboseOutput = config_.verboseOutput;
        
        auto oldConfig = config_;
        config_ = qualityConfig;
        planner_.setConfig(qualityConfig.plannerConfig);
        optimizer_.setConfig(qualityConfig.optimizerConfig);
        
        auto result = plan(start, goal);
        
        config_ = oldConfig;
        planner_.setConfig(oldConfig.plannerConfig);
        optimizer_.setConfig(oldConfig.optimizerConfig);
        
        return result;
    }
    
    /**
     * @brief 获取配置
     */
    const HighPerformanceConfig& getConfig() const { return config_; }
    
    /**
     * @brief 设置配置
     */
    void setConfig(const HighPerformanceConfig& config) {
        config_ = config;
        planner_.setConfig(config.plannerConfig);
        optimizer_.setConfig(config.optimizerConfig);
        timeParameterizer_.setConfig(config.timeConfig);
    }
    
    /**
     * @brief 启用/禁用详细输出
     */
    void setVerbose(bool verbose) {
        config_.verboseOutput = verbose;
    }
    
private:
    const RobotModel& robot_;
    CollisionChecker& checker_;
    HighPerformanceConfig config_;
    
    OptimizedPathPlanner planner_;
    OptimizedPathOptimizer optimizer_;
    OptimizedTimeParameterizer timeParameterizer_;
};

/**
 * @brief 基准测试工具
 */
class PlannerBenchmark {
public:
    struct BenchmarkResult {
        int numTrials;
        int successCount;
        double successRate;
        
        double minTime;
        double maxTime;
        double meanTime;
        double stdDevTime;
        
        double minPathLength;
        double maxPathLength;
        double meanPathLength;
        
        void print() const {
            std::cout << "\n╔══════════════════════════════════════╗" << std::endl;
            std::cout << "║       BENCHMARK RESULTS              ║" << std::endl;
            std::cout << "╠══════════════════════════════════════╣" << std::endl;
            std::cout << "║ Trials:        " << std::setw(8) << numTrials << "             ║" << std::endl;
            std::cout << "║ Success Rate:  " << std::setw(7) << std::fixed << std::setprecision(1) 
                      << successRate * 100 << "%             ║" << std::endl;
            std::cout << "╠══════════════════════════════════════╣" << std::endl;
            std::cout << "║ Time (ms)                            ║" << std::endl;
            std::cout << "║   Min:         " << std::setw(8) << std::setprecision(2) << minTime * 1000 << "             ║" << std::endl;
            std::cout << "║   Max:         " << std::setw(8) << maxTime * 1000 << "             ║" << std::endl;
            std::cout << "║   Mean:        " << std::setw(8) << meanTime * 1000 << "             ║" << std::endl;
            std::cout << "║   Std Dev:     " << std::setw(8) << stdDevTime * 1000 << "             ║" << std::endl;
            std::cout << "╠══════════════════════════════════════╣" << std::endl;
            std::cout << "║ Path Length (rad)                    ║" << std::endl;
            std::cout << "║   Min:         " << std::setw(8) << std::setprecision(4) << minPathLength << "             ║" << std::endl;
            std::cout << "║   Max:         " << std::setw(8) << maxPathLength << "             ║" << std::endl;
            std::cout << "║   Mean:        " << std::setw(8) << meanPathLength << "             ║" << std::endl;
            std::cout << "╚══════════════════════════════════════╝" << std::endl;
        }
    };
    
    static BenchmarkResult run(HighPerformancePlanner& planner,
                               const RobotModel& robot,
                               int numTrials = 100) {
        BenchmarkResult result;
        result.numTrials = numTrials;
        result.successCount = 0;
        
        std::vector<double> times;
        std::vector<double> lengths;
        
        result.minTime = std::numeric_limits<double>::infinity();
        result.maxTime = 0;
        result.minPathLength = std::numeric_limits<double>::infinity();
        result.maxPathLength = 0;
        
        for (int i = 0; i < numTrials; ++i) {
            JointConfig start = robot.randomConfig();
            JointConfig goal = robot.randomConfig();
            
            auto planResult = planner.plan(start, goal);
            
            if (planResult.isSuccess()) {
                result.successCount++;
                
                double time = planResult.performance.totalTime;
                double length = planResult.performance.optimizedPathLength;
                
                times.push_back(time);
                lengths.push_back(length);
                
                result.minTime = std::min(result.minTime, time);
                result.maxTime = std::max(result.maxTime, time);
                result.minPathLength = std::min(result.minPathLength, length);
                result.maxPathLength = std::max(result.maxPathLength, length);
            }
        }
        
        result.successRate = static_cast<double>(result.successCount) / numTrials;
        
        // 计算均值和标准差
        if (!times.empty()) {
            double sumTime = 0, sumLength = 0;
            for (size_t i = 0; i < times.size(); ++i) {
                sumTime += times[i];
                sumLength += lengths[i];
            }
            result.meanTime = sumTime / times.size();
            result.meanPathLength = sumLength / lengths.size();
            
            double sumSqDiff = 0;
            for (double t : times) {
                sumSqDiff += (t - result.meanTime) * (t - result.meanTime);
            }
            result.stdDevTime = std::sqrt(sumSqDiff / times.size());
        } else {
            result.minTime = result.maxTime = result.meanTime = result.stdDevTime = 0;
            result.minPathLength = result.maxPathLength = result.meanPathLength = 0;
        }
        
        return result;
    }
};

} // namespace palletizing
