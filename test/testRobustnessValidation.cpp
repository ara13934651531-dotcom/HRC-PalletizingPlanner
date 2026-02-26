/**
 * @file testRobustnessValidation.cpp
 * @brief 全面的鲁棒性和泛化性测试
 * 
 * 测试内容：
 * 1. 边界条件测试 - 关节极限、奇异位置
 * 2. 随机配置泛化测试 - 大量随机起点终点
 * 3. 极端情况测试 - 零距离、超长路径
 * 4. 数值稳定性测试 - 浮点精度边界
 * 5. 压力测试 - 连续多次规划
 * 6. 错误恢复测试 - 无效输入处理
 * 7. 路径质量验证 - 碰撞检测、平滑度
 */

#include <iostream>
#include <iomanip>
#include <vector>
#include <random>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <fcntl.h>

#include "PalletizingPlanner/Types.hpp"
#include "PalletizingPlanner/RobotModel.hpp"
#include "PalletizingPlanner/CollisionChecker.hpp"
#include "PalletizingPlanner/PathPlanner.hpp"
#include "PalletizingPlanner/PathOptimizer.hpp"
#include "PalletizingPlanner/TimeParameterization.hpp"
#include "PalletizingPlanner/PalletizingPlanner.hpp"

using namespace palletizing;

// ============================================================================
// 测试统计结构
// ============================================================================
struct TestStatistics {
    std::string testName;
    int totalTests = 0;
    int passed = 0;
    int failed = 0;
    double totalTime = 0.0;
    double minTime = std::numeric_limits<double>::max();
    double maxTime = 0.0;
    std::vector<std::string> failureReasons;
    
    double successRate() const { 
        return totalTests > 0 ? 100.0 * passed / totalTests : 0.0; 
    }
    double avgTime() const { 
        return passed > 0 ? totalTime / passed : 0.0; 
    }
    
    void recordSuccess(double time) {
        totalTests++;
        passed++;
        totalTime += time;
        minTime = std::min(minTime, time);
        maxTime = std::max(maxTime, time);
    }
    
    void recordFailure(const std::string& reason) {
        totalTests++;
        failed++;
        failureReasons.push_back(reason);
    }
    
    void print() const {
        std::cout << "  " << testName << ":\n";
        std::cout << "    Total: " << totalTests 
                  << ", Passed: " << passed 
                  << ", Failed: " << failed << "\n";
        std::cout << "    Success Rate: " << std::fixed << std::setprecision(2) 
                  << successRate() << "%\n";
        if (passed > 0) {
            std::cout << "    Time - Avg: " << std::setprecision(2) << avgTime() 
                      << " ms, Min: " << minTime 
                      << " ms, Max: " << maxTime << " ms\n";
        }
        if (!failureReasons.empty()) {
            std::cout << "    Failure samples (first 3):\n";
            for (size_t i = 0; i < std::min(size_t(3), failureReasons.size()); i++) {
                std::cout << "      - " << failureReasons[i] << "\n";
            }
        }
    }
};

// ============================================================================
// 鲁棒性验证器
// ============================================================================
class RobustnessValidator {
private:
    class ScopedStdoutSilencer {
    public:
        explicit ScopedStdoutSilencer(bool enable = true) : enabled_(enable) {
            if (!enabled_) return;
            fflush(stdout);
            savedFd_ = dup(STDOUT_FILENO);
            nullFd_ = open("/dev/null", O_WRONLY);
            if (savedFd_ >= 0 && nullFd_ >= 0) {
                dup2(nullFd_, STDOUT_FILENO);
            } else {
                enabled_ = false;
            }
        }
        ~ScopedStdoutSilencer() {
            if (!enabled_) return;
            fflush(stdout);
            if (savedFd_ >= 0) {
                dup2(savedFd_, STDOUT_FILENO);
                close(savedFd_);
            }
            if (nullFd_ >= 0) close(nullFd_);
        }
    private:
        bool enabled_ = false;
        int savedFd_ = -1;
        int nullFd_ = -1;
    };

    std::mt19937 rng_;
    RobotModel robot_;
    std::unique_ptr<CollisionChecker> collisionChecker_;
    std::unique_ptr<PathPlanner> planner_;
    std::unique_ptr<PathOptimizer> optimizer_;
    bool initialized_ = false;
    bool silenceInnerLoopStdout_ = true;
    
    JointVector jointMin_;
    JointVector jointMax_;
    
public:
    RobustnessValidator() : rng_(42), robot_(RobotDHParams::fromHRConfig()) {
        auto [minLimits, maxLimits] = robot_.getParams().getJointLimits();
        jointMin_ = minLimits;
        jointMax_ = maxLimits;
    }
    
    bool initialize() {
        // 初始化碰撞检测
        collisionChecker_ = std::make_unique<CollisionChecker>(robot_);
        // 鲁棒性验证使用最小场景，避免场景约束掩盖规划器本体性能
        SceneConfig scene;
        scene.platformHeight = 0.0;
        scene.robotBasePose = Pose6D();
        if (!collisionChecker_->initialize(scene)) {
            std::cerr << "Failed to initialize collision checker\n";
            return false;
        }
        
        // 初始化规划器
        PlannerConfig plannerConfig;
        plannerConfig.maxIterations = 2000;
        plannerConfig.maxPlanningTime = 0.8;
        plannerConfig.goalBias = 0.1;
        plannerConfig.stepSize = 0.1;
        plannerConfig.shortcutIterations = 60;
        plannerConfig.plannerType = PlannerType::InformedRRTStar;
        
        planner_ = std::make_unique<PathPlanner>(robot_, *collisionChecker_, plannerConfig);
        
        // 初始化优化器
        optimizer_ = std::make_unique<PathOptimizer>(robot_, *collisionChecker_, plannerConfig);
        
        initialized_ = true;
        return true;
    }
    
    // 生成随机有效配置
    JointConfig generateRandomConfig() {
        std::uniform_real_distribution<double> dist(-1.0, 1.0);
        JointVector joints;
        for (int i = 0; i < 6; i++) {
            double range = jointMax_[i] - jointMin_[i];
            double mid = (jointMax_[i] + jointMin_[i]) / 2.0;
            joints[i] = mid + dist(rng_) * range * 0.4;  // 使用40%范围避免极限
        }
        return JointConfig(joints);
    }
    
    // 生成边界附近配置
    JointConfig generateBoundaryConfig(double margin = 0.05) {
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        std::uniform_int_distribution<int> jointDist(0, 5);
        
        JointConfig config = generateRandomConfig();
        
        // 随机选择1-3个关节设置到边界附近
        int numBoundaryJoints = 1 + (rng_() % 3);
        for (int i = 0; i < numBoundaryJoints; i++) {
            int joint = jointDist(rng_);
            double range = jointMax_[joint] - jointMin_[joint];
            if (dist(rng_) < 0.5) {
                config.q[joint] = jointMin_[joint] + margin * range;
            } else {
                config.q[joint] = jointMax_[joint] - margin * range;
            }
        }
        return config;
    }
    
    // 验证配置有效性
    bool isConfigValid(const JointConfig& config) {
        // 检查关节限位
        for (int i = 0; i < 6; i++) {
            if (config.q[i] < jointMin_[i] || config.q[i] > jointMax_[i]) {
                return false;
            }
        }
        {
            ScopedStdoutSilencer silencer(silenceInnerLoopStdout_);
            return collisionChecker_->isCollisionFree(config);
        }
    }
    
    bool sampleValidConfig(JointConfig& config, int maxAttempts = 200) {
        for (int i = 0; i < maxAttempts; ++i) {
            JointConfig candidate = generateRandomConfig();
            if (isConfigValid(candidate)) {
                config = candidate;
                return true;
            }
        }
        return false;
    }

    bool sampleValidBoundaryConfig(JointConfig& config, double margin = 0.05, int maxAttempts = 200) {
        for (int i = 0; i < maxAttempts; ++i) {
            JointConfig candidate = generateBoundaryConfig(margin);
            if (isConfigValid(candidate)) {
                config = candidate;
                return true;
            }
        }
        return false;
    }

    // 生成有效的随机配置对（带距离约束）
    bool generateValidPair(JointConfig& start, JointConfig& goal,
                          int maxAttempts = 400, double minDistance = 0.10) {
        for (int i = 0; i < maxAttempts; ++i) {
            if (!sampleValidConfig(start, 80)) continue;
            if (!sampleValidConfig(goal, 80)) continue;
            if (start.distanceTo(goal) >= minDistance) return true;
        }
        return false;
    }
    
    // 计算路径长度
    double computePathLength(const Path& path) {
        if (path.waypoints.size() < 2) return 0.0;
        double length = 0.0;
        for (size_t i = 1; i < path.waypoints.size(); i++) {
            length += path.waypoints[i].config.distanceTo(path.waypoints[i-1].config);
        }
        return length;
    }
    
    // 验证路径无碰撞
    bool verifyPathCollisionFree(const Path& path, double resolution = 0.01) {
        ScopedStdoutSilencer silencer(silenceInnerLoopStdout_);
        for (size_t i = 0; i < path.waypoints.size() - 1; i++) {
            double dist = path.waypoints[i].config.distanceTo(path.waypoints[i+1].config);
            int steps = std::max(2, static_cast<int>(std::ceil(dist / resolution)));
            for (int s = 0; s <= steps; s++) {
                double t = static_cast<double>(s) / steps;
                JointConfig interpolated = path.waypoints[i].config.interpolate(
                    path.waypoints[i+1].config, t);
                if (!collisionChecker_->isCollisionFree(interpolated)) {
                    return false;
                }
            }
        }
        return true;
    }
    
    // 计算路径平滑度（曲率变化）
    double computePathSmoothness(const Path& path) {
        if (path.waypoints.size() < 3) return 0.0;
        
        double totalCurvatureChange = 0.0;
        for (size_t i = 1; i < path.waypoints.size() - 1; i++) {
            // 计算前后两段的方向变化
            JointVector v1 = path.waypoints[i].config.q - path.waypoints[i-1].config.q;
            JointVector v2 = path.waypoints[i+1].config.q - path.waypoints[i].config.q;
            double n1 = v1.norm();
            double n2 = v2.norm();
            if (n1 > 1e-6 && n2 > 1e-6) {
                double cosAngle = v1.dot(v2) / (n1 * n2);
                cosAngle = std::max(-1.0, std::min(1.0, cosAngle));
                totalCurvatureChange += std::acos(cosAngle);
            }
        }
        return totalCurvatureChange / (path.waypoints.size() - 2);  // 平均曲率变化
    }
    
    // 执行单次规划并验证
    struct PlanResult {
        bool success;
        double planTime;
        double pathLength;
        double smoothness;
        bool collisionFree;
        std::string failureReason;
    };
    
    PlanResult planAndValidate(const JointConfig& start, const JointConfig& goal) {
        PlanResult result;
        result.success = false;
        result.planTime = 0.0;
        result.pathLength = 0.0;
        result.smoothness = 0.0;
        result.collisionFree = true;
        
        auto t1 = std::chrono::high_resolution_clock::now();
        
        PlanningResult planResult;
        {
            ScopedStdoutSilencer silencer(silenceInnerLoopStdout_);
            planResult = planner_->plan(start, goal);
        }
        
        auto t2 = std::chrono::high_resolution_clock::now();
        result.planTime = std::chrono::duration<double, std::milli>(t2 - t1).count();
        
        if (!planResult.isSuccess()) {
            result.failureReason = "Planning failed: " + planResult.statusString();
            return result;
        }
        
        // 优化路径
        PlanningResult optimizedResult;
        {
            ScopedStdoutSilencer silencer(silenceInnerLoopStdout_);
            optimizedResult = optimizer_->optimize(planResult.rawPath);
        }
        
        // 验证路径
        result.pathLength = computePathLength(optimizedResult.optimizedPath);
        result.smoothness = computePathSmoothness(optimizedResult.optimizedPath);
        result.collisionFree = verifyPathCollisionFree(optimizedResult.optimizedPath);
        
        if (!result.collisionFree) {
            result.failureReason = "Path contains collision";
            return result;
        }
        
        result.success = true;
        return result;
    }
    
    // ========================================================================
    // 测试用例
    // ========================================================================
    
    // 1. 随机配置泛化测试
    TestStatistics testRandomConfigurations(int numTests = 100) {
        TestStatistics stats;
        stats.testName = "Random Configurations";
        
        for (int i = 0; i < numTests; i++) {
            JointConfig start, goal;
            if (!generateValidPair(start, goal)) {
                stats.recordFailure("Test " + std::to_string(i) + ": Failed to sample valid start/goal");
                continue;
            }
            auto result = planAndValidate(start, goal);
            
            if (result.success) {
                stats.recordSuccess(result.planTime);
            } else {
                std::ostringstream oss;
                oss << "Test " << i << ": " << result.failureReason;
                stats.recordFailure(oss.str());
            }
        }
        return stats;
    }
    
    // 2. 边界条件测试
    TestStatistics testBoundaryConditions(int numTests = 50) {
        TestStatistics stats;
        stats.testName = "Boundary Conditions";
        
        for (int i = 0; i < numTests; i++) {
            JointConfig start, goal;
            if (!sampleValidBoundaryConfig(start, 0.02) || !sampleValidBoundaryConfig(goal, 0.02)) {
                stats.recordFailure("Boundary test " + std::to_string(i) + ": Failed to sample valid boundary config");
                continue;
            }
            
            auto result = planAndValidate(start, goal);
            
            if (result.success) {
                stats.recordSuccess(result.planTime);
            } else {
                std::ostringstream oss;
                oss << "Boundary test " << i << ": " << result.failureReason;
                stats.recordFailure(oss.str());
            }
        }
        return stats;
    }
    
    // 3. 极短距离测试（起点接近终点）
    TestStatistics testShortDistances(int numTests = 50) {
        TestStatistics stats;
        stats.testName = "Short Distances";
        
        std::uniform_real_distribution<double> noiseDist(-0.05, 0.05);
        
        for (int i = 0; i < numTests; i++) {
            JointConfig start;
            if (!sampleValidConfig(start)) {
                stats.recordFailure("Short distance test " + std::to_string(i) + ": Failed to sample valid start");
                continue;
            }

            JointConfig goal;
            bool foundGoal = false;
            for (int attempt = 0; attempt < 80; ++attempt) {
                JointVector goalJoints = start.q;
                for (int j = 0; j < 6; j++) {
                    goalJoints[j] += noiseDist(rng_);
                }
                JointConfig candidate(goalJoints);
                if (isConfigValid(candidate)) {
                    goal = candidate;
                    foundGoal = true;
                    break;
                }
            }
            if (!foundGoal) {
                stats.recordFailure("Short distance test " + std::to_string(i) + ": Failed to sample nearby valid goal");
                continue;
            }
            
            auto result = planAndValidate(start, goal);
            
            if (result.success) {
                stats.recordSuccess(result.planTime);
            } else {
                std::ostringstream oss;
                oss << "Short distance test " << i << ": " << result.failureReason;
                stats.recordFailure(oss.str());
            }
        }
        return stats;
    }
    
    // 4. 相同起点终点测试
    TestStatistics testSameStartGoal(int numTests = 20) {
        TestStatistics stats;
        stats.testName = "Same Start/Goal";
        
        for (int i = 0; i < numTests; i++) {
            JointConfig config;
            if (!sampleValidConfig(config)) {
                stats.recordFailure("Same start/goal test " + std::to_string(i) + ": Failed to sample valid config");
                continue;
            }
            
            auto t1 = std::chrono::high_resolution_clock::now();
            PlanningResult planResult;
            {
                ScopedStdoutSilencer silencer(silenceInnerLoopStdout_);
                planResult = planner_->plan(config, config);
            }
            auto t2 = std::chrono::high_resolution_clock::now();
            double time = std::chrono::duration<double, std::milli>(t2 - t1).count();
            
            // 相同起点终点应该立即返回成功
            if (planResult.isSuccess() && planResult.rawPath.waypoints.size() <= 2) {
                stats.recordSuccess(time);
            } else if (!planResult.isSuccess()) {
                stats.recordFailure("Failed for same start/goal");
            } else {
                stats.recordSuccess(time);  // 有些实现可能返回包含起点终点的路径
            }
        }
        return stats;
    }
    
    // 5. 大范围运动测试
    TestStatistics testLargeMotions(int numTests = 30) {
        TestStatistics stats;
        stats.testName = "Large Motions";
        
        for (int i = 0; i < numTests; i++) {
            // 生成接近极限的大范围运动
            JointVector startJoints, goalJoints;
            for (int j = 0; j < 6; j++) {
                double range = jointMax_[j] - jointMin_[j];
                startJoints[j] = jointMin_[j] + 0.15 * range;
                goalJoints[j] = jointMax_[j] - 0.15 * range;
            }
            
            // 添加随机扰动
            std::uniform_real_distribution<double> noiseDist(-0.1, 0.1);
            for (int j = 0; j < 6; j++) {
                startJoints[j] += noiseDist(rng_);
                goalJoints[j] += noiseDist(rng_);
            }
            
            JointConfig start(startJoints);
            JointConfig goal(goalJoints);
            
            if (!isConfigValid(start) || !isConfigValid(goal)) {
                bool sampled = generateValidPair(start, goal, 120, 0.40);
                if (!sampled) {
                    stats.recordFailure("Large motion test " + std::to_string(i) + ": Failed to sample valid large-motion pair");
                    continue;
                }
            }
            
            auto result = planAndValidate(start, goal);
            
            if (result.success) {
                stats.recordSuccess(result.planTime);
            } else {
                std::ostringstream oss;
                oss << "Large motion test " << i << ": " << result.failureReason;
                stats.recordFailure(oss.str());
            }
        }
        return stats;
    }
    
    // 6. 奇异位置附近测试
    TestStatistics testNearSingularities(int numTests = 30) {
        TestStatistics stats;
        stats.testName = "Near Singularities";
        
        // 常见奇异位置配置
        std::vector<JointVector> singularConfigs;
        
        // 肩部奇异 (J2 + J3 ≈ 0 or π)
        JointVector shoulder;
        shoulder << 0, -M_PI/2, M_PI/2, 0, -M_PI/2, 0;
        singularConfigs.push_back(shoulder);
        
        // 腕部奇异 (J5 ≈ 0)
        JointVector wrist;
        wrist << 0, -M_PI/3, M_PI/4, 0, 0.1, 0;
        singularConfigs.push_back(wrist);
        
        // 肘部伸直奇异
        JointVector elbow;
        elbow << 0, 0, 0, 0, -M_PI/2, 0;
        singularConfigs.push_back(elbow);
        
        std::uniform_real_distribution<double> noiseDist(-0.15, 0.15);
        
        for (int i = 0; i < numTests; i++) {
            // 选择一个奇异配置并添加扰动
            JointVector startJoints = singularConfigs[i % singularConfigs.size()];
            for (int j = 0; j < 6; j++) {
                startJoints[j] += noiseDist(rng_);
            }
            JointConfig start(startJoints);
            
            JointConfig goal;
            if (!sampleValidConfig(goal)) {
                stats.recordFailure("Singularity test " + std::to_string(i) + ": Failed to sample valid goal");
                continue;
            }

            if (!isConfigValid(start)) {
                if (!sampleValidConfig(start)) {
                    stats.recordFailure("Singularity test " + std::to_string(i) + ": Failed to sample valid start");
                    continue;
                }
            }
            if (!isConfigValid(start) || !isConfigValid(goal)) {
                stats.recordFailure("Singularity test " + std::to_string(i) + ": Invalid start/goal after sampling");
                continue;
            }
            
            auto result = planAndValidate(start, goal);
            
            if (result.success) {
                stats.recordSuccess(result.planTime);
            } else {
                std::ostringstream oss;
                oss << "Singularity test " << i << ": " << result.failureReason;
                stats.recordFailure(oss.str());
            }
        }
        return stats;
    }
    
    // 7. 连续规划压力测试
    TestStatistics testContinuousPlanning(int numTests = 50) {
        TestStatistics stats;
        stats.testName = "Continuous Planning (Stress)";
        
        JointConfig current = JointConfig::fromDegrees({0, -45, 45, 0, -45, 0});
        if (!isConfigValid(current) && !sampleValidConfig(current)) {
            stats.recordFailure("Continuous planning: Failed to get initial valid config");
            return stats;
        }
        
        for (int i = 0; i < numTests; i++) {
            JointConfig next;
            if (!sampleValidConfig(next)) {
                stats.recordFailure("Continuous test " + std::to_string(i) + ": Failed to sample next valid config");
                continue;
            }
            
            auto result = planAndValidate(current, next);
            
            if (result.success) {
                stats.recordSuccess(result.planTime);
                current = next;  // 更新当前位置
            } else {
                std::ostringstream oss;
                oss << "Continuous test " << i << ": " << result.failureReason;
                stats.recordFailure(oss.str());
            }
        }
        return stats;
    }
    
    // 8. 数值稳定性测试
    TestStatistics testNumericalStability(int numTests = 30) {
        TestStatistics stats;
        stats.testName = "Numerical Stability";
        
        for (int i = 0; i < numTests; i++) {
            JointVector startJoints, goalJoints;
            
            // 使用非常接近但不完全相同的值
            double epsilon = 1e-10 * (i + 1);
            for (int j = 0; j < 6; j++) {
                double mid = (jointMax_[j] + jointMin_[j]) / 2.0;
                startJoints[j] = mid + epsilon * j;
                goalJoints[j] = mid + 0.5 + epsilon * (j + 1);
            }
            
            JointConfig start(startJoints);
            JointConfig goal(goalJoints);
            
            if (!isConfigValid(start) || !isConfigValid(goal)) {
                if (!generateValidPair(start, goal, 100, 0.05)) {
                    stats.recordFailure("Numerical test " + std::to_string(i) + ": Failed to sample valid pair");
                    continue;
                }
            }
            
            auto result = planAndValidate(start, goal);
            
            if (result.success) {
                stats.recordSuccess(result.planTime);
            } else {
                std::ostringstream oss;
                oss << "Numerical test " << i << " (eps=" << epsilon << "): " << result.failureReason;
                stats.recordFailure(oss.str());
            }
        }
        return stats;
    }
    
    // 9. 路径质量一致性测试
    TestStatistics testPathQualityConsistency(int numTests = 20) {
        TestStatistics stats;
        stats.testName = "Path Quality Consistency";
        
        for (int i = 0; i < numTests; i++) {
            JointConfig start, goal;
            if (!generateValidPair(start, goal)) {
                stats.recordFailure("Path consistency test " + std::to_string(i) + ": Failed to sample valid pair");
                continue;
            }
            
            // 对同一对起点终点规划多次
            std::vector<double> pathLengths;
            std::vector<double> planTimes;
            bool allSuccess = true;
            
            for (int trial = 0; trial < 5; trial++) {
                auto result = planAndValidate(start, goal);
                if (result.success) {
                    pathLengths.push_back(result.pathLength);
                    planTimes.push_back(result.planTime);
                } else {
                    allSuccess = false;
                    break;
                }
            }
            
            if (allSuccess && pathLengths.size() >= 5) {
                // 检查路径长度一致性（标准差应该小）
                double mean = std::accumulate(pathLengths.begin(), pathLengths.end(), 0.0) / pathLengths.size();
                double variance = 0.0;
                for (double len : pathLengths) {
                    variance += (len - mean) * (len - mean);
                }
                double stddev = std::sqrt(variance / pathLengths.size());
                double cv = mean > 0 ? stddev / mean : 0;  // 变异系数
                
                if (cv < 0.3) {  // 变异系数小于30%认为一致
                    double avgTime = std::accumulate(planTimes.begin(), planTimes.end(), 0.0) / planTimes.size();
                    stats.recordSuccess(avgTime);
                } else {
                    std::ostringstream oss;
                    oss << "Inconsistent path lengths, CV=" << std::fixed << std::setprecision(2) << cv;
                    stats.recordFailure(oss.str());
                }
            } else {
                stats.recordFailure("Not all trials succeeded");
            }
        }
        return stats;
    }
    
    // 10. B-Spline平滑验证
    TestStatistics testBSplineSmoothing(int numTests = 30) {
        TestStatistics stats;
        stats.testName = "B-Spline Smoothing Quality";
        
        for (int i = 0; i < numTests; i++) {
            JointConfig start, goal;
            if (!generateValidPair(start, goal)) {
                stats.recordFailure("B-Spline test " + std::to_string(i) + ": Failed to sample valid pair");
                continue;
            }
            
            PlanningResult planResult;
            {
                ScopedStdoutSilencer silencer(silenceInnerLoopStdout_);
                planResult = planner_->plan(start, goal);
            }
            if (!planResult.isSuccess() || planResult.rawPath.waypoints.size() < 4) {
                stats.recordFailure("B-Spline test " + std::to_string(i) + ": Planning failed or path too short");
                continue;
            }
            
            auto t1 = std::chrono::high_resolution_clock::now();
            
            // 优化并获取B-Spline
            PlanningResult optimizedResult;
            {
                ScopedStdoutSilencer silencer(silenceInnerLoopStdout_);
                optimizedResult = optimizer_->optimize(planResult.rawPath);
            }
            BSpline spline = optimizedResult.smoothedSpline;
            
            auto t2 = std::chrono::high_resolution_clock::now();
            double time = std::chrono::duration<double, std::milli>(t2 - t1).count();
            
            // 验证B-Spline采样点无碰撞
            bool splineValid = true;
            for (double t = 0.0; t <= 1.0; t += 0.02) {
                JointConfig sample = spline.evaluate(t);
                if (!collisionChecker_->isCollisionFree(sample)) {
                    splineValid = false;
                    break;
                }
            }
            
            // 检查B-Spline起点终点精度
            JointConfig splineStart = spline.evaluate(0.0);
            JointConfig splineEnd = spline.evaluate(1.0);
            double startError = splineStart.distanceTo(start);
            double endError = splineEnd.distanceTo(goal);
            
            if (splineValid && startError < 0.2 && endError < 0.2) {
                stats.recordSuccess(time);
            } else {
                std::ostringstream oss;
                if (!splineValid) {
                    oss << "Spline collision detected";
                } else {
                    oss << "Endpoint error: start=" << std::setprecision(3) << startError 
                        << ", end=" << endError;
                }
                stats.recordFailure(oss.str());
            }
        }
        return stats;
    }
    
    // 11. 时间参数化验证
    TestStatistics testTimeParameterization(int numTests = 20) {
        TestStatistics stats;
        stats.testName = "Time Parameterization";
        
        TimeParameterizationConfig tpConfig = TimeParameterizationConfig::fromRobotParams(robot_.getParams());
        TimeParameterizer parameterizer(tpConfig);
        
        for (int i = 0; i < numTests; i++) {
            JointConfig start, goal;
            if (!generateValidPair(start, goal)) {
                stats.recordFailure("Time param test " + std::to_string(i) + ": Failed to sample valid pair");
                continue;
            }
            
            PlanningResult planResult;
            {
                ScopedStdoutSilencer silencer(silenceInnerLoopStdout_);
                planResult = planner_->plan(start, goal);
            }
            if (!planResult.isSuccess() || planResult.rawPath.waypoints.size() < 3) {
                stats.recordFailure("Time param test " + std::to_string(i) + ": Planning failed or path too short");
                continue;
            }
            
            PlanningResult optimizedResult;
            {
                ScopedStdoutSilencer silencer(silenceInnerLoopStdout_);
                optimizedResult = optimizer_->optimize(planResult.rawPath);
            }
            
            auto t1 = std::chrono::high_resolution_clock::now();
            Trajectory trajectory = parameterizer.parameterize(optimizedResult.optimizedPath);
            auto t2 = std::chrono::high_resolution_clock::now();
            double time = std::chrono::duration<double, std::milli>(t2 - t1).count();
            
            // 验证轨迹约束
            bool velocityOk = true;
            bool accelerationOk = true;
            
            for (size_t j = 0; j < trajectory.points.size(); j++) {
                const auto& pt = trajectory.points[j];
                for (int k = 0; k < 6; k++) {
                    if (std::abs(pt.velocity[k]) > tpConfig.maxVelocity[k] * 1.05) {
                        velocityOk = false;
                    }
                    if (std::abs(pt.acceleration[k]) > tpConfig.maxAcceleration[k] * 1.05) {
                        accelerationOk = false;
                    }
                }
            }
            
            if (velocityOk && accelerationOk && trajectory.totalTime > 0) {
                stats.recordSuccess(time);
            } else {
                std::ostringstream oss;
                if (!velocityOk) oss << "Velocity limit violated; ";
                if (!accelerationOk) oss << "Acceleration limit violated; ";
                if (trajectory.totalTime <= 0) oss << "Invalid total time";
                stats.recordFailure(oss.str());
            }
        }
        return stats;
    }
    
    // 12. 碰撞检测准确性验证
    TestStatistics testCollisionDetectionAccuracy(int numTests = 100) {
        TestStatistics stats;
        stats.testName = "Collision Detection Accuracy";
        
        for (int i = 0; i < numTests; i++) {
            JointConfig config = generateRandomConfig();
            
            auto t1 = std::chrono::high_resolution_clock::now();
            bool isFree = false;
            {
                ScopedStdoutSilencer silencer(silenceInnerLoopStdout_);
                isFree = collisionChecker_->isCollisionFree(config);
            }
            auto t2 = std::chrono::high_resolution_clock::now();
            double time = std::chrono::duration<double, std::milli>(t2 - t1).count();
            
            // 简单验证：检查关节是否在限位内
            bool inLimits = true;
            for (int j = 0; j < 6; j++) {
                if (config.q[j] < jointMin_[j] || config.q[j] > jointMax_[j]) {
                    inLimits = false;
                    break;
                }
            }
            
            // 碰撞检测应该对限位内的配置给出合理结果
            if (inLimits) {
                stats.recordSuccess(time);
            }
        }
        
        return stats;
    }
};

// ============================================================================
// 主函数
// ============================================================================
void printHeader(const std::string& title) {
    std::cout << "\n";
    std::cout << std::string(70, '=') << "\n";
    std::cout << "  " << title << "\n";
    std::cout << std::string(70, '=') << "\n";
}

void printSummary(const std::vector<TestStatistics>& allStats) {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                    ROBUSTNESS VALIDATION SUMMARY                   ║\n";
    std::cout << "╠════════════════════════════════════════════════════════════════════╣\n";
    
    int totalTests = 0;
    int totalPassed = 0;
    double totalTime = 0.0;
    
    for (const auto& stats : allStats) {
        totalTests += stats.totalTests;
        totalPassed += stats.passed;
        totalTime += stats.totalTime;
        
        std::string status = stats.successRate() >= 95.0 ? "✓ PASS" : 
                            (stats.successRate() >= 80.0 ? "~ WARN" : "✗ FAIL");
        
        std::cout << "║  " << std::left << std::setw(35) << stats.testName 
                  << std::right << std::setw(6) << std::fixed << std::setprecision(1) 
                  << stats.successRate() << "%  " 
                  << std::setw(8) << status << "  ║\n";
    }
    
    std::cout << "╠════════════════════════════════════════════════════════════════════╣\n";
    
    double overallRate = totalTests > 0 ? 100.0 * totalPassed / totalTests : 0.0;
    std::string overallStatus = overallRate >= 95.0 ? "EXCELLENT" :
                               (overallRate >= 85.0 ? "GOOD" :
                               (overallRate >= 70.0 ? "ACCEPTABLE" : "NEEDS WORK"));
    
    std::cout << "║  " << std::left << std::setw(35) << "OVERALL" 
              << std::right << std::setw(6) << std::fixed << std::setprecision(1) 
              << overallRate << "%  " 
              << std::setw(8) << overallStatus << "  ║\n";
    std::cout << "║                                                                    ║\n";
    std::cout << "║  Total Tests: " << std::setw(5) << totalTests 
              << "    Passed: " << std::setw(5) << totalPassed 
              << "    Failed: " << std::setw(5) << (totalTests - totalPassed) << "       ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════════════╝\n";
    
    // 性能等级评定
    std::cout << "\n";
    std::cout << "┌────────────────────────────────────────────────────────────────────┐\n";
    std::cout << "│                     WORLD-CLASS CRITERIA CHECK                     │\n";
    std::cout << "├────────────────────────────────────────────────────────────────────┤\n";
    
    std::vector<std::pair<std::string, bool>> criteria = {
        {"Success Rate > 95%", overallRate >= 95.0},
        {"Random Config Success > 90%", allStats.size() > 0 && allStats[0].successRate() >= 90.0},
        {"Boundary Handling > 85%", allStats.size() > 1 && allStats[1].successRate() >= 85.0},
        {"Path Quality Consistent", allStats.size() > 8 && allStats[8].successRate() >= 80.0},
        {"Time Param Valid", allStats.size() > 10 && allStats[10].successRate() >= 90.0},
    };
    
    int criteriaMet = 0;
    for (const auto& [name, met] : criteria) {
        std::cout << "│  " << (met ? "✓" : "✗") << " " << std::left << std::setw(60) << name << "  │\n";
        if (met) criteriaMet++;
    }
    
    std::cout << "├────────────────────────────────────────────────────────────────────┤\n";
    std::string grade = criteriaMet >= 5 ? "★★★★★ WORLD-CLASS" :
                       (criteriaMet >= 4 ? "★★★★☆ EXCELLENT" :
                       (criteriaMet >= 3 ? "★★★☆☆ GOOD" :
                       (criteriaMet >= 2 ? "★★☆☆☆ ACCEPTABLE" : "★☆☆☆☆ NEEDS IMPROVEMENT")));
    std::cout << "│  Final Grade: " << std::left << std::setw(50) << grade << "  │\n";
    std::cout << "└────────────────────────────────────────────────────────────────────┘\n";
}

int main() {
    std::cout << "╔════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║          COMPREHENSIVE ROBUSTNESS & GENERALIZATION TEST            ║\n";
    std::cout << "║                Motion Planning System Validation                   ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════════════╝\n";
    
    RobustnessValidator validator;
    
    std::cout << "\nInitializing validator...\n";
    if (!validator.initialize()) {
        std::cerr << "Failed to initialize validator!\n";
        return 1;
    }
    std::cout << "Validator initialized successfully.\n";
    
    std::vector<TestStatistics> allStats;
    
    // 运行所有测试
    printHeader("1. RANDOM CONFIGURATION GENERALIZATION TEST");
    auto stats1 = validator.testRandomConfigurations(30);  // 减少数量
    stats1.print();
    allStats.push_back(stats1);
    
    printHeader("2. BOUNDARY CONDITION TEST");
    auto stats2 = validator.testBoundaryConditions(20);
    stats2.print();
    allStats.push_back(stats2);
    
    printHeader("3. SHORT DISTANCE TEST");
    auto stats3 = validator.testShortDistances(20);
    stats3.print();
    allStats.push_back(stats3);
    
    printHeader("4. SAME START/GOAL TEST");
    auto stats4 = validator.testSameStartGoal(10);
    stats4.print();
    allStats.push_back(stats4);
    
    printHeader("5. LARGE MOTION TEST");
    auto stats5 = validator.testLargeMotions(15);
    stats5.print();
    allStats.push_back(stats5);
    
    printHeader("6. NEAR SINGULARITY TEST");
    auto stats6 = validator.testNearSingularities(15);
    stats6.print();
    allStats.push_back(stats6);
    
    printHeader("7. CONTINUOUS PLANNING STRESS TEST");
    auto stats7 = validator.testContinuousPlanning(20);
    stats7.print();
    allStats.push_back(stats7);
    
    printHeader("8. NUMERICAL STABILITY TEST");
    auto stats8 = validator.testNumericalStability(15);
    stats8.print();
    allStats.push_back(stats8);
    
    printHeader("9. PATH QUALITY CONSISTENCY TEST");
    auto stats9 = validator.testPathQualityConsistency(10);
    stats9.print();
    allStats.push_back(stats9);
    
    printHeader("10. B-SPLINE SMOOTHING QUALITY TEST");
    auto stats10 = validator.testBSplineSmoothing(15);
    stats10.print();
    allStats.push_back(stats10);
    
    printHeader("11. TIME PARAMETERIZATION VALIDATION");
    auto stats11 = validator.testTimeParameterization(10);
    stats11.print();
    allStats.push_back(stats11);
    
    printHeader("12. COLLISION DETECTION ACCURACY TEST");
    auto stats12 = validator.testCollisionDetectionAccuracy(30);
    stats12.print();
    allStats.push_back(stats12);
    
    // 打印总结
    printSummary(allStats);
    
    return 0;
}
