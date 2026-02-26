/**
 * @file PalletizingPlanner.hpp
 * @brief 码垛规划器 - 顶层接口
 * 
 * 整合所有子模块，提供简洁的码垛任务规划接口。
 * 特别针对Pick-and-Place任务优化。
 * 
 * @author Guangdong Huayan Robotics Co., Ltd.
 * @version 1.0.0
 * @date 2026-01-29
 */

#pragma once

#include "Types.hpp"
#include "RobotModel.hpp"
#include "CollisionChecker.hpp"
#include "PathPlannerOptimized.hpp"
#include "PathOptimizer.hpp"

#include <memory>
#include <vector>
#include <functional>
#include <fstream>

namespace palletizing {

/**
 * @brief 码垛任务描述
 */
struct PalletizingTask {
    // 抓取位姿
    JointConfig pickConfig;     // 抓取位置的关节配置
    Pose6D pickPose;            // 抓取位置的笛卡尔位姿
    
    // 放置位姿  
    JointConfig placeConfig;    // 放置位置的关节配置
    Pose6D placePose;           // 放置位置的笛卡尔位姿
    
    // 接近/撤退偏移
    double approachOffset = 0.1;  // 接近偏移 [m]
    double retractOffset = 0.15;  // 撤退偏移 [m]
    
    // 安全高度
    double safeHeight = 0.3;      // 安全高度 [m]
    
    // 任务ID
    int taskId = 0;
    std::string description;
};

/**
 * @brief 码垛规划结果
 */
struct PalletizingResult {
    bool success = false;
    
    // 分段路径
    Path approachPath;      // 接近路径 (安全位置 -> 抓取预位置)
    Path pickPath;          // 抓取路径 (预位置 -> 抓取)
    Path liftPath;          // 提升路径 (抓取 -> 安全高度)
    Path transferPath;      // 转移路径 (安全高度 -> 放置安全高度)
    Path descendPath;       // 下降路径 (安全高度 -> 放置预位置)
    Path placePath;         // 放置路径 (预位置 -> 放置)
    Path retreatPath;       // 撤退路径 (放置 -> 安全位置)
    
    // 完整路径
    Path completePath;
    BSpline smoothedSpline;
    
    // 统计
    double totalPlanningTime = 0.0;
    double totalPathLength = 0.0;
    double maxCurvature = 0.0;
    
    std::string errorMessage;
};

/**
 * @brief 码垛规划器
 * 
 * 提供完整的码垛任务规划功能，包括:
 * - Pick-and-Place路径规划
 * - 多任务序列优化
 * - 实时重规划支持
 */
class PalletizingPlanner {
public:
    using ProgressCallback = std::function<void(int current, int total, const std::string& status)>;
    
    // 禁止复制和移动，避免内部引用悬空
    PalletizingPlanner(const PalletizingPlanner&) = delete;
    PalletizingPlanner& operator=(const PalletizingPlanner&) = delete;
    PalletizingPlanner(PalletizingPlanner&&) = delete;
    PalletizingPlanner& operator=(PalletizingPlanner&&) = delete;
    
    /**
     * @brief 构造函数
     * @param robotParams 机器人参数
     */
    explicit PalletizingPlanner(const RobotDHParams& robotParams = RobotDHParams::fromHRConfig())
        : robot_(robotParams),
          checker_(robot_),
          optimizedPlanner_(robot_, checker_),
          optimizer_(robot_, checker_) {
    }
    
    /**
     * @brief 初始化场景
     * @param scene 场景配置
     * @return 是否成功
     */
    bool initialize(const SceneConfig& scene = SceneConfig::defaultPalletizing()) {
        scene_ = scene;
        return checker_.initialize(scene);
    }
    
    /**
     * @brief 设置规划器配置
     */
    void setConfig(const PlannerConfig& config) {
        config_ = config;
    }
    
    /**
     * @brief 设置工具配置
     */
    void setToolConfig(const ToolConfig& tool) {
        toolConfig_ = tool;
        
        // 设置碰撞模型
        if (tool.type == ToolConfig::GripperType::Suction) {
            Eigen::Vector3d offset(0, 0, tool.toolFrame.position.z());
            checker_.setToolCollisionBall(offset, tool.toolRadius);
        }
    }
    
    /**
     * @brief 规划单个Pick-and-Place任务
     * @param task 任务描述
     * @return 规划结果
     */
    PalletizingResult planPickAndPlace(const PalletizingTask& task) {
        PalletizingResult result;
        auto startTime = std::chrono::high_resolution_clock::now();
        
        // 1. 生成关键路径点
        std::vector<JointConfig> keyConfigs = generateKeyConfigs(task);
        if (keyConfigs.empty()) {
            result.errorMessage = "Failed to generate key configurations";
            return result;
        }
        
        // 2. 规划各段路径
        Path completePath;
        
        for (size_t i = 0; i < keyConfigs.size() - 1; ++i) {
            PlanningResult segResult = optimizedPlanner_.plan(keyConfigs[i], keyConfigs[i + 1]);
            
            if (!segResult.isSuccess()) {
                result.errorMessage = "Failed to plan segment " + std::to_string(i) + 
                                     ": " + segResult.statusString();
                return result;
            }
            
            // 合并路径 (避免重复点)
            for (size_t j = (i == 0 ? 0 : 1); j < segResult.rawPath.waypoints.size(); ++j) {
                completePath.waypoints.push_back(segResult.rawPath.waypoints[j]);
            }
        }
        
        // 3. 优化路径
        PlanningResult optResult = optimizer_.optimize(completePath);
        
        if (!optResult.isSuccess()) {
            result.errorMessage = "Path optimization failed";
            return result;
        }
        
        result.completePath = optResult.optimizedPath;
        result.smoothedSpline = optResult.smoothedSpline;
        
        // 4. 计算统计信息
        auto endTime = std::chrono::high_resolution_clock::now();
        result.totalPlanningTime = std::chrono::duration<double>(endTime - startTime).count();
        result.totalPathLength = result.completePath.totalLength();
        result.maxCurvature = optResult.maxCurvature;
        result.success = true;
        
        return result;
    }
    
    /**
     * @brief 规划点到点运动
     * @param start 起始配置
     * @param goal 目标配置
     * @return 规划结果
     */
    PlanningResult planPointToPoint(const JointConfig& start, const JointConfig& goal) {
        PlanningResult result = optimizedPlanner_.plan(start, goal);
        
        if (result.isSuccess()) {
            result = optimizer_.optimize(result.rawPath);
        }
        
        return result;
    }
    
    /**
     * @brief 规划多任务序列
     * @param tasks 任务列表
     * @param startConfig 初始配置
     * @param callback 进度回调
     * @return 所有任务的规划结果
     */
    std::vector<PalletizingResult> planTaskSequence(
            const std::vector<PalletizingTask>& tasks,
            const JointConfig& startConfig,
            ProgressCallback callback = nullptr) {
        
        std::vector<PalletizingResult> results;
        results.reserve(tasks.size());
        
        JointConfig currentConfig = startConfig;
        
        for (size_t i = 0; i < tasks.size(); ++i) {
            if (callback) {
                callback(static_cast<int>(i), static_cast<int>(tasks.size()),
                        "Planning task " + std::to_string(i + 1));
            }
            
            PalletizingTask task = tasks[i];
            
            // 规划从当前位置到抓取位置的路径
            PlanningResult toPickResult = optimizedPlanner_.plan(currentConfig, task.pickConfig);
            
            if (!toPickResult.isSuccess()) {
                PalletizingResult failResult;
                failResult.errorMessage = "Failed to plan path to pick position";
                results.push_back(failResult);
                continue;
            }
            
            // 规划完整的Pick-and-Place
            PalletizingResult taskResult = planPickAndPlace(task);
            
            if (taskResult.success) {
                // 将到达抓取位置的路径合并到任务结果的接近路径前面
                Path combinedPath;
                for (const auto& wp : toPickResult.rawPath.waypoints) {
                    combinedPath.waypoints.push_back(wp);
                }
                // 避免重复第一个点
                for (size_t j = 1; j < taskResult.completePath.waypoints.size(); ++j) {
                    combinedPath.waypoints.push_back(taskResult.completePath.waypoints[j]);
                }
                taskResult.completePath = combinedPath;
                taskResult.approachPath = toPickResult.rawPath;
                
                currentConfig = task.placeConfig;
            }
            
            results.push_back(taskResult);
        }
        
        return results;
    }
    
    /**
     * @brief 检查配置是否有效
     */
    bool isConfigValid(const JointConfig& config) const {
        return robot_.isWithinLimits(config) && checker_.isCollisionFree(config);
    }
    
    /**
     * @brief 生成随机有效配置
     */
    JointConfig generateRandomValidConfig(int maxAttempts = 1000) {
        for (int i = 0; i < maxAttempts; ++i) {
            JointConfig config = robot_.randomConfig();
            if (isConfigValid(config)) {
                return config;
            }
        }
        return JointConfig();  // 返回零配置表示失败
    }
    
    /**
     * @brief 可视化路径 (输出到文件)
     */
    void savePathToFile(const Path& path, const std::string& filename) const {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "[PalletizingPlanner] 无法打开文件: " << filename << "\n";
            return;
        }
        
        file << "# Palletizing Path\n";
        file << "# Format: q1 q2 q3 q4 q5 q6 (radians)\n";
        
        for (const auto& wp : path.waypoints) {
            for (int i = 0; i < 6; ++i) {
                file << wp.config.q[i];
                if (i < 5) file << " ";
            }
            file << "\n";
        }
        
        file.close();
    }
    
    /**
     * @brief 保存B-Spline到文件
     */
    void saveSplineToFile(const BSpline& spline, const std::string& filename, 
                          int samples = 200) const {
        std::ofstream file(filename);
        if (!file.is_open()) return;
        
        file << "# B-Spline Path (sampled)\n";
        file << "# Format: t q1 q2 q3 q4 q5 q6\n";
        
        for (int i = 0; i < samples; ++i) {
            double t = static_cast<double>(i) / (samples - 1);
            JointConfig config = spline.evaluate(t);
            
            file << t;
            for (int j = 0; j < 6; ++j) {
                file << " " << config.q[j];
            }
            file << "\n";
        }
        
        file.close();
    }
    
    // 访问器
    const RobotModel& getRobot() const { return robot_; }
    CollisionChecker& getCollisionChecker() { return checker_; }
    const SceneConfig& getScene() const { return scene_; }
    
private:
    /**
     * @brief 生成关键配置点序列
     * 
     * 生成完整的8段码垛路径关键点:
     * 1. 抓取预位置 (接近偏移)
     * 2. 抓取位置
     * 3. 抓取后提升位置 (安全高度)
     * 4. 转移路径中间点 (高安全高度)
     * 5. 放置预位置上方 (安全高度)
     * 6. 放置预位置 (接近偏移)
     * 7. 放置位置
     * 8. 撤退位置 (安全高度)
     */
    std::vector<JointConfig> generateKeyConfigs(const PalletizingTask& task) {
        std::vector<JointConfig> configs;
        
        // 计算抓取和放置位姿的FK末端位置，用于生成中间路径点
        // (注: 精确的笛卡尔偏移需IK, 这里使用关节空间近似)
        auto offsetConfig = [&](const JointConfig& baseConfig, double zOffset) -> JointConfig {
            // 通过微调J5关节实现近似Z方向偏移
            // 注: 精确的笛卡尔偏移需要IK, 这里使用关节空间近似
            JointConfig offset = baseConfig;
            // J2和J3协同调整实现竖直方向偏移
            double adjustAngle = std::atan2(zOffset, robot_.getParams().a2 / 1000.0);
            offset.q[1] -= adjustAngle;  // J2向上抬
            offset.q[2] += adjustAngle * 0.5;  // J3补偿
            return robot_.clampToLimits(offset);
        };
        
        double approachH = task.approachOffset;
        double safeH = task.safeHeight;
        
        // 1. 抓取预位置 (从上方接近, 偏移approachOffset)
        configs.push_back(offsetConfig(task.pickConfig, approachH));
        
        // 2. 抓取位置
        configs.push_back(task.pickConfig);
        
        // 3. 抓取后提升到安全高度
        configs.push_back(offsetConfig(task.pickConfig, safeH));
        
        // 4. 转移路径中间点 (在安全高度平面上的线性插值中点)
        JointConfig midTransfer;
        JointConfig liftPick = offsetConfig(task.pickConfig, safeH);
        JointConfig liftPlace = offsetConfig(task.placeConfig, safeH);
        midTransfer.q = 0.5 * (liftPick.q + liftPlace.q);
        midTransfer = robot_.clampToLimits(midTransfer);
        configs.push_back(midTransfer);
        
        // 5. 放置上方安全高度
        configs.push_back(offsetConfig(task.placeConfig, safeH));
        
        // 6. 放置预位置 (从上方接近)
        configs.push_back(offsetConfig(task.placeConfig, approachH));
        
        // 7. 放置位置
        configs.push_back(task.placeConfig);
        
        // 8. 撤退位置 (提升到安全高度)
        configs.push_back(offsetConfig(task.placeConfig, safeH));
        
        return configs;
    }
    
    RobotModel robot_;
    CollisionChecker checker_;
    OptimizedPathPlanner optimizedPlanner_;  // 高性能规划器 (默认)
    PathOptimizer optimizer_;
    
    SceneConfig scene_;
    PlannerConfig config_;
    ToolConfig toolConfig_;
};

} // namespace palletizing
