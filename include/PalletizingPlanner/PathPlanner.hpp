/**
 * @file PathPlanner.hpp
 * @brief 世界顶尖路径规划器实现
 * 
 * 实现多种先进的采样式路径规划算法:
 * - Informed RRT* (Gammell et al., 2014)
 * - BIT* (Gammell et al., 2015)
 * - 自适应采样和启发式优化
 * 
 * 设计目标：
 * 1. 渐进最优 - 随时间改善路径质量
 * 2. 路径最短 - 配置空间度量优化
 * 3. 高效采样 - 椭球和启发式采样
 * 
 * @author Guangdong Huayan Robotics Co., Ltd.
 * @version 1.0.0
 * @date 2026-01-29
 */

#pragma once

#include "Types.hpp"
#include "RobotModel.hpp"
#include "CollisionChecker.hpp"

#include <random>
#include <queue>
#include <unordered_set>
#include <chrono>
#include <algorithm>
#include <future>

namespace palletizing {

/**
 * @brief RRT树节点
 */
struct RRTNode {
    int id;                      // 节点ID
    JointConfig config;          // 关节配置
    int parentId = -1;           // 父节点ID
    double costFromStart = 0.0;  // 从起点的代价
    std::vector<int> children;   // 子节点ID列表
    
    RRTNode() : id(-1) {}
    RRTNode(int nodeId, const JointConfig& cfg) : id(nodeId), config(cfg) {}
};

/**
 * @brief 椭球采样器 (用于Informed RRT*)
 */
class EllipsoidSampler {
public:
    EllipsoidSampler(const JointConfig& start, const JointConfig& goal)
        : start_(start), goal_(goal) {
        
        // 默认关节限位: 使用HR_S50-2000参数
        RobotDHParams defaultParams;
        auto [minLimits, maxLimits] = defaultParams.getJointLimits();
        jointMin_ = minLimits;
        jointMax_ = maxLimits;
        
        // 计算椭球参数
        center_ = 0.5 * (start.q + goal.q);
        cMin_ = (goal.q - start.q).norm();
        
        // 计算旋转矩阵 (从单位球到椭球)
        Eigen::VectorXd a1 = (goal.q - start.q).normalized();
        
        // 构造正交基
        Eigen::Matrix<double, 6, 6> M = Eigen::Matrix<double, 6, 6>::Identity();
        M.col(0) = a1;
        
        // Gram-Schmidt正交化
        for (int i = 1; i < 6; ++i) {
            Eigen::VectorXd v = M.col(i);
            for (int j = 0; j < i; ++j) {
                v -= v.dot(M.col(j)) * M.col(j);
            }
            if (v.norm() > 1e-9) {
                M.col(i) = v.normalized();
            }
        }
        
        rotation_ = M;
    }
    
    /**
     * @brief 在椭球内采样
     * @param cBest 当前最佳路径长度
     * @param gen 随机数生成器
     * @return 采样点
     */
    JointConfig sample(double cBest, std::mt19937& gen) {
        if (cBest >= std::numeric_limits<double>::infinity()) {
            // 无限椭球 = 均匀采样
            return sampleUniform(gen);
        }
        
        // 椭球半轴长度
        std::vector<double> r(6);
        r[0] = cBest / 2.0;
        double r_rest = std::sqrt(cBest * cBest - cMin_ * cMin_) / 2.0;
        for (int i = 1; i < 6; ++i) {
            r[i] = r_rest;
        }
        
        // 在单位球内均匀采样
        std::normal_distribution<double> normal(0.0, 1.0);
        JointVector xBall;
        for (int i = 0; i < 6; ++i) {
            xBall[i] = normal(gen);
        }
        
        double norm = xBall.norm();
        std::uniform_real_distribution<double> uniform(0.0, 1.0);
        double rad = std::pow(uniform(gen), 1.0 / 6.0);  // 6维球体采样
        xBall = xBall / norm * rad;
        
        // 缩放到椭球
        for (int i = 0; i < 6; ++i) {
            xBall[i] *= r[i];
        }
        
        // 旋转并平移
        JointVector xEllipsoid = rotation_ * xBall + center_;
        
        return JointConfig(xEllipsoid);
    }
    
private:
    JointConfig sampleUniform(std::mt19937& gen) {
        JointVector q;
        for (int i = 0; i < 6; ++i) {
            std::uniform_real_distribution<double> dist(jointMin_[i], jointMax_[i]);
            q[i] = dist(gen);
        }
        return JointConfig(q);
    }
    
    JointConfig start_, goal_;
    JointVector center_;
    JointVector jointMin_;
    JointVector jointMax_;
    double cMin_;
    Eigen::Matrix<double, 6, 6> rotation_;
};

/**
 * @brief 先进的路径规划器 - Informed RRT* + BIT*混合
 */
class PathPlanner {
public:
    /**
     * @brief 构造函数
     * @param robot 机器人模型
     * @param checker 碰撞检测器
     * @param config 规划器配置
     */
    PathPlanner(const RobotModel& robot, 
                CollisionChecker& checker,
                const PlannerConfig& config = PlannerConfig())
        : robot_(robot), checker_(checker), config_(config),
          gen_(std::random_device{}()) {
    }
    
    /**
     * @brief 规划路径
     * @param start 起始配置
     * @param goal 目标配置
     * @return 规划结果
     */
    PlanningResult plan(const JointConfig& start, const JointConfig& goal) {
        auto startTime = std::chrono::high_resolution_clock::now();
        
        PlanningResult result;
        
        // 验证起点和终点
        if (!robot_.isWithinLimits(start)) {
            result.status = PlanningStatus::InvalidStart;
            return result;
        }
        
        if (!robot_.isWithinLimits(goal)) {
            result.status = PlanningStatus::InvalidGoal;
            return result;
        }
        
        if (!checker_.isCollisionFree(start)) {
            result.status = PlanningStatus::CollisionAtStart;
            return result;
        }
        
        if (!checker_.isCollisionFree(goal)) {
            result.status = PlanningStatus::CollisionAtGoal;
            return result;
        }
        
        // 选择规划算法
        Path rawPath;
        
        switch (config_.plannerType) {
            case PlannerType::InformedRRTStar:
                rawPath = planInformedRRTStar(start, goal, result);
                break;
                
            case PlannerType::BITStar:
                rawPath = planBITStar(start, goal, result);
                break;
                
            case PlannerType::RRTStar:
                rawPath = planRRTStar(start, goal, result);
                break;
                
            default:
                rawPath = planInformedRRTStar(start, goal, result);
                break;
        }
        
        auto endPlanTime = std::chrono::high_resolution_clock::now();
        result.planningTime = std::chrono::duration<double>(endPlanTime - startTime).count();
        
        if (rawPath.empty()) {
            result.status = PlanningStatus::NoPath;
            return result;
        }
        
        result.rawPath = rawPath;
        result.status = PlanningStatus::Success;
        
        return result;
    }
    
    /**
     * @brief 更新配置
     */
    void setConfig(const PlannerConfig& config) {
        config_ = config;
    }
    
private:
    /**
     * @brief Informed RRT*算法实现
     */
    Path planInformedRRTStar(const JointConfig& start, const JointConfig& goal,
                             PlanningResult& result) {
        
        // 初始化树
        nodes_.clear();
        nodes_.reserve(config_.maxIterations);
        
        RRTNode startNode(0, start);
        startNode.costFromStart = 0.0;
        nodes_.push_back(startNode);
        
        // 初始化椭球采样器
        EllipsoidSampler ellipsoidSampler(start, goal);
        double cBest = std::numeric_limits<double>::infinity();
        int goalNodeId = -1;
        
        // 分布
        std::uniform_real_distribution<double> uniformDist(0.0, 1.0);
        
        auto startTime = std::chrono::high_resolution_clock::now();
        
        for (int iter = 0; iter < config_.maxIterations; ++iter) {
            // 检查超时
            auto now = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double>(now - startTime).count();
            if (elapsed > config_.maxPlanningTime) {
                break;
            }
            
            // 采样
            JointConfig qRand;
            if (uniformDist(gen_) < config_.goalBias && goalNodeId < 0) {
                // 目标偏向
                qRand = goal;
            } else if (config_.useInformedSampling && cBest < std::numeric_limits<double>::infinity()) {
                // 椭球采样
                qRand = ellipsoidSampler.sample(cBest, gen_);
                qRand = robot_.clampToLimits(qRand);
            } else {
                // 均匀采样
                qRand = robot_.randomConfig();
            }
            
            // 找最近节点
            int nearestId = findNearest(qRand);
            if (nearestId < 0) continue;
            
            // 扩展
            JointConfig qNew = steer(nodes_[nearestId].config, qRand, config_.stepSize);
            
            if (!robot_.isWithinLimits(qNew)) continue;
            if (!checker_.isPathCollisionFree(nodes_[nearestId].config, qNew, config_.collisionResolution)) continue;
            
            // 在邻域内找最佳父节点 (RRT*)
            double rewireRadius = computeRewireRadius();
            std::vector<int> neighbors = findNear(qNew, rewireRadius);
            
            int bestParent = nearestId;
            double bestCost = nodes_[nearestId].costFromStart + 
                             nodes_[nearestId].config.distanceTo(qNew);
            
            for (int neighborId : neighbors) {
                double newCost = nodes_[neighborId].costFromStart + 
                                nodes_[neighborId].config.distanceTo(qNew);
                
                if (newCost < bestCost) {
                    if (checker_.isPathCollisionFree(nodes_[neighborId].config, qNew, config_.collisionResolution)) {
                        bestParent = neighborId;
                        bestCost = newCost;
                    }
                }
            }
            
            // 添加新节点
            int newNodeId = static_cast<int>(nodes_.size());
            RRTNode newNode(newNodeId, qNew);
            newNode.parentId = bestParent;
            newNode.costFromStart = bestCost;
            nodes_.push_back(newNode);
            nodes_[bestParent].children.push_back(newNodeId);
            
            // 重连邻居 (RRT* rewiring)
            for (int neighborId : neighbors) {
                if (neighborId == bestParent) continue;
                
                double newCost = newNode.costFromStart + 
                                qNew.distanceTo(nodes_[neighborId].config);
                
                if (newCost < nodes_[neighborId].costFromStart) {
                    if (checker_.isPathCollisionFree(qNew, nodes_[neighborId].config, config_.collisionResolution)) {
                        // 移除旧的父子关系
                        int oldParent = nodes_[neighborId].parentId;
                        if (oldParent >= 0) {
                            auto& children = nodes_[oldParent].children;
                            children.erase(std::remove(children.begin(), children.end(), neighborId), children.end());
                        }
                        
                        // 建立新的父子关系
                        nodes_[neighborId].parentId = newNodeId;
                        nodes_[neighborId].costFromStart = newCost;
                        nodes_[newNodeId].children.push_back(neighborId);
                        
                        // 传播代价更新
                        propagateCostUpdate(neighborId);
                    }
                }
            }
            
            // 检查是否到达目标
            double distToGoal = qNew.distanceTo(goal);
            if (distToGoal < config_.stepSize) {
                if (checker_.isPathCollisionFree(qNew, goal, config_.collisionResolution)) {
                    double pathCost = newNode.costFromStart + distToGoal;
                    
                    if (pathCost < cBest) {
                        cBest = pathCost;
                        
                        // 添加目标节点或更新
                        if (goalNodeId < 0) {
                            goalNodeId = static_cast<int>(nodes_.size());
                            RRTNode goalNode(goalNodeId, goal);
                            goalNode.parentId = newNodeId;
                            goalNode.costFromStart = pathCost;
                            nodes_.push_back(goalNode);
                            nodes_[newNodeId].children.push_back(goalNodeId);
                        } else if (pathCost < nodes_[goalNodeId].costFromStart) {
                            // 更新目标节点的父节点
                            int oldParent = nodes_[goalNodeId].parentId;
                            if (oldParent >= 0) {
                                auto& children = nodes_[oldParent].children;
                                children.erase(std::remove(children.begin(), children.end(), goalNodeId), children.end());
                            }
                            
                            nodes_[goalNodeId].parentId = newNodeId;
                            nodes_[goalNodeId].costFromStart = pathCost;
                            nodes_[newNodeId].children.push_back(goalNodeId);
                        }
                    }
                }
            }
            
            result.iterations = iter + 1;
        }
        
        result.nodesExplored = static_cast<int>(nodes_.size());
        
        // 提取路径
        if (goalNodeId >= 0) {
            return extractPath(goalNodeId);
        }
        
        return Path();
    }
    
    /**
     * @brief RRT*算法 (简化版，无椭球采样)
     */
    Path planRRTStar(const JointConfig& start, const JointConfig& goal,
                     PlanningResult& result) {
        // 禁用椭球采样
        PlannerConfig tempConfig = config_;
        tempConfig.useInformedSampling = false;
        
        auto oldConfig = config_;
        config_ = tempConfig;
        
        Path path = planInformedRRTStar(start, goal, result);
        
        config_ = oldConfig;
        return path;
    }
    
    /**
     * @brief BIT*算法实现
     */
    Path planBITStar(const JointConfig& start, const JointConfig& goal,
                     PlanningResult& result) {
        // BIT*: Batch Informed Trees
        // 批量添加样本，构建隐式随机几何图，然后搜索
        
        nodes_.clear();
        
        RRTNode startNode(0, start);
        startNode.costFromStart = 0.0;
        nodes_.push_back(startNode);
        
        EllipsoidSampler ellipsoidSampler(start, goal);
        double cBest = std::numeric_limits<double>::infinity();
        int goalNodeId = -1;
        
        auto startTime = std::chrono::high_resolution_clock::now();
        int totalSamples = 0;
        
        while (totalSamples < config_.maxIterations) {
            // 检查超时
            auto now = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double>(now - startTime).count();
            if (elapsed > config_.maxPlanningTime) {
                break;
            }
            
            // 生成一批样本
            std::vector<JointConfig> batch;
            batch.reserve(config_.batchSize);
            
            for (int i = 0; i < config_.batchSize && totalSamples < config_.maxIterations; ++i) {
                JointConfig sample;
                
                if (cBest < std::numeric_limits<double>::infinity()) {
                    sample = ellipsoidSampler.sample(cBest, gen_);
                    sample = robot_.clampToLimits(sample);
                } else {
                    sample = robot_.randomConfig();
                }
                
                if (checker_.isCollisionFree(sample)) {
                    batch.push_back(sample);
                }
                
                totalSamples++;
            }
            
            // 将有效样本加入树
            for (const auto& sample : batch) {
                // 找最近节点
                int nearestId = findNearest(sample);
                if (nearestId < 0) continue;
                
                double dist = nodes_[nearestId].config.distanceTo(sample);
                if (dist > config_.rewireRadius * 2) continue;
                
                // 检查路径
                if (!checker_.isPathCollisionFree(nodes_[nearestId].config, sample, config_.collisionResolution)) {
                    continue;
                }
                
                // 添加节点
                int newNodeId = static_cast<int>(nodes_.size());
                RRTNode newNode(newNodeId, sample);
                newNode.parentId = nearestId;
                newNode.costFromStart = nodes_[nearestId].costFromStart + dist;
                nodes_.push_back(newNode);
                nodes_[nearestId].children.push_back(newNodeId);
                
                // 检查目标可达性
                double distToGoal = sample.distanceTo(goal);
                if (distToGoal < config_.stepSize * 2) {
                    if (checker_.isPathCollisionFree(sample, goal, config_.collisionResolution)) {
                        double pathCost = newNode.costFromStart + distToGoal;
                        
                        if (pathCost < cBest) {
                            cBest = pathCost;
                            
                            if (goalNodeId < 0) {
                                goalNodeId = static_cast<int>(nodes_.size());
                                RRTNode goalNode(goalNodeId, goal);
                                goalNode.parentId = newNodeId;
                                goalNode.costFromStart = pathCost;
                                nodes_.push_back(goalNode);
                            } else {
                                int oldParent = nodes_[goalNodeId].parentId;
                                if (oldParent >= 0) {
                                    auto& children = nodes_[oldParent].children;
                                    children.erase(std::remove(children.begin(), children.end(), goalNodeId), children.end());
                                }
                                nodes_[goalNodeId].parentId = newNodeId;
                                nodes_[goalNodeId].costFromStart = pathCost;
                            }
                            nodes_[newNodeId].children.push_back(goalNodeId);
                        }
                    }
                }
            }
            
            // 剪枝 (可选)
            if (cBest < std::numeric_limits<double>::infinity()) {
                pruneTree(cBest * config_.pruneThreshold);
            }
            
            result.iterations = totalSamples;
        }
        
        result.nodesExplored = static_cast<int>(nodes_.size());
        
        if (goalNodeId >= 0) {
            return extractPath(goalNodeId);
        }
        
        return Path();
    }
    
    /**
     * @brief 找最近节点
     */
    int findNearest(const JointConfig& q) const {
        int nearestId = -1;
        double minDist = std::numeric_limits<double>::infinity();
        
        for (size_t i = 0; i < nodes_.size(); ++i) {
            double dist = nodes_[i].config.distanceTo(q);
            if (dist < minDist) {
                minDist = dist;
                nearestId = static_cast<int>(i);
            }
        }
        
        return nearestId;
    }
    
    /**
     * @brief 找邻域内的节点
     */
    std::vector<int> findNear(const JointConfig& q, double radius) const {
        std::vector<int> neighbors;
        
        for (size_t i = 0; i < nodes_.size(); ++i) {
            if (nodes_[i].config.distanceTo(q) <= radius) {
                neighbors.push_back(static_cast<int>(i));
            }
        }
        
        return neighbors;
    }
    
    /**
     * @brief 向目标方向扩展
     */
    JointConfig steer(const JointConfig& from, const JointConfig& to, double stepSize) const {
        JointVector direction = to.q - from.q;
        double dist = direction.norm();
        
        if (dist <= stepSize) {
            return to;
        }
        
        direction.normalize();
        JointVector newQ = from.q + stepSize * direction;
        
        return JointConfig(newQ);
    }
    
    /**
     * @brief 计算RRT*重连半径
     */
    double computeRewireRadius() const {
        // gamma_RRT* formula
        double n = static_cast<double>(nodes_.size());
        double d = 6.0;  // 6维空间
        
        // 计算自由空间体积的近似
        double spaceMeasure = 1.0;
        auto [minLimits, maxLimits] = robot_.getParams().getJointLimits();
        for (int i = 0; i < 6; ++i) {
            spaceMeasure *= (maxLimits[i] - minLimits[i]);
        }
        
        // gamma = 2 * (1 + 1/d)^(1/d) * (mu_free / zeta_d)^(1/d)
        double gamma = 2.0 * std::pow(1.0 + 1.0/d, 1.0/d) * 
                       std::pow(spaceMeasure / std::pow(M_PI, d/2.0), 1.0/d);
        
        double radius = gamma * std::pow(std::log(n + 1) / (n + 1), 1.0/d);
        
        return std::min(radius, config_.rewireRadius);
    }
    
    /**
     * @brief 传播代价更新
     */
    void propagateCostUpdate(int nodeId) {
        std::queue<int> queue;
        queue.push(nodeId);
        
        while (!queue.empty()) {
            int currentId = queue.front();
            queue.pop();
            
            for (int childId : nodes_[currentId].children) {
                double newCost = nodes_[currentId].costFromStart + 
                                nodes_[currentId].config.distanceTo(nodes_[childId].config);
                
                if (newCost < nodes_[childId].costFromStart) {
                    nodes_[childId].costFromStart = newCost;
                    queue.push(childId);
                }
            }
        }
    }
    
    /**
     * @brief 剪枝 - 移除代价超过阈值的节点
     * @param threshold 代价阈值，超过此值的节点将被标记为无效
     */
    void pruneTree(double threshold) {
        // 标记代价超过阈值的叶节点
        // 注: 不实际删除以避免ID失效，仅断开连接
        for (size_t i = 1; i < nodes_.size(); ++i) {  // 跳过根节点
            if (nodes_[i].costFromStart > threshold && nodes_[i].children.empty()) {
                // 从父节点的children列表中移除
                int parentId = nodes_[i].parentId;
                if (parentId >= 0 && parentId < static_cast<int>(nodes_.size())) {
                    auto& siblings = nodes_[parentId].children;
                    siblings.erase(
                        std::remove(siblings.begin(), siblings.end(), static_cast<int>(i)),
                        siblings.end());
                }
                nodes_[i].parentId = -1;  // 断开
            }
        }
    }
    
    /**
     * @brief 从目标节点提取路径
     */
    Path extractPath(int goalNodeId) const {
        Path path;
        
        std::vector<int> nodeIds;
        int currentId = goalNodeId;
        
        while (currentId >= 0) {
            nodeIds.push_back(currentId);
            currentId = nodes_[currentId].parentId;
        }
        
        // 反转得到从起点到终点的路径
        std::reverse(nodeIds.begin(), nodeIds.end());
        
        for (int id : nodeIds) {
            Waypoint wp(nodes_[id].config);
            path.waypoints.push_back(wp);
        }
        
        path.updatePathParameters();
        
        return path;
    }
    
    const RobotModel& robot_;
    CollisionChecker& checker_;
    PlannerConfig config_;
    
    std::vector<RRTNode> nodes_;
    std::mt19937 gen_;
};

} // namespace palletizing
