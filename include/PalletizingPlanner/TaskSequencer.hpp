/**
 * @file TaskSequencer.hpp
 * @brief 码垛任务序列规划器
 * 
 * 实现智能的多任务序列优化：
 * 1. 最短行程TSP优化
 * 2. 多层码垛模式生成
 * 3. 任务执行状态管理
 * 
 * @author GitHub Copilot
 * @version 1.0.0
 * @date 2026-01-29
 */

#pragma once

#include "Types.hpp"
#include "RobotModel.hpp"
#include "CollisionChecker.hpp"
#include "PalletizingPlanner.hpp"

#include <algorithm>
#include <numeric>
#include <random>

namespace palletizing {

/**
 * @brief 码垛位置描述
 */
struct PalletPosition {
    int layer;          // 层号 (从0开始)
    int row;            // 行号
    int col;            // 列号
    Pose6D pose;        // 放置位姿
    bool occupied = false;  // 是否已放置
    
    // 标识
    std::string getId() const {
        return "L" + std::to_string(layer) + "_R" + std::to_string(row) + "_C" + std::to_string(col);
    }
};

/**
 * @brief 码垛模式定义
 */
struct PalletPattern {
    int numLayers;      // 总层数
    int rowsPerLayer;   // 每层行数
    int colsPerRow;     // 每行列数
    
    // 物品尺寸
    double itemLength;  // 长度 [m]
    double itemWidth;   // 宽度 [m]
    double itemHeight;  // 高度 [m]
    
    // 码垛区域原点
    Pose6D palletOrigin;
    
    // 物品间隙
    double gapX = 0.01;  // X方向间隙 [m]
    double gapY = 0.01;  // Y方向间隙 [m]
    
    // 层间旋转 (用于交错码垛)
    bool alternateOrientation = false;
    
    /**
     * @brief 生成所有码垛位置
     */
    std::vector<PalletPosition> generatePositions() const {
        std::vector<PalletPosition> positions;
        
        for (int layer = 0; layer < numLayers; ++layer) {
            for (int row = 0; row < rowsPerLayer; ++row) {
                for (int col = 0; col < colsPerRow; ++col) {
                    PalletPosition pos;
                    pos.layer = layer;
                    pos.row = row;
                    pos.col = col;
                    
                    // 计算位置
                    double x = palletOrigin.position.x() + col * (itemLength + gapX);
                    double y = palletOrigin.position.y() + row * (itemWidth + gapY);
                    double z = palletOrigin.position.z() + layer * itemHeight;
                    
                    pos.pose.position = Eigen::Vector3d(x, y, z);
                    
                    // 交错层旋转90度
                    if (alternateOrientation && layer % 2 == 1) {
                        pos.pose.orientation = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()) 
                                             * palletOrigin.orientation;
                    } else {
                        pos.pose.orientation = palletOrigin.orientation;
                    }
                    
                    positions.push_back(pos);
                }
            }
        }
        
        return positions;
    }
    
    /**
     * @brief 创建标准纸箱码垛模式
     */
    static PalletPattern standardCartonPattern() {
        PalletPattern pattern;
        pattern.numLayers = 5;
        pattern.rowsPerLayer = 4;
        pattern.colsPerRow = 3;
        pattern.itemLength = 0.4;
        pattern.itemWidth = 0.3;
        pattern.itemHeight = 0.25;
        pattern.palletOrigin = Pose6D::fromEulerZYX(-1.0, 0.5, 0.1, 0, 0, 0);
        pattern.alternateOrientation = true;
        return pattern;
    }
};

/**
 * @brief 任务序列优化器 (TSP近似解)
 */
class TaskSequenceOptimizer {
public:
    /**
     * @brief 使用贪心+2-opt优化任务顺序
     * @param tasks 任务列表
     * @param startConfig 起始配置
     * @return 优化后的任务索引顺序
     */
    static std::vector<int> optimize(const std::vector<PalletizingTask>& tasks,
                                      const JointConfig& startConfig) {
        int n = static_cast<int>(tasks.size());
        if (n <= 2) {
            std::vector<int> order(n);
            std::iota(order.begin(), order.end(), 0);
            return order;
        }
        
        // 计算距离矩阵
        std::vector<std::vector<double>> distances(n + 1, std::vector<double>(n + 1, 0.0));
        
        // 从起点到各任务的距离
        for (int i = 0; i < n; ++i) {
            distances[0][i + 1] = startConfig.distanceTo(tasks[i].pickConfig);
        }
        
        // 任务间距离 (上一个任务的放置位置到下一个任务的抓取位置)
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                if (i != j) {
                    distances[i + 1][j + 1] = tasks[i].placeConfig.distanceTo(tasks[j].pickConfig);
                }
            }
        }
        
        // 贪心构造初始解
        std::vector<int> order = greedyTSP(distances, n);
        
        // 2-opt局部优化
        order = twoOpt(order, distances);
        
        return order;
    }
    
private:
    /**
     * @brief 贪心TSP
     */
    static std::vector<int> greedyTSP(const std::vector<std::vector<double>>& dist, int n) {
        std::vector<int> order;
        std::vector<bool> visited(n, false);
        
        int current = 0;  // 从起点开始
        
        for (int i = 0; i < n; ++i) {
            double minDist = std::numeric_limits<double>::infinity();
            int next = -1;
            
            for (int j = 0; j < n; ++j) {
                if (!visited[j] && dist[current][j + 1] < minDist) {
                    minDist = dist[current][j + 1];
                    next = j;
                }
            }
            
            if (next >= 0) {
                order.push_back(next);
                visited[next] = true;
                current = next + 1;
            }
        }
        
        return order;
    }
    
    /**
     * @brief 2-opt优化
     */
    static std::vector<int> twoOpt(std::vector<int> order, 
                                    const std::vector<std::vector<double>>& dist) {
        int n = static_cast<int>(order.size());
        bool improved = true;
        
        while (improved) {
            improved = false;
            
            for (int i = 0; i < n - 1; ++i) {
                for (int j = i + 2; j < n; ++j) {
                    double delta = calculate2OptDelta(order, dist, i, j);
                    
                    if (delta < -1e-9) {
                        // 执行2-opt交换
                        std::reverse(order.begin() + i + 1, order.begin() + j + 1);
                        improved = true;
                    }
                }
            }
        }
        
        return order;
    }
    
    /**
     * @brief 计算2-opt交换的代价变化
     */
    static double calculate2OptDelta(const std::vector<int>& order,
                                     const std::vector<std::vector<double>>& dist,
                                     int i, int j) {
        int n = static_cast<int>(order.size());
        
        int a = (i == 0) ? 0 : order[i - 1] + 1;
        int b = order[i] + 1;
        int c = order[j] + 1;
        int d = (j == n - 1) ? 0 : order[j + 1] + 1;  // 回到起点或下一个
        
        double before = dist[a][b] + dist[c][d];
        double after = dist[a][c] + dist[b][d];
        
        return after - before;
    }
};

/**
 * @brief 码垛任务序列规划器
 */
class TaskSequencer {
public:
    using ProgressCallback = std::function<void(int current, int total, const std::string& status)>;
    
    TaskSequencer(PalletizingPlanner& planner)
        : planner_(planner) {}
    
    /**
     * @brief 设置码垛模式
     */
    void setPattern(const PalletPattern& pattern) {
        pattern_ = pattern;
        positions_ = pattern.generatePositions();
    }
    
    /**
     * @brief 生成完整的码垛任务序列
     * @param pickPose 抓取位置 (固定，如流水线位置)
     * @param robotModel 机器人模型用于IK
     * @return 任务列表
     */
    std::vector<PalletizingTask> generateTasks(const Pose6D& pickPose,
                                                const RobotModel& robotModel) {
        std::vector<PalletizingTask> tasks;
        
        // 为每个码垛位置创建任务
        for (size_t i = 0; i < positions_.size(); ++i) {
            if (positions_[i].occupied) continue;
            
            PalletizingTask task;
            task.taskId = static_cast<int>(i);
            task.pickPose = pickPose;
            task.placePose = positions_[i].pose;
            task.description = "Place at " + positions_[i].getId();
            
            // 简化版：使用预设的关节配置
            // 完整版应使用IK求解
            task.pickConfig = JointConfig::fromDegrees({0, -60, 30, 0, -60, 0});
            
            // 基于放置位置调整配置 (简化)
            double theta = std::atan2(positions_[i].pose.position.y(),
                                      positions_[i].pose.position.x());
            task.placeConfig = JointConfig::fromDegrees({
                theta * 180 / M_PI,
                -45 - positions_[i].layer * 5,
                30,
                0,
                -75 + positions_[i].layer * 5,
                theta * 180 / M_PI
            });
            
            tasks.push_back(task);
        }
        
        return tasks;
    }
    
    /**
     * @brief 优化任务执行顺序
     */
    std::vector<PalletizingTask> optimizeSequence(
            const std::vector<PalletizingTask>& tasks,
            const JointConfig& startConfig) {
        
        if (tasks.size() <= 1) return tasks;
        
        // 获取优化顺序
        std::vector<int> optimizedOrder = TaskSequenceOptimizer::optimize(tasks, startConfig);
        
        // 重排任务
        std::vector<PalletizingTask> optimizedTasks;
        optimizedTasks.reserve(tasks.size());
        
        for (int idx : optimizedOrder) {
            optimizedTasks.push_back(tasks[idx]);
        }
        
        return optimizedTasks;
    }
    
    /**
     * @brief 执行完整码垛序列规划
     */
    std::vector<PalletizingResult> planSequence(
            const std::vector<PalletizingTask>& tasks,
            const JointConfig& startConfig,
            ProgressCallback callback = nullptr) {
        
        return planner_.planTaskSequence(tasks, startConfig, callback);
    }
    
    /**
     * @brief 计算总行程估计
     */
    double estimateTotalDistance(const std::vector<PalletizingTask>& tasks,
                                  const JointConfig& startConfig) const {
        double total = 0.0;
        JointConfig current = startConfig;
        
        for (const auto& task : tasks) {
            total += current.distanceTo(task.pickConfig);
            total += task.pickConfig.distanceTo(task.placeConfig);
            current = task.placeConfig;
        }
        
        return total;
    }
    
    // 访问器
    const std::vector<PalletPosition>& getPositions() const { return positions_; }
    
    void markPositionOccupied(int index, bool occupied = true) {
        if (index >= 0 && index < static_cast<int>(positions_.size())) {
            positions_[index].occupied = occupied;
        }
    }
    
private:
    PalletizingPlanner& planner_;
    PalletPattern pattern_;
    std::vector<PalletPosition> positions_;
};

/**
 * @brief 码垛层规划策略
 */
enum class LayerStrategy {
    BottomUp,       // 从下往上
    NearestFirst,   // 最近优先
    SnakePattern,   // 蛇形路径
    SpiralInward    // 螺旋向内
};

/**
 * @brief 生成特定模式的层内顺序
 */
class LayerOrderGenerator {
public:
    /**
     * @brief 生成蛇形顺序
     */
    static std::vector<std::pair<int, int>> snakeOrder(int rows, int cols) {
        std::vector<std::pair<int, int>> order;
        
        for (int r = 0; r < rows; ++r) {
            if (r % 2 == 0) {
                for (int c = 0; c < cols; ++c) {
                    order.emplace_back(r, c);
                }
            } else {
                for (int c = cols - 1; c >= 0; --c) {
                    order.emplace_back(r, c);
                }
            }
        }
        
        return order;
    }
    
    /**
     * @brief 生成螺旋顺序
     */
    static std::vector<std::pair<int, int>> spiralOrder(int rows, int cols) {
        std::vector<std::pair<int, int>> order;
        
        int top = 0, bottom = rows - 1;
        int left = 0, right = cols - 1;
        
        while (top <= bottom && left <= right) {
            // 向右
            for (int c = left; c <= right; ++c) {
                order.emplace_back(top, c);
            }
            top++;
            
            // 向下
            for (int r = top; r <= bottom; ++r) {
                order.emplace_back(r, right);
            }
            right--;
            
            // 向左
            if (top <= bottom) {
                for (int c = right; c >= left; --c) {
                    order.emplace_back(bottom, c);
                }
                bottom--;
            }
            
            // 向上
            if (left <= right) {
                for (int r = bottom; r >= top; --r) {
                    order.emplace_back(r, left);
                }
                left++;
            }
        }
        
        return order;
    }
};

} // namespace palletizing
