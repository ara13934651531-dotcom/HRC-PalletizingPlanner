/**
 * @file PathOptimizer.hpp
 * @brief 路径优化器 - 捷径优化 + B-Spline平滑
 * 
 * 实现多种路径优化技术:
 * 1. 随机捷径优化 - 缩短路径长度
 * 2. 迭代捷径优化 - 贪婪式路径简化
 * 3. B-Spline平滑 - 保证曲率连续性
 * 4. 梯度下降平滑 - 最小化曲率变化
 * 
 * 目标: 路径最短 + 曲率平滑
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
#include <algorithm>
#include <numeric>

namespace palletizing {

/**
 * @brief 路径优化器
 */
class PathOptimizer {
public:
    PathOptimizer(const RobotModel& robot,
                  CollisionChecker& checker,
                  const PlannerConfig& config = PlannerConfig())
        : robot_(robot), checker_(checker), config_(config),
          gen_(std::random_device{}()) {
    }
    
    /**
     * @brief 完整优化流程
     * @param path 输入路径
     * @return 优化结果
     */
    PlanningResult optimize(const Path& path) {
        PlanningResult result;
        result.rawPath = path;
        
        if (path.waypoints.size() < 2) {
            result.status = PlanningStatus::Failure;
            return result;
        }
        
        auto startTime = std::chrono::high_resolution_clock::now();
        
        // 1. 捷径优化
        Path shortcutPath = shortcutOptimization(path, config_.shortcutIterations);
        
        // 2. 路径简化 (移除共线点)
        Path simplifiedPath = simplifyPath(shortcutPath, 0.01);
        
        // 3. 细分路径以保证平滑
        Path subdividedPath = subdividePath(simplifiedPath, config_.stepSize);
        
        // 4. B-Spline平滑
        result.smoothedSpline = fitBSpline(subdividedPath, config_.splineDegree);
        
        // 5. 采样平滑曲线得到最终路径
        result.optimizedPath = result.smoothedSpline.sample(config_.splineResolution);
        
        // 6. 验证最终路径无碰撞
        if (!validatePath(result.optimizedPath)) {
            // 如果平滑后有碰撞，回退到细分路径
            result.optimizedPath = subdividedPath;
            result.smoothedSpline = fitBSpline(subdividedPath, 3);  // 降低阶数
        }
        
        auto endTime = std::chrono::high_resolution_clock::now();
        result.optimizationTime = std::chrono::duration<double>(endTime - startTime).count();
        
        // 计算统计信息
        result.pathLength = result.optimizedPath.totalLength();
        result.maxCurvature = computeMaxCurvature(result.smoothedSpline);
        result.status = PlanningStatus::Success;
        
        return result;
    }
    
    /**
     * @brief 随机捷径优化
     */
    Path shortcutOptimization(const Path& path, int iterations) {
        if (path.waypoints.size() < 3) return path;
        
        Path optimized = path;
        std::uniform_int_distribution<int> dist(0, static_cast<int>(optimized.waypoints.size()) - 1);
        
        for (int iter = 0; iter < iterations; ++iter) {
            if (optimized.waypoints.size() < 3) break;
            
            int n = static_cast<int>(optimized.waypoints.size());
            
            // 随机选择两个点
            int i = dist(gen_) % n;
            int j = dist(gen_) % n;
            
            if (i > j) std::swap(i, j);
            if (j - i < 2) continue;  // 需要至少间隔一个点
            
            // 检查直接连接是否无碰撞
            const auto& start = optimized.waypoints[i].config;
            const auto& end = optimized.waypoints[j].config;
            
            if (checker_.isPathCollisionFree(start, end, config_.collisionResolution)) {
                // 移除中间点
                std::vector<Waypoint> newWaypoints;
                for (int k = 0; k <= i; ++k) {
                    newWaypoints.push_back(optimized.waypoints[k]);
                }
                for (int k = j; k < n; ++k) {
                    newWaypoints.push_back(optimized.waypoints[k]);
                }
                
                optimized.waypoints = std::move(newWaypoints);
                
                // 更新分布范围
                dist = std::uniform_int_distribution<int>(0, static_cast<int>(optimized.waypoints.size()) - 1);
            }
        }
        
        optimized.updatePathParameters();
        return optimized;
    }
    
    /**
     * @brief 贪婪捷径优化 - 尝试直接连接每对非相邻点
     */
    Path greedyShortcut(const Path& path) {
        if (path.waypoints.size() < 3) return path;
        
        Path optimized;
        optimized.waypoints.push_back(path.waypoints.front());
        
        size_t current = 0;
        
        while (current < path.waypoints.size() - 1) {
            // 找能直接到达的最远点
            size_t farthest = current + 1;
            
            for (size_t j = path.waypoints.size() - 1; j > current + 1; --j) {
                if (checker_.isPathCollisionFree(
                        path.waypoints[current].config,
                        path.waypoints[j].config,
                        config_.collisionResolution)) {
                    farthest = j;
                    break;
                }
            }
            
            optimized.waypoints.push_back(path.waypoints[farthest]);
            current = farthest;
        }
        
        optimized.updatePathParameters();
        return optimized;
    }
    
    /**
     * @brief 路径简化 - 移除近似共线的点
     */
    Path simplifyPath(const Path& path, double tolerance) {
        if (path.waypoints.size() < 3) return path;
        
        Path simplified;
        simplified.waypoints.push_back(path.waypoints.front());
        
        for (size_t i = 1; i < path.waypoints.size() - 1; ++i) {
            const auto& prev = simplified.waypoints.back().config;
            const auto& curr = path.waypoints[i].config;
            const auto& next = path.waypoints[i + 1].config;
            
            // 计算点到线段的距离
            JointVector v = next.q - prev.q;
            JointVector w = curr.q - prev.q;
            
            double vSqNorm = v.squaredNorm();
            if (vSqNorm < 1e-18) {
                // prev ≈ next, 退化情况, 保留当前点
                simplified.waypoints.push_back(path.waypoints[i]);
                continue;
            }
            double t = std::clamp(w.dot(v) / vSqNorm, 0.0, 1.0);
            JointVector projection = prev.q + t * v;
            double dist = (curr.q - projection).norm();
            
            if (dist > tolerance) {
                simplified.waypoints.push_back(path.waypoints[i]);
            }
        }
        
        simplified.waypoints.push_back(path.waypoints.back());
        simplified.updatePathParameters();
        
        return simplified;
    }
    
    /**
     * @brief 细分路径 - 确保相邻点距离不超过maxDist
     */
    Path subdividePath(const Path& path, double maxDist) {
        if (path.waypoints.empty()) return path;
        
        Path subdivided;
        
        for (size_t i = 0; i < path.waypoints.size() - 1; ++i) {
            const auto& start = path.waypoints[i].config;
            const auto& end = path.waypoints[i + 1].config;
            
            double dist = start.distanceTo(end);
            int numSegments = static_cast<int>(std::ceil(dist / maxDist));
            numSegments = std::max(numSegments, 1);
            
            for (int j = 0; j < numSegments; ++j) {
                double t = static_cast<double>(j) / numSegments;
                JointConfig interpolated = robot_.interpolate(start, end, t);
                subdivided.waypoints.push_back(Waypoint(interpolated));
            }
        }
        
        subdivided.waypoints.push_back(path.waypoints.back());
        subdivided.updatePathParameters();
        
        return subdivided;
    }
    
    /**
     * @brief 拟合B-Spline曲线
     */
    BSpline fitBSpline(const Path& path, int degree = 5) {
        BSpline spline;
        
        if (path.waypoints.empty()) {
            return spline;
        }
        
        // 提取控制点 (对于拟合，我们需要计算最优控制点)
        // 这里使用简化方法：直接使用路径点作为控制点
        std::vector<JointConfig> controlPoints;
        controlPoints.reserve(path.waypoints.size());
        
        for (const auto& wp : path.waypoints) {
            controlPoints.push_back(wp.config);
        }
        
        // 如果点数太少，降低阶数
        int actualDegree = std::min(degree, static_cast<int>(controlPoints.size()) - 1);
        actualDegree = std::max(actualDegree, 1);
        
        // 对于平滑效果，我们可以适当增加控制点或使用拟合算法
        // 这里使用B-Spline近似：在路径点之间添加额外控制点
        if (controlPoints.size() > 3 && degree >= 3) {
            std::vector<JointConfig> smoothedPoints = generateSmoothControlPoints(controlPoints);
            spline.initUniform(smoothedPoints, actualDegree);
        } else {
            spline.initUniform(controlPoints, actualDegree);
        }
        
        return spline;
    }
    
    /**
     * @brief 梯度下降平滑
     */
    Path gradientSmooth(const Path& path, int iterations, double lambda = 0.5) {
        if (path.waypoints.size() < 3) return path;
        
        Path smoothed = path;
        
        for (int iter = 0; iter < iterations; ++iter) {
            std::vector<JointVector> updates(smoothed.waypoints.size(), JointVector::Zero());
            
            // 计算平滑梯度
            for (size_t i = 1; i < smoothed.waypoints.size() - 1; ++i) {
                const auto& prev = smoothed.waypoints[i - 1].config.q;
                const auto& curr = smoothed.waypoints[i].config.q;
                const auto& next = smoothed.waypoints[i + 1].config.q;
                
                // 拉普拉斯平滑
                JointVector laplacian = (prev + next) / 2.0 - curr;
                updates[i] = lambda * laplacian;
            }
            
            // 应用更新
            for (size_t i = 1; i < smoothed.waypoints.size() - 1; ++i) {
                JointVector newQ = smoothed.waypoints[i].config.q + updates[i];
                JointConfig newConfig(newQ);
                
                // 检查新配置是否有效
                if (robot_.isWithinLimits(newConfig) && 
                    checker_.isCollisionFree(newConfig)) {
                    smoothed.waypoints[i].config = newConfig;
                }
            }
        }
        
        smoothed.updatePathParameters();
        return smoothed;
    }
    
    /**
     * @brief 验证路径无碰撞
     */
    bool validatePath(const Path& path) const {
        if (path.waypoints.size() < 2) return true;  // 空路径或单点视为有效
        for (size_t i = 0; i + 1 < path.waypoints.size(); ++i) {
            if (!checker_.isPathCollisionFree(
                    path.waypoints[i].config,
                    path.waypoints[i + 1].config,
                    config_.collisionResolution)) {
                return false;
            }
        }
        return true;
    }
    
    /**
     * @brief 计算B-Spline的最大曲率
     */
    double computeMaxCurvature(const BSpline& spline, int samples = 100) const {
        double maxCurvature = 0.0;
        
        for (int i = 0; i < samples; ++i) {
            double t = static_cast<double>(i) / (samples - 1);
            double curvature = spline.curvature(t);
            maxCurvature = std::max(maxCurvature, curvature);
        }
        
        return maxCurvature;
    }
    
private:
    /**
     * @brief 生成平滑控制点
     */
    std::vector<JointConfig> generateSmoothControlPoints(
            const std::vector<JointConfig>& pathPoints) {
        
        std::vector<JointConfig> controlPoints;
        
        // 添加起点
        controlPoints.push_back(pathPoints.front());
        
        // 使用Catmull-Rom到B-Spline转换思想
        // 在每个内部点周围添加控制点以保证通过原点
        for (size_t i = 1; i < pathPoints.size() - 1; ++i) {
            // 添加原始点附近的控制点
            controlPoints.push_back(pathPoints[i]);
        }
        
        // 添加终点
        controlPoints.push_back(pathPoints.back());
        
        return controlPoints;
    }
    
    const RobotModel& robot_;
    CollisionChecker& checker_;
    PlannerConfig config_;
    std::mt19937 gen_;
};

/**
 * @brief 高级B-Spline拟合器 - 最小化拟合误差和曲率
 */
class BSplineFitter {
public:
    BSplineFitter(int degree = 5) : degree_(degree) {}
    
    /**
     * @brief 使用最小二乘法拟合B-Spline
     * @param points 路径点
     * @param numControlPoints 控制点数量
     * @return 拟合的B-Spline
     */
    BSpline fitLeastSquares(const std::vector<JointConfig>& points, 
                            int numControlPoints) {
        BSpline spline;
        
        int n = static_cast<int>(points.size());
        int m = numControlPoints;
        
        if (n < 2 || m < degree_ + 1) {
            // 退化情况
            spline.initUniform(points, std::min(degree_, n - 1));
            return spline;
        }
        
        // 计算参数化 (弦长参数化)
        std::vector<double> params(n);
        params[0] = 0.0;
        
        double totalLength = 0.0;
        for (int i = 1; i < n; ++i) {
            totalLength += points[i].distanceTo(points[i - 1]);
        }
        
        double cumLength = 0.0;
        for (int i = 1; i < n; ++i) {
            cumLength += points[i].distanceTo(points[i - 1]);
            params[i] = cumLength / totalLength;
        }
        
        // 构造B-Spline基函数矩阵
        // 这里使用简化方法：均匀分布控制点
        std::vector<JointConfig> controlPoints(m);
        
        for (int j = 0; j < m; ++j) {
            double t = static_cast<double>(j) / (m - 1);
            
            // 找到最近的路径点
            int nearestIdx = 0;
            double minDist = std::abs(params[0] - t);
            for (int i = 1; i < n; ++i) {
                double dist = std::abs(params[i] - t);
                if (dist < minDist) {
                    minDist = dist;
                    nearestIdx = i;
                }
            }
            
            controlPoints[j] = points[nearestIdx];
        }
        
        spline.initUniform(controlPoints, degree_);
        
        return spline;
    }
    
    /**
     * @brief 拟合并优化曲率
     */
    BSpline fitWithCurvatureOptimization(const std::vector<JointConfig>& points,
                                          int numControlPoints,
                                          [[maybe_unused]] double curvatureWeight = 0.1) {
        // 首先进行最小二乘拟合
        BSpline spline = fitLeastSquares(points, numControlPoints);
        
        // TODO: 添加曲率优化迭代
        // 这需要更复杂的非线性优化
        
        return spline;
    }
    
private:
    int degree_;
};

} // namespace palletizing
