/**
 * @file Types.hpp
 * @brief 核心数据类型定义 - 世界顶尖码垛运动规划系统
 * 
 * 定义了运动规划系统中使用的所有基础数据类型，包括：
 * - 关节空间和笛卡尔空间表示
 * - 路径和轨迹数据结构
 * - 规划器配置参数
 * 
 * @author GitHub Copilot
 * @version 1.0.0
 * @date 2026-01-29
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <array>
#include <memory>
#include <optional>
#include <functional>
#include <limits>
#include <cmath>

namespace palletizing {

// ============================================================================
// 基础类型定义
// ============================================================================

/// 6自由度关节向量
using JointVector = Eigen::Matrix<double, 6, 1>;

/// 笛卡尔位置 (x, y, z)
using Position3D = Eigen::Vector3d;

/// 笛卡尔姿态 (四元数表示，避免万向节锁)
using Orientation = Eigen::Quaterniond;

/// 6D位姿 (位置 + 四元数姿态)
struct Pose6D {
    Position3D position;
    Orientation orientation;
    
    Pose6D() : position(Position3D::Zero()), orientation(Orientation::Identity()) {}
    Pose6D(const Position3D& pos, const Orientation& ori) : position(pos), orientation(ori) {}
    
    /// 从欧拉角构造 (ZYX顺序, 单位: rad)
    static Pose6D fromEulerZYX(double x, double y, double z, double rx, double ry, double rz) {
        Pose6D pose;
        pose.position = Position3D(x, y, z);
        pose.orientation = Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ())
                         * Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY())
                         * Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX());
        return pose;
    }
    
    /// 转换为4x4齐次变换矩阵
    Eigen::Matrix4d toMatrix() const {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = orientation.toRotationMatrix();
        T.block<3, 1>(0, 3) = position;
        return T;
    }
    
    /// 从4x4齐次变换矩阵构造
    static Pose6D fromMatrix(const Eigen::Matrix4d& T) {
        Pose6D pose;
        pose.position = T.block<3, 1>(0, 3);
        pose.orientation = Orientation(T.block<3, 3>(0, 0));
        return pose;
    }
    
    /// 位姿插值 (SLERP for orientation)
    static Pose6D interpolate(const Pose6D& start, const Pose6D& end, double t) {
        Pose6D result;
        result.position = start.position + t * (end.position - start.position);
        result.orientation = start.orientation.slerp(t, end.orientation);
        return result;
    }
};

// ============================================================================
// 关节空间配置
// ============================================================================

/// 关节空间配置点
struct JointConfig {
    JointVector q;  // 关节角度 [rad]
    
    JointConfig() : q(JointVector::Zero()) {}
    explicit JointConfig(const JointVector& joints) : q(joints) {}
    JointConfig(std::initializer_list<double> list) {
        int i = 0;
        for (double val : list) {
            if (i < 6) q[i++] = val;
        }
    }
    
    /// 从度数转换
    static JointConfig fromDegrees(const JointVector& deg) {
        JointConfig config;
        config.q = deg * M_PI / 180.0;
        return config;
    }
    
    /// 从度数转换 (初始化列表版本)
    static JointConfig fromDegrees(std::initializer_list<double> list) {
        JointVector deg = JointVector::Zero();
        int i = 0;
        for (double val : list) {
            if (i < 6) deg[i++] = val;
        }
        return fromDegrees(deg);
    }
    
    /// 转换为度数
    JointVector toDegrees() const {
        return q * 180.0 / M_PI;
    }
    
    /// 计算配置空间距离
    double distanceTo(const JointConfig& other) const {
        return (q - other.q).norm();
    }
    
    /// 加权距离 (考虑不同关节的重要性)
    double weightedDistanceTo(const JointConfig& other, const JointVector& weights) const {
        return ((q - other.q).cwiseProduct(weights)).norm();
    }
    
    /// 线性插值
    JointConfig interpolate(const JointConfig& other, double t) const {
        return JointConfig(q + t * (other.q - q));
    }
};

// ============================================================================
// 路径和轨迹表示
// ============================================================================

/// 路径航点 (配置 + 可选的笛卡尔信息)
struct Waypoint {
    JointConfig config;
    std::optional<Pose6D> cartesian;  // 可选的笛卡尔位姿缓存
    double pathParam = 0.0;           // 路径参数 [0, 1]
    
    Waypoint() = default;
    explicit Waypoint(const JointConfig& cfg) : config(cfg) {}
    Waypoint(const JointConfig& cfg, const Pose6D& pose) : config(cfg), cartesian(pose) {}
};

/// 路径 (航点序列)
struct Path {
    std::vector<Waypoint> waypoints;
    
    bool empty() const { return waypoints.empty(); }
    size_t size() const { return waypoints.size(); }
    
    /// 计算路径总长度 (关节空间)
    double totalLength() const {
        double length = 0.0;
        for (size_t i = 1; i < waypoints.size(); ++i) {
            length += waypoints[i].config.distanceTo(waypoints[i-1].config);
        }
        return length;
    }
    
    /// 更新路径参数
    void updatePathParameters() {
        if (waypoints.empty()) return;
        
        double totalLen = totalLength();
        if (totalLen < 1e-9) {
            for (auto& wp : waypoints) wp.pathParam = 0.0;
            return;
        }
        
        double cumLen = 0.0;
        waypoints[0].pathParam = 0.0;
        for (size_t i = 1; i < waypoints.size(); ++i) {
            cumLen += waypoints[i].config.distanceTo(waypoints[i-1].config);
            waypoints[i].pathParam = cumLen / totalLen;
        }
    }
};

// ============================================================================
// B-Spline 曲线表示
// ============================================================================

/// B-Spline曲线 (用于平滑路径)
struct BSpline {
    int degree = 3;                           // 曲线阶数 (默认3次)
    std::vector<JointConfig> controlPoints;  // 控制点
    std::vector<double> knotVector;          // 节点向量
    
    /// 初始化均匀B-Spline
    void initUniform(const std::vector<JointConfig>& points, int deg = 3) {
        degree = deg;
        controlPoints = points;
        
        // 构造均匀夹紧节点向量
        int n = static_cast<int>(points.size()) - 1;
        int m = n + degree + 1;
        knotVector.resize(m + 1);
        
        for (int i = 0; i <= degree; ++i) {
            knotVector[i] = 0.0;
        }
        for (int i = degree + 1; i < m - degree; ++i) {
            knotVector[i] = static_cast<double>(i - degree) / (m - 2 * degree);
        }
        for (int i = m - degree; i <= m; ++i) {
            knotVector[i] = 1.0;
        }
    }
    
    /// 在参数t处求值 (De Boor算法)
    JointConfig evaluate(double t) const {
        if (controlPoints.empty()) return JointConfig();
        
        t = std::clamp(t, 0.0, 1.0);
        
        int n = static_cast<int>(controlPoints.size()) - 1;
        
        // 找到t所在的节点区间
        int k = degree;
        for (int i = degree; i < n + 1; ++i) {
            if (t >= knotVector[i] && t < knotVector[i + 1]) {
                k = i;
                break;
            }
        }
        if (t >= 1.0 - 1e-9) k = n;
        
        // De Boor算法
        std::vector<JointVector> d(degree + 1);
        for (int j = 0; j <= degree; ++j) {
            d[j] = controlPoints[k - degree + j].q;
        }
        
        for (int r = 1; r <= degree; ++r) {
            for (int j = degree; j >= r; --j) {
                double alpha = (t - knotVector[k - degree + j]) / 
                              (knotVector[k + 1 + j - r] - knotVector[k - degree + j]);
                d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j];
            }
        }
        
        return JointConfig(d[degree]);
    }
    
    /// 计算曲线导数
    JointVector derivative(double t, int order = 1) const {
        if (order == 0) return evaluate(t).q;
        
        double h = 1e-6;
        if (order == 1) {
            auto p1 = evaluate(std::min(t + h, 1.0));
            auto p0 = evaluate(std::max(t - h, 0.0));
            return (p1.q - p0.q) / (2 * h);
        }
        
        // 高阶导数递归
        auto d1 = derivative(std::min(t + h, 1.0), order - 1);
        auto d0 = derivative(std::max(t - h, 0.0), order - 1);
        return (d1 - d0) / (2 * h);
    }
    
    /// 计算曲率 (用于评估平滑度)
    double curvature(double t) const {
        JointVector d1 = derivative(t, 1);
        JointVector d2 = derivative(t, 2);
        
        double d1_norm = d1.norm();
        if (d1_norm < 1e-9) return 0.0;
        
        // 高维空间曲率: k = |d2 - (d2·d1)d1/|d1|^2| / |d1|^2
        // 使用弗莱纳公式的推广
        double d1d2_dot = d1.dot(d2);
        JointVector d2_perp = d2 - (d1d2_dot / (d1_norm * d1_norm)) * d1;
        return d2_perp.norm() / (d1_norm * d1_norm);
    }
    
    /// 采样曲线
    Path sample(int numPoints) const {
        Path path;
        path.waypoints.reserve(numPoints);
        
        for (int i = 0; i < numPoints; ++i) {
            double t = static_cast<double>(i) / (numPoints - 1);
            Waypoint wp(evaluate(t));
            wp.pathParam = t;
            path.waypoints.push_back(wp);
        }
        
        return path;
    }
};

// ============================================================================
// 规划器配置
// ============================================================================

/// 规划器类型枚举
enum class PlannerType {
    RRT,           // 基础RRT
    RRTStar,       // RRT* (渐进最优)
    InformedRRTStar,  // Informed RRT* (椭球采样)
    BITStar,       // Batch Informed Trees
    LazyPRM,       // 延迟概率路线图
    ABITStar       // Advanced BIT*
};

/// 路径优化器类型
enum class OptimizerType {
    Shortcut,        // 捷径优化
    BSplineSmooth,   // B-Spline平滑
    STOMP,           // 随机轨迹优化
    CHOMP,           // 协变哈密顿优化
    Hybrid           // 混合优化
};

/// 规划器配置参数
struct PlannerConfig {
    // 算法选择
    PlannerType plannerType = PlannerType::InformedRRTStar;
    OptimizerType optimizerType = OptimizerType::Hybrid;
    
    // RRT/RRT*参数
    double stepSize = 0.1;           // 步长 [rad]
    double goalBias = 0.15;          // 目标偏向概率
    double rewireRadius = 0.5;       // 重连半径
    int maxIterations = 50000;       // 最大迭代次数
    double maxPlanningTime = 10.0;   // 最大规划时间 [s]
    
    // Informed RRT*参数
    bool useInformedSampling = true;
    double informedThreshold = 1.5;  // 开始椭球采样的路径长度倍率
    
    // BIT*参数
    int batchSize = 100;             // 批次大小
    double pruneThreshold = 1.05;    // 剪枝阈值
    
    // 碰撞检测参数
    double collisionResolution = 0.02;  // 碰撞检测分辨率 [rad]
    double safetyMargin = 0.01;         // 安全余量 [m]
    
    // 路径优化参数
    int shortcutIterations = 200;       // 捷径优化迭代次数
    int smoothIterations = 100;         // 平滑迭代次数
    double smoothWeight = 0.5;          // 平滑权重 [0, 1]
    
    // B-Spline参数
    int splineDegree = 5;               // 样条阶数 (5次保证加速度连续)
    int splineResolution = 100;         // 采样点数
    
    // 性能调优
    bool enableParallel = true;         // 启用并行计算
    int numThreads = 4;                 // 线程数
    bool enableCaching = true;          // 启用碰撞检测缓存
};

// ============================================================================
// 规划结果
// ============================================================================

/// 规划状态
enum class PlanningStatus {
    Success,           // 成功
    Failure,           // 失败
    Timeout,           // 超时
    InvalidStart,      // 无效起点
    InvalidGoal,       // 无效终点
    CollisionAtStart,  // 起点碰撞
    CollisionAtGoal,   // 终点碰撞
    NoPath             // 无可行路径
};

/// 规划结果
struct PlanningResult {
    PlanningStatus status = PlanningStatus::Failure;
    Path rawPath;              // 原始路径
    Path optimizedPath;        // 优化后路径
    BSpline smoothedSpline;    // 平滑样条
    
    // 统计信息
    double planningTime = 0.0;      // 规划耗时 [s]
    double optimizationTime = 0.0;  // 优化耗时 [s]
    int iterations = 0;             // 迭代次数
    int nodesExplored = 0;          // 探索节点数
    double pathLength = 0.0;        // 最终路径长度
    double maxCurvature = 0.0;      // 最大曲率
    
    bool isSuccess() const { return status == PlanningStatus::Success; }
    
    std::string statusString() const {
        switch (status) {
            case PlanningStatus::Success: return "Success";
            case PlanningStatus::Failure: return "Failure";
            case PlanningStatus::Timeout: return "Timeout";
            case PlanningStatus::InvalidStart: return "Invalid Start";
            case PlanningStatus::InvalidGoal: return "Invalid Goal";
            case PlanningStatus::CollisionAtStart: return "Collision at Start";
            case PlanningStatus::CollisionAtGoal: return "Collision at Goal";
            case PlanningStatus::NoPath: return "No Path Found";
            default: return "Unknown";
        }
    }
};

// ============================================================================
// 工具函数
// ============================================================================

namespace utils {

/// 角度归一化到 [-π, π]
inline double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

/// 关节角归一化
inline JointVector normalizeJoints(const JointVector& q) {
    JointVector result;
    for (int i = 0; i < 6; ++i) {
        result[i] = normalizeAngle(q[i]);
    }
    return result;
}

/// 线性插值
template<typename T>
inline T lerp(const T& a, const T& b, double t) {
    return a + t * (b - a);
}

/// 平滑阶跃函数 (S曲线)
inline double smoothStep(double t) {
    t = std::clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

/// 更平滑的阶跃函数 (quintic)
inline double smootherStep(double t) {
    t = std::clamp(t, 0.0, 1.0);
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}

} // namespace utils

} // namespace palletizing
