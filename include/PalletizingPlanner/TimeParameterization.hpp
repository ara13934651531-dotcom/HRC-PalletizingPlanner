/**
 * @file TimeParameterization.hpp
 * @brief 时间参数化模块 - 预留TOPP接口
 * 
 * 提供路径到轨迹的时间参数化功能：
 * 1. 梯形速度规划
 * 2. S曲线速度规划  
 * 3. 预留TOPP-RA时间最优接口
 * 
 * 设计目标：为后续时间最优轨迹规划预留接口
 * 
 * @author GitHub Copilot
 * @version 1.0.0
 * @date 2026-01-29
 */

#pragma once

#include "Types.hpp"
#include "RobotModel.hpp"

#include <algorithm>
#include <cmath>

namespace palletizing {

/**
 * @brief 轨迹点 (包含时间信息)
 */
struct TrajectoryPoint {
    JointConfig config;
    JointVector velocity;
    JointVector acceleration;
    double time = 0.0;
    double pathParam = 0.0;  // s ∈ [0, 1]
    
    TrajectoryPoint() : velocity(JointVector::Zero()), acceleration(JointVector::Zero()) {}
};

/**
 * @brief 完整轨迹
 */
struct Trajectory {
    std::vector<TrajectoryPoint> points;
    double totalTime = 0.0;
    
    bool empty() const { return points.empty(); }
    size_t size() const { return points.size(); }
    
    /// 在时间t处插值
    TrajectoryPoint interpolate(double t) const {
        if (points.empty()) return TrajectoryPoint();
        if (t <= 0) return points.front();
        if (t >= totalTime) return points.back();
        
        // 二分查找
        size_t low = 0, high = points.size() - 1;
        while (low < high - 1) {
            size_t mid = (low + high) / 2;
            if (points[mid].time <= t) {
                low = mid;
            } else {
                high = mid;
            }
        }
        
        // 线性插值
        double dt = points[high].time - points[low].time;
        double alpha = (dt > 1e-9) ? (t - points[low].time) / dt : 0.0;
        
        TrajectoryPoint result;
        result.time = t;
        result.config.q = (1 - alpha) * points[low].config.q + alpha * points[high].config.q;
        result.velocity = (1 - alpha) * points[low].velocity + alpha * points[high].velocity;
        result.acceleration = (1 - alpha) * points[low].acceleration + alpha * points[high].acceleration;
        result.pathParam = (1 - alpha) * points[low].pathParam + alpha * points[high].pathParam;
        
        return result;
    }
};

/**
 * @brief 速度规划类型
 */
enum class VelocityProfileType {
    Trapezoidal,    // 梯形速度曲线
    SCurve,         // S曲线 (七段式)
    TOPP            // 时间最优 (预留)
};

/**
 * @brief 时间参数化配置
 */
struct TimeParameterizationConfig {
    VelocityProfileType profileType = VelocityProfileType::SCurve;
    
    // 速度限制 (rad/s)
    JointVector maxVelocity;
    
    // 加速度限制 (rad/s^2)
    JointVector maxAcceleration;
    
    // Jerk限制 (rad/s^3) - 用于S曲线
    JointVector maxJerk;
    
    // 速度缩放因子 [0, 1]
    double velocityScaling = 0.8;
    
    // 采样周期 (s)
    double samplePeriod = 0.001;  // 1ms
    
    /// 从机器人参数构造
    static TimeParameterizationConfig fromRobotParams(const RobotDHParams& params) {
        TimeParameterizationConfig config;
        
        for (int i = 0; i < 6; ++i) {
            config.maxVelocity[i] = params.maxVelocity[i] * M_PI / 180.0;
            config.maxAcceleration[i] = params.maxAcceleration[i] * M_PI / 180.0;
            config.maxJerk[i] = config.maxAcceleration[i] * 10.0;  // 默认jerk
        }
        
        return config;
    }
};

/**
 * @brief 时间参数化器
 * 
 * 将几何路径转换为带时间信息的轨迹
 */
class TimeParameterizer {
public:
    TimeParameterizer(const TimeParameterizationConfig& config = TimeParameterizationConfig())
        : config_(config) {}
    
    /**
     * @brief 对路径进行时间参数化
     * @param path 输入路径
     * @return 带时间的轨迹
     */
    Trajectory parameterize(const Path& path) {
        if (path.empty()) return Trajectory();
        
        switch (config_.profileType) {
            case VelocityProfileType::Trapezoidal:
                return trapezoidalParameterization(path);
            case VelocityProfileType::SCurve:
                return sCurveParameterization(path);
            case VelocityProfileType::TOPP:
                // 预留TOPP接口
                return toppParameterization(path);
            default:
                return trapezoidalParameterization(path);
        }
    }
    
    /**
     * @brief 对B-Spline进行时间参数化
     * @param spline 输入样条
     * @param numSamples 采样点数
     * @return 带时间的轨迹
     */
    Trajectory parameterize(const BSpline& spline, int numSamples = 500) {
        Path path = spline.sample(numSamples);
        return parameterize(path);
    }
    
    /**
     * @brief 设置配置
     */
    void setConfig(const TimeParameterizationConfig& config) {
        config_ = config;
    }
    
private:
    /**
     * @brief 梯形速度曲线参数化
     */
    Trajectory trapezoidalParameterization(const Path& path) {
        Trajectory traj;
        
        if (path.waypoints.size() < 2) {
            if (!path.empty()) {
                TrajectoryPoint pt;
                pt.config = path.waypoints[0].config;
                pt.time = 0.0;
                traj.points.push_back(pt);
            }
            return traj;
        }
        
        double currentTime = 0.0;
        
        for (size_t i = 0; i < path.waypoints.size() - 1; ++i) {
            const auto& q0 = path.waypoints[i].config.q;
            const auto& q1 = path.waypoints[i + 1].config.q;
            JointVector dq = q1 - q0;
            
            // 计算每个关节所需的时间
            double segmentTime = 0.0;
            for (int j = 0; j < 6; ++j) {
                double v_max = config_.maxVelocity[j] * config_.velocityScaling;
                double a_max = config_.maxAcceleration[j] * config_.velocityScaling;
                
                double dist = std::abs(dq[j]);
                
                // 梯形速度曲线时间计算
                double t_accel = v_max / a_max;
                double d_accel = 0.5 * a_max * t_accel * t_accel;
                
                double t_joint;
                if (dist <= 2 * d_accel) {
                    // 三角形曲线 (达不到最大速度)
                    t_joint = 2.0 * std::sqrt(dist / a_max);
                } else {
                    // 梯形曲线
                    t_joint = 2 * t_accel + (dist - 2 * d_accel) / v_max;
                }
                
                segmentTime = std::max(segmentTime, t_joint);
            }
            
            segmentTime = std::max(segmentTime, 0.001);  // 最小时间
            
            // 生成该段的轨迹点
            int numPoints = static_cast<int>(std::ceil(segmentTime / config_.samplePeriod));
            numPoints = std::max(numPoints, 2);
            
            for (int k = 0; k < numPoints; ++k) {
                double t_local = static_cast<double>(k) / (numPoints - 1) * segmentTime;
                double alpha = static_cast<double>(k) / (numPoints - 1);
                
                TrajectoryPoint pt;
                pt.config.q = q0 + alpha * dq;
                pt.time = currentTime + t_local;
                pt.pathParam = path.waypoints[i].pathParam + 
                              alpha * (path.waypoints[i+1].pathParam - path.waypoints[i].pathParam);
                
                // 计算速度 (有限差分)
                if (k > 0 && k < numPoints - 1) {
                    pt.velocity = dq / segmentTime;
                }
                
                // 避免重复点
                if (k < numPoints - 1 || i == path.waypoints.size() - 2) {
                    traj.points.push_back(pt);
                }
            }
            
            currentTime += segmentTime;
        }
        
        traj.totalTime = currentTime;
        
        return traj;
    }
    
    /**
     * @brief S曲线 (七段式) 速度参数化
     */
    Trajectory sCurveParameterization(const Path& path) {
        Trajectory traj;
        
        if (path.waypoints.size() < 2) {
            if (!path.empty()) {
                TrajectoryPoint pt;
                pt.config = path.waypoints[0].config;
                pt.time = 0.0;
                traj.points.push_back(pt);
            }
            return traj;
        }
        
        double currentTime = 0.0;
        
        for (size_t i = 0; i < path.waypoints.size() - 1; ++i) {
            const auto& q0 = path.waypoints[i].config.q;
            const auto& q1 = path.waypoints[i + 1].config.q;
            JointVector dq = q1 - q0;
            
            // 计算每个关节的S曲线参数
            double segmentTime = 0.0;
            
            for (int j = 0; j < 6; ++j) {
                double v_max = config_.maxVelocity[j] * config_.velocityScaling;
                double a_max = config_.maxAcceleration[j] * config_.velocityScaling;
                double j_max = config_.maxJerk[j] * config_.velocityScaling;
                
                double dist = std::abs(dq[j]);
                if (dist < 1e-9) continue;
                
                // 七段式S曲线时间计算 (简化版)
                double t_jerk = a_max / j_max;
                double t_accel = v_max / a_max;
                
                double d_jerk = j_max * t_jerk * t_jerk * t_jerk / 6.0;
                double d_accel = v_max * t_accel - j_max * t_jerk * t_jerk * t_jerk / 3.0;
                
                double t_joint;
                if (dist <= 2 * d_jerk) {
                    // 极短距离
                    t_joint = std::pow(6 * dist / j_max, 1.0/3.0) * 2;
                } else if (dist <= 2 * d_accel) {
                    // 三角形加速度曲线
                    t_joint = 2 * t_jerk + 2 * std::sqrt((dist - 2*d_jerk) / a_max);
                } else {
                    // 完整七段曲线
                    t_joint = 2 * t_jerk + 2 * (t_accel - t_jerk) + (dist - 2*d_accel) / v_max;
                }
                
                segmentTime = std::max(segmentTime, t_joint);
            }
            
            segmentTime = std::max(segmentTime, 0.001);
            
            // 生成轨迹点 (使用平滑的五次多项式插值)
            int numPoints = static_cast<int>(std::ceil(segmentTime / config_.samplePeriod));
            numPoints = std::max(numPoints, 2);
            
            for (int k = 0; k < numPoints; ++k) {
                double t_local = static_cast<double>(k) / (numPoints - 1) * segmentTime;
                double tau = static_cast<double>(k) / (numPoints - 1);
                
                // 五次多项式平滑 (满足速度和加速度边界条件)
                double s = utils::smootherStep(tau);
                double ds = 30 * tau * tau * (tau - 1) * (tau - 1);  // 一阶导数
                double dds = 60 * tau * (tau - 1) * (2 * tau - 1);   // 二阶导数
                
                TrajectoryPoint pt;
                pt.config.q = q0 + s * dq;
                pt.time = currentTime + t_local;
                pt.velocity = (ds / segmentTime) * dq;
                pt.acceleration = (dds / (segmentTime * segmentTime)) * dq;
                pt.pathParam = path.waypoints[i].pathParam + 
                              s * (path.waypoints[i+1].pathParam - path.waypoints[i].pathParam);
                
                if (k < numPoints - 1 || i == path.waypoints.size() - 2) {
                    traj.points.push_back(pt);
                }
            }
            
            currentTime += segmentTime;
        }
        
        traj.totalTime = currentTime;
        
        return traj;
    }
    
    /**
     * @brief TOPP时间最优参数化 (预留接口)
     * 
     * 后续可集成TOPP-RA或TOPPRA库
     * 参考: https://github.com/hungpham2511/toppra
     */
    Trajectory toppParameterization(const Path& path) {
        // TODO: 集成TOPP-RA算法
        // 目前回退到S曲线
        
        // TOPP算法步骤 (预留):
        // 1. 计算路径的弧长参数化
        // 2. 计算可行速度集合 (考虑关节速度、加速度、力矩约束)
        // 3. 通过动态规划或凸优化求解时间最优速度曲线
        // 4. 积分得到时间参数化
        
        return sCurveParameterization(path);
    }
    
    TimeParameterizationConfig config_;
};

/**
 * @brief 轨迹验证器
 */
class TrajectoryValidator {
public:
    TrajectoryValidator(const RobotDHParams& params = RobotDHParams::fromHRConfig())
        : params_(params) {}
    
    /**
     * @brief 验证轨迹是否满足动力学约束
     */
    bool validate(const Trajectory& traj) const {
        for (const auto& pt : traj.points) {
            // 检查速度限制
            for (int i = 0; i < 6; ++i) {
                double maxVel = params_.maxVelocity[i] * M_PI / 180.0;
                if (std::abs(pt.velocity[i]) > maxVel * 1.05) {  // 5%容差
                    return false;
                }
            }
            
            // 检查加速度限制
            for (int i = 0; i < 6; ++i) {
                double maxAcc = params_.maxAcceleration[i] * M_PI / 180.0;
                if (std::abs(pt.acceleration[i]) > maxAcc * 1.05) {
                    return false;
                }
            }
        }
        return true;
    }
    
    /**
     * @brief 获取轨迹的最大速度和加速度
     */
    std::pair<JointVector, JointVector> getMaxValues(const Trajectory& traj) const {
        JointVector maxVel = JointVector::Zero();
        JointVector maxAcc = JointVector::Zero();
        
        for (const auto& pt : traj.points) {
            for (int i = 0; i < 6; ++i) {
                maxVel[i] = std::max(maxVel[i], std::abs(pt.velocity[i]));
                maxAcc[i] = std::max(maxAcc[i], std::abs(pt.acceleration[i]));
            }
        }
        
        return {maxVel, maxAcc};
    }
    
private:
    RobotDHParams params_;
};

} // namespace palletizing
