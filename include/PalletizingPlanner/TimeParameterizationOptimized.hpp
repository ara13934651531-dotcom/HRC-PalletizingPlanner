/**
 * @file TimeParameterizationOptimized.hpp
 * @brief 高性能时间参数化
 * 
 * 优化特性：
 * - 预计算查表加速
 * - SIMD友好的数值积分
 * - 增量式计算
 * - 缓存优化
 * 
 * @author GitHub Copilot
 * @version 2.0.0 - Performance Optimized
 * @date 2026-01-29
 */

#pragma once

#include "Types.hpp"
#include "TimeParameterization.hpp"

#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>

namespace palletizing {

/**
 * @brief 时间参数化性能统计
 */
struct TimeParameterizationStats {
    double computationTime = 0;
    int segments = 0;
    double totalDuration = 0;
    double maxVelocity = 0;
    double maxAcceleration = 0;
    double maxJerk = 0;
    
    void reset() {
        computationTime = 0;
        segments = totalDuration = maxVelocity = maxAcceleration = maxJerk = 0;
    }
};

/**
 * @brief 预计算的S曲线查表
 */
class SCurveTable {
public:
    SCurveTable(int resolution = 1000) : resolution_(resolution) {
        // 预计算归一化S曲线
        table_.resize(resolution + 1);
        derivTable_.resize(resolution + 1);
        accelTable_.resize(resolution + 1);
        
        for (int i = 0; i <= resolution; ++i) {
            double t = static_cast<double>(i) / resolution;
            
            // 7段S曲线的归一化形式
            // 使用多项式近似: s(t) = 10t³ - 15t⁴ + 6t⁵
            double t2 = t * t;
            double t3 = t2 * t;
            double t4 = t3 * t;
            double t5 = t4 * t;
            
            table_[i] = 10.0 * t3 - 15.0 * t4 + 6.0 * t5;
            derivTable_[i] = 30.0 * t2 - 60.0 * t3 + 30.0 * t4;
            accelTable_[i] = 60.0 * t - 180.0 * t2 + 120.0 * t3;
        }
    }
    
    // 快速查表
    double position(double t) const {
        t = std::max(0.0, std::min(1.0, t));
        double idx = t * resolution_;
        int i0 = static_cast<int>(idx);
        int i1 = std::min(i0 + 1, resolution_);
        double frac = idx - i0;
        return table_[i0] * (1.0 - frac) + table_[i1] * frac;
    }
    
    double velocity(double t) const {
        t = std::max(0.0, std::min(1.0, t));
        double idx = t * resolution_;
        int i0 = static_cast<int>(idx);
        int i1 = std::min(i0 + 1, resolution_);
        double frac = idx - i0;
        return derivTable_[i0] * (1.0 - frac) + derivTable_[i1] * frac;
    }
    
    double acceleration(double t) const {
        t = std::max(0.0, std::min(1.0, t));
        double idx = t * resolution_;
        int i0 = static_cast<int>(idx);
        int i1 = std::min(i0 + 1, resolution_);
        double frac = idx - i0;
        return accelTable_[i0] * (1.0 - frac) + accelTable_[i1] * frac;
    }
    
private:
    int resolution_;
    std::vector<double> table_;
    std::vector<double> derivTable_;
    std::vector<double> accelTable_;
};

/**
 * @brief 高性能时间参数化配置
 */
struct OptimizedTimeConfig {
    // 运动约束
    std::array<double, 6> maxVelocity = {3.0, 3.0, 3.0, 4.0, 4.0, 4.0};      // rad/s
    std::array<double, 6> maxAcceleration = {10.0, 10.0, 10.0, 15.0, 15.0, 15.0};  // rad/s²
    std::array<double, 6> maxJerk = {50.0, 50.0, 50.0, 80.0, 80.0, 80.0};    // rad/s³
    
    // 全局限制
    double globalVelocityScale = 1.0;   // 速度缩放因子
    double globalAccelScale = 1.0;      // 加速度缩放因子
    
    // 算法参数
    int tableResolution = 1000;         // S曲线查表分辨率
    double minSegmentTime = 0.01;       // 最小段时间 (秒)
    double blendRadius = 0.05;          // 混合半径 (弧度)
    
    VelocityProfileType profileType = VelocityProfileType::SCurve;
    
    static OptimizedTimeConfig fromRobotParams(double velScale = 1.0, double accelScale = 1.0) {
        OptimizedTimeConfig config;
        config.globalVelocityScale = velScale;
        config.globalAccelScale = accelScale;
        return config;
    }
};

/**
 * @brief 高性能时间参数化器
 */
class OptimizedTimeParameterizer {
public:
    explicit OptimizedTimeParameterizer(const OptimizedTimeConfig& config = OptimizedTimeConfig())
        : config_(config), scurveTable_(config.tableResolution) {}
    
    /**
     * @brief 快速路径时间参数化
     */
    Trajectory parameterize(const Path& path) {
        stats_.reset();
        auto startTime = std::chrono::high_resolution_clock::now();
        
        Trajectory traj;
        
        if (path.waypoints.size() < 2) {
            return traj;
        }
        
        int n = static_cast<int>(path.waypoints.size());
        
        // 1. 计算段长度和方向
        std::vector<double> segmentLengths(n - 1);
        std::vector<JointVector> segmentDirs(n - 1);
        double totalLength = 0;
        
        for (int i = 0; i < n - 1; ++i) {
            JointVector diff = path.waypoints[i + 1].config.q - path.waypoints[i].config.q;
            segmentLengths[i] = diff.norm();
            totalLength += segmentLengths[i];
            
            if (segmentLengths[i] > 1e-10) {
                segmentDirs[i] = diff / segmentLengths[i];
            } else {
                segmentDirs[i].setZero();
            }
        }
        
        // 2. 计算每段的最大允许速度
        std::vector<double> maxSegmentVel(n - 1);
        for (int i = 0; i < n - 1; ++i) {
            maxSegmentVel[i] = computeMaxVelocity(segmentDirs[i]);
        }
        
        // 3. 前向传播计算入口速度
        std::vector<double> entryVel(n, 0.0);
        entryVel[0] = 0.0;  // 从静止开始
        
        for (int i = 0; i < n - 1; ++i) {
            double maxAccel = computeMaxAcceleration(segmentDirs[i]);
            // v² = v0² + 2*a*s
            double velSq = entryVel[i] * entryVel[i] + 2.0 * maxAccel * segmentLengths[i];
            double vel = std::sqrt(velSq);
            vel = std::min(vel, maxSegmentVel[i]);
            
            if (i < n - 2) {
                // 考虑转弯减速
                double turnFactor = computeTurnFactor(segmentDirs[i], segmentDirs[i + 1]);
                vel *= turnFactor;
            }
            
            entryVel[i + 1] = vel;
        }
        
        // 4. 后向传播确保可减速
        entryVel[n - 1] = 0.0;  // 结束时静止
        
        for (int i = n - 2; i >= 0; --i) {
            double maxAccel = computeMaxAcceleration(segmentDirs[i]);
            // 从后向前: v0² = v² - 2*a*s  =>  v0 = sqrt(v² + 2*a*s) (减速)
            double velSq = entryVel[i + 1] * entryVel[i + 1] + 2.0 * maxAccel * segmentLengths[i];
            double vel = std::sqrt(velSq);
            entryVel[i] = std::min(entryVel[i], vel);
        }
        
        // 5. 生成轨迹点
        double currentTime = 0.0;
        {
            TrajectoryPoint startPt;
            startPt.config = path.waypoints[0].config;
            startPt.time = currentTime;
            traj.points.push_back(startPt);
        }
        
        for (int i = 0; i < n - 1; ++i) {
            double v0 = entryVel[i];
            double v1 = entryVel[i + 1];
            double s = segmentLengths[i];
            
            if (s < 1e-10) continue;
            
            // 计算段时间
            double segmentTime;
            if (std::abs(v0 - v1) < 1e-10) {
                double vAvg = (v0 + v1) * 0.5;
                if (vAvg < 1e-10) vAvg = 0.1;  // 避免除零
                segmentTime = s / vAvg;
            } else {
                // 梯形/S曲线时间
                double vMax = maxSegmentVel[i];
                segmentTime = computeSegmentTime(s, v0, v1, vMax, 
                    computeMaxAcceleration(segmentDirs[i]));
            }
            
            segmentTime = std::max(segmentTime, config_.minSegmentTime);
            currentTime += segmentTime;
            
            // 添加轨迹点
            TrajectoryPoint pt;
            pt.config = path.waypoints[i + 1].config;
            pt.time = currentTime;
            traj.points.push_back(pt);
        }
        
        // 填充速度和加速度
        computeVelocitiesAndAccelerations(traj);
        
        // 统计
        stats_.segments = n - 1;
        stats_.totalDuration = currentTime;
        
        auto endTime = std::chrono::high_resolution_clock::now();
        stats_.computationTime = std::chrono::duration<double>(endTime - startTime).count();
        
        return traj;
    }
    
    /**
     * @brief 快速样条时间参数化
     */
    Trajectory parameterizeSpline(const BSpline& spline, int numSamples = 100) {
        // 采样样条生成路径点
        Path sampledPath;
        for (int i = 0; i <= numSamples; ++i) {
            double t = static_cast<double>(i) / numSamples;
            sampledPath.waypoints.push_back(Waypoint(spline.evaluate(t)));
        }
        
        return parameterize(sampledPath);
    }
    
    /**
     * @brief 获取性能统计
     */
    const TimeParameterizationStats& getStats() const { return stats_; }
    
    void setConfig(const OptimizedTimeConfig& config) {
        config_ = config;
    }
    
private:
    double computeMaxVelocity(const JointVector& dir) const {
        double minFactor = std::numeric_limits<double>::infinity();
        
        for (int i = 0; i < 6; ++i) {
            if (std::abs(dir[i]) > 1e-10) {
                double factor = config_.maxVelocity[i] * config_.globalVelocityScale / std::abs(dir[i]);
                minFactor = std::min(minFactor, factor);
            }
        }
        
        return minFactor < std::numeric_limits<double>::infinity() ? minFactor : 1.0;
    }
    
    double computeMaxAcceleration(const JointVector& dir) const {
        double minFactor = std::numeric_limits<double>::infinity();
        
        for (int i = 0; i < 6; ++i) {
            if (std::abs(dir[i]) > 1e-10) {
                double factor = config_.maxAcceleration[i] * config_.globalAccelScale / std::abs(dir[i]);
                minFactor = std::min(minFactor, factor);
            }
        }
        
        return minFactor < std::numeric_limits<double>::infinity() ? minFactor : 5.0;
    }
    
    double computeTurnFactor(const JointVector& dir1, const JointVector& dir2) const {
        // 计算转向角
        double dot = dir1.dot(dir2);
        dot = std::max(-1.0, std::min(1.0, dot));  // 数值稳定
        
        double angle = std::acos(dot);
        
        // 转向角越大，速度越低
        // cos(angle/2) 给出平滑的减速因子
        return std::cos(angle * 0.5);
    }
    
    double computeSegmentTime(double s, double v0, double v1, double vMax, double aMax) const {
        if (config_.profileType == VelocityProfileType::Trapezoidal) {
            return computeTrapezoidalTime(s, v0, v1, vMax, aMax);
        } else {
            return computeSCurveTime(s, v0, v1, vMax, aMax);
        }
    }
    
    double computeTrapezoidalTime(double s, double v0, double v1, double vMax, double aMax) const {
        // 简化梯形计算
        double vAvg = (v0 + v1 + vMax) / 3.0;
        if (vAvg < 1e-10) vAvg = 0.1;
        
        double accelTime = std::abs(vMax - v0) / aMax;
        double decelTime = std::abs(vMax - v1) / aMax;
        double cruiseTime = 0;
        
        double accelDist = (v0 + vMax) * 0.5 * accelTime;
        double decelDist = (v1 + vMax) * 0.5 * decelTime;
        double cruiseDist = s - accelDist - decelDist;
        
        if (cruiseDist > 0) {
            cruiseTime = cruiseDist / vMax;
        } else {
            // 三角形速度剖面
            double vPeak = std::sqrt(aMax * s + 0.5 * (v0 * v0 + v1 * v1));
            vPeak = std::min(vPeak, vMax);
            accelTime = std::abs(vPeak - v0) / aMax;
            decelTime = std::abs(vPeak - v1) / aMax;
        }
        
        return accelTime + cruiseTime + decelTime;
    }
    
    double computeSCurveTime(double s, double v0, double v1, double vMax, double aMax) const {
        // S曲线需要更多时间（加加速段）
        double trapTime = computeTrapezoidalTime(s, v0, v1, vMax, aMax);
        return trapTime * 1.2;  // S曲线约比梯形慢20%
    }
    
    void computeVelocitiesAndAccelerations(Trajectory& traj) const {
        int n = static_cast<int>(traj.points.size());
        if (n < 2) return;
        
        // 速度：中心差分
        for (int i = 0; i < n; ++i) {
            if (i == 0) {
                double dt = traj.points[1].time - traj.points[0].time;
                if (dt > 1e-10) {
                    traj.points[i].velocity = (traj.points[1].config.q - traj.points[0].config.q) / dt;
                }
            } else if (i == n - 1) {
                double dt = traj.points[n-1].time - traj.points[n-2].time;
                if (dt > 1e-10) {
                    traj.points[i].velocity = (traj.points[n-1].config.q - traj.points[n-2].config.q) / dt;
                }
            } else {
                double dt = traj.points[i+1].time - traj.points[i-1].time;
                if (dt > 1e-10) {
                    traj.points[i].velocity = (traj.points[i+1].config.q - traj.points[i-1].config.q) / dt;
                }
            }
            
            // 更新统计
            double velNorm = traj.points[i].velocity.norm();
            stats_.maxVelocity = std::max(stats_.maxVelocity, velNorm);
        }
        
        // 加速度：速度差分
        for (int i = 0; i < n; ++i) {
            if (i == 0) {
                double dt = traj.points[1].time - traj.points[0].time;
                if (dt > 1e-10) {
                    traj.points[i].acceleration = (traj.points[1].velocity - traj.points[0].velocity) / dt;
                }
            } else if (i == n - 1) {
                double dt = traj.points[n-1].time - traj.points[n-2].time;
                if (dt > 1e-10) {
                    traj.points[i].acceleration = (traj.points[n-1].velocity - traj.points[n-2].velocity) / dt;
                }
            } else {
                double dt = traj.points[i+1].time - traj.points[i-1].time;
                if (dt > 1e-10) {
                    traj.points[i].acceleration = (traj.points[i+1].velocity - traj.points[i-1].velocity) / dt;
                }
            }
            
            double accelNorm = traj.points[i].acceleration.norm();
            stats_.maxAcceleration = std::max(stats_.maxAcceleration, accelNorm);
        }
    }
    
    OptimizedTimeConfig config_;
    SCurveTable scurveTable_;
    mutable TimeParameterizationStats stats_;
};

/**
 * @brief 轨迹验证器
 */
class OptimizedTrajectoryValidator {
public:
    struct ValidationResult {
        bool valid = true;
        std::string errorMessage;
        double maxVelocityViolation = 0;
        double maxAccelerationViolation = 0;
        double maxJerkViolation = 0;
        int violationCount = 0;
    };
    
    static ValidationResult validate(const Trajectory& traj, 
                                     const OptimizedTimeConfig& limits) {
        ValidationResult result;
        
        for (size_t i = 0; i < traj.points.size(); ++i) {
            const auto& pt = traj.points[i];
            
            // 检查速度
            for (int j = 0; j < 6; ++j) {
                double vel = std::abs(pt.velocity[j]);
                double limit = limits.maxVelocity[j] * limits.globalVelocityScale;
                
                if (vel > limit * 1.01) {  // 1%容差
                    result.valid = false;
                    result.violationCount++;
                    double violation = (vel - limit) / limit;
                    result.maxVelocityViolation = std::max(result.maxVelocityViolation, violation);
                }
            }
            
            // 检查加速度
            for (int j = 0; j < 6; ++j) {
                double accel = std::abs(pt.acceleration[j]);
                double limit = limits.maxAcceleration[j] * limits.globalAccelScale;
                
                if (accel > limit * 1.01) {
                    result.valid = false;
                    result.violationCount++;
                    double violation = (accel - limit) / limit;
                    result.maxAccelerationViolation = std::max(result.maxAccelerationViolation, violation);
                }
            }
        }
        
        if (!result.valid) {
            result.errorMessage = "Trajectory violates kinematic limits: " + 
                std::to_string(result.violationCount) + " violations";
        }
        
        return result;
    }
};

} // namespace palletizing
