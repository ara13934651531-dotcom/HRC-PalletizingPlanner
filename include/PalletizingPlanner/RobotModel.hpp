/**
 * @file RobotModel.hpp
 * @brief HR-S50协作机器人运动学模型
 * 
 * 实现了基于DH参数的正/逆运动学，以及关节限位检查。
 * 针对HR_S50-2000重载协作机器人优化。
 * 
 * @author GitHub Copilot
 * @version 1.0.0
 * @date 2026-01-29
 */

#pragma once

#include "Types.hpp"
#include <array>
#include <stdexcept>

namespace palletizing {

/**
 * @brief HR-S50机器人DH参数和运动学配置
 */
struct RobotDHParams {
    // DH参数 (单位: mm, 在使用时转换为m)
    double d1 = 296.5;   // 基座到肩关节
    double d2 = 336.2;   // 
    double d3 = 239.0;   //
    double d4 = 158.5;   //
    double d5 = 158.5;   //
    double d6 = 134.5;   // 腕部到法兰
    double a2 = 900.0;   // 下臂长度
    double a3 = 941.5;   // 上臂长度
    
    // 关节限位 (单位: deg, 在使用时转换为rad)
    std::array<double, 6> jointMin = {-360, -190, -165, -360, -360, -360};
    std::array<double, 6> jointMax = {360, 10, 165, 360, 360, 360};
    
    // 最大速度限制 (deg/s)
    std::array<double, 6> maxVelocity = {120, 120, 120, 180, 180, 180};
    
    // 最大加速度限制 (deg/s^2)
    std::array<double, 6> maxAcceleration = {121, 121, 121, 121, 121, 121};
    
    // 负载信息
    double maxPayload = 50.0;  // kg
    
    /// 从配置文件构造 (支持HR_S50-2000.hard格式)
    static RobotDHParams fromHRConfig() {
        RobotDHParams params;
        // 使用HR_S50-2000配置
        params.d1 = 296.5;
        params.d2 = 336.2;
        params.d3 = 239.0;
        params.d4 = 158.5;
        params.d5 = 158.5;
        params.d6 = 134.5;
        params.a2 = 900.0;
        params.a3 = 941.5;
        return params;
    }
    
    /// 转换为碰撞检测库需要的DH数组格式 (单位: mm)
    std::array<double, 8> toDHArray() const {
        return {d1, d2, d3, d4, d5, d6, a2, a3};
    }
    
    /// 获取关节限位 (rad)
    std::pair<JointVector, JointVector> getJointLimits() const {
        JointVector min_rad, max_rad;
        for (int i = 0; i < 6; ++i) {
            min_rad[i] = jointMin[i] * M_PI / 180.0;
            max_rad[i] = jointMax[i] * M_PI / 180.0;
        }
        return {min_rad, max_rad};
    }
};

/**
 * @brief 6自由度串联机器人运动学模型
 * 
 * 提供正运动学、雅可比矩阵计算等功能。
 * 采用改进的DH参数表示法。
 */
class RobotModel {
public:
    using JacobianMatrix = Eigen::Matrix<double, 6, 6>;
    
    explicit RobotModel(const RobotDHParams& params = RobotDHParams::fromHRConfig())
        : params_(params) {
        auto [min_limits, max_limits] = params_.getJointLimits();
        jointMin_ = min_limits;
        jointMax_ = max_limits;
    }
    
    /**
     * @brief 正运动学 - 计算末端位姿
     * @param config 关节配置 (rad)
     * @return 末端位姿
     */
    Pose6D forwardKinematics(const JointConfig& config) const {
        Eigen::Matrix4d T = computeFK(config.q);
        return Pose6D::fromMatrix(T);
    }
    
    /**
     * @brief 计算完整的FK变换矩阵
     * @param q 关节角度 (rad)
     * @return 4x4齐次变换矩阵
     */
    Eigen::Matrix4d computeFK(const JointVector& q) const {
        // 转换DH参数为米
        const double d1 = params_.d1 / 1000.0;
        const double d2 = params_.d2 / 1000.0;
        const double d3 = params_.d3 / 1000.0;
        const double d4 = params_.d4 / 1000.0;
        const double d5 = params_.d5 / 1000.0;
        const double d6 = params_.d6 / 1000.0;
        const double a2 = params_.a2 / 1000.0;
        const double a3 = params_.a3 / 1000.0;
        
        // 计算各关节的正余弦值
        std::array<double, 6> c, s;
        for (int i = 0; i < 6; ++i) {
            c[i] = std::cos(q[i]);
            s[i] = std::sin(q[i]);
        }
        
        // 使用标准DH参数计算变换矩阵
        // S系列机器人采用类UR构型
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        
        // 基于UR构型的DH参数
        // Link 1: d=d1, a=0, alpha=pi/2
        Eigen::Matrix4d T1;
        T1 << c[0], 0, s[0], 0,
              s[0], 0, -c[0], 0,
              0, 1, 0, d1,
              0, 0, 0, 1;
        
        // Link 2: d=d2, a=a2, alpha=0
        Eigen::Matrix4d T2;
        T2 << c[1], -s[1], 0, a2*c[1],
              s[1], c[1], 0, a2*s[1],
              0, 0, 1, d2,
              0, 0, 0, 1;
        
        // Link 3: d=d3, a=a3, alpha=0
        Eigen::Matrix4d T3;
        T3 << c[2], -s[2], 0, a3*c[2],
              s[2], c[2], 0, a3*s[2],
              0, 0, 1, d3,
              0, 0, 0, 1;
        
        // Link 4: d=d4, a=0, alpha=pi/2
        Eigen::Matrix4d T4;
        T4 << c[3], 0, s[3], 0,
              s[3], 0, -c[3], 0,
              0, 1, 0, d4,
              0, 0, 0, 1;
        
        // Link 5: d=d5, a=0, alpha=-pi/2
        Eigen::Matrix4d T5;
        T5 << c[4], 0, -s[4], 0,
              s[4], 0, c[4], 0,
              0, -1, 0, d5,
              0, 0, 0, 1;
        
        // Link 6: d=d6, a=0, alpha=0
        Eigen::Matrix4d T6;
        T6 << c[5], -s[5], 0, 0,
              s[5], c[5], 0, 0,
              0, 0, 1, d6,
              0, 0, 0, 1;
        
        T = T1 * T2 * T3 * T4 * T5 * T6;
        
        return T;
    }
    
    /**
     * @brief 计算雅可比矩阵
     * @param config 关节配置
     * @return 6x6雅可比矩阵
     */
    JacobianMatrix computeJacobian(const JointConfig& config) const {
        JacobianMatrix J;
        
        // 数值微分计算雅可比
        const double eps = 1e-8;
        Pose6D pose0 = forwardKinematics(config);
        
        for (int i = 0; i < 6; ++i) {
            JointVector q_plus = config.q;
            q_plus[i] += eps;
            
            Pose6D pose_plus = forwardKinematics(JointConfig(q_plus));
            
            // 位置部分
            J.block<3, 1>(0, i) = (pose_plus.position - pose0.position) / eps;
            
            // 姿态部分 (使用旋转矩阵差异的反对称部分)
            Eigen::Matrix3d R0 = pose0.orientation.toRotationMatrix();
            Eigen::Matrix3d R1 = pose_plus.orientation.toRotationMatrix();
            Eigen::Matrix3d dR = (R1 - R0) / eps * R0.transpose();
            
            J(3, i) = dR(2, 1);  // wx
            J(4, i) = dR(0, 2);  // wy
            J(5, i) = dR(1, 0);  // wz
        }
        
        return J;
    }
    
    /**
     * @brief 检查关节配置是否在限位内
     * @param config 关节配置
     * @return true如果在限位内
     */
    bool isWithinLimits(const JointConfig& config) const {
        for (int i = 0; i < 6; ++i) {
            if (config.q[i] < jointMin_[i] || config.q[i] > jointMax_[i]) {
                return false;
            }
        }
        return true;
    }
    
    /**
     * @brief 将关节配置钳制到限位内
     * @param config 关节配置
     * @return 钳制后的配置
     */
    JointConfig clampToLimits(const JointConfig& config) const {
        JointConfig clamped = config;
        for (int i = 0; i < 6; ++i) {
            clamped.q[i] = std::clamp(config.q[i], jointMin_[i], jointMax_[i]);
        }
        return clamped;
    }
    
    /**
     * @brief 计算配置空间的可操作度
     * @param config 关节配置
     * @return 可操作度指标
     */
    double manipulability(const JointConfig& config) const {
        JacobianMatrix J = computeJacobian(config);
        // 使用Yoshikawa's manipulability measure
        return std::sqrt((J * J.transpose()).determinant());
    }
    
    /**
     * @brief 生成随机关节配置
     * @return 随机配置
     */
    JointConfig randomConfig() const {
        JointConfig config;
        for (int i = 0; i < 6; ++i) {
            double range = jointMax_[i] - jointMin_[i];
            config.q[i] = jointMin_[i] + (static_cast<double>(rand()) / RAND_MAX) * range;
        }
        return config;
    }
    
    /**
     * @brief 在两个配置之间插值
     * @param start 起始配置
     * @param end 目标配置
     * @param t 插值参数 [0, 1]
     * @return 插值后的配置
     */
    JointConfig interpolate(const JointConfig& start, const JointConfig& end, double t) const {
        JointConfig result;
        result.q = start.q + t * (end.q - start.q);
        return result;
    }
    
    // 访问器
    const RobotDHParams& getParams() const { return params_; }
    const JointVector& getJointMin() const { return jointMin_; }
    const JointVector& getJointMax() const { return jointMax_; }
    
private:
    RobotDHParams params_;
    JointVector jointMin_;
    JointVector jointMax_;
};

/**
 * @brief 工具坐标系配置
 */
struct ToolConfig {
    Pose6D toolFrame;      // 工具坐标系相对法兰的变换
    double toolRadius = 0.05;  // 工具包络半径 [m]
    
    // 夹爪类型
    enum class GripperType {
        Suction,    // 吸盘
        Parallel,   // 平行夹爪
        Custom      // 自定义
    };
    
    GripperType type = GripperType::Suction;
    
    // 吸盘抓取偏移 (码垛常用)
    double approachOffset = 0.1;   // 接近偏移 [m]
    double retractOffset = 0.15;   // 撤退偏移 [m]
    
    /// 创建默认码垛吸盘工具
    static ToolConfig palletizingSuction() {
        ToolConfig tool;
        tool.type = GripperType::Suction;
        tool.toolFrame = Pose6D::fromEulerZYX(0, 0, 0.15, 0, 0, 0);
        tool.toolRadius = 0.08;
        tool.approachOffset = 0.1;
        tool.retractOffset = 0.15;
        return tool;
    }
};

} // namespace palletizing
