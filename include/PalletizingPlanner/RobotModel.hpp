/**
 * @file RobotModel.hpp
 * @brief HR-S50协作机器人参数模型与关节空间工具函数
 *
 * 仅提供机器人参数 (DH, 关节限位, 速度/加速度限制) 和关节空间工具函数。
 * 所有正/逆运动学计算必须通过 CollisionCheckerSO (libHRCInterface.so) 完成。
 *
 * ⚠️ 本模块不提供任何FK/IK实现。DH参数仅用于碰撞库初始化和参数查询。
 *
 * @author Guangdong Huayan Robotics Co., Ltd.
 * @version 2.0.0
 * @date 2026-02-26
 */

#pragma once

#include "Types.hpp"
#include <array>
#include <random>
#include <stdexcept>

namespace palletizing {

/**
 * @brief HR-S50机器人DH参数和运动学配置
 */
struct RobotDHParams {
    // DH参数 (单位: mm, 在使用时转换为m)

    double d1 = 296.5;   // 基座到肩关节 (Joint1偏移)
    double d2 = 336.2;   // 肩部横向偏移 (Joint2偏移)
    double d3 = 239.0;   // 肘部横向偏移 (Joint3偏移)
    double d4 = 158.5;   // 腕部纵向偏移 (Joint4偏移)
    double d5 = 158.5;   // 腕部横向偏移 (Joint5偏移)
    double d6 = 134.5;   // 腕部到法兰 (Joint6偏移)
    double a2 = 900.0;   // 下臂长度 (Link2)
    double a3 = 941.5;   // 上臂长度 (Link3)
    
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
 * @brief 6自由度串联机器人参数模型
 * 
 * 提供关节限位检查、随机采样、插值等关节空间工具函数。
 * FK/IK功能已由 CollisionCheckerSO (libHRCInterface.so) 提供，此处不再实现。
 */
class RobotModel {
public:
    
    explicit RobotModel(const RobotDHParams& params = RobotDHParams::fromHRConfig())
        : params_(params) {
        auto [min_limits, max_limits] = params_.getJointLimits();
        jointMin_ = min_limits;
        jointMax_ = max_limits;
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
     * @brief 生成随机关节配置
     * @return 随机配置
     */
    JointConfig randomConfig() const {
        // 使用thread_local确保线程安全
        static thread_local std::mt19937 tl_gen{std::random_device{}()};
        JointConfig config;
        for (int i = 0; i < 6; ++i) {
            std::uniform_real_distribution<double> dist(jointMin_[i], jointMax_[i]);
            config.q[i] = dist(tl_gen);
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
    double toolRadius = 0.05;  // 工具包络半径 [m] (通用默认, 吸盘见palletizingSuction)
    
    // 夹爪类型
    enum class GripperType {
        Suction,    // 吸盘
        Parallel,   // 平行夹爪
        Custom      // 自定义
    };
    
    GripperType type = GripperType::Suction;
    
    // 吸盘抓取偏移 (码垛常用)
    double approachOffset = 0.1;   // 接近偏移 [m] (箱子高度250mm/2=125mm ≈ 0.1m)
    double retractOffset = 0.15;   // 撤退偏移 [m] (接近偏移+安全余量)
    
    /// 创建HR_S50码垛吸盘工具 (参数来源: 码垛工站实测)
    static ToolConfig palletizingSuction() {
        ToolConfig tool;
        tool.type = GripperType::Suction;
        // 吸盘安装偏移: 法兰→吸盘中心 150mm (d6=134.5mm + 法兰厚度)
        tool.toolFrame = Pose6D::fromEulerZYX(0, 0, 0.15, 0, 0, 0);
        // 吸盘外径~160mm, 包络半径取80mm (含安全裕度)
        tool.toolRadius = 0.08;
        // 接近/撤退偏移 (箱子高度250mm的一半 + 安全距离)
        tool.approachOffset = 0.1;   // 100mm
        tool.retractOffset = 0.15;   // 150mm
        return tool;
    }
};

} // namespace palletizing
