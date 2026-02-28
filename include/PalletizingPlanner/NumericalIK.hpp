/**
 * @file NumericalIK.hpp
 * @brief 基于CollisionCheckerSO的数值逆运动学求解器
 *
 * 从testS50PalletizingSO.cpp和testTrajectoryOptimality.cpp中提取的共享IK代码。
 * 使用阻尼最小二乘(DLS)法求解位置级IK，支持多起点搜索。
 *
 * 单位:
 *   - 目标位置: mm (毫米)
 *   - 关节角度: rad (内部) / deg (API输入)
 *   - FK输出: mm (v1.0.0: CollisionCheckerSO.forwardKinematics返回mm)
 *
 * @author Guangdong Huayan Robotics Co., Ltd.
 * @version 1.0.0
 * @date 2026-02-25
 */

#pragma once

#include "Types.hpp"
#include "RobotModel.hpp"
#include "CollisionCheckerSO.hpp"

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>

namespace palletizing {

/// IK求解结果
struct IKResult {
    JointConfig config;
    double posError_mm = 1e10;  ///< 位置误差 (mm)
    int iterations = 0;         ///< 迭代次数
    bool converged = false;     ///< 是否收敛
};

/**
 * @brief 阻尼最小二乘(DLS)数值IK求解
 *
 * 使用Jacobian转置+DLS方法迭代求解位置级IK。
 * Jacobian通过数值差分计算(利用.so内置FK)。
 *
 * @param checker 碰撞检测器 (用于FK, 所有FK通过SO库执行)
 * @param robot   机器人模型 (提供关节限位, 避免硬编码)
 * @param target_mm 目标TCP位置 (mm, 机器人基座坐标系)
 * @param seed 初始关节配置
 * @param tol_mm 收敛容差 (mm)
 * @param maxIter 最大迭代次数
 * @param lambda DLS阻尼系数
 * @return IK求解结果
 */
inline IKResult numericalIK(CollisionCheckerSO& checker,
    const RobotModel& robot,
    const Eigen::Vector3d& target_mm, const JointConfig& seed,
    double tol_mm = 2.0, int maxIter = 500, double lambda = 5.0)
{
    IKResult result;
    JointConfig q = seed;
    const double eps = 1e-4;  // 数值差分步长 (rad)

    // 关节限位从 RobotModel 获取 (来源: RobotDHParams, 单位: rad)
    const auto& jMin = robot.getJointMin();
    const auto& jMax = robot.getJointMax();

    for (int iter = 0; iter < maxIter; iter++) {
        result.iterations = iter + 1;

        // 当前TCP位置 (v1.0.0 FK2直接返回mm)
        SO_COORD_REF tcp;
        if (!checker.forwardKinematics(q, tcp)) {
            result.posError_mm = 1e10;
            return result;
        }
        Eigen::Vector3d pos(tcp.X, tcp.Y, tcp.Z);

        // 位置误差
        Eigen::Vector3d err = target_mm - pos;
        result.posError_mm = err.norm();
        if (result.posError_mm < tol_mm) {
            result.config = q;
            result.converged = true;
            return result;
        }

        // 数值Jacobian (3x6, 位置部分)
        Eigen::Matrix<double, 3, 6> J;
        for (int j = 0; j < 6; j++) {
            JointConfig qp = q;
            qp.q[j] += eps;
            SO_COORD_REF tp;
            if (!checker.forwardKinematics(qp, tp)) {
                // FK失败, 使用零列避免NaN注入
                J.col(j).setZero();
                continue;
            }
            J.col(j) = (Eigen::Vector3d(tp.X, tp.Y, tp.Z) - pos) / eps;
        }

        // DLS: dq = J^T * (J*J^T + λ²I)^(-1) * err
        Eigen::Matrix3d JJT = J * J.transpose() + lambda * lambda * Eigen::Matrix3d::Identity();
        Eigen::Matrix<double, 6, 1> dq = J.transpose() * JJT.ldlt().solve(err);

        // 步长限制 (防止单次迭代跳跃过大导致振荡)
        constexpr double maxStep = 0.08;  // rad, 经验值 ~4.6deg
        double dqN = dq.norm();
        if (dqN > maxStep) dq *= maxStep / dqN;

        // 更新 + 关节限位钳制 (限位来自 RobotModel)
        for (int j = 0; j < 6; j++) {
            q.q[j] = std::clamp(q.q[j] + dq(j), jMin[j], jMax[j]);
        }
    }

    result.config = q;
    return result;
}

/**
 * @brief 多起点IK求解
 *
 * 依次尝试多个种子配置, 返回最佳收敛结果。
 * 如果第一轮未收敛，自动用更激进参数(更多迭代、更小阻尼)重试。
 *
 * @param checker 碰撞检测器
 * @param robot   机器人模型 (提供关节限位)
 * @param target_mm 目标TCP位置 (mm)
 * @param seeds 种子配置列表
 * @param tol_mm 收敛容差 (mm)
 * @return 最佳IK结果
 */
inline IKResult multiStartIK(CollisionCheckerSO& checker,
    const RobotModel& robot,
    const Eigen::Vector3d& target_mm, const std::vector<JointConfig>& seeds,
    double tol_mm = 2.0)
{
    IKResult best;
    best.posError_mm = 1e10;

    // 第一轮: 标准参数
    for (const auto& seed : seeds) {
        auto r = numericalIK(checker, robot, target_mm, seed, tol_mm);
        if (r.converged && r.posError_mm < best.posError_mm) best = r;
        if (best.converged && best.posError_mm < tol_mm * 0.5) break;
    }

    // 第二轮: 更激进参数 (更多迭代, 更小阻尼)
    if (!best.converged) {
        for (const auto& seed : seeds) {
            auto r = numericalIK(checker, robot, target_mm, seed, tol_mm, 1000, 2.0);
            if (r.posError_mm < best.posError_mm) best = r;
            if (r.converged) break;
        }
    }

    return best;
}

} // namespace palletizing
