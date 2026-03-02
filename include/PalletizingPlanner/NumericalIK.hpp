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
 * @param constrainHorizontal 是否约束TCP保持水平朝下
 *   true: 每次迭代后投影 q5 = π + q2 - q3 (rad), 使TCP Z轴≈(0,0,-1)
 *   对应码垛场景: 吸盘必须水平, 否则箱子脱落
 * @return IK求解结果
 */
inline IKResult numericalIK(CollisionCheckerSO& checker,
    const RobotModel& robot,
    const Eigen::Vector3d& target_mm, const JointConfig& seed,
    double tol_mm = 2.0, int maxIter = 500, double lambda = 5.0,
    bool constrainHorizontal = false)
{
    IKResult result;
    JointConfig q = seed;
    const double eps = 1e-4;  // 数值差分步长 (rad)

    // 关节限位从 RobotModel 获取 (来源: RobotDHParams, 单位: rad)
    const auto& jMin = robot.getJointMin();
    const auto& jMax = robot.getJointMax();

    // TCP水平约束初始投影: 确保起始配置满足约束
    // 完整约束集 (经 FK2 全配置扫描验证, Zdev=0°):
    //   q4 = 0          → FK2 Euler B = 0 (pitch=0, 当q1≈-90°时 B=q4)
    //   q5 = π + q2 - q3 → 结构性保证 B=0
    //   q6 = q1         → FK2 Euler C = ±180° → TCP Z=(0,0,-1)
    // 约束后只留 J1,J2,J3 自由 (3DOF 对应 3 位置方程)
    if (constrainHorizontal) {
        q.q[3] = 0.0;  // q4 = 0 → FK2 Euler B=0
        q.q[4] = M_PI + q.q[1] - q.q[2];  // q5 = π + q2 - q3
        q.q[4] = std::clamp(q.q[4], jMin[4], jMax[4]);
        q.q[5] = q.q[0];  // q6 = q1 → C=±180°
        q.q[5] = std::clamp(q.q[5], jMin[5], jMax[5]);
    }

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

        // TCP水平约束投影: 每步迭代后强制 q4=0, q5=π+q2-q3, q6=q1
        if (constrainHorizontal) {
            q.q[3] = 0.0;  // q4 = 0 → B=0
            q.q[4] = M_PI + q.q[1] - q.q[2];
            q.q[4] = std::clamp(q.q[4], jMin[4], jMax[4]);
            q.q[5] = q.q[0];  // q6 = q1 → C=±180°
            q.q[5] = std::clamp(q.q[5], jMin[5], jMax[5]);
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
 * @param constrainHorizontal 是否约束TCP保持水平朝下
 * @return 最佳IK结果
 */
inline IKResult multiStartIK(CollisionCheckerSO& checker,
    const RobotModel& robot,
    const Eigen::Vector3d& target_mm, const std::vector<JointConfig>& seeds,
    double tol_mm = 2.0, bool constrainHorizontal = false)
{
    IKResult best;
    best.posError_mm = 1e10;

    // 第一轮: 标准参数
    for (const auto& seed : seeds) {
        auto r = numericalIK(checker, robot, target_mm, seed, tol_mm, 500, 5.0, constrainHorizontal);
        if (r.converged && r.posError_mm < best.posError_mm) best = r;
        if (best.converged && best.posError_mm < tol_mm * 0.5) break;
    }

    // 第二轮: 更激进参数 (更多迭代, 更小阻尼)
    if (!best.converged) {
        for (const auto& seed : seeds) {
            auto r = numericalIK(checker, robot, target_mm, seed, tol_mm, 1000, 2.0, constrainHorizontal);
            if (r.posError_mm < best.posError_mm) best = r;
            if (r.converged) break;
        }
    }

    return best;
}

} // namespace palletizing
