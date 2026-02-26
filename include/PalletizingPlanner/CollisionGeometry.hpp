/**
 * @file CollisionGeometry.hpp
 * @brief HR_S50-2000 统一碰撞几何参数定义
 *
 * 所有碰撞检测模块 (CollisionChecker / CollisionCheckerSO) 共享此参数。
 * 参数已根据 HansAlgorithmExport S50 配置验证。
 *
 * 单位: mm (所有距离/坐标/半径)
 *
 * 胶囊体格式: [start_x, start_y, start_z, end_x, end_y, end_z, radius]
 * 球体格式:   [center_x, center_y, center_z, radius]
 *
 * @author Guangdong Huayan Robotics Co., Ltd.
 * @version 1.0.0
 * @date 2026-02-24
 */

#pragma once

namespace palletizing {

/**
 * @brief HR_S50-2000 碰撞包络几何参数
 *
 * 采用 HansAlgorithmExport .so 配置 (较保守的包络) 作为基准参数,
 * 确保静态库和动态库两套碰撞检测栈行为一致。
 */
struct S50CollisionGeometry {
    // ========================================================================
    // 胶囊体 [start_xyz, end_xyz, radius] (mm)
    // ========================================================================

    /// 基座 — 覆盖 Joint1 旋转体
    static constexpr double baseCapsule[7]     = {0, 0, 20,  0, 0, 330, 160};

    /// 下臂 — 覆盖 Link2 (Joint2 → Joint3)
    static constexpr double lowerArmCapsule[7] = {0, 0, 340, 900, 0, 340, 140};

    /// 肘部 — 覆盖 Link3 (Joint3 → Joint4)
    static constexpr double elbowCapsule[7]    = {-10, 0, 60, 941.5, 0, 60, 120};

    /// 上臂 — 覆盖 Link4 (Joint4 → Joint5)
    static constexpr double upperArmCapsule[7] = {0, 0, -50, 0, 0, 100, 100};

    // ========================================================================
    // 球体 [center_xyz, radius] (mm)
    // ========================================================================

    /// 腕部 — 球体包络 Joint5/Joint6
    static constexpr double wristBall[4]       = {0, 0, 20, 140};

    // ========================================================================
    // DH 参数 (mm)
    // ========================================================================

    /// S50 DH 参数: d1, d2, d3, d4, d5, d6, a2, a3
    static constexpr double dhParams[8] = {296.5, 336.2, 239.0, 158.5, 158.5, 134.5, 900.0, 941.5};

    // ========================================================================
    // 安全余量
    // ========================================================================

    /// 默认自碰撞安全余量 (mm)
    static constexpr double defaultSafetyMarginMm = 10.0;

    /// 默认环境碰撞安全余量 (mm)
    static constexpr double defaultEnvSafetyMarginMm = 5.0;
};

} // namespace palletizing
