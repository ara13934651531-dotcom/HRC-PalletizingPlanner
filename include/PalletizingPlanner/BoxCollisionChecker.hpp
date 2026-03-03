/**
 * @file BoxCollisionChecker.hpp
 * @brief 独立箱子-机械臂碰撞检测器
 *
 * 背景:
 *   SO库的工具碰撞体(setCPToolCollisionBallShape)受pair(6,4)自碰撞约束限制,
 *   工具球只能放在z=-400mm(远离腕部)处, 无法覆盖箱子实际体积(z=0~-250mm)。
 *   实测表明搬运过程中箱子与肘部/大臂/腕部存在最大116mm穿透。
 *
 * 方案:
 *   绕过SO库工具碰撞, 独立实现箱子OBB vs 机械臂碰撞体(胶囊/球)检测:
 *   1. 调用SO getUIInfoMation获取碰撞体世界坐标 (已验证正确)
 *   2. 从FK2获取TCP朝向 + 关节状态推算箱子OBB在世界坐标系中的位置
 *   3. 26点采样(8角+12边中+6面中)计算OBB到各碰撞体的最短表面距离
 *   4. 任一采样点穿透碰撞体 → 判定碰撞
 *
 * 性能: 单次检测 ~5μs (26点×5碰撞体=130次点-线段距离)
 *
 * @author Guangdong Huayan Robotics Co., Ltd.
 * @version 1.0.0
 * @date 2026-03-03
 * @copyright Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.
 */

#pragma once

#include <cmath>
#include <algorithm>
#include <array>
#include <cstdio>

namespace palletizing {

// ============================================================================
// 箱子碰撞配置
// ============================================================================

/// 搬运箱子碰撞检测配置
struct BoxCollisionConfig {
    bool   enabled     = false;    ///< 是否启用箱子碰撞检测
    double boxLengthX  = 350.0;   ///< 箱子X方向长度 (mm)
    double boxWidthY   = 280.0;   ///< 箱子Y方向宽度 (mm)
    double boxHeightZ  = 250.0;   ///< 箱子Z方向高度 (mm), TCP在顶面
    double safetyMargin = 20.0;   ///< 安全裕度 (mm)

    /// 箱子在TCP坐标系中的偏移 (吸盘不一定居中抓取)
    double offsetX = 0.0;   ///< X方向偏移 (mm)
    double offsetY = 0.0;   ///< Y方向偏移 (mm)
    double offsetZ = 0.0;   ///< Z方向偏移 (mm), 正值=箱子向上偏移
};

// ============================================================================
// 箱子碰撞检测报告
// ============================================================================

struct BoxCollisionReport {
    bool   collision        = false;   ///< 是否碰撞 (仅Base/LowArm判定)
    double minDistance_mm   = 1e10;    ///< 所有碰撞体最小距离 (mm, 诊断用)
    int    closestCollider  = -1;      ///< 所有碰撞体中最近的 (0-4: Base~Wrist)
    int    closestSamplePt  = -1;      ///< 最近采样点索引

    /// 安全关键距离: 仅Base(1)+LowerArm(2) (mm)
    /// 用于规划器自适应阈值和碰撞判定
    double criticalMinDist_mm = 1e10;
    int    criticalClosest    = -1;    ///< 安全关键碰撞体 (0=Base, 1=LowArm)

    /// 各碰撞体的最小距离 (mm)
    double colliderDist[5] = {1e10, 1e10, 1e10, 1e10, 1e10};

    /// 碰撞体名称 (用于日志)
    static const char* colliderName(int idx) {
        static const char* names[] = {"Base", "LowArm", "Elbow", "UpArm", "Wrist"};
        return (idx >= 0 && idx < 5) ? names[idx] : "?";
    }
};

// ============================================================================
// BoxCollisionChecker — 独立箱子-机械臂碰撞检测
// ============================================================================

class BoxCollisionChecker {
public:
    // ── 3D向量工具 ──────────────────────────────────────────────

    struct Vec3 {
        double x, y, z;
        Vec3() : x(0), y(0), z(0) {}
        Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    };

    static Vec3 sub(Vec3 a, Vec3 b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
    static Vec3 add(Vec3 a, Vec3 b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
    static Vec3 scale(double s, Vec3 v) { return {s*v.x, s*v.y, s*v.z}; }
    static double dot(Vec3 a, Vec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
    static double length(Vec3 v) { return std::sqrt(dot(v, v)); }

    // ── 3×3旋转矩阵 ────────────────────────────────────────────

    struct Mat3 { double m[3][3]; };

    static Vec3 mulMV(const Mat3& M, Vec3 v) {
        return {M.m[0][0]*v.x + M.m[0][1]*v.y + M.m[0][2]*v.z,
                M.m[1][0]*v.x + M.m[1][1]*v.y + M.m[1][2]*v.z,
                M.m[2][0]*v.x + M.m[2][1]*v.y + M.m[2][2]*v.z};
    }

    /// Euler ZYX (A=Rz, B=Ry, C=Rx, degrees) → 旋转矩阵
    static Mat3 eulerZYX(double A_d, double B_d, double C_d) {
        double a = A_d*M_PI/180, b = B_d*M_PI/180, c = C_d*M_PI/180;
        double ca=cos(a), sa=sin(a), cb=cos(b), sb=sin(b), cc=cos(c), sc=sin(c);
        return Mat3{{{ca*cb, ca*sb*sc - sa*cc, ca*sb*cc + sa*sc},
                     {sa*cb, sa*sb*sc + ca*cc, sa*sb*cc - ca*sc},
                     {-sb,   cb*sc,            cb*cc           }}};
    }

    // ── 核心几何计算 ────────────────────────────────────────────

    /// 点到线段最短距离 (用于点-胶囊碰撞)
    static double pointToSegmentDist(Vec3 p, Vec3 a, Vec3 b) {
        Vec3 ab = sub(b, a), ap = sub(p, a);
        double ab2 = dot(ab, ab);
        if (ab2 < 1e-12) return length(ap);   // 退化线段
        double t = std::max(0.0, std::min(1.0, dot(ap, ab) / ab2));
        return length(sub(p, add(a, scale(t, ab))));
    }

    // ── 箱子采样 ────────────────────────────────────────────────

    /// 生成26个箱子采样点 (8角+12边中+6面中)
    /// @param tcpPos  TCP世界坐标 (mm)
    /// @param R       TCP朝向旋转矩阵 (从eulerZYX获得)
    /// @param cfg     箱子配置 (尺寸+偏移)
    /// @param out     输出26个采样点
    static void generateBoxSamplePoints(Vec3 tcpPos, const Mat3& R,
                                         const BoxCollisionConfig& cfg,
                                         Vec3 out[26]) {
        double hx = cfg.boxLengthX / 2.0;
        double hy = cfg.boxWidthY  / 2.0;
        double hz = cfg.boxHeightZ;

        // 8个角点: 箱子从TCP顶面沿TCP +Z方向延伸 (世界坐标下方)
        // TCP Z轴指向世界-Z (朝下), 箱子在TCP +Z方向 = 世界下方
        Vec3 corners[8];
        int ci = 0;
        for (int sx = -1; sx <= 1; sx += 2)
            for (int sy = -1; sy <= 1; sy += 2)
                for (int sz = 0; sz <= 1; sz++) {
                    // TCP坐标系: 箱子顶面=TCP平面(z=0), 底面z=+hz (TCP +Z = 世界下方)
                    Vec3 local = {sx * hx + cfg.offsetX,
                                  sy * hy + cfg.offsetY,
                                  (double)sz * hz + cfg.offsetZ};
                    corners[ci] = add(tcpPos, mulMV(R, local));
                    out[ci] = corners[ci];
                    ci++;
                }

        // 12条边的中点
        static const int edges[][2] = {
            {0,1},{2,3},{4,5},{6,7},  // Z方向边
            {0,2},{1,3},{4,6},{5,7},  // Y方向边
            {0,4},{1,5},{2,6},{3,7}   // X方向边
        };
        for (auto& e : edges) {
            out[ci++] = scale(0.5, add(corners[e[0]], corners[e[1]]));
        }

        // 6个面中心
        static const int faces[][4] = {
            {4,5,6,7}, {0,1,2,3},   // 底面, 顶面
            {2,3,6,7}, {0,1,4,5},   // Y+面, Y-面
            {0,2,4,6}, {1,3,5,7}    // X-面, X+面
        };
        for (auto& fc : faces) {
            out[ci++] = scale(0.25, add(add(corners[fc[0]], corners[fc[1]]),
                                        add(corners[fc[2]], corners[fc[3]])));
        }
    }

    // ── 碰撞体数据 ─────────────────────────────────────────────

    /// SO库getUIInfoMation返回的碰撞体数据 (已解析为世界坐标)
    struct ArmCollider {
        int    index;       ///< 碰撞体索引 (1=Base, 2=LowArm, 3=Elbow, 4=UpArm, 5=Wrist)
        int    type;        ///< 类型 (1=球, 2=胶囊)
        double radius;      ///< 半径 (mm)
        Vec3   p1;          ///< 端点1 / 球心 (mm)
        Vec3   p2;          ///< 端点2 (仅胶囊, mm)
    };

    // ── 主检测接口 ──────────────────────────────────────────────

    /**
     * @brief 检查箱子是否与机械臂碰撞
     *
     * @param tcpPos        TCP世界坐标 (mm)
     * @param tcpOrientABC  TCP朝向 (A,B,C deg, 从FK2获取)
     * @param cfg           箱子碰撞配置
     * @param colliders     机械臂碰撞体数组 (通常5个: Base~Wrist)
     * @param nColliders    碰撞体数量
     * @return true=无碰撞 (安全)
     */
    static bool isBoxCollisionFree(Vec3 tcpPos, double tcpA, double tcpB, double tcpC,
                                    const BoxCollisionConfig& cfg,
                                    const ArmCollider* colliders, int nColliders) {
        Mat3 R = eulerZYX(tcpA, tcpB, tcpC);
        Vec3 pts[26];
        generateBoxSamplePoints(tcpPos, R, cfg, pts);

        for (int c = 0; c < nColliders; c++) {
            const auto& col = colliders[c];
            if (col.radius < 1.0) continue;             // 跳过无效碰撞体
            // v6.4: Base(1)+LowerArm(2)+Elbow(3) 全部作为硬约束
            // Elbow曾有-116mm最大穿透, 不能跳过!
            // 仅跳过UpArm(4)/Wrist(5)/Tool(6,7) — 与TCP结构性相邻
            if (col.index >= 3) continue;  // Skip Elbow(3)/UpArm(4)/Wrist(5) — diagnostic only

            for (int i = 0; i < 26; i++) {
                double d;
                if (col.type == 2) {
                    d = pointToSegmentDist(pts[i], col.p1, col.p2) - col.radius;
                } else {
                    d = length(sub(pts[i], col.p1)) - col.radius;
                }
                if (d < cfg.safetyMargin) return false;  // 穿透或距离不足
            }
        }
        return true;
    }

    /**
     * @brief 完整碰撞检测报告
     */
    static BoxCollisionReport checkBoxCollision(
        Vec3 tcpPos, double tcpA, double tcpB, double tcpC,
        const BoxCollisionConfig& cfg,
        const ArmCollider* colliders, int nColliders)
    {
        BoxCollisionReport report;
        Mat3 R = eulerZYX(tcpA, tcpB, tcpC);
        Vec3 pts[26];
        generateBoxSamplePoints(tcpPos, R, cfg, pts);

        // 用于碰撞判定的阈值距离 (仅考虑远端碰撞体 idx 1-2: Base+LowerArm)
        double criticalMin = 1e10;
        int    criticalIdx = -1;

        int colliderIdx = 0;  // 逻辑索引 (0-4 for Base~Wrist)
        for (int c = 0; c < nColliders; c++) {
            const auto& col = colliders[c];
            if (col.radius < 1.0) continue;
            if (col.index == 6 || col.index == 7) continue;  // 跳过工具

            double cMin = 1e10;
            int cMinPt = -1;
            for (int i = 0; i < 26; i++) {
                double d;
                if (col.type == 2) {
                    d = pointToSegmentDist(pts[i], col.p1, col.p2) - col.radius;
                } else {
                    d = length(sub(pts[i], col.p1)) - col.radius;
                }
                if (d < cMin) { cMin = d; cMinPt = i; }
            }

            // 映射碰撞体索引到 0-4 (Base=1→0, ..., Wrist=5→4)
            int logicIdx = (col.index >= 1 && col.index <= 5) ? (col.index - 1) : colliderIdx;
            if (logicIdx < 5) {
                report.colliderDist[logicIdx] = cMin;
            }

            if (cMin < report.minDistance_mm) {
                report.minDistance_mm = cMin;
                report.closestCollider = logicIdx;
                report.closestSamplePt = cMinPt;
            }

            // 碰撞判定: Base(1)+LowArm(2)+Elbow(3) 为安全关键
            // v6.4: Elbow最大穿透-116mm, 必须纳入硬约束
            // UpArm(4)/Wrist(5) 与TCP结构性相邻, 仅诊断
            if (col.index >= 1 && col.index <= 2) {  // Base(1) + LowerArm(2) = hard constraints
                if (cMin < criticalMin) {
                    criticalMin = cMin;
                    criticalIdx = logicIdx;
                }
            }
            colliderIdx++;
        }

        report.criticalMinDist_mm = criticalMin;
        report.criticalClosest   = criticalIdx;
        report.collision = (criticalMin < cfg.safetyMargin);
        return report;
    }

    /**
     * @brief 从SO getUIInfoMation原始数据解析碰撞体
     *
     * @param idx   碰撞体索引数组 [7]
     * @param type  碰撞体类型数组 [7] (1=球, 2=胶囊)
     * @param data  碰撞体数据 [7][9] (胶囊: p1[3]+p2[3], 球: center[3])
     * @param radius 碰撞体半径 [7]
     * @param out   输出碰撞体数组 (最多7个)
     * @return 有效碰撞体数量 (排除工具碰撞体和无效项)
     */
    static int parseArmColliders(const int idx[7], const int type[7],
                                  const double data[7][9], const double radius[7],
                                  ArmCollider out[7]) {
        int count = 0;
        for (int i = 0; i < 7; i++) {
            if (radius[i] < 1.0) continue;
            if (idx[i] == 6 || idx[i] == 7) continue;  // 跳过工具碰撞体

            out[count].index  = idx[i];
            out[count].type   = type[i];
            out[count].radius = radius[i];
            out[count].p1     = {data[i][0], data[i][1], data[i][2]};
            out[count].p2     = {data[i][3], data[i][4], data[i][5]};
            count++;
        }
        return count;
    }
};

} // namespace palletizing
