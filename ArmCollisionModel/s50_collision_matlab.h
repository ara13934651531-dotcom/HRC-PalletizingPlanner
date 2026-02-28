/**
 * @file s50_collision_matlab.h
 * @brief MATLAB loadlibrary 用 C 头文件 — libHRCInterface.so v1.0.0
 *        适配 HansRobot 碰撞检测与运动学公共 C API
 *
 * 单位约定:
 *   - 距离/位置:  mm (碰撞几何、环境障碍物、工具偏移)
 *   - 关节角度:    deg
 *   - 碰撞距离:    mm
 *   - FK 输出:     位置 m, 姿态 deg
 *   - IK 输入:     位置 m, 姿态 deg
 *   - getUIInfoMation: 位置 mm, 半径 mm
 *
 * @version 1.0.0 (适配 libHRCInterface v1.0.0)
 * @date 2026-02-27
 * Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.
 */

#ifndef S50_COLLISION_MATLAB_H
#define S50_COLLISION_MATLAB_H

/* ═══════════════════════════════════════════════════════════════════════
 *  类型定义 (匹配 hansTypeDef.h v1.0.0)
 * ═══════════════════════════════════════════════════════════════════════ */
typedef double          SO_LREAL;       /* RTS_IEC_LREAL = double */
typedef signed char     SO_BOOL;        /* RTS_IEC_BOOL = int8_t  */
typedef int             SO_INT;         /* RTS_IEC_INT = int (v1.0.0: was short) */
typedef long long       SO_LINT;        /* RTS_IEC_LINT = int64_t */
typedef int             SO_DINT;        /* RTS_IEC_DINT = int32_t */

/* TCP坐标结构 */
typedef struct {
    SO_LREAL X, Y, Z, A, B, C;
} MC_COORD_REF;

/* ═══════════════════════════════════════════════════════════════════════
 *  运动学初始化 (v1.0.0 新增)
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * @brief 初始化机器人类型
 * @param robotType 0=Elfin, 1=S系列(S50)
 */
extern void initilizeRobotType(SO_INT robotType);

/**
 * @brief 设置运动学参数
 * @param kinParams DH参数 [10]: S系列={d1,d2,d3,d4,d5,d6,a2,a3,0,0} (mm)
 * @return 成功/失败
 */
extern SO_INT setKinParams(SO_LREAL kinParams[10]);

/**
 * @brief 设置关节限位
 * @param upper 上限 [6] (deg)
 * @param lower 下限 [6] (deg)
 * @return 成功/失败
 */
extern SO_INT setJointSpceLimits(SO_LREAL upper[6], SO_LREAL lower[6]);

/**
 * @brief 设置工具坐标系
 * @param tool 工具参数 [6] (mm, deg)
 * @return 成功/失败
 */
extern SO_INT setToolCoordinateSystem(SO_LREAL tool[6]);

/* ═══════════════════════════════════════════════════════════════════════
 *  碰撞检测功能包
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * @brief 初始化碰撞检测算法包
 * @param robType   0=Elfin, 1=S系列
 * @param dh        DH参数 [d1..d6, a2, a3] (mm)
 * @param baseGeo   基座碰撞几何: 胶囊体 [sx,sy,sz, ex,ey,ez, r] (mm)
 * @param lowerArmGeo 下臂碰撞几何 (mm)
 * @param elbowGeo  肘部碰撞几何 (mm)
 * @param upperArmGeo 上臂碰撞几何 (mm)
 * @param wristGeo  腕部碰撞几何: 球体 [cx,cy,cz, r] (mm)
 * @param initJoint 初始关节角 [6] (deg)
 */
extern void initACAreaConstrainPackageInterface(
    SO_INT    robType,
    SO_LREAL  dh[8],
    SO_LREAL  baseGeo[7],
    SO_LREAL  lowerArmGeo[7],
    SO_LREAL  elbowGeo[7],
    SO_LREAL  upperArmGeo[7],
    SO_LREAL  wristGeo[4],
    SO_LREAL  initJoint[6]);

/**
 * @brief 更新机器人状态 (每个控制周期调用)
 * @param jointDeg  关节角 [6] (deg)
 * @param velDps    关节速度 [6] (deg/s)
 * @param accDpss   关节加速度 [6] (deg/s^2)
 */
extern void updateACAreaConstrainPackageInterface(
    SO_LREAL  jointDeg[6],
    SO_LREAL  velDps[6],
    SO_LREAL  accDpss[6]);

/**
 * @brief 自碰撞检测查询
 * @param colliderPair  输出: 最近碰撞对 [linkId1, linkId2]
 * @param distance      输出: 最小自碰撞距离 (mm), 碰撞时=0
 * @return 非0=碰撞, 0=安全
 */
extern SO_INT checkCPSelfCollisionInterface(
    SO_LINT   colliderPair[2],
    SO_LREAL  *distance);

/**
 * @brief 启用/禁用碰撞检测链接模型
 * @param flags [3]: [0]=elbow, [1]=wrist, [2]=tool; 1=启用
 */
extern void setCPSelfColliderLinkModelOpenStateInterface(
    SO_BOOL   flags[3]);

/* ═══════════════════════════════════════════════════════════════════════
 *  正逆运动学
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * @brief 正运动学计算 TCP 位姿
 * @param jointDeg  关节角 [6] (deg)
 * @param tcp       输出: TCP位姿 {X,Y,Z,A,B,C} (位置: m, 姿态: deg)
 * @return 成功/失败
 * @note  位置返回米(m)!
 */
extern SO_INT forwardKinematics2(
    SO_LREAL  jointDeg[6],
    MC_COORD_REF *tcp);

/**
 * @brief 逆运动学计算
 * @param targetTcp  目标TCP位姿 {X,Y,Z,A,B,C} (位置: m, 姿态: deg)
 * @param refJoint   参考关节角 [6] (deg)
 * @param resultJoint 输出关节角 [6] (deg)
 * @return 成功/失败
 * @note 位置单位为米(m)
 */
extern SO_INT inverseKinematics(
    MC_COORD_REF targetTcp,
    SO_LREAL  refJoint[6],
    SO_LREAL  resultJoint[6]);

/* ═══════════════════════════════════════════════════════════════════════
 *  TCP 位置管理
 * ═══════════════════════════════════════════════════════════════════════ */

extern void initTCPPositionInterface(MC_COORD_REF p);
extern void updateTCPPositionInterface(MC_COORD_REF p);
extern void setACToolCoordinateInterface(MC_COORD_REF ee2Tool);

/* ═══════════════════════════════════════════════════════════════════════
 *  工具碰撞模型
 * ═══════════════════════════════════════════════════════════════════════ */

extern SO_INT setCPToolCollisionBallShapeInterface(
    SO_LINT   toolIndex,
    SO_LREAL  offset[3],
    SO_LREAL  radius);

extern SO_INT setCPToolCollisonCapsuleShapeInterface(
    SO_LINT   toolIndex,
    SO_LREAL  startPt[3],
    SO_LREAL  endPt[3],
    SO_LREAL  radius);

extern SO_INT removeCPToolCollisonInterface(SO_LINT toolIndex);

/* ═══════════════════════════════════════════════════════════════════════
 *  环境障碍物接口
 * ═══════════════════════════════════════════════════════════════════════ */

extern SO_INT addEnvObstacleBallInterface(
    SO_LINT   envId,
    SO_LREAL  center[3],
    SO_LREAL  radius);

extern SO_INT addEnvObstacleCapsuleInterface(
    SO_LINT   envId,
    SO_LREAL  startPt[3],
    SO_LREAL  endPt[3],
    SO_LREAL  radius);

extern SO_INT removeEnvObstacleInterface(SO_LINT envId);

extern void setLinkEnvCollisionEnabledInterface(SO_BOOL flags[7]);

extern SO_INT getEnvObstacleCountInterface(void);

extern SO_INT isEnvObstacleRegisteredInterface(SO_LINT envId);

/* ═══════════════════════════════════════════════════════════════════════
 *  碰撞可视化信息
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * @brief 获取碰撞模型可视化数据 (世界坐标)
 * @param collideIndex 碰撞模型索引 [7] (Base=1..Tool2=7)
 * @param collideType  碰撞模型类型 [7] (1=Ball,2=Capsule,3=Lozenge)
 * @param dataList     碰撞模型数据 [7×9 flattened] (mm)
 * @param radiusList   碰撞模型半径 [7] (mm)
 */
extern void getUIInfoMationInterface(
    SO_DINT   collideIndex[7],
    SO_DINT   collideType[7],
    SO_LREAL  dataList[63],
    SO_LREAL  radiusList[7]);

/* ═══════════════════════════════════════════════════════════════════════
 *  安全平面 / 区域限制
 * ═══════════════════════════════════════════════════════════════════════ */

extern SO_INT addACHalfPlaneAreaInterface(MC_COORD_REF pose, SO_INT id);

extern SO_INT addACOrientedBoundingBoxAreaDefindLWHInterface(
    MC_COORD_REF pose, SO_LREAL lwh[3], SO_INT id);

extern SO_INT deleteACAreaInterface(SO_INT id);

/* ═══════════════════════════════════════════════════════════════════════
 *  区域状态查询
 * ═══════════════════════════════════════════════════════════════════════ */

extern void getACAreaOpenStateInterface(SO_INT indexList[13]);

extern SO_INT getACRelativeMotionInfoInterface(
    SO_INT id, SO_LREAL *measure, SO_INT *trendency);

extern SO_INT getACTCPInAreaStatusInterface(SO_INT id, SO_BOOL *checkResult);

extern SO_INT getACTCPInAreaListInterface(SO_INT indexList[13]);

/* ═══════════════════════════════════════════════════════════════════════
 *  急停预测
 * ═══════════════════════════════════════════════════════════════════════ */

extern void setStopPredictionTypeInterface(SO_INT stopType);

extern double printCollisionPairInterface(void);

/* ═══════════════════════════════════════════════════════════════════════
 *  坐标变换
 * ═══════════════════════════════════════════════════════════════════════ */

extern MC_COORD_REF base2UserCoordTrans(MC_COORD_REF base, MC_COORD_REF userCoord);
extern MC_COORD_REF baseTCP2BaseCoordTrans(MC_COORD_REF baseTCP, MC_COORD_REF TCP);

/* ═══════════════════════════════════════════════════════════════════════
 *  笛卡尔速度计算
 * ═══════════════════════════════════════════════════════════════════════ */

extern void calculateCartesianVelAndAccFromJoint(
    SO_LREAL  jointPos[6],
    SO_LREAL  jointVel[6],
    SO_LREAL  jointAcc[6],
    MC_COORD_REF *cartVel,
    MC_COORD_REF *cartAcc);

#endif /* S50_COLLISION_MATLAB_H */
