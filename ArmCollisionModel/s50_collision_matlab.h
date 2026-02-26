/**
 * @file s50_collision_matlab.h
 * @brief Simplified C header for MATLAB loadlibrary — S50 collision interface
 *        从 libHRCInterface.so (HansAlgorithmExport) 导出的C接口函数
 *
 * 单位约定 (CDS层):
 *   - 距离/位置:  mm
 *   - 关节角度:    deg
 *   - 碰撞距离:    mm
 *   - TCP坐标:     mm + deg
 *
 * @date 2026-02-24
 * Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.
 */

#ifndef S50_COLLISION_MATLAB_H
#define S50_COLLISION_MATLAB_H

/* ═══════════════════════════════════════════════════════════════════════
 *  类型定义 (匹配 .so 内部类型)
 * ═══════════════════════════════════════════════════════════════════════ */
typedef double          SO_LREAL;       /* RTS_IEC_LREAL = double */
typedef signed char     SO_BOOL;        /* RTS_IEC_BOOL = int8  */
typedef short           SO_INT;         /* RTS_IEC_INT = int16  */
typedef long long       SO_LINT;        /* RTS_IEC_LINT = int64 */
typedef int             SO_DINT;        /* RTS_IEC_DINT = int32 */

/* TCP坐标结构 */
typedef struct {
    SO_LREAL X, Y, Z, A, B, C;         /* mm + deg */
} MC_COORD_REF;

/* ═══════════════════════════════════════════════════════════════════════
 *  核心碰撞检测接口
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * @brief 初始化碰撞检测算法包 (调用一次)
 * @param robType   机器人类型: 0=Elfin, 1=S-Serial(S50)
 * @param dh        DH参数 [d1..d6, a2, a3] (mm)
 * @param baseGeo   基座碰撞几何 — 胶囊体 [sx,sy,sz, ex,ey,ez, radius] (mm)
 * @param lowerArmGeo 下臂碰撞几何 (mm)
 * @param elbowGeo  肘部碰撞几何 (mm)
 * @param upperArmGeo 上臂碰撞几何 (mm)
 * @param wristGeo  腕部碰撞几何 — 球体 [cx,cy,cz, radius] (mm)
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
 * @param velDps    关节速度 [6] (deg/s), 可全为0
 * @param accDpss   关节加速度 [6] (deg/s^2), 可全为0
 */
extern void updateACAreaConstrainPackageInterface(
    SO_LREAL  jointDeg[6],
    SO_LREAL  velDps[6],
    SO_LREAL  accDpss[6]);

/**
 * @brief 自碰撞检测查询
 * @param colliderPair  输出: 最近碰撞对 [linkId1, linkId2]
 * @param distance      输出: 最小自碰撞距离 (mm), 碰撞时=0
 * @return 1=碰撞, 0=安全
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
 *  正运动学
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * @brief 正运动学计算 TCP 位姿
 * @param jointDeg  关节角 [6] (deg)
 * @param tcp       输出: TCP位姿 {X,Y,Z,A,B,C} (位置: m, 始态: deg)
 * @note  注意: 位置返回米(m)而非毫米(mm)! 如需mm请手动乘以1000
 */
extern void forwardKinematics2(
    SO_LREAL  jointDeg[6],
    MC_COORD_REF *tcp);

/* ═══════════════════════════════════════════════════════════════════════
 *  工具碰撞模型
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * @brief 设置工具碰撞球模型
 * @param toolIndex  工具索引 (6=Tool1, 7=Tool2)
 * @param offset     球体偏移 [ox,oy,oz] (mm, 工具坐标系)
 * @param radius     球体半径 (mm)
 * @return 0=成功
 */
extern SO_INT setCPToolCollisionBallShapeInterface(
    SO_DINT   toolIndex,
    SO_LREAL  offset[3],
    SO_LREAL  radius);

/**
 * @brief 设置工具碰撞胶囊体模型  
 * @param toolIndex  工具索引
 * @param startPt    起点 [sx,sy,sz] (mm)
 * @param endPt      终点 [ex,ey,ez] (mm)
 * @param radius     半径 (mm)
 * @return 0=成功
 */
extern SO_INT setCPToolCollisonCapsuleShapeInterface(
    SO_DINT   toolIndex,
    SO_LREAL  startPt[3],
    SO_LREAL  endPt[3],
    SO_LREAL  radius);

/**
 * @brief 移除工具碰撞模型
 * @param toolIndex 工具索引
 * @return 0=成功
 */
extern SO_INT removeCPToolCollisonInterface(SO_DINT toolIndex);

/* ═══════════════════════════════════════════════════════════════════════
 *  环境障碍物接口
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * @brief 添加球形环境障碍物
 * @param envId   环境障碍物ID (30-45)
 * @param center  球心 [x,y,z] (mm)
 * @param radius  半径 (mm)
 * @return 0=成功
 */
extern SO_INT addEnvObstacleBallInterface(
    SO_DINT   envId,
    SO_LREAL  center[3],
    SO_LREAL  radius);

/**
 * @brief 添加胶囊体环境障碍物
 * @param envId   环境障碍物ID (30-45)
 * @param startPt 起点 [sx,sy,sz] (mm)
 * @param endPt   终点 [ex,ey,ez] (mm)
 * @param radius  半径 (mm)
 * @return 0=成功
 */
extern SO_INT addEnvObstacleCapsuleInterface(
    SO_DINT   envId,
    SO_LREAL  startPt[3],
    SO_LREAL  endPt[3],
    SO_LREAL  radius);

/**
 * @brief 移除环境障碍物
 * @param envId 障碍物ID
 * @return 0=成功
 */
extern SO_INT removeEnvObstacleInterface(SO_DINT envId);

/**
 * @brief 设置各连杆环境碰撞检测开关
 * @param flags [7]: 每个连杆的启用状态
 */
extern void setLinkEnvCollisionEnabledInterface(SO_BOOL flags[7]);

/**
 * @brief 获取环境障碍物数量
 * @return 当前注册的环境障碍物数量
 */
extern SO_DINT getEnvObstacleCountInterface(void);

/* ═══════════════════════════════════════════════════════════════════════
 *  碰撞可视化信息
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * @brief 获取碰撞模型可视化数据 (世界坐标)
 * @param collideIndex 碰撞模型索引 [7]
 * @param collideType  碰撞模型类型 [7]  (1=Ball,2=Capsule,3=Lozenge)
 * @param dataList     碰撞模型数据 [7][9] (m — 注意这里是m不是mm!)
 * @param radiusList   碰撞模型半径 [7] (m)
 */
extern void getUIInfoMationInterface(
    SO_DINT   collideIndex[7],
    SO_DINT   collideType[7],
    SO_LREAL  dataList[63],       /* 7x9 flattened, in meters */
    SO_LREAL  radiusList[7]);     /* in meters */

/**
 * @brief 打印碰撞对调试信息
 */
extern void printCollisionPairInterface(void);

#endif /* S50_COLLISION_MATLAB_H */
