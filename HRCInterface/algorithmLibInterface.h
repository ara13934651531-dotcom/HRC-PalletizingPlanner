#pragma once

/**
 * @file algorithmLibInterface.h
 * @brief HansRobot 碰撞检测与运动学公共 C API
 *
 * 本头文件声明了 libHRCInterface.so 导出的全部 57 个函数接口，
 * 覆盖坐标变换、正逆解、安全平面、区域限制、自碰撞检测、
 * 环境碰撞检测、急停预测和 TCP 管理 8 大功能模块。
 *
 * 支持型号: Elfin3 / Elfin5 / Elfin10 / Elfin15 /
 *          S05 / S10 / S20 / S30 / S50
 *
 * 单位约定: 长度 mm, 角度 deg (除特别标注 rad 的参数)
 *
 * 线程安全: 同一时刻仅允许单线程调用（非线程安全）
 *
 * @version 1.0.0
 * @date    2026-02-26
 */

/* ── 库版本信息 ── */
#define HRC_VERSION_MAJOR  1
#define HRC_VERSION_MINOR  0
#define HRC_VERSION_PATCH  0
#define HRC_VERSION_STRING "1.0.0"

#ifdef _WIN32
#define EXPORTED_SYMBOL __declspec(dllexport)
#else
#define EXPORTED_SYMBOL __attribute__((visibility("default")))
#endif


#include "hansTypeDef.h"

#ifdef __cplusplus
extern "C" {
#endif


EXPORTED_SYMBOL MC_COORD_REF translateXYZtoZYZ
(
	MC_COORD_REF input
);

EXPORTED_SYMBOL MC_COORD_REF translateZYZtoXYZ
(
	MC_COORD_REF input
);

EXPORTED_SYMBOL MC_COORD_REF base2UserCoordTrans
(
	MC_COORD_REF base,
	MC_COORD_REF userCoord
);

EXPORTED_SYMBOL MC_COORD_REF baseTCP2BaseCoordTrans
(
	MC_COORD_REF baseTCP,
	MC_COORD_REF TCP
);

EXPORTED_SYMBOL MC_COORD_REF baseVel2UserCoordTrans
(
	MC_COORD_REF baseVel,
	MC_COORD_REF userCoord
);

EXPORTED_SYMBOL MC_COORD_REF user2BaseCoordTrans
(
	MC_COORD_REF user,
	MC_COORD_REF userCoord
);


//   ***********************    正逆解   **************************************************
// 初始化机器人类型
EXPORTED_SYMBOL void initilizeRobotType(RTS_IEC_INT robotType);

// 设置杆件长度参数
EXPORTED_SYMBOL RTS_IEC_BOOL setKinParams(RTS_IEC_LREAL* kinParams);

EXPORTED_SYMBOL void setAbsoluteCalibParams
(
	RTS_IEC_LREAL scaleParam,			/* VAR_INPUT */	/* reducer scale param; */
	RTS_IEC_LREAL reducerParams[6][7],	/* VAR_INPUT */	/* reducer params; */
	RTS_IEC_LREAL jointCalib[5][6]		/* VAR_INPUT */	/* jointCalibParams; */
);

// 设置关节边界；
EXPORTED_SYMBOL RTS_IEC_BOOL setJointSpceLimits(
	RTS_IEC_LREAL* upper,
	RTS_IEC_LREAL* lower);


EXPORTED_SYMBOL RTS_IEC_BOOL setToolCoordinateSystem(RTS_IEC_LREAL* tool);

EXPORTED_SYMBOL void updateFlexibleCompensateTorqueHRC(RTS_IEC_LREAL* torque);

// 正解；
EXPORTED_SYMBOL RTS_IEC_BOOL forwardKinematics2(RTS_IEC_LREAL* jointPosition, MC_COORD_REF* endFrame);


//逆解；
EXPORTED_SYMBOL RTS_IEC_BOOL inverseKinematics(MC_COORD_REF endFrame, RTS_IEC_LREAL* jointRefPos, RTS_IEC_LREAL* jointPos);

// 计算笛卡尔速度
EXPORTED_SYMBOL void calculateCartesianVelAndAccFromJoint(
	RTS_IEC_LREAL* jointposition,			/* VAR_INPUT */
	RTS_IEC_LREAL* jointVelocity,			/* VAR_INPUT */
	RTS_IEC_LREAL* jointAcceleration,			/* VAR_INPUT */
	MC_COORD_REF* cartVel,				/* VAR_OUTPUT */
	MC_COORD_REF* cartAcc				/* VAR_OUTPUT */);

/**
 * @brief 检查工具是否安全
 * @param pose 工具的位姿坐标
 * @return 如果工具在安全状态返回 TRUE，否则返回 FALSE
 */
extern RTS_IEC_BOOL isToolSafeInterface(MC_COORD_REF pose);
/**
 * @brief 设置线性轴的方向
 * @param direction //设置直线轴的运动方向，只能[1,0,0],[0,1,0],[0,0,1];
 */
EXPORTED_SYMBOL void setLinearAxisDirection(RTS_IEC_LREAL* direction);

////////////////////////////////////////////// 新版本安全平面相关接口 ///////////////////////////////////////////////

/**
 * @brief 功能包的初始化函数
 * @param robType 机器人的类型，枚举值 Elfin = 0, S Seirial = 1,  
 * @param dh 机器人的 Denavit-Hartenberg 参数。如果是 Elfin，dh = {d1,d4,d6,a2, 0, 0, 0, 0}, dh = {d1, d2, d3, d4. d5, d6, a2, a3}; 单位： mm
 * @param baseGeometry base 包络的几何坐标 ,形状为胶囊体 单位： mm
 * @param lowerArmGeometry lowerArm 包络的几何坐标 ,形状为胶囊体 单位： mm
 * @param elbowGeometry elbow 包络的几何坐标 ,形状为胶囊体 单位： mm
 *  @param upperArmGeometry upperArm 包络的几何坐标 ,形状为胶囊体 单位： mm
 *  @param wristGeometry wrist 包络的几何坐标 ,形状为球体 单位： mm
 * @param jointPos 机器人关节坐标 (q1, q2, q3, q4, q5, q6) 单位： deg
 * 
 * 构造球体的 ball 的数据结构 数据长度为 4：
 * 	数据 [0-2]: 球体中心相对球体参考系偏移向量 格式：{x, y, z}
 *  数据 [3]: 球体半径
 * 
 * 构造球体的 capsule 的数据结构 数据长度为 7：
 * 	数据 [0-2]: 胶囊体起始端点相对胶囊参考系在参考系下的坐标 格式：{x, y, z}
 *  数据 [3-5]: 胶囊体结束端点相对胶囊参考系在参考系下的坐标 格式：{x, y, z}
 *  数据 [6]: 胶囊体半径
 */

void initACAreaConstrainPackageInterface(
                              RTS_IEC_INT robType, 
                              RTS_IEC_LREAL dh[8], 
                              RTS_IEC_LREAL baseGeometry[7], 
                              RTS_IEC_LREAL lowerArmGeometry[7], 
                              RTS_IEC_LREAL elbowGeometry[7], 
                              RTS_IEC_LREAL upperArmGeometry[7], 
                              RTS_IEC_LREAL wristGeometry[4], 
                              RTS_IEC_LREAL jointPos[6]);

/**
 * @brief 每周期外部状态信息的更新函数
 * @param jointPositions 包含机器人更新后关节位置向量 。这个参数用于更新机器人碰撞模型 单位：mm, deg
 * @param jointVelocitys 包含机器人更新后关节速度向量 。这个参数暂时没有用上，以后可能用用于计算碰撞模型的碰撞预测 单位：mm, deg
 * @param position 这个数值用于计算区域限制功能，当前周期的 TCP 姿态信息 (x, y, z, Rx, Ry, Rz) 单位：mm, rad
 * @param velocity 这个参数用于计算TCP的碰撞预测功能，当前周期的 TCP 运动线速度 (x, y, z) 单位：mm/s^1
 */
void updateACAreaConstrainPackageInterface(RTS_IEC_LREAL jointPositions[6], 
                        RTS_IEC_LREAL jointVelocity[6], RTS_IEC_LREAL jointAccelerations[6]);



////////////////////////////////////////////// 设置类接口  ///////////////////////////////////////////////


/**
 * @brief 添加TCP
 * @param ee2Tool TCP坐标 (X, Y, Z, Rx, Ry, Rz)  单位：mm, deg
 * @return 
 */
EXPORTED_SYMBOL void setACToolCoordinateInterface(MC_COORD_REF ee2Tool);

/**
 * @brief 添加一个区域限制半平面
 * @param pose 区域限制半平面的参考坐标系 (Rx, Ry, Rz) 欧拉矩阵的相乘顺序为 RzRyRx 单位：mm, deg
 * @param id 区域限制半平面的 ID (许可范围为 0-7)
 * @return 新添加的半平面的 ID
 */
EXPORTED_SYMBOL RTS_IEC_INT addACHalfPlaneAreaInterface(MC_COORD_REF pose, RTS_IEC_INT id);

/**
 * @brief 使用长宽高添加一个有向包络框限制区域
 * @param pose 有向包络框的参考坐标系，原点在 OBB 的一个顶点上，长宽高三条边分别落在坐标系的正方向上 单位：mm, deg
 * @param lwh 有向包络框的长宽高信息 (长度 3，格式 长宽高)
 * @param id 有向包络框的 ID (许可范围为 8-12)
 * @return 新添加的有向包络框的 ID
 */
EXPORTED_SYMBOL RTS_IEC_INT addACOrientedBoundingBoxAreaDefindLWHInterface(MC_COORD_REF pose, RTS_IEC_LREAL lwh[3], RTS_IEC_INT id);


EXPORTED_SYMBOL RTS_IEC_INT addACOrientedBoundingBoxAreaDefindOffsetLWHInterface(MC_COORD_REF pose, RTS_IEC_LREAL offset[3], RTS_IEC_LREAL lwh[3], RTS_IEC_INT id);

/**
 * @brief 使用对角角点定义一个有向包络框限制区域
 * @param pose 有向包络框的参考坐标系，原点在 OBB 的一个顶点上，长宽高三条边分别落在坐标系 单位：mm, deg
 * @param p1 有向包络框的一个边界点，详情见手册示意图，格式为 (x, y, z)，基于坐标系 单位：mm
 * @param p2 有向包络框的另一个边界点，详情见手册示意图，格式为 (x, y, z)，基于坐标系 单位：mm
 * @param id 有向包络框的 ID (许可范围为 8-12)
 * @return 新添加的有向包络框的 ID
 */
EXPORTED_SYMBOL RTS_IEC_INT addACOrientedBoundingBoxAreaDefine2PInterface(MC_COORD_REF pose, RTS_IEC_LREAL p1[3], RTS_IEC_LREAL p2[3], RTS_IEC_INT id);



/** 
 * 查询碰撞接口，返回发生碰撞的连杆的id，是否发生自碰撞。
 * 
 * 
 * 数据格式如下图所示: 四列分别是 碰撞连杆id，碰撞连杆类型，碰撞体半径r, 和连杆数据
   Index   Type    Radius  Data
   -------------------------------
    1      2    0.08    [ 0.0000, 0.0000, 0.0200, 0.0000, 0.0000, 0.1800, 0.0000, 0.0000, 0.0000 ]
    2      2    0.0600  [ -0.0738, -0.1050, 0.4584, 0.0037, -0.1050, 0.1790, 0.0000, 0.0000, 0.0000 ]
    3      2    0.0600  [ -0.0712, -0.0000, 0.4488, -0.1858, -0.0000, 0.4133, 0.0000, 0.0000, 0.0000 ]
    4      2    0.0400  [ -0.3807, -0.0900, 0.3531, -0.2374, -0.0900, 0.3974, 0.0000, 0.0000, 0.0000 ]
    5      1    0.0900  [ -0.3554, -0.0000, 0.3099, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000 ]
    6      2    0.0500  [ -0.3017, -0.0000, 0.2180, -0.1375, -0.0000, 0.1750, 0.0000, 0.0000, 0.0000 ]
    7      2    0.0700  [ -0.1375, -0.0000, 0.1750, -0.1165, -0.0000, 0.0598, 0.0000, 0.0000, 0.0000 ]

 * @param collideIndex  该参数表示发生碰撞的连杆对的id, 长度为 7
    Base = 1,                // 基座
    LowerArm = 2,            // 下臂 
    Elbow = 3,               // 肘部 
    UpperArm = 4,            // 上臂
    Wrist = 5,               // 腕部 
    Tool1 = 6,               // 末端工具1 
    Tool2 = 7,               // 末端工具2 
 * @param collideType   该参数表述碰撞体类型, 长度为 7
    None = 0,                // 没有定义
    Ball = 1,                // 球碰撞模型
    Capsule = 2,             // 胶囊碰撞模型
    Lozenge = 3,             // 棱体碰撞模型
    Plane = 4                // 平面碰撞模型 
 * @param dataList      该参数表述碰撞体 UI 数据, 大小为长度为 7 行9列 [7][9]
    1. Ball 球碰撞模型
    data 数据的【0-2】元素表示在 base 坐标下的球心偏移，格式为 （x,y,z）单位 mm
    data 数据的【3-8】元素填充为 0
    --------
    2. Capsule 胶囊碰撞模型
    data 数据的【0-2】元素表示在 base 坐标下的一个胶囊端点，格式为 （x,y,z）单位 mm
    data 数据的【3-5】元素表示在 base 坐标下的另一个胶囊端点，格式为 （x,y,z）单位 mm
   --------
    3. Lozenge 棱体碰撞模型
    data 数据的【0-2】元素表示圆角框在base下的几何中心坐标，格式为偏移（x,y,z）单位 mm
    data 数据的【3-5】元素表示欧拉角（ZYX）（Rx,Ry,Rz）单位 deg
    data 数据的【6-8】元素表示圆角框的xyz方向对应的长宽高（x,y,z）单位 mm

 * @param radiusList     该参数表述碰撞体半径, 长度为 7，单位 mm
    
 */

EXPORTED_SYMBOL void getUIInfoMationInterface(
    RTS_IEC_DINT collideIndex_out[7], 
    RTS_IEC_DINT collideType_out[7], 
    RTS_IEC_LREAL dataList_out[7][9], 
    RTS_IEC_LREAL radiusList_out[7]);

/** 
 * 将工具的碰撞模型设置为球体。
 * 
 * @param toolIndex 修改的工具 index，当前最多支持两个工具，只能选择 1 或者 2；
 * @param offset 球体中心相对于工具 TCP 坐标系中的偏移（x,y,z）Unit:（mm）；
 * @param radius 球体的半径 Unit:（mm）；
 * @return  判断是否添加成功 
 */
EXPORTED_SYMBOL RTS_IEC_INT setCPToolCollisionBallShapeInterface(RTS_IEC_LINT toolIndex, RTS_IEC_LREAL offset[3], RTS_IEC_LREAL radius);

/** 
 * 将工具的碰撞模型设置为胶囊体。
 * 
 * @param toolIndex 修改的工具 index，当前最多支持两个工具，只能选择 1 或者 2；
 * @param startPoint 胶囊体相对于工具 TCP 坐标系的起始点坐标 （x,y,z）Unit:（mm）；
 * @param endPoint 胶囊体相对于工具 TCP 坐标系的终止点坐标 （x,y,z）Unit:（mm）；
 * @param radius 胶囊体的半径  Unit:（mm）；
 * @return  判断是否添加成功 
 */
EXPORTED_SYMBOL RTS_IEC_INT setCPToolCollisonCapsuleShapeInterface(RTS_IEC_LINT toolIndex, RTS_IEC_LREAL startPoint[3], RTS_IEC_LREAL endPoint[3], RTS_IEC_LREAL radius);


/** 
 * 将工具的模型移除。
 * @param toolIndex 修改的工具 index，当前最多支持两个工具，只能选择 1 或者 2；
 * @return  判断是否删除成功 
**/
EXPORTED_SYMBOL RTS_IEC_INT removeCPToolCollisonInterface(RTS_IEC_LINT toolIndex);


/** 
 * 查询碰撞接口，返回发生碰撞的连杆的id，是否发生自碰撞。
 * 
 * @param colliderPair 如果接口返回值是true，该参数表示发生碰撞的连杆对的id，如果返回 false ，表示所有执行检测碰撞对中距离最小的碰撞对的距离。格式 长度为2 的 EcU32Vector 。
    Base = 1,                // 基座
    LowerArm = 2,            // 下臂 
    Elbow = 3,               // 肘部 
    UpperArm = 4,            // 上臂
    Wrist = 5,               // 腕部 
    Tool1 = 6,               // 末端工具1 
    Tool2 = 7,               // 末端工具2
 * @param distance 在没有发生碰撞的情况下 distance 表示 当前待检测连杆的最小距离， 如果发生了碰撞 distance 等于 0  Unit:（mm）。
 * @return 如果检测到自碰撞则返回 true，否则返回 false。
 */
EXPORTED_SYMBOL RTS_IEC_INT checkCPSelfCollisionInterface(RTS_IEC_LINT* colliderPair, RTS_IEC_LREAL* distance);


/** 
 * 设置本体连杆和虚拟墙交互的打开状态。
 * @param input 长度为 3 的bool型向量， 分别对应肘部连杆，腕部连杆和肩部连杆。false 为关闭， true为打开
 * input[0] 是否打开肘部连杆和虚拟墙的碰撞检测
 * input[1] 是否打开腕部连杆和虚拟墙的碰撞检测
 * input[2] 是否打开工具连杆和虚拟墙的碰撞检测
 */
EXPORTED_SYMBOL void setCPSelfColliderLinkModelOpenStateInterface(RTS_IEC_BOOL* input);


/**
 * @brief 删除一个限制区域
 * @param id 要删除的限制区域的 ID
 * @return 是否成功删除限制区域
 */
EXPORTED_SYMBOL RTS_IEC_INT deleteACAreaInterface(RTS_IEC_INT id);


////////////////////////////////////////////// 查询类接口  ///////////////////////////////////////////////
/**
 * @brief 获取当前正在进行碰撞检测的区域 ID 列表
 * @param indexList 存储当前正在进行碰撞检测的区域 ID 列表 （长度13, 0-7 指代安全平面，8-12 指代有向包络）
 */
EXPORTED_SYMBOL void getACAreaOpenStateInterface(RTS_IEC_INT indexList[13]);

/**
 * @brief 查询区域的相对运动信息
 * @param id 区域的 ID
 * @param measure 返回的相对平面的度量，
 *        对于开区域（如平面），measure 表示 TCP 到平面的距离（平面内的话返回负值）。
 *        对于闭区域（如 OBB），measure 表示 TCP 到几何深度的极大值的绝对误差作为度量。
 * @param trendency 相对运动状态（MoveIn，MoveOut，RelativeStatic）
 * @return 是否成功获取相对运动信息
 */
EXPORTED_SYMBOL RTS_IEC_INT getACRelativeMotionInfoInterface(RTS_IEC_INT id, RTS_IEC_LREAL* measure, RTS_IEC_INT* trendency);

/**
 * @brief 查询当前 TCP 是否在某一区域内部
 * @param id 区域的 ID
 * @param checkResult 返回 TCP 是否在区域内部的结果
 * @return 是否成功执行查询
 */
EXPORTED_SYMBOL RTS_IEC_INT getACTCPInAreaStatusInterface(RTS_IEC_INT id, RTS_IEC_BOOL* checkResult);

/**
 * @brief 查询当前 TCP 是否在所有区域外部
 * @param indexList 存储当前 TCP 在内部的区域 ID 列表 （长度13, 0-7 指代安全平面，8-12 指代有向包络）
 * @return 是否成功执行查询
 */
EXPORTED_SYMBOL RTS_IEC_BOOL getACTCPInAreaListInterface(RTS_IEC_INT indexList[13]);

/**
 * @brief 设置虚拟墙开始产生阻尼力时的距离阈值
 * @param depth:	虚拟墙限制区域自动驱动阻尼带的距离阈值 。单位：毫米
 * @return 无返回值
 */
EXPORTED_SYMBOL RTS_IEC_BOOL setDepthThresholdForDampingAreaInterface(RTS_IEC_LREAL depth);

EXPORTED_SYMBOL RTS_IEC_BOOL getSafePlaneDampingFactorInterface(RTS_IEC_LREAL* dampFactor);

////////////////////////////////////////////// 急停相关接口  ///////////////////////////////////////////////
/**
 * @brief 设置停止预测类型，不同类型对应不同加速度约束和加加速度约束
 * @param stopType 停止预测类型，枚举值：
 * - Cat0 = 1, 使用 cat0 碰撞急停
 * - Cat1 = 2, 使用 cat1 碰撞急停
 * - Cat2 = 3, 使用 cat2 碰撞急停，版本预留类型，目前暂时没有做该功能
 * - none = 4, 不使用碰撞急停位置预测
 * @return 无返回值
 */
EXPORTED_SYMBOL void setStopPredictionTypeInterface(RTS_IEC_INT stopType);

/**
 * @brief 设置停止类型和减速，
 * @param stopType 停止类型，枚举值：
 * - Cat0 = 1, 使用 cat0 碰撞急, 默认 850 °/s^2  8500 °/s^3
 * - Cat1 = 2, 使用 cat1 碰撞急停, 默认 800 °/s^2  8000 °/s^3
 * - Cat2 = 3, 使用 cat2 碰撞急停，版本预留类型，目前暂时没有做该功能
 * - none = 4, 不使用碰撞急停位置预测
 * @param accel 关节最大加速度约束
 * @param jeck 关节最大加加速度约束
 */
EXPORTED_SYMBOL void setStopTypeAndDecelerationInterface(RTS_IEC_INT stopType, RTS_IEC_LREAL* accel, RTS_IEC_LREAL* jeck);

/**
 * @brief 计算关节刹车距离
 * @param jointPos 当前关节位置
 * @param jointVel 当前关节速度
 * @param jointAcc 当前关节加速度
 * @param stop_distance 在当前速度和加速度下刹车所需距离
 * @param stopPos 在当前速度和加速度下刹车停止后关节位置
 */
EXPORTED_SYMBOL void calJointDeaccelerationDistanceInterface(RTS_IEC_LREAL* jointPos, RTS_IEC_LREAL* jointVel, RTS_IEC_LREAL* jointAcc, RTS_IEC_LREAL* stop_distance, RTS_IEC_LREAL* stopPos);

EXPORTED_SYMBOL double printCollisionPairInterface();




////////////////////////////////////////////// TCP 更新类接口  ///////////////////////////////////////////////

/**
 * @brief 初始化 TCP 位置信息
 * @param p TCP 初始化姿态 (x, y, z, Rx, Ry, Rz) 单位：mm, deg
 */
EXPORTED_SYMBOL void initTCPPositionInterface(MC_COORD_REF p);

/**
 * @brief 更新 TCP 位置信息
 * @param p 当前周期的 TCP 姿态信息 (x, y, z, Rx, Ry, Rz) 单位：mm, deg
 */
EXPORTED_SYMBOL void updateTCPPositionInterface(MC_COORD_REF p);


//=============================================================================
// [env-collision] fix(R2.H2): 环境碰撞检测 CDS 接口导出声明
//=============================================================================

/** @brief 添加球形环境障碍物, envId [30,45], center(mm), radius(mm) */
EXPORTED_SYMBOL RTS_IEC_INT addEnvObstacleBallInterface(
    RTS_IEC_LINT envId, RTS_IEC_LREAL center[3], RTS_IEC_LREAL radius);

/** @brief 添加胶囊体环境障碍物, envId [30,45], startPt/endPt(mm), radius(mm) */
EXPORTED_SYMBOL RTS_IEC_INT addEnvObstacleCapsuleInterface(
    RTS_IEC_LINT envId, RTS_IEC_LREAL startPt[3],
    RTS_IEC_LREAL endPt[3], RTS_IEC_LREAL radius);

/** @brief 添加棱体环境障碍物, ref2local[6](deg,mm), offset[3](mm), xLen/yLen/zLen 尺寸(mm), radius(mm) */
EXPORTED_SYMBOL RTS_IEC_INT addEnvObstacleLozengeInterface(
    RTS_IEC_LINT envId, RTS_IEC_LREAL ref2local[6],
    RTS_IEC_LREAL offset[3], RTS_IEC_LREAL xLen,
    RTS_IEC_LREAL yLen, RTS_IEC_LREAL zLen, RTS_IEC_LREAL radius);

/** @brief 删除环境障碍物, envId [30,45] */
EXPORTED_SYMBOL RTS_IEC_INT removeEnvObstacleInterface(RTS_IEC_LINT envId);

/** @brief 更新环境障碍物位姿, pose[6]=[rx,ry,rz(deg),tx,ty,tz(mm)] */
EXPORTED_SYMBOL RTS_IEC_INT updateEnvObstaclePoseInterface(
    RTS_IEC_LINT envId, RTS_IEC_LREAL pose[6]);

/** @brief 设置连杆-环境碰撞开关, flags[7]={Base,LowerArm,Elbow,UpperArm,Wrist,Tool1,Tool2} */
EXPORTED_SYMBOL void setLinkEnvCollisionEnabledInterface(RTS_IEC_BOOL flags[7]);

/** @brief 获取已注册环境障碍物数量 */
EXPORTED_SYMBOL RTS_IEC_INT getEnvObstacleCountInterface();

/** @brief 查询环境障碍物是否已注册, envId [30,45] */
EXPORTED_SYMBOL RTS_IEC_INT isEnvObstacleRegisteredInterface(RTS_IEC_LINT envId);

#ifdef __cplusplus
}
#endif