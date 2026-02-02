#include "InterfaceDataStruct.h"

#ifndef COMPLIME_CPP_INTERFACE
#ifdef __cplusplus
extern "C" {
#endif
#endif


//   ********************************* 老版本安全平面相关接口   ***********************************************
/**
 * @brief 初始化机器人工具位置
 * @param robotPosition 机器人的工具位置坐标
 * @return 无返回值
 */
extern void initRobotToolPositionInterface(MC_COORD_REF robotPosition);

/**
 * @brief 获取TCP末端与安全平面的距离和趋势
 * @param robotPosition 机器人人的 TCP 位姿
 * @param id 查询的安全平面 ID
 * @param distance 返回 TCP 末端与安全平面的相对距离，安全平面的 Z 轴方向定义为安全平面的正方向
 * @param trendency 返回 TCP 相对平面的运动趋势
 * @return 操作成功返回整数值，失败返回错误代码
 */
extern RTS_IEC_INT getDistanceInterface(
    MC_COORD_REF robotPosition,  // 机器人人的 TCP 位姿
    RTS_IEC_INT id,              // 查询的安全平面 ID
    RTS_IEC_LREAL* distance,     // TCP 末端与安全平面的相对距离
    RTS_IEC_INT* trendency       // TCP 相对平面的运动趋势
);

/**
 * @brief 更新机器人位置
 * @param robotPosition 更新后的机器人工具位置坐标
 * @return 无返回值
 */
extern void updateRobotPositionInterface(MC_COORD_REF robotPosition);

/**
 * @brief 获取平面状态
 * @param openList 返回的平面状态列表，指示哪些平面是打开的
 * @return 无返回值
 */
extern void getPlaneStatesInterface(RTS_IEC_BOOL* openList);

/**
 * @brief 移除指定的安全平面
 * @param id 要移除的安全平面 ID
 * @return 操作成功返回整数值，失败返回错误代码
 */
extern RTS_IEC_INT removeSafePlaneInterface(RTS_IEC_INT id);

/**
 * @brief 设置安全平面
 * @param pose 安全平面的参考坐标系的姿态，坐标系的 XY 平面为安全平面，Z 轴是安全平面的正方向。格式为 (x, y, z, Rx, Ry, Rz) 欧拉矩阵的相乘顺序为 RzRyRx
 * @param id 安全平面 ID
 * @return 操作成功返回整数值，失败返回错误代码
 */
extern RTS_IEC_INT setSafePlaneInterface(
    MC_COORD_REF pose,  // 安全平面的参考坐标系的姿态
    RTS_IEC_INT id      // 安全平面 ID
);

/**
 * @brief 检查工具是否安全
 * @param pose 工具的位姿坐标
 * @return 如果工具在安全状态返回 TRUE，否则返回 FALSE
 */
extern RTS_IEC_BOOL isToolSafeInterface(MC_COORD_REF pose);



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
                        RTS_IEC_LREAL jointVelocity[6]);



////////////////////////////////////////////// 设置类接口  ///////////////////////////////////////////////


/**
 * @brief 添加TCP
 * @param ee2Tool TCP坐标 (X, Y, Z, Rx, Ry, Rz)  单位：mm, deg
 * @return 
 */
extern void setACToolCoordinateInterface(MC_COORD_REF ee2Tool);

/**
 * @brief 添加一个区域限制半平面
 * @param pose 区域限制半平面的参考坐标系 (Rx, Ry, Rz) 欧拉矩阵的相乘顺序为 RzRyRx 单位：mm, deg
 * @param id 区域限制半平面的 ID (许可范围为 0-7)
 * @return 新添加的半平面的 ID
 */
extern RTS_IEC_INT addACHalfPlaneAreaInterface(MC_COORD_REF pose, RTS_IEC_INT id);

/**
 * @brief 使用长宽高添加一个有向包络框限制区域
 * @param pose 有向包络框的参考坐标系，原点在 OBB 的一个顶点上，长宽高三条边分别落在坐标系的正方向上 单位：mm, deg
 * @param lwh 有向包络框的长宽高信息 (长度 3，格式 长宽高)
 * @param id 有向包络框的 ID (许可范围为 8-12)
 * @return 新添加的有向包络框的 ID
 */
extern RTS_IEC_INT addACOrientedBoundingBoxAreaDefindLWHInterface(MC_COORD_REF pose, RTS_IEC_LREAL lwh[3], RTS_IEC_INT id);


extern RTS_IEC_INT addACOrientedBoundingBoxAreaDefindOffsetLWHInterface(MC_COORD_REF pose, RTS_IEC_LREAL offset[3], RTS_IEC_LREAL lwh[3], RTS_IEC_INT id);

/**
 * @brief 使用对角角点定义一个有向包络框限制区域
 * @param pose 有向包络框的参考坐标系，原点在 OBB 的一个顶点上，长宽高三条边分别落在坐标系 单位：mm, deg
 * @param p1 有向包络框的一个边界点，详情见手册示意图，格式为 (x, y, z)，基于坐标系 单位：mm
 * @param p2 有向包络框的另一个边界点，详情见手册示意图，格式为 (x, y, z)，基于坐标系 单位：mm
 * @param id 有向包络框的 ID (许可范围为 8-12)
 * @return 新添加的有向包络框的 ID
 */
extern RTS_IEC_INT addACOrientedBoundingBoxAreaDefine2PInterface(MC_COORD_REF pose, RTS_IEC_LREAL p1[3], RTS_IEC_LREAL p2[3], RTS_IEC_INT id);



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
    data 数据的【0-2】元素表示在 base 坐标下的球心偏移，格式为 （x,y,z）单位 m
    data 数据的【3-8】元素填充为 0
    --------
    2. Capsule 胶囊碰撞模型
    data 数据的【0-2】元素表示在 base 坐标下的一个胶囊端点，格式为 （x,y,z）单位 m
    data 数据的【3-5】元素表示在 base 坐标下的另一个胶囊端点，格式为 （x,y,z）单位 m
   --------
    2. Capsule 胶囊碰撞模型
    data 数据的【0-6】元素表示圆角框在base下的几何中心坐标，格式为偏移+欧拉角（ZYX）（x,y,z,Rx,Ry,Rz）单位 m/rad
    data 数据的【7-8】元素表示圆角框的xyz方向对应的长宽高（x,y,z）单位 m/rad

 * @param radiusList     该参数表述碰撞体半径, 长度为 7
    
 */

extern void getUIInfoMationInterface(
    RTS_IEC_DINT collideIndex_out[7], 
    RTS_IEC_DINT collideType_out[7], 
    RTS_IEC_LREAL dataList_out[7][9], 
    RTS_IEC_LREAL radiusList_out[7]);

/** 
 * 将工具的碰撞模型设置为球体。
 * 
 * @param toolIndex 修改的工具 index，当前最多支持两个工具，只能选择 1 或者 2；
 * @param offset 球体中心相对于工具 TCP 坐标系中的偏移（x,y,z）Unit:（m）；
 * @param radius 球体的半径 Unit:（m）；
 * @return  判断是否添加成功 
 */
extern RTS_BOOL setCPToolCollisionBallShapeInterface(RTS_IEC_LINT toolIndex, RTS_IEC_LREAL offset[3], RTS_IEC_LREAL radius);

/** 
 * 将工具的碰撞模型设置为胶囊体。
 * 
 * @param toolIndex 修改的工具 index，当前最多支持两个工具，只能选择 1 或者 2；
 * @param startPoint 胶囊体相对于工具 TCP 坐标系的起始点坐标 （x,y,z）Unit:（m）；
 * @param endPoint 胶囊体相对于工具 TCP 坐标系的终止点坐标 （x,y,z）Unit:（m）；
 * @param radius 胶囊体的半径  Unit:（m）；
 * @return  判断是否添加成功 
 */
extern RTS_BOOL setCPToolCollisonCapsuleShapeInterface(RTS_IEC_LINT toolIndex, RTS_IEC_LREAL startPoint[3], RTS_IEC_LREAL endPoint[3], RTS_IEC_LREAL radius);


/** 
 * 将工具的模型移除。
 * @param toolIndex 修改的工具 index，当前最多支持两个工具，只能选择 1 或者 2；
 * @return  判断是否删除成功 
**/
extern RTS_BOOL removeCPToolCollisonInterface(RTS_IEC_LINT toolIndex);


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
 * @param distance 在没有发生碰撞的情况下 distance 表示 当前待检测连杆的最小距离， 如果发生了碰撞 distance 等于 0  Unit:（m）。
 * @return 如果检测到自碰撞则返回 true，否则返回 false。
 */
extern RTS_BOOL checkCPSelfCollisionInterface(RTS_IEC_LINT* colliderPair, RTS_IEC_LREAL* distance);


/** 
 * 设置本体连杆和虚拟墙交互的打开状态。
 * @param input 长度为 3 的bool型向量， 分别对应肘部连杆，腕部连杆和肩部连杆。false 为关闭， true为打开
 * input[0] 是否打开肘部连杆和虚拟墙的碰撞检测
 * input[1] 是否打开腕部连杆和虚拟墙的碰撞检测
 * input[2] 是否打开工具连杆和虚拟墙的碰撞检测
 */
extern void setCPSelfColliderLinkModelOpenStateInterface(RTS_IEC_BOOL* input);


/**
 * @brief 删除一个限制区域
 * @param id 要删除的限制区域的 ID
 * @return 是否成功删除限制区域
 */
extern RTS_IEC_INT deleteACAreaInterface(RTS_IEC_INT id);


////////////////////////////////////////////// 查询类接口  ///////////////////////////////////////////////
/**
 * @brief 获取当前正在进行碰撞检测的区域 ID 列表
 * @param indexList 存储当前正在进行碰撞检测的区域 ID 列表 （长度13, 0-7 指代安全平面，8-12 指代有向包络）
 */
extern void getACAreaOpenStateInterface(RTS_IEC_INT indexList[13]);

/**
 * @brief 查询区域的相对运动信息
 * @param id 区域的 ID
 * @param measure 返回的相对平面的度量，
 *        对于开区域（如平面），measure 表示 TCP 到平面的距离（平面内的话返回负值）。
 *        对于闭区域（如 OBB），measure 表示 TCP 到几何深度的极大值的绝对误差作为度量。
 * @param trendency 相对运动状态（MoveIn，MoveOut，RelativeStatic）
 * @return 是否成功获取相对运动信息
 */
extern RTS_IEC_INT getACRelativeMotionInfoInterface(RTS_IEC_INT id, RTS_IEC_LREAL* measure, RTS_IEC_INT* trendency);

/**
 * @brief 查询当前 TCP 是否在某一区域内部
 * @param id 区域的 ID
 * @param checkResult 返回 TCP 是否在区域内部的结果
 * @return 是否成功执行查询
 */
extern RTS_IEC_INT getACTCPInAreaStatusInterface(RTS_IEC_INT id, RTS_IEC_BOOL* checkResult);

/**
 * @brief 查询当前 TCP 是否在所有区域外部
 * @param indexList 存储当前 TCP 在内部的区域 ID 列表 （长度13, 0-7 指代安全平面，8-12 指代有向包络）
 * @return 是否成功执行查询
 */
extern RTS_IEC_BOOL getACTCPInAreaListInterface(RTS_IEC_INT indexList[13]);


/**
 * @brief 设置停止预测类型
 * @param stopType 停止预测类型，枚举值：
 * - Cat0 = 1, 使用 cat0 碰撞急停
 * - Cat1 = 2, 使用 cat1 碰撞急停
 * - Cat2 = 3, 使用 cat2 碰撞急停，版本预留类型，目前暂时没有做该功能
 * - none = 4, 不使用碰撞急停位置预测
 * @return 无返回值
 */
extern void setStopPredictionTypeInterface(RTS_IEC_INT stopType);

/**
 * @brief 设置停止类型和减速
 * @param stopType 停止类型，枚举值：
 * - Cat0 = 1, 使用 cat0 碰撞急, 默认 200 rad/s^2
 * - Cat1 = 2, 使用 cat1 碰撞急停, 默认 60 rad/s^2
 * - Cat2 = 3, 使用 cat2 碰撞急停，版本预留类型，目前暂时没有做该功能
 * - none = 4, 不使用碰撞急停位置预测

 */
extern void setStopTypeAndDecelerationInterface(RTS_IEC_INT stopType, RTS_IEC_LREAL* accel);


/**
 * @brief 设置虚拟墙开始产生阻尼力时的距离阈值
 * @param depth:	虚拟墙限制区域自动驱动阻尼带的距离阈值 。单位：毫米
 * @return 无返回值
 */
extern RTS_IEC_BOOL setDepthThresholdForDampingAreaInterface(RTS_IEC_LREAL depth);

extern RTS_IEC_BOOL getSafePlaneDampingFactorInterface(RTS_IEC_LREAL* dampFactor);

extern double printCollisionPairInterface();
#ifndef COMPLIME_CPP_INTERFACE
#ifdef __cplusplus
}
#endif
#endif