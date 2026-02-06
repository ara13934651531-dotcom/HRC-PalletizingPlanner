/**
 * @file CollisionChecker.hpp
 * @brief 碰撞检测模块 - 封装HRC碰撞检测库 (v2.0 完整碰撞检测)
 * 
 * 提供高效的碰撞检测接口，支持：
 * - 机器人自碰撞检测 (全连杆对: Base/LowerArm/Elbow/UpperArm/Wrist/Tool)
 * - 连杆与环境障碍物碰撞检测 (肘部/腕部/工具 vs 安全平面/OBB)
 * - TCP区域入侵检测 (安全平面/OBB内部判断)
 * - 碰撞检测缓存优化
 * 
 * v2.0 变更:
 *   - 启用 setCPSelfColliderLinkModelOpenStateInterface 开启连杆-虚拟墙碰撞
 *   - 整合 getACTCPInAreaListInterface 查询TCP区域入侵
 *   - 分离自碰撞距离与环境碰撞距离
 *   - 新增 CollisionReport 详细碰撞报告
 * 
 * @author GitHub Copilot
 * @version 2.0.0
 * @date 2026-02-05
 */

#pragma once

#include "Types.hpp"
#include "RobotModel.hpp"

// HRC碰撞检测库接口
extern "C" {
#include <InterfaceDataStruct.h>
#include <algorithmLibInterface.h>
}

#include <unordered_map>
#include <mutex>
#include <memory>
#include <string>
#include <sstream>

namespace palletizing {

// ============================================================================
// 碰撞连杆枚举 (与HRC库一致)
// ============================================================================

/// HRC连杆ID枚举
enum class LinkId : int {
    None     = 0,
    Base     = 1,   // 基座
    LowerArm = 2,   // 下臂
    Elbow    = 3,   // 肘部
    UpperArm = 4,   // 上臂
    Wrist    = 5,   // 腕部
    Tool1    = 6,   // 末端工具1
    Tool2    = 7    // 末端工具2
};

/// 连杆名称转换
inline const char* linkIdToName(int id) {
    switch (id) {
        case 1: return "Base";
        case 2: return "LowerArm";
        case 3: return "Elbow";
        case 4: return "UpperArm";
        case 5: return "Wrist";
        case 6: return "Tool1";
        case 7: return "Tool2";
        default: return "Unknown";
    }
}

// ============================================================================
// 碰撞检测报告
// ============================================================================

/**
 * @brief 详细碰撞检测报告
 * 
 * 包含自碰撞、环境碰撞、TCP区域入侵的完整信息
 */
struct CollisionReport {
    // === 总体结果 ===
    bool collisionFree = true;          // 总体是否无碰撞
    
    // === 自碰撞 ===
    bool selfCollision = false;         // 是否发生自碰撞
    double selfMinDistance = 1e10;      // 自碰撞最小距离 [m]
    int selfColliderPairA = 0;          // 碰撞连杆A的ID
    int selfColliderPairB = 0;          // 碰撞连杆B的ID
    
    // === 环境碰撞 (TCP区域入侵) ===
    bool envCollision = false;          // TCP是否侵入环境区域
    int envViolatedAreaCount = 0;       // 被侵入的区域数量
    int envViolatedAreas[13] = {0};     // 被侵入的区域ID列表
    
    // === 连杆-虚拟墙碰撞 ===
    bool linkWallCollision = false;     // 连杆是否与虚拟墙碰撞
    bool linkWallEnabled = false;       // 连杆-虚拟墙检测是否已启用
    
    // === 碰撞体几何信息 (来自getUIInfoMationInterface) ===
    struct LinkGeometry {
        int index;                      // 连杆ID
        int type;                       // 碰撞体类型 (1=Ball, 2=Capsule)
        double radius;                  // 碰撞体半径 [m]
        double data[9];                 // 几何数据 (base坐标系下) [m]
    };
    LinkGeometry linkGeometries[7];     // 7个连杆的碰撞体信息
    bool hasGeometryInfo = false;       // 是否包含几何信息
    
    // === 关节限位 ===
    bool jointLimitViolation = false;   // 是否超关节限位
    
    /// 生成可读的报告字符串
    std::string toString() const {
        std::ostringstream ss;
        ss << "=== 碰撞检测报告 ===\n";
        ss << "总体: " << (collisionFree ? "✅ 安全" : "❌ 碰撞") << "\n";
        
        if (jointLimitViolation) {
            ss << "  ⚠️ 关节限位超限\n";
        }
        
        ss << "自碰撞: " << (selfCollision ? "❌ " : "✅ ")
           << "最小距离=" << selfMinDistance << "m";
        if (selfCollision) {
            ss << " 碰撞对=[" << linkIdToName(selfColliderPairA) 
               << "-" << linkIdToName(selfColliderPairB) << "]";
        }
        ss << "\n";
        
        ss << "环境碰撞: " << (envCollision ? "❌ " : "✅ ");
        if (envCollision) {
            ss << "侵入区域数=" << envViolatedAreaCount << " IDs=[";
            for (int i = 0; i < envViolatedAreaCount; ++i) {
                if (i > 0) ss << ",";
                ss << envViolatedAreas[i];
            }
            ss << "]";
        }
        ss << "\n";
        
        if (linkWallEnabled) {
            ss << "连杆-虚拟墙: " << (linkWallCollision ? "❌ 碰撞" : "✅ 安全") << "\n";
        }
        
        return ss.str();
    }
};

/**
 * @brief OBB障碍物定义
 */
struct OBBObstacle {
    int id;                  // 障碍物ID (8-12)
    Pose6D pose;             // 位姿
    Eigen::Vector3d lwh;     // 长宽高 [m]
    std::string name;        // 可选名称
    
    OBBObstacle() : id(-1), lwh(Eigen::Vector3d::Zero()) {}
    
    OBBObstacle(int obsId, const Pose6D& p, const Eigen::Vector3d& dimensions, 
                const std::string& n = "")
        : id(obsId), pose(p), lwh(dimensions), name(n) {}
};

/**
 * @brief 安全平面定义
 */
struct SafePlane {
    int id;         // 平面ID (0-7)
    Pose6D pose;    // 位姿 (XY平面为安全平面，Z轴为正方向)
    std::string name;
    
    SafePlane() : id(-1) {}
    SafePlane(int planeId, const Pose6D& p, const std::string& n = "")
        : id(planeId), pose(p), name(n) {}
};

/**
 * @brief 场景配置 - 码垛环境描述
 */
struct SceneConfig {
    // 机器人安装平台
    double platformHeight = 0.0;    // 平台高度 [m]
    Pose6D robotBasePose;           // 机器人基座位姿
    
    // 集装箱区域 (蓝色框架)
    struct PalletZone {
        Pose6D pose;               // 区域位姿
        double length = 1.2;       // 长度 [m]
        double width = 1.0;        // 宽度 [m]
        double height = 1.8;       // 高度 [m]
        int layerCount = 5;        // 层数
        double layerHeight = 0.3;  // 每层高度 [m]
    };
    std::vector<PalletZone> palletZones;
    
    // 流水线
    struct ConveyorBelt {
        Pose6D pose;
        double length = 2.0;
        double width = 0.6;
        double height = 0.8;
    };
    std::optional<ConveyorBelt> conveyor;
    
    // 静态障碍物
    std::vector<OBBObstacle> obstacles;
    
    // 安全平面
    std::vector<SafePlane> safePlanes;
    
    // 工作区域限制
    double workspaceRadius = 2.5;  // 工作空间半径 [m]
    
    /// 创建默认码垛场景
    static SceneConfig defaultPalletizing() {
        SceneConfig scene;
        
        // Robot Platform Height 750mm
        scene.platformHeight = 0.75;
        scene.robotBasePose = Pose6D::fromEulerZYX(0, 0, 0.75, 0, 0, 0);
        
        // Pallet Zone (Inside Frame)
        // Frame starts X=1.1, Center X=2.0. Center Y=1.0.
        PalletZone pallet;
        pallet.pose = Pose6D::fromEulerZYX(2.0, 1.0, 0, 0, 0, 0);
        pallet.length = 1.2;
        pallet.width = 1.0;
        pallet.height = 2.0;
        scene.palletZones.push_back(pallet);
        
        // Conveyor (Front Right)
        // Center (0.3, -1.2). Height 0.8 (Solid block for safety)
        ConveyorBelt belt;
        belt.pose = Pose6D::fromEulerZYX(0.3, -1.2, 0.4, 0, 0, 0);
        belt.length = 3.0;
        belt.width = 0.6;
        belt.height = 0.8;
        scene.conveyor = belt;
        
        // Ground
        SafePlane ground;
        ground.id = 0;
        ground.pose = Pose6D::fromEulerZYX(0, 0, 0, 0, 0, 0);  // Z=0 Plane
        ground.name = "Ground";
        scene.safePlanes.push_back(ground);
        
        return scene;
    }
};

/**
 * @brief 碰撞检测器 - 封装HRC碰撞检测库 (v2.0 完整碰撞检测)
 * 
 * 支持三层碰撞检测:
 * 1. 连杆-连杆 自碰撞 (checkCPSelfCollisionInterface)
 * 2. 连杆-虚拟墙 碰撞 (setCPSelfColliderLinkModelOpenStateInterface)
 * 3. TCP-区域 入侵检测 (getACTCPInAreaListInterface)
 */
class CollisionChecker {
public:
    /**
     * @brief 构造函数
     * @param robot 机器人模型
     */
    explicit CollisionChecker(const RobotModel& robot)
        : robot_(robot), initialized_(false), linkWallEnabled_(false) {
    }
    
    /**
     * @brief 初始化碰撞检测系统
     * @param scene 场景配置
     * @param enableLinkWall 是否启用连杆-虚拟墙碰撞检测 (默认启用)
     * @return 是否初始化成功
     */
    bool initialize(const SceneConfig& scene = SceneConfig::defaultPalletizing(),
                    bool enableLinkWall = true) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        scene_ = scene;
        
        // 获取DH参数
        auto dhArray = robot_.getParams().toDHArray();
        RTS_IEC_LREAL dh[8];
        for (int i = 0; i < 8; ++i) {
            dh[i] = dhArray[i];
        }
        
        // S系列机器人 (type = 1)
        RTS_IEC_INT robType = 1;
        
        // 连杆碰撞几何 (基于S50_collision.json实际参数, mm)
        // 胶囊体格式: [start_x, start_y, start_z, end_x, end_y, end_z, radius]
        RTS_IEC_LREAL baseGeometry[7]     = {0, 0, 30,  0, 0, 336.2, 130};
        RTS_IEC_LREAL lowerArmGeometry[7] = {0, 0, 280, 900, 0, 280, 130};
        RTS_IEC_LREAL elbowGeometry[7]    = {-20, 0, 80, 941.5, 0, 80, 100};
        RTS_IEC_LREAL upperArmGeometry[7] = {0, 0, -60, 0, 0, 120, 60};
        
        // 腕部球体: [x, y, z, radius] (mm)
        RTS_IEC_LREAL wristGeometry[4] = {0, 0, 30, 120};
        
        // 初始关节位置 (deg)
        RTS_IEC_LREAL jointPos[6] = {0, 0, 0, 0, 0, 0};
        
        // 调用HRC库初始化
        initACAreaConstrainPackageInterface(
            robType, dh,
            baseGeometry, lowerArmGeometry, elbowGeometry,
            upperArmGeometry, wristGeometry, jointPos
        );
        
        // ★ v2.0: 启用连杆-虚拟墙碰撞检测
        if (enableLinkWall) {
            enableLinkWallCollision(true, true, true);
        }
        
        // 设置场景障碍物
        setupSceneObstacles(scene);
        
        initialized_ = true;
        return true;
    }
    
    /**
     * @brief 启用/禁用连杆与虚拟墙的碰撞检测
     * @param elbow 是否检测肘部连杆
     * @param wrist 是否检测腕部连杆
     * @param tool 是否检测工具连杆
     */
    void enableLinkWallCollision(bool elbow, bool wrist, bool tool) {
        RTS_IEC_BOOL input[3] = {
            static_cast<RTS_IEC_BOOL>(elbow),
            static_cast<RTS_IEC_BOOL>(wrist),
            static_cast<RTS_IEC_BOOL>(tool)
        };
        setCPSelfColliderLinkModelOpenStateInterface(input);
        linkWallEnabled_ = (elbow || wrist || tool);
    }
    
    /**
     * @brief 检查单个配置是否无碰撞 (v2.0: 三层检测)
     * @param config 关节配置
     * @return true如果无碰撞
     */
    bool isCollisionFree(const JointConfig& config) const {
        if (!initialized_) return false;
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        // 首先检查关节限位
        if (!robot_.isWithinLimits(config)) {
            return false;
        }
        
        // 更新机器人配置
        RTS_IEC_LREAL jointPos[6], jointVel[6] = {0};
        for (int i = 0; i < 6; ++i) {
            jointPos[i] = config.q[i] * 180.0 / M_PI;  // rad → deg
        }
        
        updateACAreaConstrainPackageInterface(jointPos, jointVel);
        
        // ★ 第1层: 自碰撞检测 (所有连杆对)
        RTS_IEC_LINT colliderPair[2];
        RTS_IEC_LREAL selfDistance;
        RTS_BOOL selfCollision = checkCPSelfCollisionInterface(colliderPair, &selfDistance);
        
        if (selfCollision) {
            return false;
        }
        
        // 自碰撞安全余量检查
        if (selfDistance < safetyMargin_) {
            return false;
        }
        
        // ★ 第2层: TCP区域入侵检测
        RTS_IEC_INT areaList[13] = {0};
        RTS_IEC_BOOL tcpAllOutside = getACTCPInAreaListInterface(areaList);
        
        if (!tcpAllOutside) {
            // TCP侵入了某个区域
            return false;
        }
        
        return true;
    }
    
    /**
     * @brief 获取完整碰撞检测报告 (v2.0)
     * @param config 关节配置
     * @param includeGeometry 是否包含碰撞体几何信息
     * @return 详细碰撞报告
     */
    CollisionReport getCollisionReport(const JointConfig& config,
                                       bool includeGeometry = false) const {
        CollisionReport report;
        
        if (!initialized_) {
            report.collisionFree = false;
            return report;
        }
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        // 关节限位检查
        if (!robot_.isWithinLimits(config)) {
            report.collisionFree = false;
            report.jointLimitViolation = true;
            return report;
        }
        
        // 更新机器人配置
        RTS_IEC_LREAL jointPos[6], jointVel[6] = {0};
        for (int i = 0; i < 6; ++i) {
            jointPos[i] = config.q[i] * 180.0 / M_PI;
        }
        updateACAreaConstrainPackageInterface(jointPos, jointVel);
        
        // ★ 第1层: 自碰撞检测
        RTS_IEC_LINT colliderPair[2] = {0, 0};
        RTS_IEC_LREAL selfDistance = 0;
        RTS_BOOL selfColl = checkCPSelfCollisionInterface(colliderPair, &selfDistance);
        
        report.selfCollision = (selfColl != 0);
        report.selfMinDistance = selfColl ? 0.0 : selfDistance;
        report.selfColliderPairA = static_cast<int>(colliderPair[0]);
        report.selfColliderPairB = static_cast<int>(colliderPair[1]);
        
        // ★ 第2层: TCP区域入侵检测
        RTS_IEC_INT areaList[13] = {0};
        RTS_IEC_BOOL tcpAllOutside = getACTCPInAreaListInterface(areaList);
        
        if (!tcpAllOutside) {
            report.envCollision = true;
            int count = 0;
            for (int i = 0; i < 13; ++i) {
                if (areaList[i] != 0) {
                    report.envViolatedAreas[count++] = i;
                }
            }
            report.envViolatedAreaCount = count;
        }
        
        // ★ 连杆-虚拟墙状态
        report.linkWallEnabled = linkWallEnabled_;
        // 注: 连杆-虚拟墙碰撞结果通过checkCPSelfCollisionInterface的返回值体现
        // 当连杆-虚拟墙开启时, selfCollision返回true也可能是连杆碰虚拟墙
        
        // ★ 碰撞体几何信息 (可用于可视化)
        if (includeGeometry) {
            RTS_IEC_DINT collideIndex[7];
            RTS_IEC_DINT collideType[7];
            RTS_IEC_LREAL dataList[7][9];
            RTS_IEC_LREAL radiusList[7];
            
            getUIInfoMationInterface(collideIndex, collideType, dataList, radiusList);
            
            for (int i = 0; i < 7; ++i) {
                report.linkGeometries[i].index = static_cast<int>(collideIndex[i]);
                report.linkGeometries[i].type = static_cast<int>(collideType[i]);
                report.linkGeometries[i].radius = radiusList[i];
                for (int j = 0; j < 9; ++j) {
                    report.linkGeometries[i].data[j] = dataList[i][j];
                }
            }
            report.hasGeometryInfo = true;
        }
        
        // 总体判断
        report.collisionFree = !report.selfCollision && !report.envCollision 
                               && (report.selfMinDistance >= safetyMargin_);
        
        return report;
    }
    
    /**
     * @brief 检查路径段是否无碰撞
     * @param start 起始配置
     * @param end 目标配置
     * @param resolution 检测分辨率 [rad]
     * @return true如果整段路径无碰撞
     */
    bool isPathCollisionFree(const JointConfig& start, const JointConfig& end,
                             double resolution = 0.02) const {
        double dist = start.distanceTo(end);
        int numChecks = static_cast<int>(std::ceil(dist / resolution)) + 1;
        numChecks = std::max(numChecks, 2);
        
        for (int i = 0; i <= numChecks; ++i) {
            double t = static_cast<double>(i) / numChecks;
            JointConfig interpolated = robot_.interpolate(start, end, t);
            
            if (!isCollisionFree(interpolated)) {
                return false;
            }
        }
        
        return true;
    }
    
    /**
     * @brief 获取配置的碰撞距离 (v2.0: 返回自碰撞距离)
     * @param config 关节配置
     * @return 最小自碰撞距离 [m]，0表示碰撞
     */
    double getCollisionDistance(const JointConfig& config) const {
        if (!initialized_) return -1.0;
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        // 更新机器人配置
        RTS_IEC_LREAL jointPos[6], jointVel[6] = {0};
        for (int i = 0; i < 6; ++i) {
            jointPos[i] = config.q[i] * 180.0 / M_PI;
        }
        
        updateACAreaConstrainPackageInterface(jointPos, jointVel);
        
        // 获取自碰撞距离
        RTS_IEC_LINT colliderPair[2];
        RTS_IEC_LREAL distance;
        RTS_BOOL collision = checkCPSelfCollisionInterface(colliderPair, &distance);
        
        return collision ? 0.0 : distance;
    }
    
    /**
     * @brief 查询TCP到指定区域的距离和运动趋势
     * @param areaId 区域ID (0-7: 安全平面, 8-12: OBB)
     * @param[out] measure 距离度量 [m] (平面: 到面距离; OBB: 到几何深度度量)
     * @param[out] tendency 运动趋势 (MoveIn/MoveOut/Static)
     * @return 是否查询成功
     */
    bool getAreaDistance(int areaId, double& measure, int& tendency) const {
        if (!initialized_) return false;
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        RTS_IEC_LREAL meas;
        RTS_IEC_INT trend;
        RTS_IEC_INT result = getACRelativeMotionInfoInterface(
            static_cast<RTS_IEC_INT>(areaId), &meas, &trend);
        
        if (result >= 0) {
            measure = meas;
            tendency = static_cast<int>(trend);
            return true;
        }
        return false;
    }
    
    /**
     * @brief 查询TCP是否在指定区域内部
     * @param areaId 区域ID
     * @return true如果TCP在区域内部
     */
    bool isTCPInArea(int areaId) const {
        if (!initialized_) return false;
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        RTS_IEC_BOOL inArea;
        RTS_IEC_INT result = getACTCPInAreaStatusInterface(
            static_cast<RTS_IEC_INT>(areaId), &inArea);
        
        return (result >= 0) && inArea;
    }
    
    /**
     * @brief 获取当前碰撞体的世界坐标几何信息 (用于可视化)
     * @param[out] geometries 7个连杆的碰撞体几何信息数组
     */
    void getCollisionGeometries(CollisionReport::LinkGeometry geometries[7]) const {
        if (!initialized_) return;
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        RTS_IEC_DINT collideIndex[7];
        RTS_IEC_DINT collideType[7];
        RTS_IEC_LREAL dataList[7][9];
        RTS_IEC_LREAL radiusList[7];
        
        getUIInfoMationInterface(collideIndex, collideType, dataList, radiusList);
        
        for (int i = 0; i < 7; ++i) {
            geometries[i].index = static_cast<int>(collideIndex[i]);
            geometries[i].type = static_cast<int>(collideType[i]);
            geometries[i].radius = radiusList[i];
            for (int j = 0; j < 9; ++j) {
                geometries[i].data[j] = dataList[i][j];
            }
        }
    }
    
    /**
     * @brief 添加OBB障碍物
     * @param obstacle OBB障碍物
     * @return 是否添加成功
     */
    bool addObstacle(const OBBObstacle& obstacle) {
        if (!initialized_) return false;
        if (obstacle.id < 8 || obstacle.id > 12) return false;
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        MC_COORD_REF pose;
        pose.X = obstacle.pose.position.x() * 1000.0;  // m -> mm
        pose.Y = obstacle.pose.position.y() * 1000.0;
        pose.Z = obstacle.pose.position.z() * 1000.0;
        
        // 从四元数提取欧拉角
        Eigen::Vector3d euler = obstacle.pose.orientation.toRotationMatrix().eulerAngles(2, 1, 0);
        pose.A = euler[2] * 180.0 / M_PI;  // rx
        pose.B = euler[1] * 180.0 / M_PI;  // ry
        pose.C = euler[0] * 180.0 / M_PI;  // rz
        
        RTS_IEC_LREAL lwh[3] = {
            obstacle.lwh.x() * 1000.0,  // m -> mm
            obstacle.lwh.y() * 1000.0,
            obstacle.lwh.z() * 1000.0
        };
        
        RTS_IEC_INT result = addACOrientedBoundingBoxAreaDefindLWHInterface(
            pose, lwh, obstacle.id
        );
        
        if (result >= 0) {
            obstacles_[obstacle.id] = obstacle;
            return true;
        }
        
        return false;
    }
    
    /**
     * @brief 添加安全平面
     * @param plane 安全平面
     * @return 是否添加成功
     */
    bool addSafePlane(const SafePlane& plane) {
        if (!initialized_) return false;
        if (plane.id < 0 || plane.id > 7) return false;
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        MC_COORD_REF pose;
        pose.X = plane.pose.position.x() * 1000.0;
        pose.Y = plane.pose.position.y() * 1000.0;
        pose.Z = plane.pose.position.z() * 1000.0;
        
        Eigen::Vector3d euler = plane.pose.orientation.toRotationMatrix().eulerAngles(2, 1, 0);
        pose.A = euler[2] * 180.0 / M_PI;
        pose.B = euler[1] * 180.0 / M_PI;
        pose.C = euler[0] * 180.0 / M_PI;
        
        RTS_IEC_INT result = addACHalfPlaneAreaInterface(pose, plane.id);
        
        if (result >= 0) {
            safePlanes_[plane.id] = plane;
            return true;
        }
        
        return false;
    }
    
    /**
     * @brief 移除障碍物
     * @param id 障碍物ID
     * @return 是否移除成功
     */
    bool removeObstacle(int id) {
        if (!initialized_) return false;
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        RTS_IEC_INT result = deleteACAreaInterface(id);
        
        if (result >= 0) {
            obstacles_.erase(id);
            return true;
        }
        
        return false;
    }
    
    /**
     * @brief 获取当前活跃的碰撞检测区域列表
     * @param[out] areaIds 活跃区域ID列表 (长度13, 0-7安全平面, 8-12 OBB)
     */
    void getActiveAreas(int areaIds[13]) const {
        if (!initialized_) return;
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        RTS_IEC_INT list[13];
        getACAreaOpenStateInterface(list);
        for (int i = 0; i < 13; ++i) {
            areaIds[i] = static_cast<int>(list[i]);
        }
    }
    
    /**
     * @brief 设置安全余量
     * @param margin 安全余量 [m]
     */
    void setSafetyMargin(double margin) {
        safetyMargin_ = margin;
    }
    
    /**
     * @brief 设置工具碰撞模型 (球体)
     */
    bool setToolCollisionBall(const Eigen::Vector3d& offset, double radius) {
        RTS_IEC_LREAL off[3] = {offset.x(), offset.y(), offset.z()};  // 单位: m
        return setCPToolCollisionBallShapeInterface(1, off, radius);
    }
    
    /**
     * @brief 设置工具碰撞模型 (胶囊体)
     */
    bool setToolCollisionCapsule(const Eigen::Vector3d& start, 
                                  const Eigen::Vector3d& end, 
                                  double radius) {
        RTS_IEC_LREAL sp[3] = {start.x(), start.y(), start.z()};
        RTS_IEC_LREAL ep[3] = {end.x(), end.y(), end.z()};
        return setCPToolCollisonCapsuleShapeInterface(1, sp, ep, radius);
    }
    
    /**
     * @brief 移除工具碰撞模型
     */
    bool removeToolCollision(int toolIndex = 1) {
        return removeCPToolCollisonInterface(static_cast<RTS_IEC_LINT>(toolIndex));
    }
    
    // 访问器
    bool isInitialized() const { return initialized_; }
    const SceneConfig& getScene() const { return scene_; }
    bool isLinkWallEnabled() const { return linkWallEnabled_; }
    
private:
    /**
     * @brief 设置场景中的所有障碍物
     */
    void setupSceneObstacles(const SceneConfig& scene) {
        int obbId = 8;
        
        // 添加集装箱区域作为OBB
        for (const auto& pallet : scene.palletZones) {
            if (obbId > 12) break;
            
            OBBObstacle obs;
            obs.id = obbId++;
            obs.pose = pallet.pose;
            obs.lwh = Eigen::Vector3d(pallet.length, pallet.width, pallet.height);
            obs.name = "PalletZone";
            
            addObstacle(obs);
        }
        
        // 添加流水线
        if (scene.conveyor && obbId <= 12) {
            OBBObstacle belt;
            belt.id = obbId++;
            belt.pose = scene.conveyor->pose;
            belt.lwh = Eigen::Vector3d(
                scene.conveyor->length,
                scene.conveyor->width,
                scene.conveyor->height
            );
            belt.name = "Conveyor";
            
            addObstacle(belt);
        }
        
        // 添加安全平面
        for (const auto& plane : scene.safePlanes) {
            addSafePlane(plane);
        }
    }
    
    const RobotModel& robot_;
    SceneConfig scene_;
    
    std::unordered_map<int, OBBObstacle> obstacles_;
    std::unordered_map<int, SafePlane> safePlanes_;
    
    double safetyMargin_ = 0.01;  // 默认1cm安全余量
    bool initialized_;
    bool linkWallEnabled_;        // v2.0: 连杆-虚拟墙检测状态
    
    mutable std::mutex mutex_;
};

} // namespace palletizing
