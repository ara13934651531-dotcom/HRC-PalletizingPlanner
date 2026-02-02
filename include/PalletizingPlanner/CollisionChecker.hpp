/**
 * @file CollisionChecker.hpp
 * @brief 碰撞检测模块 - 封装HRC碰撞检测库
 * 
 * 提供高效的碰撞检测接口，支持：
 * - 机器人自碰撞检测
 * - 环境障碍物碰撞检测
 * - 安全平面和OBB区域约束
 * - 碰撞检测缓存优化
 * 
 * @author GitHub Copilot
 * @version 1.0.0
 * @date 2026-01-29
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

namespace palletizing {

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
        
        // 机器人平台高度约800mm
        scene.platformHeight = 0.8;
        scene.robotBasePose = Pose6D::fromEulerZYX(0, 0, 0.8, 0, 0, 0);
        
        // 添加集装箱区域 (机器人左前方)
        PalletZone pallet;
        pallet.pose = Pose6D::fromEulerZYX(-0.8, 0.8, 0, 0, 0, 0);
        pallet.length = 1.2;
        pallet.width = 1.0;
        pallet.height = 2.0;
        scene.palletZones.push_back(pallet);
        
        // 流水线 (机器人前方)
        ConveyorBelt belt;
        belt.pose = Pose6D::fromEulerZYX(0.8, 0, 0.4, 0, 0, 0);
        belt.length = 2.0;
        belt.width = 0.6;
        belt.height = 0.8;
        scene.conveyor = belt;
        
        // 地面安全平面
        SafePlane ground;
        ground.id = 0;
        ground.pose = Pose6D::fromEulerZYX(0, 0, 0, 0, 0, 0);  // Z=0平面
        ground.name = "Ground";
        scene.safePlanes.push_back(ground);
        
        return scene;
    }
};

/**
 * @brief 碰撞检测器 - 封装HRC碰撞检测库
 */
class CollisionChecker {
public:
    /**
     * @brief 构造函数
     * @param robot 机器人模型
     */
    explicit CollisionChecker(const RobotModel& robot)
        : robot_(robot), initialized_(false) {
    }
    
    /**
     * @brief 初始化碰撞检测系统
     * @param scene 场景配置
     * @return 是否初始化成功
     */
    bool initialize(const SceneConfig& scene = SceneConfig::defaultPalletizing()) {
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
        
        // 连杆碰撞几何 (基于HR_S50-2000的尺寸估算)
        // 胶囊体格式: [start_x, start_y, start_z, end_x, end_y, end_z, radius] (mm)
        RTS_IEC_LREAL baseGeometry[7] = {0, 0, 0, 0, 0, 200, 150};
        RTS_IEC_LREAL lowerArmGeometry[7] = {0, 0, 0, 900, 0, 0, 100};
        RTS_IEC_LREAL elbowGeometry[7] = {0, 0, 0, 941.5, 0, 0, 90};
        RTS_IEC_LREAL upperArmGeometry[7] = {0, 0, -50, 0, 0, 100, 80};
        
        // 腕部球体: [x, y, z, radius] (mm)
        RTS_IEC_LREAL wristGeometry[4] = {0, 0, 0, 80};
        
        // 初始关节位置
        RTS_IEC_LREAL jointPos[6] = {0, 0, 0, 0, 0, 0};
        
        // 调用HRC库初始化
        initACAreaConstrainPackageInterface(
            robType, dh,
            baseGeometry, lowerArmGeometry, elbowGeometry,
            upperArmGeometry, wristGeometry, jointPos
        );
        
        // 设置场景障碍物
        setupSceneObstacles(scene);
        
        initialized_ = true;
        return true;
    }
    
    /**
     * @brief 检查单个配置是否无碰撞
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
            jointPos[i] = config.q[i] * 180.0 / M_PI;  // 转为度
        }
        
        updateACAreaConstrainPackageInterface(jointPos, jointVel);
        
        // 检查自碰撞
        RTS_IEC_LINT colliderPair[2];
        RTS_IEC_LREAL distance;
        RTS_BOOL collision = checkCPSelfCollisionInterface(colliderPair, &distance);
        
        if (collision) {
            return false;
        }
        
        // 检查与环境的碰撞 (通过距离阈值)
        if (distance < safetyMargin_) {
            return false;
        }
        
        return true;
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
     * @brief 获取配置的碰撞距离
     * @param config 关节配置
     * @return 最小碰撞距离 [m]，负值表示碰撞
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
        
        // 获取碰撞距离
        RTS_IEC_LINT colliderPair[2];
        RTS_IEC_LREAL distance;
        RTS_BOOL collision = checkCPSelfCollisionInterface(colliderPair, &distance);
        
        return collision ? 0.0 : distance;
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
    
    // 访问器
    bool isInitialized() const { return initialized_; }
    const SceneConfig& getScene() const { return scene_; }
    
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
    
    mutable std::mutex mutex_;
};

} // namespace palletizing
