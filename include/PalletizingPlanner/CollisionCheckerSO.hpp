/**
 * @file CollisionCheckerSO.hpp
 * @brief 基于 libHRCInterface.so (HansAlgorithmExport) 的碰撞检测器
 *
 * 动态加载独立开发的碰撞检测 .so 模块，替代旧的静态库方案。
 * 优势:
 *   - 单一 .so 文件，无需 libCmpAgu.a / libhansKinematics.a
 *   - 支持环境障碍物碰撞检测 (球/胶囊/棱体)
 *   - 更新的 API: updateAC 支持加速度参数
 *   - 内置正/逆运动学 (可用于TCP位姿验证)
 *
 * 单位约定 (所有 .so 接口):
 *   - 碰撞几何/距离: mm
 *   - 关节角度: deg (接口层自动 rad→deg 转换)
 *   - 碰撞距离输出: mm
 *   - FK输出位置: m (米, 非mm!), 姿态: deg
 *   - IK输入位置: m (米, 非mm!), 姿态: deg
 *
 * @author Guangdong Huayan Robotics Co., Ltd.
 * @version 3.0.0
 * @date 2026-02-23
 */

#pragma once

#include "Types.hpp"
#include "RobotModel.hpp"
#include "CollisionGeometry.hpp"

#include <dlfcn.h>
#include <unistd.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>
#include <sstream>
#include <mutex>
#include <unordered_map>
#include <chrono>

namespace palletizing {

// ============================================================================
// 类型定义 (与 hansTypeDef.h 一致)
// ============================================================================
typedef int8_t   SO_BOOL;
typedef int      SO_INT;
typedef int64_t  SO_LINT;
typedef double   SO_LREAL;
typedef int32_t  SO_DINT;

struct SO_COORD_REF {
    SO_LREAL X, Y, Z, A, B, C;
};

// ============================================================================
// 性能计时器
// ============================================================================

/// 分层计时统计
struct TimingStats {
    double initTime_ms      = 0;   // 初始化耗时
    double updateTime_us    = 0;   // 平均update调用耗时 (微秒)
    double selfCheckTime_us = 0;   // 平均自碰撞检测耗时 (微秒)
    double envCheckTime_us  = 0;   // 平均环境检测耗时 (微秒)
    double fkTime_us        = 0;   // 累计正运动学耗时 (微秒)
    double ikTime_us        = 0;   // 累计逆运动学耗时 (微秒)
    double totalCheckTime_us = 0;  // 累计单次完整检测耗时 (微秒)
    int    callCount        = 0;   // 碰撞检测调用次数
    int    fkCallCount      = 0;   // FK调用次数 (独立计数)
    int    ikCallCount      = 0;   // IK调用次数 (独立计数)
    int    selfCollCount    = 0;   // 自碰撞次数
    int    envCollCount     = 0;   // 环境碰撞次数
    int    cacheHits        = 0;   // 缓存命中次数
    
    void reset() { *this = TimingStats{}; }
    
    std::string toString() const {
        char buf[1024];
        snprintf(buf, sizeof(buf),
            "  ┌ 碰撞检测性能分析 (%d 次调用) ─────────────────────\n"
            "  │ 初始化:      %.2f ms\n"
            "  │ update:      %.2f μs/次  (状态更新)\n"
            "  │ 自碰撞检测:  %.2f μs/次  (selfCollision)\n"
            "  │ 环境检测:    %.2f μs/次  (envCollision)\n"
            "  │ 正运动学:    %.2f μs/次  (FK, %d次调用)\n"
            "  │ 逆运动学:    %.2f μs/次  (IK, %d次调用)\n"
            "  │ 单次合计:    %.2f μs/次  (total)\n"
            "  │ 缓存命中率:  %.1f%%  (%d/%d)\n"
            "  │ 碰撞统计:    自碰撞=%d  环境=%d\n"
            "  └─────────────────────────────────────────────────\n",
            callCount, initTime_ms,
            updateTime_us, selfCheckTime_us, envCheckTime_us,
            fkTime_us, fkCallCount, ikTime_us, ikCallCount,
            totalCheckTime_us,
            callCount > 0 ? 100.0 * cacheHits / callCount : 0.0,
            cacheHits, callCount,
            selfCollCount, envCollCount);
        return std::string(buf);
    }
};

// ============================================================================
// 碰撞检测报告 (v3.0)
// ============================================================================
struct CollisionReportSO {
    bool collisionFree   = true;
    
    // 自碰撞
    bool selfCollision   = false;
    double selfMinDist_mm = 1e10;   // 单位 mm
    int selfPairA = 0, selfPairB = 0;
    
    // 环境碰撞
    bool envCollision    = false;
    int  envViolatedCount = 0;
    int  envViolatedAreas[13] = {};
    
    // TCP 位姿 (由 .so 内置FK计算)
    SO_COORD_REF tcpPose = {};     // 位置: m, 姿态: deg (注意: .so FK返回米而非mm)
    bool hasTcpPose = false;
    
    /// 总体是否安全
    bool isSafe(double marginMm = 10.0) const {
        return !selfCollision && !envCollision && selfMinDist_mm > marginMm;
    }
};

// ============================================================================
// 函数指针类型
// ============================================================================
using FnInitRobotType = void(*)(SO_INT);
using FnSetKinParams  = SO_BOOL(*)(SO_LREAL*);
using FnSetJointLimits = SO_BOOL(*)(SO_LREAL*, SO_LREAL*);
using FnInitAC = void(*)(SO_INT, SO_LREAL[8], SO_LREAL[7], SO_LREAL[7],
                          SO_LREAL[7], SO_LREAL[7], SO_LREAL[4], SO_LREAL[6]);
using FnUpdateAC = void(*)(SO_LREAL[6], SO_LREAL[6], SO_LREAL[6]);
using FnCheckSelfColl = SO_INT(*)(SO_LINT*, SO_LREAL*);
using FnSetColliderOpen = void(*)(SO_BOOL*);
using FnGetUIInfo = void(*)(SO_DINT[7], SO_DINT[7], SO_LREAL[7][9], SO_LREAL[7]);
using FnGetAreaList = SO_BOOL(*)(SO_INT[13]);
using FnGetAreaOpen = void(*)(SO_INT[13]);
using FnAddOBB_LWH = SO_INT(*)(SO_COORD_REF, SO_LREAL[3], SO_INT);
using FnAddHalfPlane = SO_INT(*)(SO_COORD_REF, SO_INT);
using FnDeleteArea = SO_INT(*)(SO_INT);
using FnGetRelMotion = SO_INT(*)(SO_INT, SO_LREAL*, SO_INT*);
using FnGetTcpInArea = SO_INT(*)(SO_INT, SO_BOOL*);
using FnForwardKin = SO_BOOL(*)(SO_LREAL*, SO_COORD_REF*);
using FnInverseKin = SO_BOOL(*)(SO_COORD_REF, SO_LREAL*, SO_LREAL*);
using FnSetToolBall = SO_INT(*)(SO_LINT, SO_LREAL[3], SO_LREAL);
using FnSetToolCapsule = SO_INT(*)(SO_LINT, SO_LREAL[3], SO_LREAL[3], SO_LREAL);
using FnRemoveTool = SO_INT(*)(SO_LINT);
using FnPrintCollPair = double(*)();
using FnAddEnvBall = SO_INT(*)(SO_LINT, SO_LREAL[3], SO_LREAL);
using FnAddEnvCapsule = SO_INT(*)(SO_LINT, SO_LREAL[3], SO_LREAL[3], SO_LREAL);
using FnAddEnvLozenge = SO_INT(*)(SO_LINT, SO_LREAL[6], SO_LREAL[3],
                                   SO_LREAL, SO_LREAL, SO_LREAL, SO_LREAL);
using FnRemoveEnv = SO_INT(*)(SO_LINT);
using FnSetLinkEnvFlags = void(*)(SO_BOOL[7]);
using FnGetEnvCount = SO_INT(*)();
using FnInitTCPPos = void(*)(SO_COORD_REF);
using FnUpdateTCPPos = void(*)(SO_COORD_REF);
using FnSetStopType = void(*)(SO_INT, SO_LREAL*, SO_LREAL*);
using FnCalcCartVelAcc = void(*)(SO_LREAL*, SO_LREAL*, SO_LREAL*,
                                  SO_COORD_REF*, SO_COORD_REF*);

// ============================================================================
// CollisionCheckerSO — 新一代碰撞检测器
// ============================================================================

class CollisionCheckerSO {
public:
    explicit CollisionCheckerSO(const RobotModel& robot)
        : robot_(robot), handle_(nullptr), initialized_(false) {}
    
    ~CollisionCheckerSO() {
        if (handle_) {
            dlclose(handle_);
            handle_ = nullptr;
        }
    }
    
    CollisionCheckerSO(const CollisionCheckerSO&) = delete;
    CollisionCheckerSO& operator=(const CollisionCheckerSO&) = delete;
    
    /**
     * @brief 初始化: 加载.so 并配置S50碰撞模型
     * @param soPath .so文件路径
     * @param enableLinkWall 启用连杆-虚拟墙碰撞
     * @return 成功/失败
     */
    bool initialize(const std::string& soPath = "",
            bool enableLinkWall = true) {
        
        auto t0 = std::chrono::high_resolution_clock::now();
        std::lock_guard<std::mutex> lock(mutex_);
        
        // SO 库路径搜索链: 显式参数 → 环境变量 → 相对路径 → 系统库路径
        std::string resolvedPath = soPath;
        if (resolvedPath.empty()) {
            if (const char* env = std::getenv("HRC_LIB_PATH"))
                resolvedPath = env;
            else if (access("../lib/libHRCInterface.so", F_OK) == 0)
                resolvedPath = "../lib/libHRCInterface.so";
            else if (access("lib/libHRCInterface.so", F_OK) == 0)
                resolvedPath = "lib/libHRCInterface.so";
            else
                resolvedPath = "libHRCInterface.so";  // 依赖 LD_LIBRARY_PATH
        }
        
        // 加载 .so
        handle_ = dlopen(resolvedPath.c_str(), RTLD_LAZY);
        if (!handle_) {
            fprintf(stderr, "CollisionCheckerSO: dlopen失败: %s\n"
                    "  搜索路径: %s\n"
                    "  提示: 设置环境变量 HRC_LIB_PATH 或将 .so 文件放到 lib/ 目录\n",
                    dlerror(), resolvedPath.c_str());
            return false;
        }
        
        // 加载所有函数指针
        if (!loadSymbols()) {
            dlclose(handle_);
            handle_ = nullptr;
            return false;
        }
        
        // S50 DH参数 + 统一碰撞几何 (从 CollisionGeometry.hpp 引用)
        SO_LREAL dh[8];
        for (int i = 0; i < 8; i++) dh[i] = S50CollisionGeometry::dhParams[i];
        
        SO_LREAL baseGeo[7], lowerArmGeo[7], elbowGeo[7], upperArmGeo[7];
        for (int i = 0; i < 7; i++) {
            baseGeo[i]     = S50CollisionGeometry::baseCapsule[i];
            lowerArmGeo[i] = S50CollisionGeometry::lowerArmCapsule[i];
            elbowGeo[i]    = S50CollisionGeometry::elbowCapsule[i];
            upperArmGeo[i] = S50CollisionGeometry::upperArmCapsule[i];
        }
        SO_LREAL wristGeo[4];
        for (int i = 0; i < 4; i++) wristGeo[i] = S50CollisionGeometry::wristBall[i];
        
        // 初始关节 (deg)
        SO_LREAL initJoint[6] = {0, 0, 0, 0, 0, 0};
        
        // 调用初始化
        initAC_(1, dh, baseGeo, lowerArmGeo, elbowGeo, upperArmGeo, wristGeo, initJoint);
        
        // 启用连杆-虚拟墙
        if (enableLinkWall) {
            SO_BOOL flags[3] = {1, 1, 1};
            setColliderOpen_(flags);
            linkWallEnabled_ = true;
        }
        
        initialized_ = true;
        
        auto t1 = std::chrono::high_resolution_clock::now();
        timing_.initTime_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

         auto toMmDisplay = [](double v) {
             return std::abs(v) < 10.0 ? v * 1000.0 : v;
         };
         double dhDisp[8];
         for (int i = 0; i < 8; ++i) dhDisp[i] = toMmDisplay(dh[i]);
         double baseR  = toMmDisplay(baseGeo[6]);
         double lArmR  = toMmDisplay(lowerArmGeo[6]);
         double elbowR = toMmDisplay(elbowGeo[6]);
         double uArmR  = toMmDisplay(upperArmGeo[6]);
         double wristR = toMmDisplay(wristGeo[3]);
        
        printf("CollisionCheckerSO: ✅ libHRCInterface.so 初始化成功\n");
        printf("  DH: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f] mm\n",
             dhDisp[0],dhDisp[1],dhDisp[2],dhDisp[3],dhDisp[4],dhDisp[5],dhDisp[6],dhDisp[7]);
        printf("  碰撞几何: base=R%.0f, lArm=R%.0f, elbow=R%.0f, uArm=R%.0f, wrist=R%.0f mm\n",
             baseR, lArmR, elbowR, uArmR, wristR);
        printf("  初始化耗时: %.2f ms\n", timing_.initTime_ms);
        
        return true;
    }
    
    // ========================================================================
    // 碰撞检测接口
    // ========================================================================
    
    /**
     * @brief 快速碰撞检测 (自碰撞 + 环境碰撞)
     * @param config 关节配置 (内部rad)
     * @return true = 无碰撞
     */
    bool isCollisionFree(const JointConfig& config) const {
        if (!initialized_) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (!robot_.isWithinLimits(config)) return false;
        
        // 逐层计时
        auto t0 = std::chrono::high_resolution_clock::now();
        
        // update
        SO_LREAL jDeg[6], vel[6] = {0}, acc[6] = {0};
        for (int i = 0; i < 6; i++) jDeg[i] = config.q[i] * 180.0 / M_PI;
        updateAC_(jDeg, vel, acc);
        
        auto t1 = std::chrono::high_resolution_clock::now();
        
        // 自碰撞
        SO_LINT pair[2] = {0,0};
        SO_LREAL dist = 0;
        SO_INT coll = checkSelfColl_(pair, &dist);
        
        auto t2 = std::chrono::high_resolution_clock::now();
        
        // 环境碰撞 (如已注册障碍物)
        bool envColl = false;
        if (getAreaList_) {
            SO_INT areaList[13] = {};
            SO_BOOL outside = getAreaList_(areaList);
            if (!outside) envColl = true;
        }
        
        auto t3 = std::chrono::high_resolution_clock::now();
        
        // 累计计时 (timing_ 为 mutable)
        timing_.callCount++;
        timing_.updateTime_us    += std::chrono::duration<double, std::micro>(t1 - t0).count();
        timing_.selfCheckTime_us += std::chrono::duration<double, std::micro>(t2 - t1).count();
        timing_.envCheckTime_us  += std::chrono::duration<double, std::micro>(t3 - t2).count();
        timing_.totalCheckTime_us += std::chrono::duration<double, std::micro>(t3 - t0).count();
        if (coll) timing_.selfCollCount++;
        if (envColl) timing_.envCollCount++;
        
        if (coll) return false;
        if (dist < safetyMarginMm_) return false;
        if (envColl) return false;
        
        return true;
    }
    
    /**
     * @brief 完整碰撞报告 (自碰撞 + 环境 + TCP位姿)
     * @param config 关节配置 (内部rad)
     * @param computeTcp 是否用.so内置FK计算TCP
     * @return 碰撞报告
     */
    CollisionReportSO getCollisionReport(const JointConfig& config,
                                          bool computeTcp = false) const {
        CollisionReportSO report;
        if (!initialized_) { report.collisionFree = false; return report; }
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (!robot_.isWithinLimits(config)) {
            report.collisionFree = false;
            return report;
        }
        
        auto t0 = std::chrono::high_resolution_clock::now();
        
        // update
        SO_LREAL jDeg[6], vel[6] = {0}, acc[6] = {0};
        for (int i = 0; i < 6; i++) jDeg[i] = config.q[i] * 180.0 / M_PI;
        updateAC_(jDeg, vel, acc);
        
        auto t1 = std::chrono::high_resolution_clock::now();
        
        // 自碰撞
        SO_LINT pair[2] = {0,0};
        SO_LREAL dist = 0;
        SO_INT coll = checkSelfColl_(pair, &dist);
        report.selfCollision = (coll != 0);
        report.selfMinDist_mm = coll ? 0.0 : dist;
        report.selfPairA = (int)pair[0];
        report.selfPairB = (int)pair[1];
        
        auto t2 = std::chrono::high_resolution_clock::now();
        
        // 环境 / TCP区域
        SO_INT areaList[13] = {};
        SO_BOOL outside = getAreaList_(areaList);
        if (!outside) {
            report.envCollision = true;
            int cnt = 0;
            for (int i = 0; i < 13; i++) {
                if (areaList[i] != 0) report.envViolatedAreas[cnt++] = i;
            }
            report.envViolatedCount = cnt;
        }
        
        auto t3 = std::chrono::high_resolution_clock::now();
        
        // TCP 位姿 (可选)
        if (computeTcp && forwardKin_) {
            SO_LREAL jPos[6];
            for (int i = 0; i < 6; i++) jPos[i] = config.q[i] * 180.0 / M_PI;
            forwardKin_(jPos, &report.tcpPose);
            report.hasTcpPose = true;
        }
        
        auto t4 = std::chrono::high_resolution_clock::now();
        
        report.collisionFree = !report.selfCollision && !report.envCollision
                               && report.selfMinDist_mm > safetyMarginMm_;
        
        // 累计计时 (timing_ 为 mutable)
        timing_.callCount++;
        timing_.updateTime_us    += std::chrono::duration<double, std::micro>(t1 - t0).count();
        timing_.selfCheckTime_us += std::chrono::duration<double, std::micro>(t2 - t1).count();
        timing_.envCheckTime_us  += std::chrono::duration<double, std::micro>(t3 - t2).count();
        timing_.fkTime_us        += std::chrono::duration<double, std::micro>(t4 - t3).count();
        timing_.totalCheckTime_us += std::chrono::duration<double, std::micro>(t4 - t0).count();
        if (coll) timing_.selfCollCount++;
        if (report.envCollision) timing_.envCollCount++;
        
        return report;
    }
    
    /**
     * @brief 获取碰撞距离 (mm)
     */
    double getCollisionDistanceMm(const JointConfig& config) const {
        if (!initialized_) return -1;
        std::lock_guard<std::mutex> lock(mutex_);
        
        SO_LREAL jDeg[6], vel[6]={0}, acc[6]={0};
        for (int i = 0; i < 6; i++) jDeg[i] = config.q[i] * 180.0 / M_PI;
        updateAC_(jDeg, vel, acc);
        
        SO_LINT pair[2]; SO_LREAL dist;
        SO_INT coll = checkSelfColl_(pair, &dist);
        return coll ? 0.0 : dist;
    }
    
    /**
     * @brief 路径碰撞检测 (批量, 减少锁开销)
     */
    bool isPathCollisionFree(const JointConfig& start, const JointConfig& end,
                              double resolution = 0.02) const {
        if (!initialized_) return false;
        
        double d = start.distanceTo(end);
        int nChecks = std::max(2, (int)std::ceil(d / resolution) + 1);
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        for (int i = 0; i <= nChecks; i++) {
            double t = (double)i / nChecks;
            JointConfig interp = start.interpolate(end, t);
            
            if (!robot_.isWithinLimits(interp)) return false;
            
            SO_LREAL jDeg[6], vel[6]={0}, acc[6]={0};
            for (int j = 0; j < 6; j++) jDeg[j] = interp.q[j] * 180.0 / M_PI;
            updateAC_(jDeg, vel, acc);
            
            SO_LINT pair[2]; SO_LREAL dist;
            if (checkSelfColl_(pair, &dist)) return false;
            if (dist < safetyMarginMm_) return false;
            
            // 环境碰撞检查
            if (getAreaList_) {
                SO_INT areaList[13] = {};
                SO_BOOL outside = getAreaList_(areaList);
                if (!outside) return false;
            }
        }
        return true;
    }
    
    // ========================================================================
    // TCP 位姿接口 (利用 .so 内置运动学)
    // ========================================================================
    
    /**
     * @brief 正运动学 (使用.so内置)
     * @param config 关节配置 (rad)
     * @param[out] pose 输出TCP位姿 (位置: m, 始态: deg) 注意: 位置返回米(m)而非毫米(mm)
     * @return 成功/失败
     */
    bool forwardKinematics(const JointConfig& config, SO_COORD_REF& pose) const {
        if (!initialized_ || !forwardKin_) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        
        auto t0 = std::chrono::high_resolution_clock::now();
        SO_LREAL jDeg[6];
        for (int i = 0; i < 6; i++) jDeg[i] = config.q[i] * 180.0 / M_PI;
        SO_BOOL ok = forwardKin_(jDeg, &pose);
        auto t1 = std::chrono::high_resolution_clock::now();
        
        timing_.fkTime_us += std::chrono::duration<double, std::micro>(t1 - t0).count();
        timing_.fkCallCount++;
        
        return ok;
    }
    
    /**
     * @brief 逆运动学 (使用.so内置)
     * @param pose 目标TCP位姿 (位置: m, 始态: deg) 注意: 位置单位为米(m)
     * @param refConfig 参考关节配置 (rad)
     * @param[out] result 输出关节配置 (rad)
     * @return 成功/失败
     */
    bool inverseKinematics(const SO_COORD_REF& pose, const JointConfig& refConfig,
                            JointConfig& result) const {
        if (!initialized_ || !inverseKin_) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        
        auto t0 = std::chrono::high_resolution_clock::now();
        SO_LREAL refDeg[6], outDeg[6];
        for (int i = 0; i < 6; i++) refDeg[i] = refConfig.q[i] * 180.0 / M_PI;
        
        SO_COORD_REF p = pose;
        SO_BOOL ok = inverseKin_(p, refDeg, outDeg);
        
        auto t1 = std::chrono::high_resolution_clock::now();
        timing_.ikTime_us += std::chrono::duration<double, std::micro>(t1 - t0).count();
        timing_.ikCallCount++;
        
        if (ok) {
            for (int i = 0; i < 6; i++) result.q[i] = outDeg[i] * M_PI / 180.0;
        }
        return ok;
    }
    
    /**
     * @brief 计算笛卡尔速度 + 加速度
     */
    bool calcCartesianVelAcc(const JointConfig& config,
                              const JointVector& jVel, const JointVector& jAcc,
                              SO_COORD_REF& cartVel, SO_COORD_REF& cartAcc) const {
        if (!initialized_ || !calcCartVelAcc_) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        
        SO_LREAL jPos[6], vel[6], acc[6];
        for (int i = 0; i < 6; i++) {
            jPos[i] = config.q[i] * 180.0 / M_PI;
            vel[i]  = jVel[i] * 180.0 / M_PI;
            acc[i]  = jAcc[i] * 180.0 / M_PI;
        }
        calcCartVelAcc_(jPos, vel, acc, &cartVel, &cartAcc);
        return true;
    }
    
    // ========================================================================
    // 环境障碍物接口
    // ========================================================================
    
    bool addEnvObstacleBall(int id, const Eigen::Vector3d& center_mm, double radius_mm) {
        if (!initialized_ || !addEnvBall_) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        SO_LREAL c[3] = {center_mm.x(), center_mm.y(), center_mm.z()};
        return addEnvBall_(id, c, radius_mm) >= 0;
    }
    
    bool addEnvObstacleCapsule(int id, const Eigen::Vector3d& start_mm,
                                const Eigen::Vector3d& end_mm, double radius_mm) {
        if (!initialized_ || !addEnvCapsule_) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        SO_LREAL s[3] = {start_mm.x(), start_mm.y(), start_mm.z()};
        SO_LREAL e[3] = {end_mm.x(), end_mm.y(), end_mm.z()};
        return addEnvCapsule_(id, s, e, radius_mm) >= 0;
    }
    
    bool removeEnvObstacle(int id) {
        if (!initialized_ || !removeEnv_) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        return removeEnv_(id) >= 0;
    }
    
    void setLinkEnvCollisionEnabled(bool flags[7]) {
        if (!initialized_ || !setLinkEnvFlags_) return;
        std::lock_guard<std::mutex> lock(mutex_);
        SO_BOOL f[7];
        for (int i = 0; i < 7; i++) f[i] = flags[i] ? 1 : 0;
        setLinkEnvFlags_(f);
    }
    
    // ========================================================================
    // 工具碰撞模型
    // ========================================================================
    
    bool setToolBall(int toolIdx, const Eigen::Vector3d& offset_mm, double radius_mm) {
        if (!initialized_ || !setToolBall_) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        SO_LREAL off[3] = {offset_mm.x(), offset_mm.y(), offset_mm.z()};
        return setToolBall_(toolIdx, off, radius_mm) >= 0;
    }
    
    bool setToolCapsule(int toolIdx, const Eigen::Vector3d& start_mm,
                         const Eigen::Vector3d& end_mm, double radius_mm) {
        if (!initialized_ || !setToolCapsule_) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        SO_LREAL s[3] = {start_mm.x(), start_mm.y(), start_mm.z()};
        SO_LREAL e[3] = {end_mm.x(), end_mm.y(), end_mm.z()};
        return setToolCapsule_(toolIdx, s, e, radius_mm) >= 0;
    }
    
    /**
     * @brief 移除工具碰撞体
     * @param toolIdx 工具索引 (6=Tool1, 7=Tool2)
     * @return 成功返回true
     */
    bool removeTool(int toolIdx) {
        if (!initialized_ || !removeTool_) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        return removeTool_((SO_LINT)toolIdx) == 0;
    }
    
    // ========================================================================
    // 性能与配置
    // ========================================================================
    
    void setSafetyMarginMm(double marginMm) { safetyMarginMm_ = marginMm; }
    double getSafetyMarginMm() const { return safetyMarginMm_; }
    
    /// 获取分层计时统计 (平均值)
    TimingStats getTimingStats() const {
        TimingStats avg = timing_;
        if (avg.callCount > 0) {
            avg.updateTime_us    /= avg.callCount;
            avg.selfCheckTime_us /= avg.callCount;
            avg.envCheckTime_us  /= avg.callCount;
            avg.totalCheckTime_us /= avg.callCount;
        }
        // FK/IK 使用各自独立的计数器
        if (avg.fkCallCount > 0) {
            avg.fkTime_us /= avg.fkCallCount;
        }
        if (avg.ikCallCount > 0) {
            avg.ikTime_us /= avg.ikCallCount;
        }
        return avg;
    }
    
    void resetTimingStats() { timing_.reset(); }
    
    bool isInitialized() const { return initialized_; }
    
private:
    bool loadSymbols() {
        #define LOAD(var, name) \
            var = (decltype(var))dlsym(handle_, #name); \
            if (!var) { fprintf(stderr, "dlsym失败: %s\n", #name); return false; }
        
        LOAD(initAC_,         initACAreaConstrainPackageInterface);
        LOAD(updateAC_,       updateACAreaConstrainPackageInterface);
        LOAD(checkSelfColl_,  checkCPSelfCollisionInterface);
        LOAD(setColliderOpen_,setCPSelfColliderLinkModelOpenStateInterface);
        LOAD(getUIInfo_,      getUIInfoMationInterface);
        LOAD(getAreaList_,    getACTCPInAreaListInterface);
        LOAD(getAreaOpen_,    getACAreaOpenStateInterface);
        
        #undef LOAD
        
        // 可选接口 (不阻断初始化)
        #define TRY_LOAD(var, name) var = (decltype(var))dlsym(handle_, #name)
        
        TRY_LOAD(addOBB_LWH_,     addACOrientedBoundingBoxAreaDefindLWHInterface);
        TRY_LOAD(addHalfPlane_,    addACHalfPlaneAreaInterface);
        TRY_LOAD(deleteArea_,      deleteACAreaInterface);
        TRY_LOAD(getRelMotion_,    getACRelativeMotionInfoInterface);
        TRY_LOAD(getTcpInArea_,    getACTCPInAreaStatusInterface);
        TRY_LOAD(forwardKin_,      forwardKinematics2);
        TRY_LOAD(inverseKin_,      inverseKinematics);
        TRY_LOAD(setToolBall_,     setCPToolCollisionBallShapeInterface);
        TRY_LOAD(setToolCapsule_,  setCPToolCollisonCapsuleShapeInterface);
        TRY_LOAD(removeTool_,      removeCPToolCollisonInterface);
        TRY_LOAD(printCollPair_,   printCollisionPairInterface);
        TRY_LOAD(addEnvBall_,      addEnvObstacleBallInterface);
        TRY_LOAD(addEnvCapsule_,   addEnvObstacleCapsuleInterface);
        TRY_LOAD(addEnvLozenge_,   addEnvObstacleLozengeInterface);
        TRY_LOAD(removeEnv_,       removeEnvObstacleInterface);
        TRY_LOAD(setLinkEnvFlags_, setLinkEnvCollisionEnabledInterface);
        TRY_LOAD(getEnvCount_,     getEnvObstacleCountInterface);
        TRY_LOAD(initTCPPos_,      initTCPPositionInterface);
        TRY_LOAD(updateTCPPos_,    updateTCPPositionInterface);
        TRY_LOAD(setStopType_,     setStopTypeAndDecelerationInterface);
        TRY_LOAD(calcCartVelAcc_,  calculateCartesianVelAndAccFromJoint);
        
        #undef TRY_LOAD
        
        return true;
    }
    
    const RobotModel& robot_;
    void* handle_;
    bool initialized_;
    bool linkWallEnabled_ = false;
    double safetyMarginMm_ = 10.0;  // 默认10mm安全余量
    mutable std::mutex mutex_;
    mutable TimingStats timing_;
    
    // 必需函数指针
    FnInitAC         initAC_          = nullptr;
    FnUpdateAC       updateAC_        = nullptr;
    FnCheckSelfColl  checkSelfColl_   = nullptr;
    FnSetColliderOpen setColliderOpen_ = nullptr;
    FnGetUIInfo      getUIInfo_       = nullptr;
    FnGetAreaList    getAreaList_     = nullptr;
    FnGetAreaOpen    getAreaOpen_     = nullptr;
    
    // 可选函数指针
    FnAddOBB_LWH     addOBB_LWH_     = nullptr;
    FnAddHalfPlane   addHalfPlane_    = nullptr;
    FnDeleteArea     deleteArea_      = nullptr;
    FnGetRelMotion   getRelMotion_    = nullptr;
    FnGetTcpInArea   getTcpInArea_    = nullptr;
    FnForwardKin     forwardKin_      = nullptr;
    FnInverseKin     inverseKin_      = nullptr;
    FnSetToolBall    setToolBall_     = nullptr;
    FnSetToolCapsule setToolCapsule_  = nullptr;
    FnRemoveTool     removeTool_      = nullptr;
    FnPrintCollPair  printCollPair_   = nullptr;
    FnAddEnvBall     addEnvBall_      = nullptr;
    FnAddEnvCapsule  addEnvCapsule_   = nullptr;
    FnAddEnvLozenge  addEnvLozenge_   = nullptr;
    FnRemoveEnv      removeEnv_       = nullptr;
    FnSetLinkEnvFlags setLinkEnvFlags_ = nullptr;
    FnGetEnvCount    getEnvCount_     = nullptr;
    FnInitTCPPos     initTCPPos_      = nullptr;
    FnUpdateTCPPos   updateTCPPos_    = nullptr;
    FnSetStopType    setStopType_     = nullptr;
    FnCalcCartVelAcc calcCartVelAcc_  = nullptr;
};

} // namespace palletizing
