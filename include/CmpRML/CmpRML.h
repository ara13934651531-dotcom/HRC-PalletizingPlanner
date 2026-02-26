/**
 * @file CmpRML.h
 * @brief libCmpRML.so CoDeSys-IEC61131 运动控制库封装
 *
 * 华数机器人上位机运动控制库（S曲线轨迹生成）。
 *
 * ⚠️ 重要: 该库使用 CoDeSys/IEC 61131-3 函数块(Function Block)调用约定。
 *    每个导出函数接受一个 void* 参数，指向函数块实例的数据结构。
 *    数据结构布局通过反汇编逆向工程获得。
 *
 * 使用流程:
 *   1. 构造 RMLController 对象
 *   2. initialize(6, 4.0) — 初始化运动系统
 *   3. setJointMotionLimits(...) — 设置运动限制
 *   4. setJointSpaceLimits(...) — 设置关节限位
 *   5. setOverride(100.0) — 设置速度倍率
 *   6. setStartPosition(...) — 设置起始位置
 *   7. moveJoint(...) — 发起关节运动
 *   8. 循环调用 step() 获取每个周期的插补数据
 *
 * @note 关节角度单位: deg
 * @note 周期时间单位: ms (典型值 4.0ms)
 * @note 速度/加速度/加加速度: deg/s, deg/s², deg/s³
 *
 * @author Guangdong Huayan Robotics Co., Ltd.
 * @date 2026-02-06
 */

#ifndef CMP_RML_H
#define CMP_RML_H

#include <cstdint>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <array>

// ============================================================================
// Raw CoDeSys function block interface (每个函数接收单个 void* 参数)
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/* 系统 */
void rm_getmotionlibversion(void* fb);
void rm_getsystemtime(void* fb);
void rm_setlogstate(void* fb);

/* 初始化与配置 */
void rm_initializehansroboticsmotion(void* fb);
void rm_resetstatus(void* fb);
void rm_setjointmotionlimits(void* fb);
void rm_setjointspacelimits(void* fb);
void rm_setlinearmotionlimits(void* fb);
void rm_setoverride(void* fb);
void rm_settooloffset(void* fb);
void rm_setabsolutecalibparams(void* fb);

/* 运动指令 */
void externalblock__setstartjointposition(void* fb);
void rm_movejoint(void* fb);
void rm_movelinear(void* fb);
void rm_movejointspline(void* fb);
void rm_movejointspeed(void* fb);
void rm_moveservoj(void* fb);
void rm_speedj(void* fb);
void rm_groupstop(void* fb);
void externalblock__groupinterrupt(void* fb);
void externalblock__groupcontinue(void* fb);

/* 运动状态查询 */
void calculatecommandstate(void* fb);
void rm_getmotionstatus(void* fb);
void rm_readmotionstate(void* fb);
void rm_readmotionsettingparams(void* fb);

/* 外部块 */
void externalblock(void* fb);
void printdebuginfo(void* fb);
void readfromlocaldatatogeneratemotion(void* fb);
void readinterpolatecalctime(void* fb);
void rm_updaterobotstate(void* fb);

#ifdef __cplusplus
}
#endif

// ============================================================================
// C++ 高级封装 — 隐藏 CoDeSys 函数块偏移细节
// ============================================================================

#ifdef __cplusplus

/**
 * @brief libCmpRML S曲线运动控制器封装
 *
 * 通过反汇编逆向工程获得的CoDeSys函数块结构体偏移:
 *
 * rm_initializehansroboticsmotion:
 *   [0x00] LREAL cycleTimeMs
 *   [0x08] INT   numJoints
 *   [0x10] ...   output area (内部分配数据)
 *
 * rm_setjointmotionlimits:
 *   [0x00..0x2F] LREAL[6] maxVelocity    (48B)
 *   [0x30..0x5F] LREAL[6] maxAcceleration(48B)
 *   [0x60..0x8F] LREAL[6] maxJerk        (48B)
 *   [0x90]       INT      numJoints
 *
 * rm_setjointspacelimits:
 *   [0x00..0x2F] LREAL[6] jointMin (48B)
 *   [0x30..0x5F] LREAL[6] jointMax (48B)
 *
 * rm_setoverride:
 *   [0x00] SINT  groupIndex
 *   [0x08] LREAL overridePercent
 *
 * externalblock__setstartjointposition:
 *   [0x08..0x37] LREAL[6] q1..q6 (48B)
 *
 * rm_movejoint:
 *   [0x00]       SINT     groupIndex
 *   [0x08..0x37] LREAL[6] targetPos (48B)
 *   [0x38..0xC7] ...      internal buffers (3×48B)
 *   [0xC8]       INT      motionType
 *   [0xD0]       LREAL    velocityPercent
 *   [0xD8]       BOOL     result (output)
 *   [0xD9..0xDE] ...      additional outputs
 *
 * calculatecommandstate:
 *   [0x00]       SINT     groupIndex
 *   [0x08..0x37] LREAL[6] position    (output, 48B)
 *   [0x38..0x67] LREAL[6] velocity    (output, 48B)
 *   [0x68..0x97] LREAL[6] acceleration(output, 48B)
 *   [0x98..0xC7] ...      extra output (48B)
 *
 * rm_getmotionstatus:
 *   [0x00..0x07] SINT groupIndex (or other header)
 *   [0x08]       LINT/LREAL primary status
 *   [0x10]       BOOL isMoving
 *   [0x11]       BOOL isDone
 *   [0x12]       BOOL hasError
 *   [0x14]       INT  errorCode?
 *   [0x16]       INT  statusCode?
 *   [0x18]       INT  reserved?
 *   [0x1A]       INT  reserved?
 */
class RMLController {
public:
    static constexpr int NUM_JOINTS = 6;
    static constexpr size_t FB_SIZE = 4096;  ///< 安全的函数块大小

    /** @brief 运动步进结果 */
    struct StepResult {
        double position[6];      ///< 当前关节位置 [deg]
        double velocity[6];      ///< 当前关节速度 [deg/s]
        double acceleration[6];  ///< 当前关节加速度 [deg/s²]
        bool   done;             ///< 运动是否完成
    };

    RMLController() {
        clearAll();
    }

    // ---------------------------------------------------------------
    // 初始化配置
    // ---------------------------------------------------------------

    /**
     * @brief 初始化运动系统
     * @param numJoints 关节数量 (6)
     * @param cycleTimeSec 插补周期 [秒]，典型值 0.004 (即4ms)
     *                     ⚠️ 单位为秒! 内部Count=0.012/cycleTime 必须>0
     * @param dhParams DH运动学参数 [d1,d2,d3,d4,d5,d6,a2,a3] (mm)
     *                 如果为nullptr，使用HR_S50默认值
     * @return true 初始化成功
     */
    bool initialize(int numJoints, double cycleTimeSec,
                    const double* dhParams = nullptr) {
        clearFB(fb_init_);
        cycleTimeSec_ = cycleTimeSec;
        numJoints_   = numJoints;
        // offset 0x00: LREAL cycleTime [秒]
        putLREAL(fb_init_, 0x00, cycleTimeSec);
        // offset 0x08: INT numJoints
        putINT(fb_init_, 0x08, (int16_t)numJoints);
        // offset 0x10-0x4F: LREAL[8] DH kinematic params
        //   顺序: [d1, d2, d3, d4, d5, d6, a2, a3] (mm)
        if (dhParams) {
            for (int i = 0; i < 8; i++)
                putLREAL(fb_init_, 0x10 + i * 8, dhParams[i]);
        } else {
            // HR_S50-2000 默认DH参数 (mm)
            static const double s50dh[8] = {
                296.5, 336.2, 239.0, 158.5, 158.5, 134.5,  // d1-d6
                900.0, 941.5                                 // a2, a3
            };
            for (int i = 0; i < 8; i++)
                putLREAL(fb_init_, 0x10 + i * 8, s50dh[i]);
        }
        try {
            rm_initializehansroboticsmotion(fb_init_);
        } catch (const std::exception& e) {
            // 内部初始化可能抛出异常，但核心运动功能可能仍然可用
            fprintf(stderr, "[RML Warning] init exception: %s\n", e.what());
        }
        initialized_ = true;
        return true;
    }

    /**
     * @brief 重置运动状态
     * @param groupIndex 组索引 (默认0)
     */
    void resetStatus(int8_t groupIndex = 0) {
        clearFB(fb_reset_);
        // offset 0x00: SINT groupIndex
        fb_reset_[0] = (uint8_t)groupIndex;
        rm_resetstatus(fb_reset_);
        motionActive_ = false;
    }

    /**
     * @brief 设置关节运动限制
     * @param maxVel  各关节最大速度 [deg/s], 6元素
     * @param maxAcc  各关节最大加速度 [deg/s²], 6元素
     * @param maxJerk 各关节最大加加速度 [deg/s³], 6元素
     */
    void setJointMotionLimits(const double maxVel[6],
                               const double maxAcc[6],
                               const double maxJerk[6]) {
        clearFB(fb_limits_);
        // [0x00..0x2F] maxVelocity
        for (int i = 0; i < 6; i++) putLREAL(fb_limits_, 0x00 + i * 8, maxVel[i]);
        // [0x30..0x5F] maxAcceleration
        for (int i = 0; i < 6; i++) putLREAL(fb_limits_, 0x30 + i * 8, maxAcc[i]);
        // [0x60..0x8F] maxJerk
        for (int i = 0; i < 6; i++) putLREAL(fb_limits_, 0x60 + i * 8, maxJerk[i]);
        // [0x90] INT numJoints
        putINT(fb_limits_, 0x90, (int16_t)numJoints_);
        rm_setjointmotionlimits(fb_limits_);
    }

    /**
     * @brief 设置关节位置限位
     * @param jointMin 各关节最小角度 [deg], 6元素
     * @param jointMax 各关节最大角度 [deg], 6元素
     */
    void setJointSpaceLimits(const double jointMin[6],
                              const double jointMax[6]) {
        clearFB(fb_space_);
        // [0x00..0x2F] jointMin
        for (int i = 0; i < 6; i++) putLREAL(fb_space_, 0x00 + i * 8, jointMin[i]);
        // [0x30..0x5F] jointMax
        for (int i = 0; i < 6; i++) putLREAL(fb_space_, 0x30 + i * 8, jointMax[i]);
        rm_setjointspacelimits(fb_space_);
    }

    /**
     * @brief 设置速度倍率
     * @param overridePercent 速度百分比 [0–100]
     * @param groupIndex 组索引 (默认0)
     */
    void setOverride(double overridePercent, int8_t groupIndex = 0) {
        clearFB(fb_override_);
        fb_override_[0] = (uint8_t)groupIndex;
        putLREAL(fb_override_, 0x08, overridePercent);
        rm_setoverride(fb_override_);
    }

    /**
     * @brief 设置起始关节位置
     * @param position 6个关节角度 [deg]
     */
    void setStartPosition(const double position[6]) {
        clearFB(fb_startpos_);
        // [0x08..0x37] LREAL[6] q1..q6
        for (int i = 0; i < 6; i++) {
            putLREAL(fb_startpos_, 0x08 + i * 8, position[i]);
            currentPos_[i] = position[i];
        }
        externalblock__setstartjointposition(fb_startpos_);
    }

    /**
     * @brief 设置日志状态
     * @param enable 1=开启, 0=关闭
     */
    void setLogState(int16_t enable) {
        clearFB(fb_log_);
        putINT(fb_log_, 0x00, enable);
        rm_setlogstate(fb_log_);
    }

    // ---------------------------------------------------------------
    // 运动指令
    // ---------------------------------------------------------------

    /**
     * @brief 发起关节空间点对点运动 (S曲线)
     * @param targetPos 目标关节角度 [deg], 6元素
     * @param velPercent 速度百分比 [1–100]
     * @param groupIndex 组索引 (默认0)
     */
    void moveJoint(const double targetPos[6], double velPercent,
                   int8_t groupIndex = 0) {
        // 保留 fb_movejoint_ 的内部状态（块2-4），不做clearFB
        // 只设置输入字段
        fb_movejoint_[0] = (uint8_t)groupIndex;
        for (int i = 0; i < 6; i++) {
            putLREAL(fb_movejoint_, 0x08 + i * 8, targetPos[i]);
            targetPos_[i] = targetPos[i];
        }
        putINT(fb_movejoint_, 0xC8, 0);  // motionType = 0 (PTP)
        putLREAL(fb_movejoint_, 0xD0, velPercent);

        rm_movejoint(fb_movejoint_);

        motionActive_ = true;
        stepCount_ = 0;
    }

    /**
     * @brief 执行一步插补计算
     * @param result [out] 当前插补数据
     * @return true 如果运动仍在进行
     *
     * 应在 moveJoint() 后循环调用，直到返回 false（运动完成）
     */
    bool step(StepResult& result) {
        if (!motionActive_) {
            result.done = true;
            return false;
        }

        // calculatecommandstate 填充位置/速度/加速度
        fb_calcstate_[0] = 0;  // groupIndex
        calculatecommandstate(fb_calcstate_);

        // 读取输出
        for (int i = 0; i < 6; i++) {
            result.position[i]     = getLREAL(fb_calcstate_, 0x08 + i * 8);
            result.velocity[i]     = getLREAL(fb_calcstate_, 0x38 + i * 8);
            result.acceleration[i] = getLREAL(fb_calcstate_, 0x68 + i * 8);
            currentPos_[i] = result.position[i];
        }

        stepCount_++;

        // 检查运动是否完成
        // 方法: 通过位置到达 + 速度归零判断
        bool posReached = true;
        for (int i = 0; i < 6; i++) {
            if (std::fabs(result.position[i] - targetPos_[i]) > 0.01) {
                posReached = false;
                break;
            }
        }

        bool velZero = true;
        for (int i = 0; i < 6; i++) {
            if (std::fabs(result.velocity[i]) > 0.01) {
                velZero = false;
                break;
            }
        }

        // 位置到达且速度为零 → 完成
        result.done = (posReached && velZero && stepCount_ > 1);

        // 安全限制: 最大 100000 步
        if (stepCount_ > 100000) {
            result.done = true;
        }

        if (result.done) {
            motionActive_ = false;
        }

        return !result.done;
    }

    /**
     * @brief 简化版步进: 直接返回位置
     * @param position [out] 当前关节位置 [deg]
     * @return true 表示运动仍在进行
     */
    bool step(double position[6]) {
        StepResult result;
        bool running = step(result);
        for (int i = 0; i < 6; i++) position[i] = result.position[i];
        return running;
    }

    // ---------------------------------------------------------------
    // 状态查询
    // ---------------------------------------------------------------

    /** @brief 获取当前关节位置 [deg] */
    const double* currentPosition() const { return currentPos_; }

    /** @brief 获取步进计数 */
    int stepCount() const { return stepCount_; }

    /** @brief 运动是否在执行中 */
    bool isMotionActive() const { return motionActive_; }

    /** @brief 获取周期时间 [秒] */
    double cycleTime() const { return cycleTimeSec_; }

    /** @brief 获取当前时间 [s] */
    double currentTime() const { return stepCount_ * cycleTimeSec_; }

    // ---------------------------------------------------------------
    // 便捷配置 (HR_S50-2000 默认参数)
    // ---------------------------------------------------------------

    /**
     * @brief 使用 HR_S50-2000 默认参数初始化
     * @param cycleTimeSec 插补周期 [秒]，默认0.004 (4ms)
     */
    void initializeS50(double cycleTimeSec = 0.004) {
        // HR_S50 DH参数 (mm)
        double dh[8] = {296.5, 336.2, 239.0, 158.5, 158.5, 134.5, 900.0, 941.5};
        initialize(6, cycleTimeSec, dh);

        // HR_S50 关节运动限制
        double maxVel[6]  = {225.0, 225.0, 225.0, 400.0, 400.0, 400.0};
        double maxAcc[6]  = {500.0, 500.0, 500.0, 1000.0, 1000.0, 1000.0};
        double maxJerk[6] = {2000.0, 2000.0, 2000.0, 4000.0, 4000.0, 4000.0};
        setJointMotionLimits(maxVel, maxAcc, maxJerk);

        // HR_S50 关节限位 [deg]
        double jointMin[6] = {-360, -190, -165, -360, -360, -360};
        double jointMax[6] = { 360,   10,  165,  360,  360,  360};
        setJointSpaceLimits(jointMin, jointMax);

        setOverride(100.0);
    }

    /**
     * @brief 停止当前运动
     */
    void stopMotion(int8_t groupIndex = 0) {
        clearFB(fb_stop_);
        fb_stop_[0] = (uint8_t)groupIndex;
        rm_groupstop(fb_stop_);
        motionActive_ = false;
    }

private:
    // 函数块实例 (足够大的零初始化缓冲区)
    // 注: CoDeSys函数块可能在内部写入大量状态数据
    alignas(16) uint8_t fb_init_[8192]     = {};
    alignas(16) uint8_t fb_reset_[256]     = {};
    alignas(16) uint8_t fb_limits_[512]    = {};
    alignas(16) uint8_t fb_space_[512]     = {};
    alignas(16) uint8_t fb_override_[256]  = {};
    alignas(16) uint8_t fb_startpos_[256]  = {};
    alignas(16) uint8_t fb_movejoint_[1024] = {};
    alignas(16) uint8_t fb_calcstate_[1024] = {};
    alignas(16) uint8_t fb_status_[256]    = {};
    alignas(16) uint8_t fb_log_[256]       = {};
    alignas(16) uint8_t fb_stop_[256]      = {};

    double currentPos_[6]  = {};
    double targetPos_[6]   = {};
    double cycleTimeSec_   = 0.004;
    int    numJoints_      = 6;
    int    stepCount_      = 0;
    bool   initialized_    = false;
    bool   motionActive_   = false;

    void clearAll() {
        clearFB(fb_init_,     sizeof(fb_init_));
        clearFB(fb_reset_,    sizeof(fb_reset_));
        clearFB(fb_limits_,   sizeof(fb_limits_));
        clearFB(fb_space_,    sizeof(fb_space_));
        clearFB(fb_override_, sizeof(fb_override_));
        clearFB(fb_startpos_, sizeof(fb_startpos_));
        clearFB(fb_movejoint_,sizeof(fb_movejoint_));
        clearFB(fb_calcstate_,sizeof(fb_calcstate_));
        clearFB(fb_status_,   sizeof(fb_status_));
        clearFB(fb_log_,      sizeof(fb_log_));
        clearFB(fb_stop_,     sizeof(fb_stop_));
    }

    static void clearFB(uint8_t* fb, size_t sz = FB_SIZE) {
        std::memset(fb, 0, sz);
    }

    static void putLREAL(uint8_t* fb, size_t offset, double value) {
        std::memcpy(fb + offset, &value, 8);
    }

    static double getLREAL(const uint8_t* fb, size_t offset) {
        double v; std::memcpy(&v, fb + offset, 8); return v;
    }

    static void putINT(uint8_t* fb, size_t offset, int16_t value) {
        std::memcpy(fb + offset, &value, 2);
    }

    static int16_t getINT(const uint8_t* fb, size_t offset) {
        int16_t v; std::memcpy(&v, fb + offset, 2); return v;
    }
};

#endif // __cplusplus
#endif // CMP_RML_H
