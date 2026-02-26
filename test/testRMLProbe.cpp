/**
 * @file testRMLProbe.cpp
 * @brief libCmpRML CoDeSys函数块接口验证 — 单次初始化+完整运动测试
 *
 * 使用反汇编逆向得出的CoDeSys结构体偏移进行S曲线运动验证。
 * 每个函数仅通过void*传递函数块数据。
 *
 * 关键发现 (反汇编):
 *   - calculatecommandstate 内部检查全局运动标志 (BSS偏移 0x564390)
 *   - 该标志仅由 ComponentEntry (CoDeSys PLC循环) 设置为1
 *   - 没有CoDeSys运行时, 需要手动设置该标志
 *   - rm_movejoint 在 0x108380 处实际计算轨迹数据并存入内部缓冲
 *   - 运动标志地址 = calculatecommandstate地址 + 0x2863E0
 *   - 基时间戳   = calculatecommandstate地址 + 0x286678 (int64 微秒)
 *   - 运动开始时间 = calculatecommandstate地址 + 0x2863D8 (double 秒)
 *
 * @date 2026-02-06
 */

#include "CmpRML/CmpRML.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <signal.h>
#include <setjmp.h>
#include <stdexcept>
#include <dlfcn.h>
#include <sys/time.h>
#include <unistd.h>

static sigjmp_buf jumpBuf;
static volatile sig_atomic_t gotSignal = 0;

void signalHandler(int sig) {
    gotSignal = sig;
    siglongjmp(jumpBuf, 1);
}

// ============================================================================
// 测试1: 底层原始调用 (完整的init → config → move → step流程)
// ============================================================================
void testRawMotionPipeline() {
    printf("\n═══════════════════════════════════════════════════\n");
    printf("  测试1: 原始CoDeSys函数块 - 完整运动流程\n");
    printf("═══════════════════════════════════════════════════\n\n");

    // 在堆上分配大缓冲区，避免堆栈溢出
    static alignas(16) uint8_t fb_init[8192];
    static alignas(16) uint8_t fb_reset[256];
    static alignas(16) uint8_t fb_limits[512];
    static alignas(16) uint8_t fb_space[512];
    static alignas(16) uint8_t fb_override[256];
    static alignas(16) uint8_t fb_startpos[256];
    static alignas(16) uint8_t fb_movejoint[1024];
    static alignas(16) uint8_t fb_calcstate[1024];
    static alignas(16) uint8_t fb_external[4096];
    static alignas(16) uint8_t fb_generate[4096];
    static alignas(16) uint8_t fb_upstate[4096];
    static alignas(16) uint8_t fb_status[1024];
    static alignas(16) uint8_t fb_groupctrl[256];
    static alignas(16) uint8_t fb_readstate[1024];
    static alignas(16) uint8_t fb_systime[256];
    static alignas(16) uint8_t fb_itpcalc[256];

    memset(fb_init, 0, sizeof(fb_init));
    memset(fb_reset, 0, sizeof(fb_reset));
    memset(fb_limits, 0, sizeof(fb_limits));
    memset(fb_space, 0, sizeof(fb_space));
    memset(fb_override, 0, sizeof(fb_override));
    memset(fb_startpos, 0, sizeof(fb_startpos));
    memset(fb_movejoint, 0, sizeof(fb_movejoint));
    memset(fb_calcstate, 0, sizeof(fb_calcstate));
    memset(fb_external, 0, sizeof(fb_external));
    memset(fb_generate, 0, sizeof(fb_generate));
    memset(fb_upstate, 0, sizeof(fb_upstate));
    memset(fb_status, 0, sizeof(fb_status));
    memset(fb_groupctrl, 0, sizeof(fb_groupctrl));
    memset(fb_readstate, 0, sizeof(fb_readstate));
    memset(fb_systime, 0, sizeof(fb_systime));
    memset(fb_itpcalc, 0, sizeof(fb_itpcalc));

    auto putd = [](uint8_t* buf, size_t off, double v) {
        memcpy(buf + off, &v, 8);
    };
    auto getd = [](const uint8_t* buf, size_t off) -> double {
        double v; memcpy(&v, buf + off, 8); return v;
    };
    auto puti16 = [](uint8_t* buf, size_t off, int16_t v) {
        memcpy(buf + off, &v, 2);
    };

    // Step 1: 日志开启
    printf("  [1] rm_setlogstate(1)\n");
    {
        alignas(16) uint8_t fb[256] = {};
        puti16(fb, 0x00, 1);
        rm_setlogstate(fb);
    }

    // Step 2: 获取版本
    printf("  [2] rm_getmotionlibversion\n");
    {
        alignas(16) uint8_t fb[256] = {};
        rm_getmotionlibversion(fb);
        int16_t ver;
        memcpy(&ver, fb, 2);
        printf("      → 版本: %d\n", ver);
    }

    // Step 3: 初始化
    // ⚠️ 关键发现: cycleTime单位为秒(s), 不是毫秒!
    //   内部计算 Count = int(0.012 / cycleTime), Count 必须 > 0
    //   cycleTime=0.004s → Count=3 ✓   cycleTime=4.0 → Count=0 ✗
    double cycleTimeSec = 0.004;  // 4ms = 0.004s
    printf("  [3] rm_initializehansroboticsmotion(6, %.4fs)\n", cycleTimeSec);
    putd(fb_init, 0x00, cycleTimeSec);       // cycleTime [秒!]
    puti16(fb_init, 0x08, 6);                 // numJoints
    // DH params at offset 0x10: d1,d2,d3,d4,d5,d6,a2,a3 (mm)
    putd(fb_init, 0x10, 296.5);
    putd(fb_init, 0x18, 336.2);
    putd(fb_init, 0x20, 239.0);
    putd(fb_init, 0x28, 158.5);
    putd(fb_init, 0x30, 158.5);
    putd(fb_init, 0x38, 134.5);
    putd(fb_init, 0x40, 900.0);
    putd(fb_init, 0x48, 941.5);
    try {
        rm_initializehansroboticsmotion(fb_init);
        printf("      → 初始化成功\n");
    } catch (const std::exception& e) {
        printf("      → 初始化异常(已捕获): %s\n", e.what());
    }

    // Step 4: 重置
    printf("  [4] rm_resetstatus(0)\n");
    fb_reset[0] = 0;
    rm_resetstatus(fb_reset);

    // Step 5: 设置运动限制
    printf("  [5] rm_setjointmotionlimits\n");
    double mv[] = {225, 225, 225, 400, 400, 400};
    double ma[] = {500, 500, 500, 1000, 1000, 1000};
    double mj[] = {2000, 2000, 2000, 4000, 4000, 4000};
    memcpy(fb_limits + 0x00, mv, 48);
    memcpy(fb_limits + 0x30, ma, 48);
    memcpy(fb_limits + 0x60, mj, 48);
    puti16(fb_limits, 0x90, 6);
    rm_setjointmotionlimits(fb_limits);

    // Step 6: 设置关节限位
    printf("  [6] rm_setjointspacelimits\n");
    double jmax[] = {360, 10, 165, 360, 360, 360};    // upper limits
    double jmin[] = {-360, -190, -165, -360, -360, -360}; // lower limits
    // 依据汇编: [0x00]=upper, [0x30]=lower (根据输出逆向修正)
    memcpy(fb_space + 0x00, jmax, 48);
    memcpy(fb_space + 0x30, jmin, 48);
    rm_setjointspacelimits(fb_space);

    // Step 7: 设置倍率
    printf("  [7] rm_setoverride(100.0)\n");
    fb_override[0] = 0;
    putd(fb_override, 0x08, 100.0);
    rm_setoverride(fb_override);

    // Step 8: 设置起始位置
    printf("  [8] externalblock__setstartjointposition\n");
    double startPos[] = {0, -90, 0, 0, 90, 0};
    memcpy(fb_startpos + 0x08, startPos, 48);
    externalblock__setstartjointposition(fb_startpos);
    printf("      → 起始位置: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]\n",
           startPos[0], startPos[1], startPos[2],
           startPos[3], startPos[4], startPos[5]);

    // Step 9: 发起运动
    printf("  [9] rm_movejoint → 目标位置: [45, -60, 30, 0, 60, 0]\n");
    double targetPos[] = {45, -60, 30, 0, 60, 0};
    fb_movejoint[0] = 0;  // groupIndex
    memcpy(fb_movejoint + 0x08, targetPos, 48);
    puti16(fb_movejoint, 0xC8, 0);   // motionType
    putd(fb_movejoint, 0xD0, 100.0); // velPercent
    rm_movejoint(fb_movejoint);
    printf("      → movejoint result byte: %d\n", (int)fb_movejoint[0xD8]);

    // 显式激活运动组 (若内部状态机需要continue信号)
    fb_groupctrl[0] = 0;
    externalblock__groupcontinue(fb_groupctrl);

    // 预热一次“PLC周期”相关函数，触发内部状态机
    fb_generate[0] = 0;
    fb_external[0] = 0;
    fb_upstate[0] = 0;
    fb_status[0] = 0;
    readfromlocaldatatogeneratemotion(fb_generate);
    externalblock(fb_external);
    rm_updaterobotstate(fb_upstate);
    rm_getmotionstatus(fb_status);
        rm_readmotionstate(fb_readstate);
        printf("      → status bytes: moving=%d done=%d err=%d code=%d\n",
            (int)fb_status[0x10], (int)fb_status[0x11], (int)fb_status[0x12],
            (int)(int16_t)(fb_status[0x14] | (fb_status[0x15] << 8)));

    // ========================================================================
    // 关键: 手动设置全局运动标志
    // ========================================================================
    // calculatecommandstate 内部(0x2f13ed)检查全局标志:
    //   cmpb $0x0, 0x272f9c(%rip)   → 地址 0x564390
    //   jne 0x2f1cb0                → 若!=0则执行运动计算
    //
    // 该标志仅由 ComponentEntry (CoDeSys PLC运行时) 设置
    // 没有 CoDeSys 运行时, 我们需要手动设置
    //
    // 地址计算: calculatecommandstate 在文件偏移 0x2ddfb0
    //           运动标志在文件偏移 0x564390
    //           差值 = 0x564390 - 0x2ddfb0 = 0x2863E0
    //
    // 同时需要初始化:
    //   基时间戳 (微秒, int64): offset +0x286678  (用于时间零点)
    //   运动开始时间 (秒, double): offset +0x2863D8

    printf("\n  [FLAG] 通过dlsym计算全局运动标志地址...\n");
    void* calc_fn = dlsym(RTLD_DEFAULT, "calculatecommandstate");
    using ComponentEntryFn = void(*)();
    ComponentEntryFn componentEntry = (ComponentEntryFn)dlsym(RTLD_DEFAULT, "ComponentEntry");
    if (!calc_fn) {
        printf("  [错误] dlsym失败: %s\n", dlerror());
        return;
    }
    printf("      ComponentEntry @ %p\n", (void*)componentEntry);

    // 计算全局变量运行时地址
    char* base = (char*)calc_fn;
    volatile char* motion_flag  = (volatile char*)(base + 0x2863E0);  // 0x564390
    volatile double* start_time = (volatile double*)(base + 0x2863D8); // 0x564388
    volatile int64_t* base_ts   = (volatile int64_t*)(base + 0x286678); // 0x564628

    printf("      calculatecommandstate @ %p\n", calc_fn);
    printf("      motion_flag @ %p (当前值: %d)\n", (void*)motion_flag, (int)*motion_flag);
    printf("      start_time  @ %p (当前值: %.6f)\n", (void*)start_time, *start_time);
    printf("      base_ts     @ %p (当前值: %ld)\n", (void*)base_ts, (long)*base_ts);

    // 初始化时间基准
    struct timeval tv;
    gettimeofday(&tv, NULL);
    int64_t now_us = (int64_t)tv.tv_sec * 1000000L + tv.tv_usec;
    double now_sec = (double)now_us / 1e6;

    *base_ts = now_us;        // 设置时间零点
    *start_time = now_sec;    // 设置运动开始时间
    *motion_flag = 1;         // ← 关键: 激活运动!

    printf("      → 已设置 motion_flag=1, base_ts=%ld, start_time=%.6f\n",
           (long)*base_ts, *start_time);

    // Step 10: 循环步进 calculatecommandstate
    printf("\n  [10] 开始S曲线插补循环 (calculatecommandstate)...\n");
    printf("      周期: 4ms (0.004s), 最大步数: 50000\n\n");

    struct sigaction sa, oldSa, oldBus;
    sa.sa_handler = signalHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGSEGV, &sa, &oldSa);
    sigaction(SIGBUS, &sa, &oldBus);

    int totalSteps = 0;
    double peakVel = 0, peakAcc = 0;
    bool motionStarted = false;

    // 创建输出文件
    FILE* fp = fopen("data/rml_scurve_profile.txt", "w");
    if (fp) {
        fprintf(fp, "# S-Curve Motion Profile (libCmpRML v142)\n");
        fprintf(fp, "# time_s q1 q2 q3 q4 q5 q6 v1 v2 v3 v4 v5 v6 a1 a2 a3 a4 a5 a6\n");
    }

    if (sigsetjmp(jumpBuf, 1) == 0) {
        for (int step = 0; step < 50000; step++) {
            if (componentEntry) {
                componentEntry();
            }

            rm_getsystemtime(fb_systime);
            readinterpolatecalctime(fb_itpcalc);

            // 确保运动标志保持为1 (calculatecommandstate内部可能清除它)
            if (*motion_flag == 0 && !motionStarted) {
                gettimeofday(&tv, NULL);
                now_us = (int64_t)tv.tv_sec * 1000000L + tv.tv_usec;
                now_sec = (double)now_us / 1e6;
                *base_ts = now_us;
                *start_time = now_sec;
                *motion_flag = 1;
            }

            // 模拟一个PLC周期: 生成运动 → 执行外部块 → 更新机器人状态 → 读命令状态
            fb_generate[0] = 0;
            fb_external[0] = 0;
            fb_upstate[0] = 0;
            fb_status[0] = 0;
            readfromlocaldatatogeneratemotion(fb_generate);
            externalblock(fb_external);
            rm_updaterobotstate(fb_upstate);
            rm_getmotionstatus(fb_status);
            rm_readmotionstate(fb_readstate);

            // 若仍未起动，周期性发送groupcontinue并重发movejoint触发
            if (!motionStarted && step > 0 && step % 100 == 0) {
                fb_groupctrl[0] = 0;
                externalblock__groupcontinue(fb_groupctrl);
                rm_movejoint(fb_movejoint);
            }

            // 调用 calculatecommandstate 读出插补数据
            fb_calcstate[0] = 0;  // groupIndex
            calculatecommandstate(fb_calcstate);

            // 读取输出位置/速度/加速度
            double pos[6], vel[6], acc[6];
            for (int i = 0; i < 6; i++) {
                pos[i] = getd(fb_calcstate, 0x08 + i * 8);
                vel[i] = getd(fb_calcstate, 0x38 + i * 8);
                acc[i] = getd(fb_calcstate, 0x68 + i * 8);
            }

            totalSteps++;
            double t = totalSteps * 4.0 / 1000.0;

            // 写入文件
            if (fp) {
                fprintf(fp, "%.4f", t);
                for (int i = 0; i < 6; i++) fprintf(fp, " %.6f", pos[i]);
                for (int i = 0; i < 6; i++) fprintf(fp, " %.6f", vel[i]);
                for (int i = 0; i < 6; i++) fprintf(fp, " %.6f", acc[i]);
                fprintf(fp, "\n");
            }

            // 检查是否有运动开始（速度不为0）
            for (int i = 0; i < 6; i++) {
                if (std::fabs(vel[i]) > 0.001) motionStarted = true;
                if (std::fabs(vel[i]) > peakVel) peakVel = std::fabs(vel[i]);
                if (std::fabs(acc[i]) > peakAcc) peakAcc = std::fabs(acc[i]);
            }

            // 每100步或特殊事件打印
            if (step < 5 || step % 200 == 0 || (step > 0 && !motionStarted && step < 20)) {
                printf("  [%5d] t=%.3fs pos=[%7.2f,%7.2f,%7.2f,%7.2f,%7.2f,%7.2f]"
                       " vel=[%7.1f,%7.1f,%7.1f,%7.1f,%7.1f,%7.1f] flag=%d\n",
                       step, t, pos[0], pos[1], pos[2], pos[3], pos[4], pos[5],
                       vel[0], vel[1], vel[2], vel[3], vel[4], vel[5],
                       (int)*motion_flag);
            }

            // 检查完成
            bool posReached = true;
            bool velZero = true;
            for (int i = 0; i < 6; i++) {
                if (std::fabs(pos[i] - targetPos[i]) > 0.01) posReached = false;
                if (std::fabs(vel[i]) > 0.01) velZero = false;
            }

            if (posReached && velZero && step > 10) {
                printf("\n  ✓ 运动完成! 步数=%d 时间=%.3fs\n", totalSteps, t);
                printf("    最终位置: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n",
                       pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
                printf("    峰值速度: %.2f deg/s\n", peakVel);
                printf("    峰值加速度: %.2f deg/s²\n", peakAcc);
                break;
            }

            // 如果超过500步还没有运动开始，可能需要不同方法
            if (step == 500 && !motionStarted) {
                printf("  [警告] 500步后无运动检测到，停止\n");
                printf("         这可能意味着初始化未完全成功\n");
                // 尝试直接打印 calcstate 的原始数据
                printf("  [调试] calcstate fb [0x00-0x0F]: ");
                for (int i = 0; i < 16; i++) printf("%02x ", fb_calcstate[i]);
                printf("\n  [调试] calcstate fb [0x08-0xC7]: ");
                for (int i = 0x08; i < 0x38; i += 8)
                    printf("%.4f ", getd(fb_calcstate, i));
                printf("\n");
                break;
            }

            // 真实4ms周期推进，模拟PLC循环
            usleep(4000);
        }
    } else {
        printf("  [错误] 崩溃于步骤 %d (signal %d)\n", totalSteps, (int)gotSignal);
    }

    if (fp) fclose(fp);
    sigaction(SIGSEGV, &oldSa, nullptr);
    sigaction(SIGBUS, &oldBus, nullptr);

    if (!motionStarted) {
        printf("\n  [状态] 运动未启动，库可能需要更多初始化调用\n");
        printf("         检查: 是否需要调用 externalblock() 或其他循环函数\n");
    }
}

// ============================================================================
// 测试2: 使用RMLController封装的完整运动
// ============================================================================
void testRMLControllerMotion() {
    printf("\n═══════════════════════════════════════════════════\n");
    printf("  测试2: RMLController 封装运动测试\n");
    printf("═══════════════════════════════════════════════════\n\n");

    // 使用堆分配避免栈溢出
    auto* rml = new RMLController();

    printf("  初始化 RMLController (S50参数)...\n");
    rml->initializeS50(0.004);
    printf("  设置倍率100%%...\n");
    rml->setOverride(100.0);

    double startPos[] = {0, -90, 0, 0, 90, 0};
    printf("  设置起始位置: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]\n",
           startPos[0], startPos[1], startPos[2],
           startPos[3], startPos[4], startPos[5]);
    rml->setStartPosition(startPos);

    double targetPos[] = {45, -60, 30, 0, 60, 0};
    printf("  发起运动 → 目标: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]\n",
           targetPos[0], targetPos[1], targetPos[2],
           targetPos[3], targetPos[4], targetPos[5]);
    rml->moveJoint(targetPos, 100.0);

    printf("\n  开始循环步进...\n");
    RMLController::StepResult result;
    int count = 0;

    struct sigaction sa, oldSa;
    sa.sa_handler = signalHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGSEGV, &sa, &oldSa);

    if (sigsetjmp(jumpBuf, 1) == 0) {
        while (rml->step(result) && count < 50000) {
            count++;
            if (count <= 5 || count % 200 == 0) {
                double t = count * 4.0 / 1000.0;
                printf("  [%5d] t=%.3fs pos=[%7.2f,%7.2f,%7.2f,%7.2f,%7.2f,%7.2f]\n",
                       count, t,
                       result.position[0], result.position[1],
                       result.position[2], result.position[3],
                       result.position[4], result.position[5]);
            }
        }
        printf("  最终步数: %d (%.3f s)\n", count, count * 4.0 / 1000.0);
        printf("  最终位置: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n",
               result.position[0], result.position[1],
               result.position[2], result.position[3],
               result.position[4], result.position[5]);
    } else {
        printf("  [错误] 崩溃于步骤 %d\n", count);
    }

    sigaction(SIGSEGV, &oldSa, nullptr);
    delete rml;
}

// ============================================================================
int main() {
    printf("╔═══════════════════════════════════════════════════════╗\n");
    printf("║   libCmpRML CoDeSys函数块接口验证                      ║\n");
    printf("║   反汇编逆向工程 → S曲线运动验证                        ║\n");
    printf("╚═══════════════════════════════════════════════════════╝\n");

    testRawMotionPipeline();
    // testRMLControllerMotion(); // 需要二次初始化，暂时跳过

    printf("\n════════════════════════════════════════════════════\n");
    printf("  探针测试完成\n");
    printf("════════════════════════════════════════════════════\n");
    return 0;
}
