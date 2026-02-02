#include <InterfaceDataStruct.h>
#include <algorithmLibInterface.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <sys/resource.h>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <string>
#include <iomanip>
#include <cstdint>  // 新增：用于uintptr_t类型

// -------------------------- 新增：精确堆栈统计（字节单位） --------------------------
static uintptr_t initial_stack_ptr = 0;  // 主线程栈底基准（程序启动时记录）

// 获取当前栈指针（x86_64架构，GCC编译器）
inline uintptr_t get_current_stack_ptr() {
    uintptr_t sp;
    // 汇编指令读取x86_64栈指针寄存器%rsp（直接获取当前栈顶地址）
    __asm__ __volatile__("mov %%rsp, %0" : "=r"(sp));
    return sp;
}

// 初始化栈底基准（必须在程序启动早期调用，此时栈使用量最小）
void init_stack_benchmark() {
    initial_stack_ptr = get_current_stack_ptr();
}

// 计算当前栈的实际使用量（单位：字节Byte）
size_t get_actual_stack_usage_byte() {
    if (initial_stack_ptr == 0) {
        std::cerr << "错误：未初始化栈基准，请先调用init_stack_benchmark()" << std::endl;
        return 0;
    }

    uintptr_t current_sp = get_current_stack_ptr();
    // 栈从高地址向低地址生长，使用量 = 栈底地址 - 当前栈顶地址（取绝对值避免异常架构误差）
    size_t usage = (initial_stack_ptr > current_sp) 
                 ? (initial_stack_ptr - current_sp) 
                 : (current_sp - initial_stack_ptr);
    
    return usage;  // 直接返回字节数，不做KB转换
}
// -------------------------- 精确堆栈统计结束 --------------------------

// 获取当前进程的内存使用情况（KB）
size_t getMemoryUsage() {
    std::ifstream statusFile("/proc/self/status");
    std::string line;
    size_t vmSize = 0; // 虚拟内存大小
    size_t vmRSS = 0;  // 常驻内存大小
    
    while (std::getline(statusFile, line)) {
        if (line.find("VmSize:") != std::string::npos) {
            std::istringstream iss(line);
            std::string key;
            iss >> key >> vmSize;
        } else if (line.find("VmRSS:") != std::string::npos) {
            std::istringstream iss(line);
            std::string key;
            iss >> key >> vmRSS;
        }
    }
    
    // 返回常驻内存大小（更准确反映实际使用的物理内存）
    return vmRSS;
}

// 原堆栈估算函数（修改为字节单位，与精确统计统一）
size_t getStackUsageApprox_byte() {
    char stack_probe[1024];  // 探测栈指针的临时数组
    void* stack_ptr = &stack_probe;  // 当前栈内地址（用于计算）
    
    std::ifstream mapsFile("/proc/self/maps");
    std::string line;
    size_t stack_start = 0;  // 栈段起始地址（低地址）
    size_t stack_end = 0;    // 栈段结束地址（高地址）
    
    while (std::getline(mapsFile, line)) {
        if (line.find("[stack]") != std::string::npos) {
            std::istringstream iss(line);
            std::string range;
            iss >> range;
            size_t dash_pos = range.find('-');
            // 十六进制转十进制，获取栈段地址范围（单位：字节）
            stack_start = std::stoul(range.substr(0, dash_pos), nullptr, 16);
            stack_end = std::stoul(range.substr(dash_pos + 1), nullptr, 16);
            break;
        }
    }
    
    if (stack_start && stack_end) {
        size_t current_stack_addr = reinterpret_cast<size_t>(stack_ptr);
        // 估算栈使用量 = 栈段结束地址 - 当前栈内地址（单位：字节）
        return stack_end - current_stack_addr;
    }
    
    return 0;  // 未找到栈段时返回0
}


// 获取系统CPU利用率
double getCpuUsage() {
    static uint64_t lastTotalUser = 0, lastTotalUserLow = 0;
    static uint64_t lastTotalSys = 0, lastTotalIdle = 0;

    double percent;
    FILE* file;
    uint64_t totalUser, totalUserLow, totalSys, totalIdle, total;

    file = fopen("/proc/stat", "r");
    fscanf(file, "cpu %llu %llu %llu %llu", &totalUser, &totalUserLow,
           &totalSys, &totalIdle);
    fclose(file);

    if (lastTotalUser == 0) {
        lastTotalUser = totalUser;
        lastTotalUserLow = totalUserLow;
        lastTotalSys = totalSys;
        lastTotalIdle = totalIdle;
        return 0.0;
    }

    total = (totalUser - lastTotalUser) + (totalUserLow - lastTotalUserLow) +
            (totalSys - lastTotalSys);
    percent = total;
    total += (totalIdle - lastTotalIdle);
    percent = total ? (percent / total) * 100 : 0.0;

    lastTotalUser = totalUser;
    lastTotalUserLow = totalUserLow;
    lastTotalSys = totalSys;
    lastTotalIdle = totalIdle;

    return percent;
}

// 获取当前进程的CPU利用率
double getProcessCpuUsage() {
    static uint64_t lastUtime = 0, lastStime = 0;
    static auto lastTime = std::chrono::steady_clock::now();
    
    std::ifstream statFile("/proc/self/stat");
    std::string line;
    std::getline(statFile, line);
    std::istringstream iss(line);
    
    // 跳过前13个字段
    std::string dummy;
    for (int i = 0; i < 13; ++i) iss >> dummy;
    
    uint64_t utime, stime;
    iss >> utime >> stime;
    
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime).count() / 1000.0;
    lastTime = now;
    
    if (lastUtime == 0) {
        lastUtime = utime;
        lastStime = stime;
        return 0.0;
    }
    
    uint64_t totalTime = (utime - lastUtime) + (stime - lastStime);
    lastUtime = utime;
    lastStime = stime;
    
    // 获取系统时钟滴答频率
    long clockTicks = sysconf(_SC_CLK_TCK);
    
    return (totalTime / static_cast<double>(clockTicks)) / elapsed * 100.0;
}

int main()
{
    // 1. 初始化栈基准（程序最开始调用，确保栈使用量最小，统计最准确）
    init_stack_benchmark();

    // 2. 机器人模型初始化（保留原逻辑）
    RTS_IEC_INT robType = 0;  // 设置为 Elfin机器人类型
    RTS_IEC_LREAL dh[8] = {0.220, 0.420, 0.1565, 0.380};  // DH参数
    RTS_IEC_LREAL baseGeometry[7] = {0.00, 0.00, 0.02, 0.00, 0.00, 0.23, 0.11};    // 基座几何参数
    RTS_IEC_LREAL lowerArmGeometry[7] = {0.00, 0.00, 0.22, 0.85, 0.00, 0.22, 0.11};// 下臂几何参数
    RTS_IEC_LREAL elbowGeometry[7] = {-0.01, 0.00, 0.06, 0.71, 0.00, 0.06, 0.08};  // 肘部几何参数
    RTS_IEC_LREAL upperArmGeometry[7] = {0.00, 0.00, -0.05, 0.00, 0.00, 0.10, 0.05};// 上臂几何参数
    RTS_IEC_LREAL wristGeometry[4] = {0.00, 0.00, 0.02, 0.10};  // 腕部几何参数
    RTS_IEC_LREAL jointPos[6] = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00};  // 初始关节位置
    
    // 初始化碰撞检测包
    initACAreaConstrainPackageInterface(robType, dh, baseGeometry, lowerArmGeometry, 
                                        elbowGeometry, upperArmGeometry, wristGeometry, jointPos);

    // 3. 碰撞检测相关变量初始化
    RTS_IEC_LREAL jointVelocity[6] = {0, 0, 0, 0, 0, 0};  // 关节速度（初始为0）
    RTS_IEC_LINT colliderPair[2];  // 碰撞对索引
    RTS_IEC_LREAL distance;        // 碰撞距离


    // 4. 资源监控初始化：记录初始状态（堆栈用字节，内存用KB）
    std::cout << "==================== 资源监控启动 ====================" << std::endl;
    size_t initialMemoryUsage = getMemoryUsage();                  // 初始内存（KB）
    size_t initialStackApprox = getStackUsageApprox_byte();        // 初始堆栈估算值（字节）
    size_t initialStackActual = get_actual_stack_usage_byte();     // 初始堆栈精确值（字节）
    
    std::cout << "初始状态：" << std::endl;
    std::cout << "  - 物理内存使用量：" << initialMemoryUsage << " KB" << std::endl;
    std::cout << "  - 堆栈估算使用量：" << initialStackApprox << " Byte" << std::endl;
    std::cout << "  - 堆栈精确使用量：" << initialStackActual << " Byte" << std::endl;
    std::cout << "======================================================" << std::endl;

    // 5. 碰撞检测循环（10次迭代，每2次输出一次资源状态）
    for (int i = 0; i < 100000; i++) {
        // 更新关节位置和速度，执行自碰撞检测
        updateACAreaConstrainPackageInterface(jointPos, jointVelocity);
        RTS_BOOL ret = checkCPSelfCollisionInterface(colliderPair, &distance);
        
        // 每2次迭代输出资源监控结果（避免输出过于频繁）
        if (i % 10000 == 0) {
            size_t currentMemory = getMemoryUsage();                  // 当前内存（KB）
            size_t currentStackApprox = getStackUsageApprox_byte();   // 当前堆栈估算（字节）
            size_t currentStackActual = get_actual_stack_usage_byte();// 当前堆栈精确（字节）
            double sysCpu = getCpuUsage();                            // 系统CPU利用率（%）
            double procCpu = getProcessCpuUsage();                   // 进程CPU利用率（%）
            
            std::cout << "迭代次数：" << i 
                      << " | 内存：" << currentMemory << " KB"
                      << " | 栈估算：" << currentStackApprox << " Byte"
                      << " | 栈精确：" << currentStackActual << " Byte"
                      << " | 系统CPU：" << std::fixed << std::setprecision(1) << sysCpu << "%"
                      << " | 进程CPU：" << std::fixed << std::setprecision(1) << procCpu << "%" 
                      << std::endl;
        }
    }

    // 6. 输出最终统计结果
    std::cout << "======================================================" << std::endl;
    
    size_t finalMemory = getMemoryUsage();                  // 最终内存（KB）
    size_t finalStackApprox = getStackUsageApprox_byte();   // 最终堆栈估算（字节）
    size_t finalStackActual = get_actual_stack_usage_byte();// 最终堆栈精确（字节）
    
    std::cout << "最终状态 & 变化量：" << std::endl;
    std::cout << "  - 物理内存：" << finalMemory << " KB（变化：" << (finalMemory - initialMemoryUsage) << " KB）" << std::endl;
    std::cout << "  - 堆栈估算：" << finalStackApprox << " Byte（变化：" << (finalStackApprox - initialStackApprox) << " Byte）" << std::endl;
    std::cout << "  - 堆栈精确：" << finalStackActual << " Byte（变化：" << (finalStackActual - initialStackActual) << " Byte）" << std::endl;
    std::cout << "======================================================" << std::endl;

    return 0;
}