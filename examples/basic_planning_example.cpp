/**
 * @file basic_planning_example.cpp
 * @brief 基础路径规划示例 | Basic Path Planning Example
 * 
 * @copyright Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.
 * @author Huayan Robotics
 * @contact yuesj@huayan-robotics.com
 * @website https://www.huayan-robotics.com
 */

#include "PalletizingPlanner/PalletizingPlanner.hpp"
#include <iostream>
#include <iomanip>

using namespace palletizing;

int main() {
    std::cout << "╔══════════════════════════════════════════════════════════╗\n";
    std::cout << "║   Huayan Robotics - Motion Planning Example              ║\n";
    std::cout << "║   https://www.huayan-robotics.com                        ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════╝\n\n";

    // 1. 创建规划器实例
    std::cout << "📦 初始化规划器...\n";
    PalletizingPlanner planner;
    
    if (!planner.initialize()) {
        std::cerr << "❌ 规划器初始化失败!\n";
        return 1;
    }
    std::cout << "✅ 规划器初始化成功\n\n";

    // 2. 定义起始和目标关节配置 (单位: 度)
    JointConfig start = JointConfig::fromDegrees({0, -90, 30, 0, -60, 0});
    JointConfig goal = JointConfig::fromDegrees({45, -60, 45, 30, -45, 45});

    std::cout << "📍 起始位置 (deg): ";
    for (int i = 0; i < 6; ++i) std::cout << std::fixed << std::setprecision(1) 
                                          << start.q[i] * 180.0 / M_PI << " ";
    std::cout << "\n";

    std::cout << "🎯 目标位置 (deg): ";
    for (int i = 0; i < 6; ++i) std::cout << std::fixed << std::setprecision(1) 
                                          << goal.q[i] * 180.0 / M_PI << " ";
    std::cout << "\n\n";

    // 3. 执行路径规划
    std::cout << "🚀 开始规划...\n";
    auto startTime = std::chrono::high_resolution_clock::now();
    
    PlanningResult result = planner.planPointToPoint(start, goal);
    
    auto endTime = std::chrono::high_resolution_clock::now();
    double planningTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();

    // 4. 输出结果
    std::cout << "\n═══════════════════════════════════════════════════════════\n";
    if (result.isSuccess()) {
        std::cout << "✅ 规划成功!\n";
        std::cout << "   ├─ 规划时间: " << std::fixed << std::setprecision(2) 
                  << planningTime << " ms\n";
        std::cout << "   ├─ 路径点数: " << result.optimizedPath.size() << "\n";
        std::cout << "   └─ 路径长度: " << std::fixed << std::setprecision(4) 
                  << result.pathLength << " rad\n";
    } else {
        std::cout << "❌ 规划失败: " << result.errorMessage << "\n";
    }
    std::cout << "═══════════════════════════════════════════════════════════\n";

    return result.isSuccess() ? 0 : 1;
}
