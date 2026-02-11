/**
 * @file palletizing_example.cpp
 * @brief 码垛任务规划示例 | Palletizing Task Planning Example
 * 
 * @copyright Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.
 * @author Huayan Robotics
 * @contact yuesj@huayan-robotics.com
 * @website https://www.huayan-robotics.com
 */

#include "PalletizingPlanner/PalletizingPlanner.hpp"
#include "PalletizingPlanner/TaskSequencer.hpp"
#include <iostream>
#include <iomanip>
#include <vector>

using namespace palletizing;

int main() {
    std::cout << "╔══════════════════════════════════════════════════════════╗\n";
    std::cout << "║   Huayan Robotics - Palletizing Example                  ║\n";
    std::cout << "║   Elfin Series Collaborative Robot                       ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════╝\n\n";

    // 1. 创建规划器
    PalletizingPlanner planner;
    planner.initialize();

    // 2. 定义码垛目标点 (3x3 网格)
    std::vector<JointConfig> pickPoints;
    std::vector<JointConfig> placePoints;

    // 取料点 - 单一位置
    JointConfig pickPoint = JointConfig::fromDegrees({0, -60, 45, 0, -75, 0});

    // 放料点 - 3x3 网格 (9个位置)
    double baseAngles[9][6] = {
        {30, -45, 30, 0, -75, 30},   // 位置 1
        {35, -45, 30, 0, -75, 35},   // 位置 2
        {40, -45, 30, 0, -75, 40},   // 位置 3
        {30, -50, 35, 0, -75, 30},   // 位置 4
        {35, -50, 35, 0, -75, 35},   // 位置 5
        {40, -50, 35, 0, -75, 40},   // 位置 6
        {30, -55, 40, 0, -75, 30},   // 位置 7
        {35, -55, 40, 0, -75, 35},   // 位置 8
        {40, -55, 40, 0, -75, 40},   // 位置 9
    };

    std::cout << "📦 码垛任务配置:\n";
    std::cout << "   ├─ 取料点: 1 个\n";
    std::cout << "   └─ 放料点: 9 个 (3x3 网格)\n\n";

    for (int i = 0; i < 9; ++i) {
        pickPoints.push_back(pickPoint);
        placePoints.push_back(JointConfig::fromDegrees({
            baseAngles[i][0], baseAngles[i][1], baseAngles[i][2],
            baseAngles[i][3], baseAngles[i][4], baseAngles[i][5]
        }));
    }

    // 3. 使用 TSP 优化任务序列
    std::cout << "🔄 优化码垛任务序列...\n";
    
    // 构建PalletizingTask列表
    std::vector<PalletizingTask> tasks;
    for (int i = 0; i < 9; ++i) {
        PalletizingTask task;
        task.taskId = i;
        task.pickConfig = pickPoints[i];
        task.placeConfig = placePoints[i];
        task.description = "Task " + std::to_string(i + 1);
        tasks.push_back(task);
    }
    
    TaskSequencer sequencer(planner);
    JointConfig homeConfig = JointConfig::fromDegrees({0, -60, 45, 0, -75, 0});
    auto optimizedTasks = sequencer.optimizeSequence(tasks, homeConfig);
    
    std::cout << "   优化后顺序: ";
    for (size_t i = 0; i < optimizedTasks.size(); ++i) {
        std::cout << optimizedTasks[i].taskId + 1;
        if (i < optimizedTasks.size() - 1) std::cout << " → ";
    }
    std::cout << "\n\n";

    // 4. 执行规划
    std::cout << "🚀 开始码垛规划...\n";
    std::cout << "───────────────────────────────────────────────────────────\n";

    int successCount = 0;
    double totalTime = 0.0;

    for (size_t i = 0; i < 3 && i < optimizedTasks.size(); ++i) {  // 演示前3个任务
        const auto& task = optimizedTasks[i];
        
        auto startTime = std::chrono::high_resolution_clock::now();
        
        // 规划 取料点 → 放料点
        PlanningResult result = planner.planPointToPoint(
            task.pickConfig, 
            task.placeConfig
        );
        
        auto endTime = std::chrono::high_resolution_clock::now();
        double planTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
        totalTime += planTime;

        if (result.isSuccess()) {
            successCount++;
            std::cout << "   ✅ 任务 " << task.taskId + 1 << ": " 
                      << std::fixed << std::setprecision(1) << planTime << " ms, "
                      << result.optimizedPath.size() << " 路径点\n";
        } else {
            std::cout << "   ❌ 任务 " << task.taskId + 1 << ": 规划失败\n";
        }
    }

    // 5. 输出统计
    std::cout << "───────────────────────────────────────────────────────────\n";
    std::cout << "\n📊 统计结果:\n";
    std::cout << "   ├─ 成功率: " << successCount << "/3 (" 
              << std::fixed << std::setprecision(1) 
              << (successCount * 100.0 / 3) << "%)\n";
    std::cout << "   ├─ 总耗时: " << std::fixed << std::setprecision(1) 
              << totalTime << " ms\n";
    std::cout << "   └─ 平均耗时: " << std::fixed << std::setprecision(1) 
              << totalTime / 3 << " ms/任务\n";

    std::cout << "\n══════════════════════════════════════════════════════════\n";
    std::cout << "  🤖 Huayan Robotics | 用机器人技术为人类服务\n";
    std::cout << "══════════════════════════════════════════════════════════\n";

    return 0;
}
