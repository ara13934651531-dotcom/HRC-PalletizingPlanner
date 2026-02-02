# 示例程序 | Examples

本目录包含 HRC 运动规划系统的使用示例。

## 📁 示例列表

| 文件 | 描述 |
|------|------|
| `basic_planning_example.cpp` | 基础点到点路径规划 |
| `palletizing_example.cpp` | 完整码垛任务规划示例 |

## 🔨 编译运行

```bash
# 在项目根目录
mkdir -p build && cd build
cmake ..
make

# 运行示例
./bin/basic_planning_example
./bin/palletizing_example
```

## 📝 快速开始

### 最简示例

```cpp
#include "PalletizingPlanner/PalletizingPlanner.hpp"

using namespace palletizing;

int main() {
    // 1. 创建并初始化规划器
    PalletizingPlanner planner;
    planner.initialize();

    // 2. 定义起点和终点
    JointConfig start = JointConfig::fromDegrees({0, -90, 30, 0, -60, 0});
    JointConfig goal = JointConfig::fromDegrees({45, -60, 45, 30, -45, 45});

    // 3. 执行规划
    PlanningResult result = planner.planPointToPoint(start, goal);

    // 4. 使用结果
    if (result.isSuccess()) {
        for (const auto& waypoint : result.optimizedPath) {
            // 发送到机器人控制器...
        }
    }

    return 0;
}
```

---

**Huayan Robotics** | https://www.huayan-robotics.com
