# 贡献指南 | Contributing Guidelines

感谢您对 HRC 协作机器人运动规划系统的关注！我们欢迎所有形式的贡献。

Thank you for your interest in the HRC Collaborative Robot Motion Planning System! We welcome all forms of contributions.

---

## 🌟 如何贡献 | How to Contribute

### 报告问题 | Reporting Issues

1. 在创建新 Issue 之前，请先搜索现有 Issue
2. 使用清晰、描述性的标题
3. 提供详细的问题描述和复现步骤
4. 包含系统信息（操作系统、编译器版本等）

### 提交代码 | Submitting Code

#### 1. Fork 仓库
```bash
git clone https://github.com/huayan-robotics/HRC-PalletizingPlanner.git
cd HRC-PalletizingPlanner
```

#### 2. 创建特性分支
```bash
git checkout -b feature/your-feature-name
```

#### 3. 编写代码并测试
```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
HRC_LIB_PATH=../lib/libHRCInterface.so ../bin/testS50PalletizingSO
```

#### 4. 提交更改
```bash
git add .
git commit -m "feat: add your feature description"
```

#### 5. 推送并创建 PR
```bash
git push origin feature/your-feature-name
```

---

## 📝 代码风格 | Code Style

### C++ 代码规范

- 使用 **C++17** 标准
- 缩进使用 **4 个空格**
- 文件名使用 **PascalCase**（如 `PathPlanner.hpp`）
- 类名使用 **PascalCase**
- 函数名使用 **camelCase**
- 常量使用 **UPPER_SNAKE_CASE**
- 所有代码放在 `palletizing` 命名空间内

### 示例

```cpp
namespace palletizing {

class PathPlanner {
public:
    static constexpr double DEFAULT_STEP_SIZE = 0.1;
    
    bool planPath(const JointConfig& start, const JointConfig& goal);
    
private:
    double stepSize_;
};

} // namespace palletizing
```

---

## 📋 提交信息规范 | Commit Message Convention

使用 [Conventional Commits](https://www.conventionalcommits.org/) 规范：

```
<type>(<scope>): <description>

[optional body]

[optional footer]
```

### Type 类型

| Type | Description |
|------|-------------|
| `feat` | 新功能 |
| `fix` | Bug 修复 |
| `docs` | 文档更新 |
| `style` | 代码格式调整 |
| `refactor` | 代码重构 |
| `perf` | 性能优化 |
| `test` | 测试相关 |
| `chore` | 构建/工具相关 |

### 示例

```
feat(planner): add BIT* algorithm support

- Implemented Batch Informed Trees algorithm
- Added lazy collision checking
- Improved planning speed by 30%

Closes #123
```

---

## ✅ Pull Request 检查清单 | PR Checklist

- [ ] 代码已编译通过
- [ ] 所有测试通过
- [ ] 添加了必要的测试
- [ ] 更新了相关文档
- [ ] 提交信息符合规范
- [ ] PR 描述清晰完整

---

## 🔒 行为准则 | Code of Conduct

请尊重所有贡献者，保持友好、专业的交流氛围。

---

## 📞 联系我们 | Contact

- **Email**: yuesj@huayan-robotics.com
- **Website**: https://www.huayan-robotics.com
- **Issues**: [GitHub Issues](https://github.com/huayan-robotics/HRC-PalletizingPlanner/issues)

---

<p align="center">
  <b>🤖 广东华沿机器人有限公司 | Guangdong Huayan Robotics Co., Ltd.</b><br>
  <i>用机器人技术为人类服务</i>
</p>
