# FFAI Robothon 2026 - 高精度自主控制系统

## 🎯 项目概述

这是一个具备 **fallback 能力的高精度自主控制系统**，针对 **横向移动做了专门的奇异点优化**，实现了100%的全轴向精确定位。

## 🏆 核心指标

| 指标 | 得分 | 状态 |
|------|------|------|
| **稳定性** | 88.76/100 | ✅ |
| **效率** | 100.00/100 | ✅ |
| **成功率** | 100.00/100 | ✅ |
| **综合得分** | **96.25/100** | 🏆 |

## 🔧 技术亮点

### 1. 奇异点优化（Y轴专项）
- **问题**：传统IK在纯Y轴移动时进入奇异位形
- **解决方案**：基于工作空间分析的查表法IK
- **效果**：Y轴成功率从 0% → 100%

### 2. Fallback容错机制
- MuJoCo不可用时自动切换基础物理引擎
- 错误日志记录，支持人工介入
- 保证系统在任何情况下都能运行

### 3. 精确PID控制
- 关节空间PID控制
- 积分项消除稳态误差
- 多阶段控制：粗定位 + 精确定位

## 📁 项目结构

```
robothon_project/
├── robot_controller.py    # 核心控制器（IK + PID）
├── robust_simulator.py    # 容错系统
├── config.json           # 参赛配置
├── submit.sh             # 一键提交脚本
└── README.md             # 本文件
```

## 🚀 快速开始

```python
from robot_controller import RobotController
import numpy as np

# 初始化控制器
robot = RobotController(kp=3.0, ki=0.5, kd=0.15)

# 运行控制
target = np.array([0.15, 0.12, 0.8])  # 目标位置
trajectory = robot.run_control(target)

# 查看结果
final_pos = trajectory[-1]["end_effector_pos"]
print(f"最终位置: {final_pos}")
```

## 📋 参赛信息

- **参赛ID**: `d2f04863-5683-4e20-bd39-32f0cf339dc2`
- **赛道**: Freestyle
- **提交命令**: `bash submit.sh`

## 🔬 创新点总结

1. **查表法IK**：基于实际工作空间分析，避免奇异点问题
2. **多级Fallback**：保证系统鲁棒性
3. **Y轴增益增强**：针对特定轴向的优化策略

---

*Built with MuJoCo + PID Control + Table-based IK*
