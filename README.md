# FFAI Robothon 2026 - 自由赛道项目

## 🎯 比赛目标

**赛道：Freestyle（自由方向）**

我们的目标是开发一个智能机器人控制系统，展示先进的运动规划和控制算法。

## 👤 参赛信息

- **参赛ID**: `d2f04863-5683-4e20-bd39-32f0cf339dc2`
- **配置文件**: `config.json`（提交脚本自动读取）

## 📁 项目结构

```
robothon_project/
├── robot_controller.py   # 核心控制器类
├── config.json          # 参赛配置
├── README.md            # 本文件
└── ...（后续添加更多模块）
```

## 🚀 第一个改进点

### 优化机械臂的平滑运动逻辑

**目标**: 实现关节空间的平滑轨迹生成，避免运动中的突变和抖动。

**计划尝试的方法**:
1. **五次多项式插值** - 保证位置、速度、加速度连续
2. **S曲线速度规划** - 平滑加减速过程
3. **关节空间避障** - 在运动规划中考虑碰撞检测

**预期效果**:
- 运动更平滑自然
- 减少机械磨损
- 提高任务完成精度

## 🔧 快速开始

```python
from robot_controller import RobotController
import numpy as np

# 初始化控制器
robot = RobotController()

# 重置环境
state = robot.reset()

# 执行控制动作
action = np.array([0.5, 0.3])  # 关节1和关节2的控制信号
state = robot.step(action)

# 获取当前状态
print(f"末端位置: {state['end_effector_pos']}")
print(f"关节角度: {state['joint_angles']}")
```

## 📝 开发日志

- **2026-06-14**: 项目初始化，验证 MuJoCo 环境，创建基础控制器类

## 📚 参考资料

- [MuJoCo 官方文档](https://mujoco.org/)
- [FFAI Robothon 2026 赛事规则]
