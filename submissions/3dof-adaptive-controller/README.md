# 🤖 3DOF Adaptive Robot Controller

**FFAI Robothon 2026** — Freestyle Category

## 📋 项目摘要

一个**自适应容错机器人控制系统**，基于3DOF（三自由度）机械臂，通过DLS逆运动学和Safe Zone检测，在MuJoCo仿真中实现精确的多点位到达任务。核心创新在于**3DOF机械臂的自适应控制策略**——通过动态增益调度和奇异点规避，在受限自由度下实现高精度运动控制。

### 机器人平台
- **类型：** 3DOF串联机械臂 + 2指平行夹爪
- **模型：** MuJoCo MJCF（独立 `robot.xml` 文件）
- **关节约束：** ±180°(底座旋转), ±135°(肩部俯仰), ±150°(肘部俯仰)
- **传感器：** 触觉传感器 + 末端执行器位置 + 方块位置

### 任务目标
5点到达任务：机械臂从初始位置精确移动到5个预设目标点，验证运动控制精度和稳定性。

### 技术方案
1. **DLS逆运动学：** Jacobian伪逆 + 阻尼最小二乘（Damped Least Squares）
2. **自适应增益：** 超高增益30.0 + 极小阻尼0.002，大幅提升收敛速度
3. **Safe Zone检测：** 检测关节接近奇异点时自动3倍调整阻尼参数
4. **Motor Actuator控制：** 通过力矩控制实现平滑运动

### 核心特性
- ✅ 5/5目标全部到达（100%通过率）
- ✅ 平均误差18.8mm（阈值20mm内）
- ✅ 平均290步收敛（高效控制）
- ✅ 纯MuJoCo + NumPy，无外部依赖

### 当前局限
- 仅测试了5点到达任务，未实现抓取/推动等复杂操控
- 3DOF自由度限制了工作空间范围

### 未来改进
- 增加推块任务（A→B操控）
- 优化3DOF下的灵巧操控策略
- 添加遥操作界面（键盘/手柄控制）

## 🚀 运行方式

### 快速开始
```bash
# 安装依赖
pip install mujoco numpy

# 运行评估
python3 robot_controller.py

# 录制Demo视频
python3 record_demo_v2.py
```

### 手动测试
```python
from robot_controller import RobotController
import numpy as np

ctrl = RobotController()
ctrl.reset()
trajectory = ctrl.move_to(np.array([0.3, 0, 0.5]))
print(f"Final position: {ctrl.get_ee_pos()}")
```

## 📊 评估结果

| 指标 | 得分 |
|------|------|
| 到达成功率 | 100% (5/5) |
| 平均精度 | 81.2 |
| 效率 | 75.8 |
| **综合分数** | **85.7** |

### 详细测试结果

| 目标 | 目标位置 | 实际位置 | 误差 | 步数 | 状态 |
|------|----------|----------|------|------|------|
| T1: Forward | [0.3, 0, 0.5] | [0.30, 0, 0.52] | 16.9mm | 190 | ✅ |
| T2: Precise | [0.3, 0, 0.4] | [0.30, 0, 0.42] | 19.7mm | 223 | ✅ |
| T3: Lateral | [-0.2, 0, 0.4] | [-0.21, 0, 0.42] | 19.5mm | 311 | ✅ |
| T4: Diagonal | [0.15, 0, 0.5] | [0.15, 0, 0.52] | 18.8mm | 364 | ✅ |
| T5: Wide | [-0.15, 0, 0.5] | [-0.15, 0, 0.52] | 19.1mm | 364 | ✅ |

## 🔬 MuJoCo深度

### MJCF模型配置
- **独立XML文件：** `robot.xml`（规范的MuJoCo场景文件）
- **物理引擎：** timestep=2ms, 重力=-9.81m/s², 碰撞启用
- **关节：** 3个hinge关节 + 2个slide夹爪关节
- **执行器：** 3个motor（力矩控制）+ 2个position（夹爪）
- **传感器：** 触觉传感器 + 末端位置 + 方块位置

### 控制频率
- 仿真频率：500Hz（2ms timestep）
- 控制循环：每步1次IK计算

## 💡 创新点

### 1. 自适应Safe Zone控制
传统IK在奇异点附近发散。我们通过**实时检测末端执行器与奇异点的距离**，动态调整阻尼参数：
- 距离 < 0.18m：阻尼×3（0.01），防止发散
- 距离 ≥ 0.18m：阻尼=0.002，最大化收敛速度

### 2. 超高增益策略
通过实验验证，**增益30.0**在3DOF模型下达到最优平衡：
- 收敛速度：平均290步（约0.6秒仿真时间）
- 稳定性：100%成功率，无振荡

### 3. 3DOF受限下的高精度控制
在仅3个自由度的限制下，通过DLS算法实现平均18.8mm精度，证明了**自适应控制策略在受限机械臂上的有效性**。

## 📁 项目结构

```
submissions/3dof-adaptive-controller/
├── robot.xml              # MuJoCo MJCF模型（独立文件）
├── robot_controller.py    # 核心控制器（IK + 自适应增益）
├── record_demo_v2.py      # Demo录制脚本
├── demo.mp4               # Demo视频（67秒）
├── registration.json      # UUID注册
├── README.md              # 项目说明
├── config.json            # 竞赛配置
└── evaluation_report.json # 评估报告
```

## 🔧 技术规格

| 参数 | 值 |
|------|-----|
| 机器人模型 | 3DOF Articulated Arm |
| 控制频率 | 500 Hz |
| IK求解器 | DLS (λ=0.002) |
| 增益 | 30.0 |
| 收敛阈值 | 0.02m |
| 最大迭代 | 1200步 |

---

**参赛者：** jiayongzhang6-rgb  
**UUID：** d2f04863-5683-4e20-bd39-32f0cf339dc2  
**团队：** Hermes Robothon Team
