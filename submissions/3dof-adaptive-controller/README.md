# 🤖 3DOF Adaptive Robot Controller

**FFAI Robothon 2026** — Freestyle Category

## 项目摘要

一个自适应容错机器人控制系统，基于3DOF（三自由度）机械臂，使用数值逆运动学（IK）实现精确的多点位到达任务。

### 机器人平台
- **类型：** 3DOF串联机械臂 + 2指平行夹爪
- **模型：** 自定义MuJoCo MJCF模型
- **关节约束：** ±180°(底座旋转), ±120°(两段俯仰)

### 任务目标
5点到达任务：机械臂从初始位置精确移动到5个预设目标点，验证运动控制精度和稳定性。

### 技术方案
1. **数值逆运动学（IK）：** 基于Jacobian伪逆的DLS（Damped Least Squares）算法
2. **自适应增益：** 超高增益30.0 + 极小阻尼0.002，大幅提升收敛速度
3. **Safe Zone检测：** 检测关节接近奇异点时自动调整阻尼参数
4. **Motor Actuator控制：** 通过力矩控制实现平滑运动

### 核心特性
- ✅ 5/5目标全部到达（100%通过率）
- ✅ 平均误差18.8mm
- ✅ 平均290步收敛
- ✅ 无需外部依赖（纯MuJoCo + NumPy）

### 当前局限
- 仅测试了5点到达任务，未实现抓取/推动等复杂操控
- 3DOF机械臂自由度有限，无法实现任意姿态控制

### 未来改进
- 实现完整的抓取-放置任务
- 增加视觉反馈控制
- 优化轨迹规划

## 如何运行

### 环境要求
```bash
pip install mujoco numpy
```

### 运行步骤
```bash
# 克隆仓库
git clone https://github.com/jiayongzhang6-rgb/robothon-submission.git
cd robothon-submission

# 运行评估
python3 -c "
import sys; sys.path.insert(0, '.')
from robot_controller import RobotController
import numpy as np

robot = RobotController()
targets = [
    ('方块上方', np.array([0.3, 0, 0.5])),
    ('方块位置', np.array([0.3, 0, 0.4])),
    ('目标区域', np.array([-0.2, 0, 0.4])),
    ('左前方', np.array([0.15, 0, 0.5])),
    ('左上方', np.array([-0.15, 0, 0.5])),
]
for name, target in targets:
    robot.reset()
    ok, steps = robot.move_to(target)
    ee = robot.get_ee_pos()
    err = np.linalg.norm(target - ee) * 1000
    print(f'{\"✅\" if ok else \"❌\"} {name}: err={err:.1f}mm steps={steps}')
"
```

### 运行Demo视频
```bash
python3 record_demo.py
# 输出: /tmp/robothon_v8_demo.mp4
```

## Demo视频
见 `demo_video.mp4`（30秒，5点到达演示）

## 技术亮点
1. **极简设计：** 仅3个自由度实现精确3D空间定位
2. **高效IK：** 超高增益+自适应阻尼，290步内收敛
3. **鲁棒性：** Safe Zone检测避免奇异点问题
4. **零依赖：** 仅需MuJoCo和NumPy
