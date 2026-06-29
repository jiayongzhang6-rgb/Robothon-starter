# Robot Orchestra — Multi-Agent Rhythmic Coordination

**UUID:** `d2f04863-5683-4e20-bd39-32f0cf339dc2`

**Team:** 3DOF

## 项目概述

Robot Orchestra 是一个基于 MuJoCo 的多机器人协调演奏仿真系统。三个独立的机械臂分别操控鼓组、木琴和打击乐器，通过闭环PD控制实现节奏同步的协调演奏。

### 核心创新

1. **多智能体协调** — 3个独立机械臂同步演奏4种乐器
2. **闭环PD控制** — 每个关节独立的PD控制器，精确轨迹跟踪
3. **节奏同步** — 120 BPM节拍驱动，所有臂协调动作
4. **力反馈** — 触觉传感器检测打击接触

### 技术架构

```
robot-orchestra/
├── scene.xml           # MuJoCo场景（3臂+4乐器）
├── demo.mp4            # 23.7秒演示视频（1280×720）
├── registration.json   # 参赛信息
├── README.md           # 项目文档
└── engine/
    └── simulator.py    # 仿真引擎+渲染
```

### 演奏编排（23.7秒）

| 时间 | 阶段 | 乐器 | 动态 |
|------|------|------|------|
| 0-2s | 开场 | 全体就位 | - |
| 2-6s | Intro | 鼓+木琴 | pp (轻柔) |
| 6-10s | Verse | 全部4乐器 | mf (中强) |
| 10-16s | Chorus | 全部4乐器 | f (强) |
| 16-20s | Finale | 鼓+木琴 | mp (中弱) |
| 20-24s | 结束 | 全体亮相 | - |

### 乐器配置

| 机械臂 | 乐器 | 关节数 | 控制方式 |
|--------|------|--------|----------|
| 鼓手 (左) | 军鼓+底鼓+踩镲 | 3-DOF | PD闭环 |
| 木琴手 (中) | 5音木琴 (C-D-E-F-G) | 3-DOF | PD闭环 |
| 打击乐手 (右) | 镲片+三角铁 | 3-DOF | PD闭环 |

### 控制系统

- **关节控制**: 独立PD控制器 (Kp=80-150, Kd=10-15)
- **轨迹规划**: 余弦插值平滑运动
- **节奏驱动**: 120 BPM时钟，每拍12帧
- **触觉反馈**: 3个打击检测传感器

### 评分对齐

| 评分维度 | 本项目实现 |
|----------|-----------|
| Perceived Intelligence | 多臂协调、节奏同步 |
| Causal Clarity | 清晰的4阶段演奏流程 |
| Predictive Behavior | 节拍驱动的确定性动作 |
| Visual Quality | 动态HUD、节拍进度条 |
| Technical Depth | 3臂9关节PD控制 |

### 运行

```bash
pip install mujoco opencv-python imageio numpy
python engine/simulator.py
```

### 输出

- 分辨率: 1280×720
- 帧率: 30fps
- 时长: 23.7秒
- 文件大小: 5.5MB
