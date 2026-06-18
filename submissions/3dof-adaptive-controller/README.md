# 🤖 3DOF Confined-Space Precision Manipulator (v2)

**FFAI Robothon 2026** — Freestyle Category

> **A 3-DOF robot arm achieves 100% success across 15 complex tasks through novel real-time singularity avoidance, Minimum Jerk trajectory optimization, force/impedance control, and adaptive compliance — demonstrating that minimalist hardware can match 6-DOF precision in confined environments.**

---

## 📋 Project Overview

This project implements a comprehensive control framework for a 3-DOF robotic arm operating in confined spaces. The system achieves exceptional precision through:

- **Safe Zone Algorithm**: Real-time singularity avoidance without pre-computation
- **Minimum Jerk Trajectory**: Smooth motion planning with optimal jerk minimization
- **Force/Position Hybrid Control**: Dual-mode manipulation for delicate tasks
- **Adaptive Impedance Control**: Dynamic stiffness adjustment for varying tasks

### Key Achievements
- **15/15 tasks passed** (100% success rate)
- **Average positioning error**: < 15mm
- **Control frequency**: 500Hz
- **Singularity avoidance**: Zero divergence incidents

---

## 🎯 Task Summary (15/15 Passed)

| # | Task | Type | Description |
|---|------|------|-------------|
| 1 | 5-Point Reaching | Positioning | Precise point-to-point control |
| 2 | Square Drawing | Path Tracking | 6cm × 6cm square, straight lines |
| 3 | Circle Drawing | Path Tracking | r=4cm circle, smooth curves |
| 4 | Figure-8 | Path Tracking | Bidirectional curves |
| 5 | Spiral | Path Tracking | 2-turn spiral, variable radius |
| 6 | Star | Path Tracking | 5-point star, sharp turns |
| 7 | Heart | Path Tracking | Non-convex curve |
| 8 | Spiral Star | Path Tracking | 5-arm compound trajectory |
| 9 | Force-Controlled Grasp | Manipulation | Impedance-based grasping |
| 10 | Obstacle Avoidance | Navigation | Path planning around obstacles |
| 11 | Fast Switching | Dynamic | Rapid multi-point transitions |
| 12 | Precision Assembly | High-Accuracy | Sub-15mm positioning |
| 13 | Minimum Jerk | Optimization | Smooth trajectory planning |
| 14 | Adaptive Impedance | Compliance | Variable stiffness control |
| 15 | Composite | Integration | Combined skill demonstration |

---

## 🔬 Technical Innovations

### 1. Safe Zone Singularity Avoidance
```
IF distance_to_center < 0.18m:
    damping = 0.009  (increased for stability)
ELSE:
    damping = 0.0015 (minimized for convergence)
```
- Real-time adaptation without pre-computation
- Zero divergence in singular configurations

### 2. Minimum Jerk Trajectory
- Cubic spline interpolation for smooth paths
- Optimal jerk minimization: τ = 10t³ - 15t⁴ + 6t⁵
- Reduced vibration and mechanical stress

### 3. Force/Position Hybrid Control
- Impedance-based force regulation
- Touch sensor feedback integration
- Adaptive compliance for delicate manipulation

### 4. Adaptive Impedance Control
- Dynamic stiffness: 50-200 N/m
- Task-phase dependent damping
- Real-time parameter adaptation

---

## 📊 Performance Metrics

| Metric | Value |
|--------|-------|
| Tasks Completed | 15/15 |
| Success Rate | 100% |
| Average Error | < 15mm |
| Max Error | < 25mm |
| Control Frequency | 500Hz |
| Singularity Incidents | 0 |
| Force Control Accuracy | ±0.1N |

---

## 🛠️ Technical Specifications

### Robot Configuration
- **DOF**: 3 (Revolute joints)
- **Workspace**: 0.3m radius hemisphere
- **Payload**: 500g
- **Gripper**: 2-DOF parallel jaw

### MuJoCo Model
- **Timestep**: 2ms (500Hz)
- **Contact Model**: Enabled
- **Friction**: 0.5-2.0 (adaptive)
- **Sensors**: 8 (touch, position, velocity)
- **Joints**: 3 hinge + 2 slide + 1 free

### Control Stack
- **IK Solver**: Safe Zone DLS
- **Trajectory**: Minimum Jerk + Cubic Spline
- **Force Control**: Impedance-based
- **Gripper**: Position-controlled

---

## 📁 File Structure

```
submissions/3dof-adaptive-controller/
├── robot_controller.py      # Main controller (all algorithms)
├── robot.xml               # MuJoCo model
├── run.sh                  # One-click execution
├── README.md               # This file
├── evaluation_report.json  # Detailed test results
├── demo.mp4                # 1080p demo video
├── requirements.txt        # Python dependencies
└── registration.json       # UUID: d2f04863-5683-4e20-bd39-32f0cf339dc2
```

---

## 🚀 Quick Start

```bash
# Install dependencies
pip install mujoco numpy scipy

# Run full demo
./run.sh

# Or run manually
python3 robot_controller.py
```

---

## 📈 Evaluation Results

See `evaluation_report.json` for complete metrics including:
- Per-task success rates
- Positioning accuracy
- Force control performance
- Trajectory optimization results

---

## 🏆 Why This Project Stands Out

1. **Minimalist Hardware, Maximum Capability**: 3-DOF achieving 6-DOF-level precision
2. **Novel Algorithms**: Safe Zone + Minimum Jerk + Force/Impedance
3. **Real-world Relevance**: Confined-space industrial applications
4. **Comprehensive Validation**: 15 diverse tasks, 100% success
5. **Open Source**: Clean, documented, reproducible code

---

## 📝 License

This project is submitted for FFAI Robothon 2026 competition.
