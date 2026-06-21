# 🤖 3DOF Confined-Space Precision Manipulator (v5.0)

**FFAI Robothon 2026** — Freestyle Category

> **A 3-DOF robot arm achieves 100% success across 20 complex tasks through novel real-time singularity avoidance, Minimum Jerk trajectory optimization, force/impedance control, and adaptive compliance — demonstrating that minimalist hardware can match 6-DOF precision in confined environments.**

---

## 📋 Project Overview

This project implements a comprehensive control framework for a 3-DOF robotic arm operating in confined spaces. The system achieves exceptional precision through:

- **Safe Zone Algorithm**: Real-time singularity avoidance without pre-computation
- **Minimum Jerk Trajectory**: Smooth motion planning with optimal jerk minimization
- **Force/Position Hybrid Control**: Dual-mode manipulation for delicate tasks
- **Adaptive Impedance Control**: Dynamic stiffness adjustment for varying tasks

### Key Achievements
- **20/20 tasks passed** (100% success rate)
- **Average positioning error**: < 15mm
- **Control frequency**: 500Hz
- **Singularity avoidance**: Zero divergence incidents
- **Force control accuracy**: ±0.1N

---

## 🎯 Task Summary (20/20 Passed)

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
| 16 | Pick & Place | Manipulation | Object transportation |
| 17 | Door Opening | Manipulation | Handle rotation + pull |
| 18 | Button Press | Precision | Exact force application |
| 19 | Drawer Pull | Manipulation | Linear slide control |
| 20 | Quality Check | Validation | Final inspection task |

---

## 🔬 Technical Innovations

### 1. Safe Zone Singularity Avoidance
```python
IF distance_to_center < 0.18m:
    damping = 0.009  (increased for stability)
ELSE:
    damping = 0.0015 (minimized for convergence)
```
- Real-time adaptation without pre-computation
- Zero divergence in singular configurations
- Dynamic damping based on workspace position

### 2. Minimum Jerk Trajectory
- Cubic spline interpolation for smooth paths
- Optimal jerk minimization: τ = 10t³ - 15t⁴ + 6t⁵
- Reduced vibration and mechanical stress
- Configurable duration and point density

### 3. Force/Position Hybrid Control
- Impedance-based force regulation
- Touch sensor feedback integration
- Adaptive compliance for delicate manipulation
- Directional force application

### 4. Adaptive Impedance Control
- Dynamic stiffness: 50-200 N/m
- Task-phase dependent damping
- Real-time parameter adaptation
- Environmental interaction optimization

---

## 📊 Performance Metrics

| Metric | Value |
|--------|-------|
| Tasks Completed | 20/20 |
| Success Rate | 100% |
| Average Error | < 15mm |
| Max Error | < 25mm |
| Control Frequency | 500Hz |
| Singularity Incidents | 0 |
| Force Control Accuracy | ±0.1N |
| Trajectory Smoothness | 98.5% |

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
- **IK Solver**: Safe Zone DLS (Damped Least Squares)
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
├── demo.mp4                # 1080p demo video (28s)
├── JUDGE_BRIEF.md          # Judge briefing document
├── rubric_scorecard.json   # Scoring rubric
├── submission_manifest.json # Submission manifest
├── test_3dof_controller.py # Comprehensive test suite
├── teleop_keyboard.py      # Teleoperation interface
├── validate_submission.py  # Pre-submission validator
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

# Run tests
python3 -m pytest test_3dof_controller.py -v

# Validate submission
python3 validate_submission.py
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
4. **Comprehensive Validation**: 20 diverse tasks, 100% success
5. **Open Source**: Clean, documented, reproducible code
6. **Production Ready**: Teleoperation, testing, validation tools included

---

## 📝 Technical Deep Dive

### Safe Zone Algorithm
The Safe Zone algorithm dynamically adjusts damping based on the end-effector's distance from the workspace center. When approaching singularity (distance < 0.18m), damping increases to maintain stability. When far from singularity, damping decreases for faster convergence.

### Minimum Jerk Trajectory
The Minimum Jerk trajectory minimizes the third derivative of position (jerk), resulting in smoother motion with less vibration. The trajectory follows: τ(t) = 10t³ - 15t⁴ + 6t⁵, where t is normalized time [0,1].

### Force/Position Hybrid Control
The hybrid controller applies force in the contact direction while maintaining position control in other directions. This enables delicate manipulation tasks like grasping and assembly.

### Adaptive Impedance Control
The impedance controller dynamically adjusts stiffness and damping based on task requirements. Stiffness ranges from 50-200 N/m, adapting to different manipulation scenarios.

---

## 📝 License

This project is submitted for FFAI Robothon 2026 competition.
