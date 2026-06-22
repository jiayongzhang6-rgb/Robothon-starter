# 🤖 3DOF Confined-Space Precision Manipulator

**FFAI Robothon 2026** — Freestyle Category

> **A 3-DOF robot arm achieves 100% success across 15 complex tasks through Safe Zone singularity avoidance + Force/Impedance control — demonstrating that minimalist hardware can match 6-DOF precision in confined environments.**

---

## 📋 Project Overview

### Key Achievement
| Metric | Value |
|--------|-------|
| Tasks completed | **15** |
| Success rate | **100%** |
| Average error | **< 10mm** |
| Control frequency | **500Hz** |
| Force accuracy | **±0.1N** |

---

## 🔬 Core Innovation

### 1. Safe Zone Algorithm
Real-time adaptive damping based on distance to workspace center:
- Distance < 0.18m → Damping ×3 (prevent divergence)
- Distance ≥ 0.18m → Damping = 0.001 (maximum convergence)

### 2. Force/Position Hybrid Control
- Force control in specified direction
- Position control in other directions
- Touch sensor feedback

### 3. Adaptive Impedance Control
- Dynamic stiffness adjustment (50-200 N/m)
- Task-phase-dependent parameters

---

## 🎯 Tasks (15/15 Passed)

| # | Task | Type | Waypoints |
|---|------|------|-----------|
| 1 | 5-Point Reaching | Positioning | 5 |
| 2 | Square (6cm) | Path Tracking | 20 |
| 3 | Circle (r=4cm) | Path Tracking | 16 |
| 4 | Figure-8 | Path Tracking | 20 |
| 5 | Spiral (2 turns) | Path Tracking | 24 |
| 6 | Star (5-point) | Path Tracking | 20 |
| 7 | Heart | Path Tracking | 30 |
| 8 | Spiral Star | Path Tracking | 30 |
| 9 | Force-Controlled Grasp | Manipulation | 7 |
| 10 | Obstacle Avoidance | Navigation | 4 |
| 11 | Fast Multi-Point | Dynamic | 5 |
| 12 | Precision Assembly | High Accuracy | 12 |
| 13 | Minimum Jerk Trajectory | Trajectory Optimization | 20 |
| 14 | Adaptive Impedance | Variable Stiffness | 3 |
| 15 | Composite Task | 综合演示 | 34 |

---

## 🛠️ Technical Details

### Control System
- Algorithm: Safe Zone DLS + Force/Impedance
- Control Frequency: 500Hz
- IK Gain: 35.0
- Damping: 0.0015-0.009 (adaptive)

### MuJoCo Model
- Joints: 3 hinge + 2 slide + 1 free
- Sensors: 8
- Timestep: 2ms
- Collisions: Enabled

---

## 🚀 Quick Start

```bash
pip install -r requirements.txt
python robot_controller.py
```

---

## 📊 Evaluation

See `evaluation_report.json` for detailed metrics.
