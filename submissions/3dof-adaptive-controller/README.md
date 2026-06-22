# 🤖 3DOF Confined-Space Precision Manipulator

**FFAI Robothon 2026** — Freestyle Category

> **A 3-DOF robot arm achieves 100% success across 8 complex tasks through Safe Zone singularity avoidance — demonstrating that minimalist hardware can match 6-DOF precision in confined environments.**

---

## 📋 Key Achievement

| Metric | Value |
|--------|-------|
| Tasks completed | **8** |
| Success rate | **100%** |
| Average error | **< 10mm** |
| Control frequency | **500Hz** |
| Innovation | **Real-time Safe Zone** |

---

## 🎯 Tasks (8/8 Passed)

| # | Task | Type | Waypoints |
|---|------|------|-----------|
| 1 | 5-Point Reaching | Positioning | 5 |
| 2 | Square (6cm) | Path Tracking | 20 |
| 3 | Circle (r=4cm) | Path Tracking | 16 |
| 4 | Figure-8 | Path Tracking | 20 |
| 5 | Spiral (2 turns) | Path Tracking | 24 |
| 6 | Star (5-point) | Path Tracking | 20 |
| 7 | Heart | Path Tracking | 30 |
| 8 | Force-Controlled Grasp | Manipulation | 7 |

---

## 💡 Innovation: Safe Zone Singularity Avoidance

### The Problem
3-DOF arms suffer from **kinematic singularity** — when the arm reaches certain configurations, small Cartesian errors require infinite joint velocities. Traditional solutions require pre-computation of singularity regions.

### Our Solution: Real-Time Safe Zone
We implement **adaptive damping** that responds in real-time:

```python
def safe_zone_damping(ee_pos):
    dist = np.linalg.norm(ee_pos - WORKSPACE_CENTER)
    return DAMPING_NEAR if dist < SINGULARITY_DISTANCE else DAMPING_FAR
```

**Key Parameters:**
- `SINGULARITY_DISTANCE = 0.18m` — detection radius
- `DAMPING_NEAR = 0.009` — high damping near singularity
- `DAMPING_FAR = 0.0015` — low damping in safe regions

### Ablation Study Results

| Configuration | Success Rate | Avg Error | Avg Steps |
|--------------|--------------|-----------|-----------|
| **With Safe Zone** | **100%** | **8.2mm** | **450** |
| Without Safe Zone | 62.5% | 72.8mm | 1200 |
| **Improvement** | **+37.5%** | **-88.7%** | **-62.5%** |

**Conclusion:** Safe Zone enables real-time singularity avoidance without pre-computation, achieving 100% task success with < 10mm error.

---

## 🛠️ Technical Details

### Control System
- Algorithm: Safe Zone DLS (Damped Least Squares)
- Control Frequency: 500Hz
- IK Gain: 35.0
- Adaptive Damping: 0.0015-0.009

### Force/Impedance Control
- Implicit force control via position commands
- Adaptive impedance with variable stiffness
- Contact detection for grasp tasks

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
