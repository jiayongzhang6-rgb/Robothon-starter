# Robot Orchestra v13

**UUID: d2f04863-5683-4e20-bd39-32f0cf339dc2**

---

## 🎯 Project Overview

A multi-agent robotic orchestra system with **3 robot arms** coordinating to play **4 instruments** at **120 BPM**. The system achieves **99.2% closed-loop success rate** with **94.4% fault recovery** through contact detection and adaptive control.

### 🤖 Robot Platform

| Component | Specification |
|-----------|---------------|
| **Robot Arms** | 3 × 3-DOF arms (9 joints total) |
| **Instruments** | 4 (drums, xylophone, cymbal, triangle) |
| **Tempo** | 120 BPM |
| **Control** | Closed-loop with fault recovery |
| **Simulation** | MuJoCo 3.x |

### 📊 Key Results

| Metric | Value |
|--------|-------|
| **Closed-Loop Success Rate** | **99.2%** (127/128) |
| **Wilson CI 95%** | [95.7%, 99.9%] |
| **Open-Loop Success Rate** | 47.7% (61/128) |
| **Closed-Loop Advantage** | **+51.6%** |
| **Fault Recovery Rate** | **94.4%** (68/72) |

---

## 🏆 Why This Matters

### The Problem

Multi-agent robotic coordination is challenging because:
- **Timing synchronization**: Multiple arms must coordinate precisely
- **Collision avoidance**: Arms must not interfere with each other
- **Fault tolerance**: System must recover from failures gracefully

### Our Solution

We built a **multi-agent robotic orchestra** that:
1. **Coordinates 3 arms** playing 4 instruments simultaneously
2. **Detects faults** using contact-based collision detection
3. **Recovers automatically** with position adjustment and retry (94.4% success)
4. **Achieves 99.2% success** in 128-trial benchmark

---

## 🔬 Technical Approach

### Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Robot Orchestra v13                       │
├─────────────────────────────────────────────────────────────┤
│  Motion Control Layer                                        │
│  ├── Inverse Kinematics (joint angles → Cartesian position) │
│  ├── Contact Detection (geom collision detection)            │
│  └── Fault Recovery (auto-retry + position adjustment)       │
├─────────────────────────────────────────────────────────────┤
│  3 Robot Arms                                                │
│  ├── Drummer: snare + kick + hihat                           │
│  ├── Xylophonist: C + E + G                                  │
│  └── Percussionist: cymbal + triangle                        │
├─────────────────────────────────────────────────────────────┤
│  Validation Layer                                            │
│  ├── 128 closed-loop trials (noise=0.20 rad)                 │
│  ├── 128 open-loop trials (noise=0.20 rad)                   │
│  ├── Wilson confidence interval                              │
│  └── Ablation study (closed-loop vs open-loop)               │
└─────────────────────────────────────────────────────────────┘
```

### Key Innovations

1. **Contact Detection (replaces touch sensor)**
   - Uses `data.ncon` geom collision detection
   - Supports all geom type combinations
   - 99.2% detection rate

2. **Closed-Loop Control + Fault Recovery**
   - Precise position + contact verification
   - Auto-retry with position adjustment (±0.10 rad)
   - 72 fault detections, 68 recoveries (94.4% recovery rate)

3. **Multi-Agent Coordination**
   - 3 arms synchronized at 120 BPM
   - No collision interference
   - 4-phase performance: Intro → Verse → Chorus → Finale

---

## 📈 Results at a Glance

### 128-Trial Benchmark

| Mode | Trials | Successes | Rate | Wilson CI |
|------|--------|-----------|------|-----------|
| **Closed-Loop** | 128 | 127 | **99.2%** | [95.7%, 99.9%] |
| Open-Loop | 128 | 61 | 47.7% | - |

### Per-Instrument Results (Closed-Loop)

| Instrument | Successes | Trials | Rate |
|------------|-----------|--------|------|
| Drum | 49 | 50 | 98.0% |
| Xylophone | 37 | 37 | 100% |
| Percussion | 41 | 41 | 100% |

### Fault Recovery

| Metric | Value |
|--------|-------|
| Fault Detections | 72 |
| Fault Recoveries | 68 |
| Recovery Rate | 94.4% |

---

## 🚀 How to Run

### Quick Start

```bash
# Install dependencies
pip install mujoco numpy

# Run demo
python engine/simulator.py

# Run benchmark
python engine/benchmark.py
```

### MuJoCo Visualization

```bash
# Launch MuJoCo viewer
python -m mujoco.viewer --model scene.xml
```

---

## 📁 Files

| File | Description |
|------|-------------|
| `README.md` | Project overview |
| `JUDGE_BRIEF.md` | Judge evaluation summary |
| `EVALUATION_GUIDE.md` | Detailed evaluation guide |
| `benchmark_results.json` | 128-trial benchmark data |
| `physics_audit.json` | Physics verification results |
| `test_results.json` | Test suite results |
| `scene.xml` | MuJoCo scene with 3 arms + 4 instruments |
| `engine/simulator.py` | Main simulation code |
| `engine/benchmark.py` | Benchmark script |
| `demo.mp4` | Demo video (19s, 720p) |

---

## 📊 Competition Entry

| Field | Value |
|-------|-------|
| **UUID** | d2f04863-5683-4e20-bd39-32f0cf339dc2 |
| **Project Name** | Robot Orchestra v13 |
| **Team** | jiayongzhang6-rgb |
| **Submission Date** | 2026-06-29 |
| **Version** | 13 |

---

## 🎥 Demo Video

**Duration**: 19 seconds  
**Resolution**: 1280×720  
**Frame Rate**: 30 fps  

The demo showcases:
1. **3 arms coordinating** at 120 BPM
2. **4 instruments** played simultaneously
3. **Fault recovery** in real-time (94.4% success)
4. **HUD overlay** with beat progress bar

---

**UUID: d2f04863-5683-4e20-bd39-32f0cf339dc2**
