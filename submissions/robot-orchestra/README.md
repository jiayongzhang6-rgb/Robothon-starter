# Robot Orchestra v15 — Multi-Agent Rhythmic Coordination

**UUID: d2f04863-5683-4e20-bd39-32f0cf339dc2**

---

## 🎯 Project Overview

A multi-agent robotic orchestra system with **3 robot arms** coordinating to play **4 instruments** across **3 different musical pieces** at **120 BPM**. The system achieves **99.2% closed-loop success rate** with **94.4% fault recovery** through contact detection and adaptive control.

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
| **Musical Pieces** | 3 (March, Waltz, Finale) |

---

## 🎵 Musical Repertoire

The orchestra performs **3 distinct musical pieces**, each showcasing different instrument combinations and coordination patterns:

### Piece 1: March (进行曲)
- **Lead**: Drummer (snare + kick)
- **Accompaniment**: Xylophone (C + E)
- **Pattern**: Strong downbeat, 4/4 time
- **Coordination**: Drummer drives tempo, others follow

### Piece 2: Waltz (华尔兹)
- **Lead**: Xylophone (C + E + G triad)
- **Accompaniment**: Triangle + soft snare
- **Pattern**: 3/4 time, melodic emphasis
- **Coordination**: Xylophone leads, arms alternate

### Piece 3: Finale (终曲)
- **All instruments** at full intensity
- **Pattern**: Accelerating rhythm, all 4 instruments
- **Coordination**: Full 3-arm synchronization

```python
# Multi-song repertoire definition
SONGS = {
    'march': {
        'drummer': [(0.0, snare), (0.25, rest), (0.5, kick), (0.75, rest)],
        'xylophone': [(0.0, C_note), (0.5, E_note)],
        'perc': [(0.0, rest), (0.5, cymbal)],
    },
    'waltz': {
        'drummer': [(0.0, snare), (0.33, rest), (0.66, rest)],
        'xylophone': [(0.0, C_note), (0.33, E_note), (0.66, G_note)],
        'perc': [(0.0, triangle), (0.5, rest)],
    },
    'climax': {
        'drummer': [(0.0, snare), (0.15, kick), (0.30, hihat), 
                    (0.45, snare), (0.60, kick), (0.75, rest)],
        'xylophone': [(0.0, C), (0.2, E), (0.4, G), (0.6, E), (0.8, C)],
        'perc': [(0.0, cymbal), (0.25, triangle), (0.5, cymbal), (0.75, triangle)],
    },
}
```

---

## 🏆 Why This Matters

### The Problem

Multi-agent robotic coordination is challenging because:
- **Timing synchronization**: Multiple arms must coordinate precisely
- **Collision avoidance**: Arms must not interfere with each other
- **Fault tolerance**: System must recover from failures gracefully
- **Musical variety**: Different pieces require different coordination patterns

### Our Solution

We built a **multi-agent robotic orchestra** that:
1. **Coordinates 3 arms** playing 4 instruments simultaneously
2. **Performs 3 different musical pieces** (March, Waltz, Finale)
3. **Detects faults** using contact-based collision detection
4. **Recovers automatically** with position adjustment and retry (94.4% success)
5. **Achieves 99.2% success** in 128-trial benchmark

---

## 🔬 Technical Approach

### Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Robot Orchestra v15                       │
├─────────────────────────────────────────────────────────────┤
│  Music Repertoire Layer                                      │
│  ├── March: drum-lead, 4/4 time                             │
│  ├── Waltz: xylophone-lead, 3/4 time                        │
│  └── Finale: all-instruments, full sync                     │
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

1. **Multi-Song Repertoire (NEW)**
   - 3 distinct musical pieces with different coordination patterns
   - March (drum-lead), Waltz (xylophone-lead), Finale (all-instruments)
   - Demonstrates system versatility and extensibility

2. **Contact Detection (replaces touch sensor)**
   - Uses `data.ncon` geom collision detection
   - Supports all geom type combinations
   - 99.2% detection rate

3. **Closed-Loop Control + Fault Recovery**
   - Precise position + contact verification
   - Auto-retry with position adjustment (±0.10 rad)
   - 72 fault detections, 68 recoveries (94.4% recovery rate)

4. **Multi-Agent Coordination**
   - 3 arms synchronized at 120 BPM
   - No collision interference
   - Dynamic role assignment per song

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

### Per-Song Results

| Song | Arms Used | Instruments | Coordination Pattern |
|------|-----------|-------------|---------------------|
| March | 3 | 4 | Drum-lead, 4/4 |
| Waltz | 3 | 4 | Xylophone-lead, 3/4 |
| Finale | 3 | 4 | Full sync, all instruments |

### Fault Recovery

| Metric | Value |
|--------|-------|
| Fault Detections | 72 |
| Fault Recoveries | 68 |
| Recovery Rate | 94.4% |

---

## 🎥 Demo Video

**Duration**: 19 seconds | **Resolution**: 1920×1080 | **Frame Rate**: 30 fps

The demo showcases:
1. **Opening**: Orchestra tuning (wide shot)
2. **March**: Drum solo → Xylophone melody (6 camera angles)
3. **Waltz**: All arms in 3/4 time (close-up shots)
4. **Finale**: Full orchestra climax (wide + top-down)
5. **HUD**: BPM counter, beat progress, sync error, fault recovery rate

**Camera Angles**: 6 dynamic views (wide, drummer, xylophone, percussion, close-up, top-down)

---

## 🚀 How to Run

```bash
pip install mujoco numpy

# Run demo with multi-song repertoire
python engine/simulator.py

# Run 128-trial benchmark
python engine/benchmark.py

# Render professional video
python render_v15_professional.py
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
| `scene.xml` | MuJoCo scene with 3 arms + 4 instruments |
| `engine/simulator.py` | Main simulation code |
| `engine/benchmark.py` | Benchmark script |
| `render_v15_professional.py` | Professional multi-angle video renderer |
| `demo.mp4` | 19s demo video (1920×1080, 3 songs, 6 angles) |

---

**UUID: d2f04863-5683-4e20-bd39-32f0cf339dc2**
