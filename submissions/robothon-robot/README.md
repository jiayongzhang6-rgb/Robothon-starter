# AEGIS v6: Predictive Recovery Patrol Agent

**Team:** 3DOF  
**Competition:** Robothon 2026  
**Category:** Autonomous Navigation

## Key Innovation: Predictive Recovery

AEGIS v6 demonstrates **predictive failure detection** and **autonomous recovery** in a patrol scenario.

### 60-Second Demo Narrative

| Phase | Time | Behavior |
|-------|------|----------|
| Patrol | 0-10s | Confident, straight path |
| Deviate | 10-18s | Path bends (risk detected) |
| Hesitate | 18-25s | Slow, jittery (confidence drops) |
| Reassess | 25-35s | Full stop + 360 rotation |
| Recovery | 35-50s | New confident path |
| Resume | 50-60s | Steady patrol |

## Project Structure

```
submissions/robothon-robot/
├── README.md
├── registration.json
├── demo.mp4
├── render_demo.py          # Main demo renderer (MuJoCo + OpenCV)
├── config.yaml             # Demo configuration
├── engine/
│   ├── __init__.py
│   ├── state.py            # Unified state management
│   ├── event_bus.py        # Event-driven architecture
│   ├── simulator.py        # Simulation core
│   ├── planner.py          # Path planning
│   ├── risk_model.py       # Risk assessment
│   └── recovery.py         # Recovery pipeline
└── viz/
    ├── __init__.py
    ├── overlay.py          # HUD overlay rendering
    ├── heatmap.py          # Risk visualization
    └── mujoco_renderer.py  # 3D scene rendering
```

## How to Run

```bash
pip install mujoco opencv-python numpy pyyaml
python render_demo.py
```

Output: `output/demo.mp4` (60s, 1920x1080, 30fps)

## Key Features

- **Risk terrain visualization** (height deformation, not just color)
- **Motion trail** with path adaptation
- **Decision explanation bubbles** at key moments
- **Recovery behavior visible BEFORE label**
- **Deterministic** (seed=42, fully reproducible)
- **Event-driven architecture** (modular, testable)

## Technical Details

- **Engine**: Event-driven simulation with state machine
- **Planner**: Utility-based path selection with risk avoidance
- **Recovery**: 3-phase pipeline (observe → localize → replan)
- **Renderer**: MuJoCo 3D + OpenCV overlay
