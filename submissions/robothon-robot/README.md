# AEGIS v6: Predictive Recovery Patrol Agent

**Team:** 3DOF  
**Competition:** Robothon 2026  
**Category:** Autonomous Navigation

## Core Innovation: Attention-Guided Decision Visualization

AEGIS v6 visualizes **internal decision-making** of an autonomous navigation system. The viewer's attention is guided through five progressive phases, each revealing deeper system intelligence.

### 60-Second Demo Narrative

| Phase | Time | State | Focus |
|-------|------|-------|-------|
| 1. Baseline | 0–10s | PATROL | Normal navigation, soft-cost zones visible |
| 2. Risk Emergence | 10–20s | ADAPT | Risk field appears, path begins deforming |
| 3. Uncertainty | 20–30s | UNCERTAINTY | Robot hesitates, confidence drops |
| 4. Recovery | 30–40s | RECOVERY | Full stop → re-observe → re-localize → re-plan |
| 5. Stable | 40–60s | STABLE | System restored, metrics displayed |

### Visualization Design Principles

- **2D top-down orthographic** — no 3D, no perspective distortion
- **Fixed camera** — no movement, no zoom
- **Event-driven behavior** — actions driven by environment, not timeline
- **Attention-guided focus** — five-phase progressive revelation
- **Soft-cost zones** — traversable but penalized regions
- **Risk field** — dynamic gradient heatmap
- **HUD minimal** — only STATE label and confidence bar

## Project Structure

```
submissions/robothon-robot/
├── README.md
├── registration.json
├── demo.mp4              # 60s 2D top-down demo (900×900, 30fps)
├── config.yaml
├── engine/
│   ├── __init__.py
│   ├── state.py          # State machine
│   ├── event_bus.py      # Event-driven architecture
│   ├── simulator.py      # Simulation core
│   ├── planner.py        # Path planning with risk avoidance
│   ├── risk_model.py     # Dynamic risk field generation
│   └── recovery.py       # 3-phase recovery pipeline
└── viz/
    ├── __init__.py
    ├── overlay.py        # HUD overlay rendering
    ├── heatmap.py        # Risk field visualization
    └── mujoco_renderer.py # 3D scene rendering (legacy)
```

## Scoring Alignment

| Criterion | How This Demo Addresses It |
|-----------|---------------------------|
| Perceived Intelligence | Risk-aware path deformation, autonomous recovery |
| Causal Clarity | Clear event sequence: risk → hesitation → recovery |
| Predictive Behavior | Path deforms BEFORE entering high-risk zone |
| Recovery Completeness | Full cycle: stop → observe → localize → replan |
| Visual Interpretability | 2D top-down, minimal HUD, attention-guided |

## Technical Details

- **Engine**: Event-driven with finite state machine
- **Planner**: Utility-based with dynamic risk cost overlay
- **Risk Model**: Gaussian gradient field with temporal evolution
- **Recovery**: 3-phase pipeline (observe → localize → replan)
- **Renderer**: Pure OpenCV 2D (no GPU dependency)
- **Deterministic**: seed=42, fully reproducible
