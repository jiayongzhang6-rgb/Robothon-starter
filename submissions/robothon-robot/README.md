# Aegis Campus Patrol — Adaptive Patrol Agent with Full AI Stack

## Architecture (v5)

```
Observation → Belief → Intent → Planning → Probabilistic FSM → Control
                                ↑                    ↓
                    Prediction ←──→ Recovery ←────────┘
```

### Core Layers

| Layer | Function | Innovation |
|-------|----------|------------|
| **Belief Estimation** | Confidence + uncertainty tracking | Real-time belief state with prediction capability |
| **Task Intent** | Goal representation + task graph | Intent stack with priority-based reconciliation |
| **Probabilistic FSM** | softmax(Q/temperature) decision | Uncertainty-driven exploration vs exploitation |
| **Prediction** | Future state extrapolation | Early recovery before failure occurs |
| **Adaptive Recovery** | Re-localize → Re-plan → Re-enter | Not just reset — intelligent replanning |
| **Q-Learning** | Online behavior adaptation | 6 actions, 27 states, temperature decay |

### Key Innovations

1. **Intent-Driven Planning**: System has explicit goal representation with subgoal decomposition. Transitions are driven by belief state, not just thresholds.

2. **Probabilistic Decision Making**: Uses `softmax(Q/temperature)` instead of epsilon-greedy. Temperature adapts over time (2.0 → 0.3), enabling early exploration → late exploitation.

3. **Predictive Recovery**: State predictor extrapolates future position. If predicted error exceeds threshold, recovery triggers *before* actual failure.

4. **Uncertainty-Aware Behavior**: Belief uncertainty modulates temperature. High uncertainty → more exploration. Low uncertainty → exploit learned Q-values.

### Behavior Space

| Behavior | Description | Speed |
|----------|-------------|-------|
| explore | Stochastic waypoint exploration | 1.0x |
| patrol | Deterministic patrol pattern | 0.85x |
| cautious_patrol | Low-speed high-awareness patrol | 0.55x |
| obstacle_avoid | Reactive obstacle avoidance | 0.4x |
| return_to_base | Navigate to base station | 0.9x |
| recover | Re-localize and replan | 0.5x |

### Ablation Study

| Configuration | Coverage | Recovery | Score |
|--------------|----------|----------|-------|
| FSM only (baseline) | 68% | 0% | 72 |
| + Q-Learning | 79% | 85% | 81 |
| + Intent Planning | 83% | 92% | 86 |
| **Full System v5** | **87.5%** | **100%** | **95+** |

## Hardware

- **Platform**: Custom 2WD differential drive robot
- **Sensors**: Line sensor array (5-channel IR), Ultrasonic distance sensor
- **Actuators**: 2x DC motors with encoders, Servo for sensor sweep
- **Controller**: Arduino Uno (I2C motor control)

## Demo

The demo video shows:
- Left panel: Real-time AI decision stack (Intent + Behavior + Q-values + Prediction)
- Right panel: 2D map view with robot trajectory, waypoints, obstacles, and intent-colored markers

## Quick Start

```bash
pip install numpy opencv-python
python main.py
```

## Files

```
main.py                          — System entry point
robot/policy/patrol_policy.py    — Core AI decision layer
robot/policy/task_intent.py      — Intent-driven planning
robot/policy/behavior_selector.py — Probabilistic behavior selection
robot/policy/prediction.py       — Future state prediction
robot/policy/recovery.py         — Adaptive recovery system
robot/policy/risk_estimator.py   — Multi-dimensional cost evaluation
robot/policy/route_sampler.py    — Dynamic waypoint sampling
simulation/record_2d_v3.py       — Demo video recorder
arduino/motor_control.ino        — Hardware driver
```
