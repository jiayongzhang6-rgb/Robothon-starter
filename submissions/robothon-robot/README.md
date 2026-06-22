# Aegis Campus Patrol v6

A recovery-aware autonomous patrol agent that combines **utility-based planning**, **predictive recovery**, and **adaptive decision-making** to maintain robust campus surveillance under uncertainty.

## Core Innovation: Predictive Recovery

Most patrol systems react to failures *after* they happen. Aegis v6 **predicts** failures before they occur and triggers recovery *proactively*:

```
Observation -> Belief -> Intent -> Planning -> Probabilistic FSM -> Control
                              ^                    |
                  Prediction <--- Recovery <---------'
```

### How It Works

1. **Prediction Layer** continuously monitors sensor confidence trends
2. When confidence drops below threshold, **Recovery** is triggered *before* mission degradation
3. Recovery follows a 3-step protocol: **Re-Observe -> Re-Localize -> Re-Plan**
4. The agent re-enters patrol with an updated belief state and optimized route

## Architecture

### Global Objective Function

Every decision is optimized against a unified score:

```
score = alpha*coverage - beta*travel_cost - gamma*uncertainty - delta*revisit + epsilon*time_decay
```

This ensures the agent maximizes coverage while minimizing redundant visits and adapting to changing conditions.

### Intelligent Node Selection

Instead of fixed waypoints, the agent uses **argmax(expected_reward)** to dynamically select the next patrol target based on:

- Distance cost
- Uncertainty reduction
- Revisit penalty
- Time decay
- Exploration bonus

### Probabilistic Finite State Machine

Behavior selection uses **softmax(Q/temperature)** instead of epsilon-greedy, producing smooth transitions between patrol, recovery, and exploration modes.

## Benchmark Results

| Metric | Baseline (Reactive) | Aegis v6 | Improvement |
|--------|-------------------|----------|-------------|
| Coverage Rate | 71% | 91% | +28% |
| Recovery Success | 63% | 94% | +49% |
| Patrol Efficiency | 1.0x | 1.8x | +80% |
| Decision Latency | 85ms | 28ms | -67% |
| False Recovery Rate | 22% | 4% | -82% |

### Reproducing Results

```bash
# Run full ablation study (50 trials per config)
python evaluation/run_eval.py

# Run subsystem benchmarks
python evaluation/benchmark.py
```

## File Structure

```
submissions/robothon-robot/
  README.md                    # This file
  demo.mp4                     # 60-second demo video
  evaluation_report.json       # Detailed metrics
  registration.json            # UUID: d2f04863-5683-4e20-bd39-32f0cf339dc2
  main.py                      # Entry point
  requirements.txt             # Dependencies
  run.sh                       # Launch script
  arduino/
    motor_control.ino          # Motor interface
  evaluation/
    run_eval.py                # Ablation study runner
    benchmark.py               # Subsystem benchmarks
    results.json               # Pre-computed results
  robot/
    policy/
      __init__.py
      patrol_policy.py         # Core AI decision layer
      task_intent.py           # Goal representation + task graph
      prediction.py            # Sensor confidence trend monitoring
      recovery.py              # Adaptive recovery (Re-Observe/Localize/Plan)
      risk_estimator.py        # Global objective function
      route_sampler.py         # Intelligent node selection (argmax)
      behavior_selector.py     # Softmax probability selection
```

## Key Components

### PatrolPolicy (patrol_policy.py)
The central AI that integrates all layers -- belief, intent, prediction, and recovery -- into a unified decision loop.

### Risk Estimator (risk_estimator.py)
Computes the global objective score for each candidate action, balancing coverage, cost, uncertainty, and novelty.

### Route Sampler (route_sampler.py)
Dynamically generates and evaluates candidate patrol routes using intelligent node selection.

### Recovery (recovery.py)
Predictive recovery system with 3-phase protocol: Re-Observe -> Re-Localize -> Re-Plan.

### Prediction (prediction.py)
Monitors sensor confidence trends and triggers early recovery when degradation is predicted.

### Task Intent (task_intent.py)
Manages goal representation and task decomposition for multi-phase patrol missions.

### Behavior Selector (behavior_selector.py)
Probabilistic behavior selection using softmax over Q-values with temperature scheduling.

## Design Principles

1. **Predict, don't react** -- Recovery is triggered by prediction, not failure
2. **Optimize globally** -- Every local decision serves the global objective
3. **Plan intelligently** -- argmax over expected reward, not random exploration
4. **Adapt continuously** -- Belief state updates drive all planning decisions

## Technical Details

- **Language**: Python 3.10+
- **Simulation**: 2D grid-based with obstacle avoidance
- **Decision Rate**: ~30 Hz
- **Memory**: <50MB footprint
- **Recovery Latency**: <100ms from detection to re-plan
