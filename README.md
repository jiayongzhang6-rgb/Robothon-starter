# Adaptive Campus Patrol Agent with Hierarchical Q-learning Decision Policy

An adaptive campus patrol agent that combines hierarchical decision policy, risk-aware routing, and online Q-learning to enable dynamic, context-driven navigation behavior in complex environments.

## System Architecture

The system follows a hierarchical decision-making pipeline:

```
Patrol Policy (Q-learning agent)
        ↓
Behavior Selector (adaptive strategy selection)
        ↓
Route Sampler (stochastic waypoint generation)
        ↓
Risk Estimator (multi-factor cost evaluation)
        ↓
Low-level Navigation Controller
```

**Design principle:** Decision-making is separated from control execution.

## Key Innovations

### 1. Hierarchical Decision Policy
Replaces traditional FSM-based navigation with a structured policy hierarchy enabling adaptive decision-making.

### 2. Risk-Aware Route Selection
Routes are selected using a multi-dimensional cost function:
- **Distance cost** (1.0×): proximity to target
- **Obstacle risk** (2.5×): collision avoidance (highest priority)
- **Novelty bonus** (−0.5×): unexplored areas preferred
- **Smoothness** (0.3×): heading-aligned paths favored

### 3. Online Q-learning Behavior Adaptation
Behavior selection is dynamically adjusted using Q-learning with ε-greedy exploration (ε=0.2). The agent learns from environmental feedback (reward signal) and updates its policy at every step.

### 4. Emergent Behavior Distribution
The system exhibits emergent behavioral patterns rather than hard-coded state transitions:

```
patrol            : 28.8%  ██████████████
return_to_base    : 23.3%  ███████████
explore           : 16.5%  ████████
obstacle_avoid    : 14.3%  ███████
cautious_patrol   :  8.7%  ████
recover           :  8.3%  ████
```

## Decision Trace

Every step is logged with full reasoning:

```
step 6: sampled ['intersection_2', 'zone_c', 'zone_d', 'base']
        → selected intersection_2 (cost=-0.07)
        → behavior=explore
        → Q-values: {explore: 1.41, patrol: 0.92, return: 0.55}

step 7: sampled ['zone_a', 'zone_c', 'zone_d', 'base']
        → selected zone_a (cost=0.093)
        → behavior=cautious_patrol
        → Q-values: {cautious: 0.8, explore: 1.2, patrol: 0.5}
```

This transparent reasoning distinguishes the system from scripted navigation demos.

## Performance

| Metric | Value |
|--------|-------|
| Decision frequency | 20Hz |
| Candidates per step | 4 |
| Exploration rate (ε) | 0.2 |
| Learning rate (α) | 0.3 |
| Discount factor (γ) | 0.9 |
| Waypoint graph nodes | 9 |
| Zone coverage | All zones visited |

## Project Structure

```
robothon-robot/
├── main.py                          # Strategy simulation
├── README.md
├── evaluation_report.json
├── demo.mp4
├── robot/
│   ├── policy/                      # AI Decision Layer
│   │   ├── patrol_policy.py         # Core policy + decision logging
│   │   ├── route_sampler.py         # Stochastic waypoint generation
│   │   ├── risk_estimator.py        # Multi-factor cost evaluation
│   │   └── behavior_selector.py     # Q-learning behavior selection
│   ├── controller/
│   ├── sensors/
│   ├── tasks/
│   └── recovery/
└── simulation/
```

## Quick Start

```bash
python main.py
```

## Technical Details

- **Architecture:** Hierarchical Policy (not FSM)
- **Behavior selection:** Q-learning with ε-greedy exploration
- **Route selection:** Stochastic sampling + risk-based scoring
- **Control:** PID-based low-level navigation
- **Sensors:** 5-sensor weighted line detection

## License

MIT License

---

**Developer:** jiayongzhang6-rgb  
**Project:** Robothon 2026  
**Version:** 4.0.0
