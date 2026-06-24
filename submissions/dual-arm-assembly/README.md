# Space Module Dual-Arm Assembly

Two Franka Emika Panda arms collaboratively assemble space station modules.

## Overview

This project demonstrates dual-arm robotic coordination for space station assembly tasks. Two 7-DOF Franka Emika Panda arms work together to grasp, transfer, and assemble modular space station components.

## Task Sequence

1. **Home Position** - Arms return to initial configuration
2. **Reach Blue Module** - Left arm approaches blue module
3. **Grasp Module** - Left arm grasps the blue module
4. **Lift & Reach** - Left arm lifts module, right arm approaches red module
5. **Handoff** - Module transfer between arms
6. **Assembly** - Precise module alignment and connection
7. **Grab Green** - Right arm grasps green module
8. **Final Assembly** - Complete space module assembly

## Technical Highlights

- **Dual-Arm Coordination**: Two 7-DOF arms working synchronously
- **Analytical IK**: Jacobian-based inverse kinematics for real-time trajectory generation
- **Force Feedback**: Gripper force monitoring for secure grasping
- **Visual Feedback**: Status panels showing task phase, gripper state, and progress

## Files

- `franka_controller.py` - Main robot control logic
- `test_franka_controller.py` - Unit tests
- `demo.mp4` - Demo video (18 seconds, 960x540)
- `README.md` - This file
- `JUDGE_BRIEF.md` - Technical details for judges

## Robot Configuration

- **Left Arm**: Base at x=-0.5, reaching to workspace center
- **Right Arm**: Base at x=+0.5, reaching to workspace center
- **Modules**: 3 colored space station components (blue, red, green)
- **Grippers**: Parallel jaw grippers with force control

## Requirements

- MuJoCo
- Python 3.8+
- NumPy
- Matplotlib
