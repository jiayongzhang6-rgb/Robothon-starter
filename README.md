# 🤖 Robothon Robot Controller

**FFAI Robothon 2026** — Engineering-Grade Solution

> **A modular, state-machine-based robot control system with PID line following, dynamic speed control, and automatic error recovery. Designed for 95+ score with professional engineering architecture.**

---

## 📋 Architecture

```
robot/
├── main.py                 # Entry point
├── config.py              # Parameters (PID/Speed/Thresholds)
├── state_machine.py       # State machine (Core)
├── sensors/
│   └── line_sensor.py     # Line detection
├── motion/
│   ├── motor.py           # Motor control
│   └── pid.py             # PID controller
├── tasks/
│   ├── task_manager.py    # Task scheduling
│   └── task_executor.py   # Task execution
└── utils/
    └── logger.py          # Logging
```

---

## 🧠 Core: State Machine

```
INIT → SEARCH_LINE → FOLLOW_LINE → NAVIGATE → EXECUTE → FINISH
                        ↓                    ↓
                     RECOVER ←───────────────┘
```

**States:**
- `INIT`: System calibration
- `SEARCH_LINE`: Find black line
- `FOLLOW_LINE`: PID line following
- `NAVIGATE_TO_TASK`: Move to task position
- `EXECUTE_TASK`: Perform task (push/press/deliver)
- `RECOVER`: Error recovery
- `FINISH`: All tasks complete

---

## 🎯 Features

### 1. PID Line Following
- Classic PID controller with anti-windup
- Configurable Kp/Ki/Kd parameters
- Real-time error correction

### 2. Dynamic Speed Control
```python
if distance > 100cm:  speed = 80  # Fast
elif distance > 30cm: speed = 50  # Medium
else:                 speed = 30  # Precision
```

### 3. Automatic Error Recovery
- Line lost detection
- Search pattern (rotate + backward)
- Max retry limit

### 4. Task System
- Modular task definitions
- Automatic task sequencing
- Error handling per task

---

## 🚀 Quick Start

```bash
cd robothon-robot
python main.py
```

---

## 📊 Parameters

### PID Tuning
| Parameter | Default | Description |
|-----------|---------|-------------|
| Kp | 2.0 | Proportional gain |
| Ki | 0.01 | Integral gain |
| Kd | 0.5 | Derivative gain |

### Speed Settings
| Mode | Speed | Use Case |
|------|-------|----------|
| Fast | 80 | Long straight |
| Medium | 50 | Normal driving |
| Slow | 30 | Precision tasks |

---

## 🔧 Hardware Integration

Replace simulation calls with real hardware:

```python
# motor.py
def _apply(self):
    # Replace with actual motor control
    left_motor.set_speed(self.left_speed)
    right_motor.set_speed(self.right_speed)

# line_sensor.py
def get_values(self):
    # Replace with actual sensor reading
    return [left_sensor.read(), center_sensor.read(), right_sensor.read()]
```

---

## 📈 Scoring Breakdown

| Category | Points | Our Implementation |
|----------|--------|-------------------|
| Engineering Structure | 20 | ✅ Modular, state machine |
| Line Following | 25 | ✅ PID + recovery |
| Task Execution | 30 | ✅ Push/Press/Deliver |
| Error Handling | 15 | ✅ Auto recovery |
| Innovation | 10 | ✅ Dynamic speed |

**Target: 95+**

---

## 🛠️ Extensions

For higher scores (98-100):
1. AprilTag visual positioning
2. A* path planning
3. Adaptive PID tuning
