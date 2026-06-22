# 🏆 Robothon Robot Controller - Championship Edition

**FFAI Robothon 2026** — Engineering-Grade Solution

> **A state-machine-based robot control system with PID line following, 3-layer recovery, and dynamic speed control. Designed for 95+ score.**

---

## 🧠 System Architecture

```
┌─────────────────────────────────────────────────────┐
│                   MAIN CONTROLLER                   │
│                   (State Machine)                    │
└───────────────────────┬─────────────────────────────┘
                        │
      ┌─────────────────┼─────────────────┐
      │                 │                 │
┌─────┴─────┐   ┌──────┴──────┐   ┌─────┴─────┐
│   SENSOR  │   │   CONTROL   │   │    TASK   │
│   LAYER   │   │    LAYER    │   │    LAYER  │
└─────┬─────┘   └──────┬──────┘   └─────┬─────┘
      │                │                │
      ▼                ▼                ▼
 5-Sensor         PID + Motor       Task Executor
 Line Detect      Driver            (Push/Press/Deliver)
      │
      └──────────── Recovery System ────────────────┘
```

---

## 📁 Project Structure

```
robothon-robot/
├── main.py                     # Entry point
├── config.py                   # Parameters
├── state_machine.py            # State machine (Core)
├── robot/
│   ├── controller/
│   │   ├── pid.py              # PID controller
│   │   ├── motor.py            # Motor control
│   │   └── line_follow.py      # Line following
│   ├── sensors/
│   │   ├── line_sensor.py      # 5-sensor line detect
│   │   └── vision.py           # Vision (optional)
│   ├── tasks/
│   │   ├── task_manager.py     # Task scheduling
│   │   └── task_executor.py    # Task execution
│   ├── recovery/
│   │   └── recovery.py         # 3-layer recovery
│   └── utils/
│       ├── timer.py            # Timing
│       └── logger.py           # Logging
├── arduino/
│   └── motor_control.ino       # Arduino driver
└── demo.mp4                    # Demo video
```

---

## 🧭 State Machine

```
INIT → CALIBRATION → LINE_FOLLOW → INTERSECTION → TASK_ALIGN → TASK_EXECUTE
                ↓                      ↓                           ↓
             RECOVER ←─────────────────┘                           │
                │                                                  │
                └──────────────────────────────────────────────────┘
```

**States:**
- `INIT`: System startup
- `CALIBRATION`: Sensor calibration
- `LINE_FOLLOW`: PID line following
- `INTERSECTION`: Detect intersection
- `TASK_ALIGN`: Align to task
- `TASK_EXECUTE`: Execute task
- `RECOVERY`: Error recovery
- `FINISH`: All tasks complete

---

## 🎯 Core Features

### 1. PID Line Following
```python
Kp = 20, Ki = 0, Kd = 14
error = weighted_error(sensor_values)
correction = pid.compute(error)
motor.set(base - correction, base + correction)
```

### 2. 5-Sensor Weighted Error
```python
weights = [-2, -1, 0, 1, 2]
error = sum(w * v for w, v in zip(weights, sensors))
```

### 3. Dynamic Speed Control
```python
if abs(error) < 1:  speed = 85  # Fast
elif abs(error) < 3: speed = 65  # Medium
else:                speed = 45  # Slow
```

### 4. 3-Layer Recovery
1. **Rotate Search**: -30°, +30°, -60°, +60°
2. **Backward Search**: Move back 20cm
3. **Wide Search**: ±90° rotation

---

## ⚙️ Tuning Manual

### Phase 1: Basic
```
Ki = 0, Kd = 0
Only tune Kp
Goal: Follow line with slight oscillation
```

### Phase 2: Damping
```
Kd = 10~16
Goal: Reduce oscillation
```

### Phase 3: Fine-tune
```
Kp ±10%
Final: Kp=20, Ki=0, Kd=14
```

---

## 🚀 Quick Start

```bash
# Python
cd robothon-robot
python main.py

# Arduino
Open arduino/motor_control.ino in Arduino IDE
Upload to board
```

---

## 📊 Scoring Breakdown

| Category | Points | Implementation |
|----------|--------|----------------|
| Engineering Structure | 20 | ✅ Modular, state machine |
| Line Following | 25 | ✅ PID + 5-sensor |
| Task Execution | 30 | ✅ Push/Press/Deliver |
| Error Handling | 15 | ✅ 3-layer recovery |
| Innovation | 10 | ✅ Dynamic speed |

**Target: 95+**

---

## 🔧 Hardware Integration

### Arduino Pins
| Pin | Function |
|-----|----------|
| 5,6 | Motor PWM |
| 7-10 | Motor direction |
| A0-A4 | 5-sensor input |

### Motor Driver
```c
void setMotor(int left, int right) {
    digitalWrite(7, left > 0);
    digitalWrite(8, left <= 0);
    analogWrite(5, abs(left));
    digitalWrite(9, right > 0);
    digitalWrite(10, right <= 0);
    analogWrite(6, abs(right));
}
```
