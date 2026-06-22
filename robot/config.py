#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
冠军级配置 - Robothon比赛参数
"""

# ============ 速度参数 ============
BASE_SPEED = 75
TURN_SPEED = 60
TASK_SPEED = 40
RECOVERY_SPEED = 50

# ============ PID参数（比赛现场调参） ============
PID = {
    "kp": 20,
    "ki": 0.0,
    "kd": 14,
}

# ============ 传感器参数 ============
LINE_LOST_THRESHOLD = 250
SENSOR_WEIGHTS = [-2, -1, 0, 1, 2]  # 5传感器加权

# ============ 恢复参数 ============
RECOVERY_ANGLES = [-30, 30, -60, 60]
RECOVERY_BACKWARD = 20  # cm

# ============ 任务参数 ============
ALIGN_THRESHOLD = 2  # 像素
TASK_TIMEOUT = 5  # 秒

# ============ 状态定义 ============
from enum import Enum

class State(Enum):
    INIT = 0
    CALIBRATION = 1
    LINE_FOLLOW = 2
    INTERSECTION = 3
    TASK_ALIGN = 4
    TASK_EXECUTE = 5
    RECOVERY = 6
    FINISH = 7
