#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
最终版HYBRID_DEMO控制器 - 冲95+
核心：Safe Mode保底 + Performance Mode证明上限
"""

import time
import math

# ============== HYBRID_DEMO配置 ==============
MODE = "HYBRID_DEMO"

SAFE = {
    "BASE_SPEED": 55,
    "KP": 15,
    "KI": 0,
    "KD": 20,
    "SMOOTH_FACTOR": 0.7,
    "SMOOTH_THRESHOLD": 5,
}

PERFORMANCE = {
    "BASE_SPEED": 75,
    "KP": 20,
    "KI": 0,
    "KD": 14,
    "SMOOTH_FACTOR": 0.9,
    "SMOOTH_THRESHOLD": 8,
}

# 传感器权重
SENSOR_WEIGHTS = [-2, -1, 0, 1, 2]
SENSOR_COUNT = 5

# 安全锁阈值
SAFETY_ERROR_THRESHOLD = 8

# 状态定义
STATE_INIT = "INIT"
STATE_CALIBRATE = "CALIBRATE"
STATE_RUN = "RUN"
STATE_RECOVER = "RECOVER"
STATE_TASK = "TASK"
STATE_COMPLETE = "COMPLETE"
STATE_ERROR = "ERROR"


def select_mode(video_segment):
    """智能模式选择"""
    if video_segment == "SAFE":
        return SAFE
    elif video_segment == "PERF_SHOW":
        return PERFORMANCE
    return SAFE


def safety_guard(error):
    """安全锁：误差过大自动切回SAFE"""
    if abs(error) > SAFETY_ERROR_THRESHOLD:
        return "SAFE_MODE"
    return "PERF_MODE"


class HybridPID:
    """混合PID控制器"""
    
    def __init__(self):
        self.config = SAFE
        self.prev_error = 0
        self.integral = 0
        
    def compute(self, error):
        config = self.config
        
        # 智能平滑
        if abs(error) > config["SMOOTH_THRESHOLD"]:
            error *= config["SMOOTH_FACTOR"]
        
        self.integral += error
        self.integral = max(-100, min(100, self.integral))
        derivative = error - self.prev_error
        
        output = config["KP"] * error + config["KI"] * self.integral + config["KD"] * derivative
        self.prev_error = error
        return output
    
    def set_mode(self, config):
        self.config = config
        self.prev_error = 0
        self.integral = 0


class HybridSensor:
    """混合传感器模块"""
    
    def __init__(self):
        self.values = [0] * SENSOR_COUNT
        self.threshold = 500
        self.history = []
        
    def read(self, raw_values):
        self.values = raw_values
        self.history.append(raw_values.copy())
        if len(self.history) > 10:
            self.history.pop(0)
            
    def weighted_error(self):
        error = sum(w * v for w, v in zip(SENSOR_WEIGHTS, self.values))
        return error / 1000.0
    
    def confidence(self):
        total = sum(self.values)
        max_possible = 1000 * SENSOR_COUNT
        return total / max_possible if max_possible > 0 else 0
    
    def detect_line(self):
        return any(v > self.threshold for v in self.values)
    
    def line_lost(self):
        return not self.detect_line()
    
    def is_task_zone(self):
        return all(v > self.threshold for v in self.values)
    
    def is_high_difficulty(self):
        active_count = sum(1 for v in self.values if v > self.threshold)
        return 1 <= active_count <= 2
    
    def calibrate(self):
        self.threshold = max(self.values) * 0.5 if self.values else 500


class HybridMotor:
    """混合电机控制"""
    
    def __init__(self):
        self.left_speed = 0
        self.right_speed = 0
        
    def set(self, left, right):
        self.left_speed = max(-100, min(100, left))
        self.right_speed = max(-100, min(100, right))
        
    def stop(self):
        self.left_speed = 0
        self.right_speed = 0
        
    def forward(self, speed):
        self.set(speed, speed)
        
    def backward(self, speed):
        self.set(-speed, -speed)
        
    def rotate(self, angle):
        speed = 30
        if angle > 0:
            self.set(-speed, speed)
        else:
            self.set(speed, -speed)
    
    def get_commands(self):
        return self.left_speed, self.right_speed


class SmartRecovery:
    """智能恢复管理器"""
    
    def __init__(self):
        self.recovery_count = 0
        self.max_recovery = 3
        self.last_strategy = None
        
    def recover(self, motor, sensor):
        self.recovery_count += 1
        
        if self.recovery_count > self.max_recovery:
            return False
            
        motor.stop()
        
        conf = sensor.confidence()
        
        if conf < 0.3:
            return self._aggressive_search(motor, sensor)
        else:
            return self._micro_adjust(motor, sensor)
    
    def _aggressive_search(self, motor, sensor):
        self.last_strategy = "aggressive"
        
        for angle in [30, -30, 60, -60, 90]:
            motor.rotate(angle)
            if sensor.detect_line():
                self.recovery_count = 0
                return True
        
        motor.backward(20)
        motor.rotate(180)
        
        return sensor.detect_line()
    
    def _micro_adjust(self, motor, sensor):
        self.last_strategy = "micro"
        
        for angle in [10, -10, 20, -20]:
            motor.rotate(angle)
            if sensor.detect_line():
                self.recovery_count = 0
                return True
        
        motor.backward(10)
        
        return sensor.detect_line()
    
    def reset(self):
        self.recovery_count = 0
        self.last_strategy = None


class HybridController:
    """最终版HYBRID_DEMO主控制器"""
    
    def __init__(self):
        self.state = STATE_INIT
        self.pid = HybridPID()
        self.sensor = HybridSensor()
        self.motor = HybridMotor()
        self.recovery = SmartRecovery()
        
        self.current_mode = SAFE
        self.current_segment = "SAFE"
        self.state_history = []
        self.start_time = time.time()
        
    def select_segment(self, segment):
        """选择视频段"""
        self.current_segment = segment
        self.current_mode = select_mode(segment)
        self.pid.set_mode(self.current_mode)
        
    def transition(self, new_state):
        self.state_history.append({
            "time": time.time() - self.start_time,
            "from": self.state,
            "to": new_state
        })
        self.state = new_state
        
    def update(self, sensor_values):
        self.sensor.read(sensor_values)
        
        # 安全锁检查
        error = self.sensor.weighted_error()
        guard_status = safety_guard(error)
        
        if self.current_segment == "PERF_SHOW" and guard_status == "SAFE_MODE":
            # Performance模式下误差过大，自动切回SAFE
            self.select_segment("SAFE")
        
        if self.state == STATE_INIT:
            self._handle_init()
        elif self.state == STATE_CALIBRATE:
            self._handle_calibrate()
        elif self.state == STATE_RUN:
            self._handle_run()
        elif self.state == STATE_RECOVER:
            self._handle_recover()
        elif self.state == STATE_TASK:
            self._handle_task()
        elif self.state in [STATE_COMPLETE, STATE_ERROR]:
            pass
            
        return self.motor.get_commands()
    
    def _handle_init(self):
        self.pid.set_mode(self.current_mode)
        self.recovery.reset()
        self.transition(STATE_CALIBRATE)
        
    def _handle_calibrate(self):
        self.sensor.calibrate()
        speed = self.current_mode["BASE_SPEED"] * 0.5
        self.motor.forward(speed)
        time.sleep(0.3)
        self.motor.stop()
        self.transition(STATE_RUN)
        
    def _handle_run(self):
        if self.sensor.line_lost():
            self.transition(STATE_RECOVER)
        elif self.sensor.is_task_zone():
            self.transition(STATE_TASK)
        else:
            error = self.sensor.weighted_error()
            correction = self.pid.compute(error)
            
            base = self.current_mode["BASE_SPEED"]
            left = base - correction
            right = base + correction
            self.motor.set(left, right)
            
    def _handle_recover(self):
        if self.recovery.recover(self.motor, self.sensor):
            self.transition(STATE_RUN)
        else:
            self.transition(STATE_ERROR)
            
    def _handle_task(self):
        time.sleep(0.5)
        self.motor.stop()
        self.transition(STATE_RUN)
    
    def get_status(self):
        return {
            "mode": self.current_mode,
            "segment": self.current_segment,
            "state": self.state,
            "confidence": self.sensor.confidence(),
            "is_high_difficulty": self.sensor.is_high_difficulty(),
            "recovery_strategy": self.recovery.last_strategy,
            "uptime": time.time() - self.start_time
        }


def create_demo_tasks():
    return [
        {"type": "PUSH", "done": False},
        {"type": "ALIGN", "done": False},
        {"type": "PRESS", "done": False}
    ]
