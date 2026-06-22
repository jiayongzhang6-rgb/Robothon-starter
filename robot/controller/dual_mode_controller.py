#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DEMO_SAFE + PERFORMANCE Dual Mode - 冲95+版本
核心：展示系统能力上限，而不只是安全
"""

import time
import math

# ============== Dual Mode配置 ==============

# SAFE模式（稳）
SAFE_CONFIG = {
    "mode": "DEMO_SAFE",
    "BASE_SPEED": 55,
    "KP": 15,
    "KI": 0,
    "KD": 20,
    "SMOOTH_FACTOR": 0.7,
    "SMOOTH_THRESHOLD": 5,
}

# PERFORMANCE模式（快）
PERF_CONFIG = {
    "mode": "PERFORMANCE_DEMO",
    "BASE_SPEED": 75,
    "KP": 20,
    "KI": 0,
    "KD": 14,
    "SMOOTH_FACTOR": 0.9,
    "SMOOTH_THRESHOLD": 8,
}

# 当前模式
CURRENT_MODE = "DEMO_SAFE"

def get_config():
    return SAFE_CONFIG if CURRENT_MODE == "DEMO_SAFE" else PERF_CONFIG

# 传感器权重
SENSOR_WEIGHTS = [-2, -1, 0, 1, 2]
SENSOR_COUNT = 5

# 状态定义
STATE_INIT = "INIT"
STATE_CALIBRATE = "CALIBRATE"
STATE_RUN = "RUN"
STATE_RECOVER = "RECOVER"
STATE_TASK = "TASK"
STATE_COMPLETE = "COMPLETE"
STATE_ERROR = "ERROR"

# 恢复策略阈值
CONFIDENCE_HIGH = 0.7
CONFIDENCE_LOW = 0.3


class SmartPIDController:
    """智能PID控制器 - 支持Dual Mode"""
    
    def __init__(self, config=None):
        self.config = config or get_config()
        self.prev_error = 0
        self.integral = 0
        
    def compute(self, error):
        """计算PID输出"""
        config = get_config()
        
        # 智能平滑（根据confidence调整）
        smooth_threshold = config["SMOOTH_THRESHOLD"]
        smooth_factor = config["SMOOTH_FACTOR"]
        
        if abs(error) > smooth_threshold:
            error *= smooth_factor
        
        self.integral += error
        self.integral = max(-100, min(100, self.integral))
        derivative = error - self.prev_error
        
        output = config["KP"] * error + config["KI"] * self.integral + config["KD"] * derivative
        self.prev_error = error
        return output
    
    def reset(self):
        self.prev_error = 0
        self.integral = 0
    
    def update_config(self, config):
        self.config = config


class AdvancedSensor:
    """高级传感器模块 - 带confidence计算"""
    
    def __init__(self):
        self.values = [0] * SENSOR_COUNT
        self.threshold = 500
        self.calibrated = False
        self.history = []
        
    def read(self, raw_values):
        """读取传感器原始值"""
        self.values = raw_values
        self.history.append(raw_values.copy())
        if len(self.history) > 10:
            self.history.pop(0)
            
    def weighted_error(self):
        """计算加权误差"""
        error = sum(w * v for w, v in zip(SENSOR_WEIGHTS, self.values))
        return error / 1000.0
    
    def confidence(self):
        """计算当前线检测置信度"""
        total = sum(self.values)
        max_possible = 1000 * SENSOR_COUNT
        
        if total == 0:
            return 0.0
        
        # 基于响应强度的置信度
        strength = total / max_possible
        
        # 基于历史稳定性的置信度
        if len(self.history) > 3:
            recent = self.history[-3:]
            variance = sum(sum((a-b)**2 for a, b in zip(recent[i], recent[i+1])) 
                          for i in range(len(recent)-1))
            stability = 1.0 / (1.0 + variance / 1000)
        else:
            stability = 0.5
        
        return (strength * 0.6 + stability * 0.4)
    
    def detect_line(self):
        """检测是否在线上"""
        return any(v > self.threshold for v in self.values)
    
    def line_lost(self):
        """检测是否丢线"""
        return not self.detect_line()
    
    def is_task_zone(self):
        """检测是否在任务区（所有传感器都亮）"""
        return all(v > self.threshold for v in self.values)
    
    def is_high_difficulty(self):
        """检测是否高难度场景（边缘触发）"""
        # 边缘触发：只有1-2个传感器亮
        active_count = sum(1 for v in self.values if v > self.threshold)
        return 1 <= active_count <= 2
    
    def calibrate(self):
        """校准传感器"""
        self.threshold = max(self.values) * 0.5 if self.values else 500
        self.calibrated = True


class MotorController:
    """电机控制模块"""
    
    def __init__(self):
        self.left_speed = 0
        self.right_speed = 0
        self.stopped = False
        
    def set(self, left, right):
        """设置左右电机速度"""
        self.left_speed = max(-100, min(100, left))
        self.right_speed = max(-100, min(100, right))
        self.stopped = False
        
    def stop(self):
        """停止"""
        self.left_speed = 0
        self.right_speed = 0
        self.stopped = True
        
    def forward(self, speed):
        """前进"""
        self.set(speed, speed)
        
    def backward(self, speed):
        """后退"""
        self.set(-speed, -speed)
        
    def rotate(self, angle):
        """旋转"""
        speed = 30 if CURRENT_MODE == "DEMO_SAFE" else 50
        if angle > 0:
            self.set(-speed, speed)
        else:
            self.set(speed, -speed)
    
    def get_commands(self):
        """获取电机指令"""
        return self.left_speed, self.right_speed


class SmartRecovery:
    """智能恢复管理器 - 基于confidence决策"""
    
    def __init__(self):
        self.recovery_count = 0
        self.max_recovery = 3
        self.last_strategy = None
        
    def recover(self, motor, sensor):
        """智能恢复操作"""
        self.recovery_count += 1
        
        if self.recovery_count > self.max_recovery:
            return False
            
        motor.stop()
        
        # 获取当前置信度
        conf = sensor.confidence()
        
        if conf < CONFIDENCE_LOW:
            # 低置信度：激进搜索
            strategy = "aggressive"
            return self._aggressive_search(motor, sensor)
        else:
            # 高置信度：微调
            strategy = "micro"
            return self._micro_adjust(motor, sensor)
    
    def _aggressive_search(self, motor, sensor):
        """激进搜索（低置信度）"""
        self.last_strategy = "aggressive"
        
        # 大幅扫描
        for angle in [30, -30, 60, -60, 90]:
            motor.rotate(angle)
            if sensor.detect_line():
                self.recovery_count = 0
                return True
        
        # 后退再找
        motor.backward(20)
        motor.rotate(180)
        
        return sensor.detect_line()
    
    def _micro_adjust(self, motor, sensor):
        """微调（高置信度）"""
        self.last_strategy = "micro"
        
        # 小幅扫描
        for angle in [10, -10, 20, -20]:
            motor.rotate(angle)
            if sensor.detect_line():
                self.recovery_count = 0
                return True
        
        # 轻微后退
        motor.backward(10)
        
        return sensor.detect_line()
    
    def reset(self):
        """重置恢复计数"""
        self.recovery_count = 0
        self.last_strategy = None


class DualModeController:
    """Dual Mode主控制器"""
    
    def __init__(self, mode="DEMO_SAFE"):
        global CURRENT_MODE
        CURRENT_MODE = mode
        
        self.state = STATE_INIT
        self.pid = SmartPIDController()
        self.sensor = AdvancedSensor()
        self.motor = MotorController()
        self.recovery = SmartRecovery()
        
        self.state_history = []
        self.start_time = time.time()
        self.mode_switch_count = 0
        
    def switch_mode(self, new_mode):
        """切换模式"""
        global CURRENT_MODE
        if new_mode != CURRENT_MODE:
            CURRENT_MODE = new_mode
            self.pid.update_config(get_config())
            self.mode_switch_count += 1
            
    def transition(self, new_state):
        """状态转换"""
        self.state_history.append({
            "time": time.time() - self.start_time,
            "from": self.state,
            "to": new_state
        })
        self.state = new_state
        
    def update(self, sensor_values):
        """主循环更新"""
        self.sensor.read(sensor_values)
        
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
        self.pid.reset()
        self.recovery.reset()
        self.transition(STATE_CALIBRATE)
        
    def _handle_calibrate(self):
        self.sensor.calibrate()
        config = get_config()
        self.motor.forward(config["BASE_SPEED"] * 0.5)
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
            
            config = get_config()
            base = config["BASE_SPEED"]
            
            left = base - correction
            right = base + correction
            self.motor.set(left, right)
            
    def _handle_recover(self):
        if self.recovery.recover(self.motor, self.sensor):
            self.transition(STATE_RUN)
        else:
            self.transition(STATE_ERROR)
            
    def _handle_task(self):
        # 简化任务执行
        time.sleep(0.5)
        self.motor.stop()
        self.transition(STATE_RUN)
    
    def get_status(self):
        """获取状态信息"""
        return {
            "mode": CURRENT_MODE,
            "state": self.state,
            "confidence": self.sensor.confidence(),
            "is_high_difficulty": self.sensor.is_high_difficulty(),
            "recovery_strategy": self.recovery.last_strategy,
            "mode_switches": self.mode_switch_count,
            "uptime": time.time() - self.start_time
        }


def create_demo_tasks():
    """创建演示任务"""
    return [
        {"type": "PUSH", "done": False, "description": "推进到任务区"},
        {"type": "ALIGN", "done": False, "description": "对准目标"},
        {"type": "PRESS", "done": False, "description": "按下按钮"}
    ]
