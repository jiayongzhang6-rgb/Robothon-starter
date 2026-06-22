#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DEMO_SAFE模式 - 零失败移动机器人控制器
核心原则：宁可慢，不出错；宁可重复，也不冒险
"""

import time
import math

# ============== 全局配置 ==============
MODE = "DEMO_SAFE"
BASE_SPEED = 55  # 降速保证稳定
KP = 15  # 降低攻击性
KD = 20  # 增强稳定性
KI = 0

RECOVERY_ENABLE = True
STRICT_LINE_CHECK = True
TASK_TIMEOUT = 8  # 防卡死

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


class PIDController:
    """PID控制器 - 低攻击性版本"""
    
    def __init__(self, kp=KP, ki=KI, kd=KD):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        
    def compute(self, error):
        """计算PID输出"""
        # 强平滑（防抖）
        if abs(error) > 5:
            error *= 0.7
        
        self.integral += error
        self.integral = max(-100, min(100, self.integral))
        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output
    
    def reset(self):
        """重置PID"""
        self.prev_error = 0
        self.integral = 0


class LineSensor:
    """5传感器巡线模块"""
    
    def __init__(self):
        self.values = [0] * SENSOR_COUNT
        self.calibrated = False
        self.threshold = 500
        
    def read(self, raw_values):
        """读取传感器原始值"""
        self.values = raw_values
        
    def weighted_error(self):
        """计算加权误差"""
        error = sum(w * v for w, v in zip(SENSOR_WEIGHTS, self.values))
        return error / 1000.0
    
    def detect_line(self):
        """检测是否在线上"""
        return any(v > self.threshold for v in self.values)
    
    def line_lost(self):
        """检测是否丢线"""
        return not self.detect_line()
    
    def is_task_zone(self):
        """检测是否在任务区（所有传感器都亮）"""
        return all(v > self.threshold for v in self.values)
    
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
        """旋转（正数左转，负数右转）"""
        speed = 30
        if angle > 0:
            self.set(-speed, speed)
        else:
            self.set(speed, -speed)
    
    def get_commands(self):
        """获取电机指令"""
        return self.left_speed, self.right_speed


class TaskExecutor:
    """任务执行器"""
    
    def __init__(self):
        self.current_task = None
        self.task_list = []
        self.task_index = 0
        self.start_time = 0
        
    def set_tasks(self, tasks):
        """设置任务列表"""
        self.task_list = tasks
        self.task_index = 0
        self.current_task = tasks[0] if tasks else None
        
    def start_task(self):
        """开始当前任务"""
        self.start_time = time.time()
        
    def is_timeout(self):
        """检查是否超时"""
        return (time.time() - self.start_time) > TASK_TIMEOUT
    
    def execute(self, motor, vision):
        """执行任务"""
        if not self.current_task:
            return True
            
        if self.is_timeout():
            motor.stop()
            return False
            
        task_type = self.current_task.get("type", "")
        
        if task_type == "PUSH":
            motor.forward(30)
            
        elif task_type == "ALIGN":
            offset = vision.get_offset() if vision else 0
            motor.rotate(offset * 0.5)
            
        elif task_type == "PRESS":
            motor.forward(20)
            time.sleep(0.5)
            motor.stop()
            
        return self.current_task.get("done", False)
    
    def next_task(self):
        """下一个任务"""
        self.task_index += 1
        if self.task_index < len(self.task_list):
            self.current_task = self.task_list[self.task_index]
            return True
        return False


class RecoveryManager:
    """丢线恢复管理器"""
    
    def __init__(self):
        self.recovery_count = 0
        self.max_recovery = 3
        
    def recover(self, motor, sensor):
        """执行恢复操作"""
        self.recovery_count += 1
        
        if self.recovery_count > self.max_recovery:
            return False
            
        motor.stop()
        
        # 左右慢扫
        for angle in [20, -20, 40, -40, 60]:
            motor.rotate(angle)
            if sensor.detect_line():
                self.recovery_count = 0
                return True
        
        # 后退再找
        motor.backward(15)
        motor.rotate(90)
        
        return sensor.detect_line()
    
    def reset(self):
        """重置恢复计数"""
        self.recovery_count = 0


class SafeController:
    """DEMO_SAFE模式主控制器"""
    
    def __init__(self):
        self.state = STATE_INIT
        self.pid = PIDController()
        self.sensor = LineSensor()
        self.motor = MotorController()
        self.task_executor = TaskExecutor()
        self.recovery = RecoveryManager()
        
        self.state_history = []
        self.start_time = time.time()
        
    def transition(self, new_state):
        """状态转换"""
        self.state_history.append({
            "time": time.time() - self.start_time,
            "from": self.state,
            "to": new_state
        })
        self.state = new_state
        
    def update(self, sensor_values, tasks=None):
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
            
        elif self.state == STATE_COMPLETE:
            pass
            
        elif self.state == STATE_ERROR:
            pass
            
        return self.motor.get_commands()
    
    def _handle_init(self):
        """处理初始化状态"""
        self.pid.reset()
        self.recovery.reset()
        self.transition(STATE_CALIBRATE)
        
    def _handle_calibrate(self):
        """处理校准状态"""
        self.sensor.calibrate()
        self.motor.forward(30)
        time.sleep(0.5)
        self.motor.stop()
        self.transition(STATE_RUN)
        
    def _handle_run(self):
        """处理运行状态"""
        if self.sensor.line_lost():
            self.transition(STATE_RECOVER)
        elif self.sensor.is_task_zone():
            self.transition(STATE_TASK)
            self.task_executor.start_task()
        else:
            error = self.sensor.weighted_error()
            correction = self.pid.compute(error)
            left = BASE_SPEED - correction
            right = BASE_SPEED + correction
            self.motor.set(left, right)
            
    def _handle_recover(self):
        """处理恢复状态"""
        if self.recovery.recover(self.motor, self.sensor):
            self.transition(STATE_RUN)
        else:
            self.transition(STATE_ERROR)
            
    def _handle_task(self):
        """处理任务状态"""
        success = self.task_executor.execute(self.motor, None)
        if success:
            if self.task_executor.next_task():
                self.transition(STATE_RUN)
            else:
                self.transition(STATE_COMPLETE)
        elif self.task_executor.is_timeout():
            self.transition(STATE_RUN)
    
    def get_status(self):
        """获取状态信息"""
        return {
            "mode": MODE,
            "state": self.state,
            "error": self.sensor.weighted_error(),
            "line_detected": self.sensor.detect_line(),
            "motor_left": self.motor.left_speed,
            "motor_right": self.motor.right_speed,
            "recovery_count": self.recovery.recovery_count,
            "uptime": time.time() - self.start_time
        }


def create_demo_tasks():
    """创建演示任务"""
    return [
        {"type": "PUSH", "done": False, "description": "推进到任务区"},
        {"type": "ALIGN", "done": False, "description": "对准目标"},
        {"type": "PRESS", "done": False, "description": "按下按钮"}
    ]
