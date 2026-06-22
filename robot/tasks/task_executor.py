#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务执行器 - 执行具体任务
"""


class TaskExecutor:
    """任务执行器"""
    
    def __init__(self, motors, sensors):
        self.motors = motors
        self.sensors = sensors
        
    def execute(self, task):
        """执行任务"""
        print(f"[EXECUTOR] 执行任务: {task.name} (类型: {task.type})")
        
        if task.type == "PUSH":
            return self._execute_push(task)
        elif task.type == "PRESS":
            return self._execute_press(task)
        elif task.type == "DELIVER":
            return self._execute_deliver(task)
        else:
            print(f"[EXECUTOR] 未知任务类型: {task.type}")
            return False
    
    def _execute_push(self, task):
        """执行推送任务"""
        # 对齐物体
        if not self._align_to_object():
            return False
        
        # 推送
        distance = task.params.get('distance', 30)
        self.motors.set_speed(50, 50)
        # 实际硬件时等待到达目标距离
        self.motors.stop()
        return True
    
    def _execute_press(self, task):
        """执行按压任务"""
        # 对齐标记
        if not self._align_marker():
            return False
        
        # 按压
        self.motors.set_speed(30, 30)
        # 实际硬件时控制按压力度和时间
        import time
        time.sleep(0.5)
        self.motors.stop()
        return True
    
    def _execute_deliver(self, task):
        """执行配送任务"""
        # 移动到目标
        self.motors.move_toward(0)
        
        # 释放物体
        # 实际硬件时控制夹爪
        self.motors.stop()
        return True
    
    def _align_to_object(self):
        """对齐物体"""
        for _ in range(10):
            # 实际硬件时使用视觉
            offset = 0  # vision.get_offset()
            if abs(offset) < 2:
                return True
            correction = offset * 0.5
            self.motors.rotate(correction)
        return False
    
    def _align_marker(self):
        """对齐标记"""
        for _ in range(10):
            # 实际硬件时使用视觉
            offset = 0  # vision.get_marker_offset()
            if abs(offset) < 2:
                return True
            correction = offset * 0.5
            self.motors.rotate(correction)
        return False
