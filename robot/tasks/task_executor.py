#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
冠军级任务执行器
"""


class TaskExecutor:
    """任务执行器"""
    
    def __init__(self, motor, vision):
        self.motor = motor
        self.vision = vision
        
    def execute(self, task):
        """执行任务"""
        print(f"[EXEC] 执行: {task.name} ({task.type})")
        
        if task.type == "PUSH":
            return self._push(task)
        elif task.type == "PRESS":
            return self._press(task)
        elif task.type == "DELIVER":
            return self._deliver(task)
        else:
            print(f"[EXEC] 未知类型: {task.type}")
            return False
    
    def _push(self, task):
        """推送任务"""
        # 接近物体
        self.motor.forward(50)
        
        # 推送
        distance = task.params.get('distance', 30)
        self.motor.forward(60)
        
        # 完成
        self.motor.stop()
        return True
    
    def _press(self, task):
        """按压任务"""
        # 对齐
        self.vision.align_to_task()
        
        # 按压
        self.motor.forward(40)
        import time
        time.sleep(0.5)
        self.motor.stop()
        return True
    
    def _deliver(self, task):
        """配送任务"""
        # 导航到目标
        self.motor.forward(50)
        
        # 释放
        self.motor.stop()
        return True
