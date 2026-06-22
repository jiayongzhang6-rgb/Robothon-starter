#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电机控制模块
"""

class MotorController:
    """电机控制器"""
    
    def __init__(self):
        self.left_speed = 0
        self.right_speed = 0
        self.running = False
        
    def set_speed(self, left, right):
        """设置左右电机速度"""
        self.left_speed = max(-100, min(100, left))
        self.right_speed = max(-100, min(100, right))
        self._apply()
        
    def stop(self):
        """停止电机"""
        self.left_speed = 0
        self.right_speed = 0
        self._apply()
        self.running = False
        
    def rotate(self, angle):
        """旋转指定角度"""
        if angle > 0:
            self.set_speed(30, -30)
        else:
            self.set_speed(-30, 30)
        
    def backward(self, distance):
        """后退指定距离"""
        self.set_speed(-35, -35)
        
    def move_toward(self, direction):
        """朝指定方向移动"""
        speed = 50
        self.set_speed(speed - direction * 0.5, speed + direction * 0.5)
        
    def _apply(self):
        """应用速度到硬件（模拟）"""
        # 实际硬件时替换为真实控制
        pass
