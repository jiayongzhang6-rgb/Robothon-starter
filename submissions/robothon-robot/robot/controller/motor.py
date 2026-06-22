#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
冠军级电机控制 - 支持Arduino硬件
"""

from ..config import BASE_SPEED, TURN_SPEED, TASK_SPEED, RECOVERY_SPEED


class MotorController:
    """电机控制器 - 双路H桥"""
    
    def __init__(self):
        self.left_speed = 0
        self.right_speed = 0
        self.running = False
        
    def set(self, left, right):
        """设置左右电机速度"""
        self.left_speed = max(-255, min(255, int(left)))
        self.right_speed = max(-255, min(255, int(right)))
        self._apply()
        
    def stop(self):
        """停止"""
        self.left_speed = 0
        self.right_speed = 0
        self._apply()
        self.running = False
        
    def forward(self, speed=None):
        """前进"""
        speed = speed or BASE_SPEED
        self.set(speed, speed)
        
    def backward(self, distance=None):
        """后退"""
        speed = RECOVERY_SPEED
        self.set(-speed, -speed)
        
    def rotate(self, angle):
        """旋转"""
        if angle > 0:
            self.set(TURN_SPEED, -TURN_SPEED)
        else:
            self.set(-TURN_SPEED, TURN_SPEED)
        
    def dynamic_speed(self, error):
        """动态速度控制"""
        abs_error = abs(error)
        if abs_error < 1:
            return 85  # 高速
        elif abs_error < 3:
            return 65  # 中速
        else:
            return 45  # 低速
    
    def _apply(self):
        """应用到硬件"""
        # Arduino版本：
        # void setMotor(int left, int right) {
        #     left = constrain(left, -255, 255);
        #     right = constrain(right, -255, 255);
        #     digitalWrite(7, left > 0);
        #     digitalWrite(8, left <= 0);
        #     analogWrite(5, abs(left));
        #     digitalWrite(9, right > 0);
        #     digitalWrite(10, right <= 0);
        #     analogWrite(6, abs(right));
        # }
        pass
