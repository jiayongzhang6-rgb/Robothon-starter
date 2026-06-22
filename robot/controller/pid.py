#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
冠军级PID控制器
"""

class PID:
    """工业级PID控制器"""
    
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev = 0
        self.integral = 0
        self.integral_limit = 100
        
    def compute(self, error):
        """计算PID输出"""
        # 积分项（带限幅）
        self.integral += error
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        
        # 微分项
        derivative = error - self.prev
        
        # PID输出
        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )
        
        self.prev = error
        return output
    
    def reset(self):
        """重置PID"""
        self.prev = 0
        self.integral = 0
    
    def tune(self, kp=None, ki=None, kd=None):
        """在线调参"""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
