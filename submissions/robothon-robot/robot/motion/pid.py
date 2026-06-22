#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PID控制器
"""

class PID:
    """经典PID控制器"""
    
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.integral_limit = 100  # 积分限幅
        
    def compute(self, error):
        """计算PID输出"""
        # 积分项（带限幅）
        self.integral += error
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        
        # 微分项
        derivative = error - self.prev_error
        
        # PID输出
        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )
        
        self.prev_error = error
        return output
    
    def reset(self):
        """重置PID状态"""
        self.prev_error = 0
        self.integral = 0
