#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
冠军级巡线传感器 - 5路加权
"""

from ..config import LINE_LOST_THRESHOLD, SENSOR_WEIGHTS


class LineSensor:
    """5路巡线传感器"""
    
    def __init__(self):
        self.values = [0, 0, 0, 0, 0]  # 5路传感器
        self.calibrated = False
        self.threshold = LINE_LOST_THRESHOLD
        
    def calibrate(self):
        """校准传感器"""
        # 实际硬件时读取黑线/白线基准
        self.calibrated = True
        
    def get_values(self):
        """获取传感器值"""
        # 实际硬件时读取真实值
        return self.values
    
    def weighted_error(self):
        """计算加权误差"""
        from ..controller.line_follow import weighted_error
        return weighted_error(self.get_values(), SENSOR_WEIGHTS)
    
    def detect_line(self):
        """检测黑线"""
        return any(v > self.threshold for v in self.get_values())
    
    def line_lost(self):
        """判断丢线"""
        return not self.detect_line()
    
    def is_intersection(self):
        """判断十字路口"""
        # 十字路口：中间3个以上传感器同时检测到黑线
        values = self.get_values()
        count = sum(1 for v in values if v > self.threshold)
        return count >= 3
    
    def is_left_turn(self):
        """判断左转"""
        values = self.get_values()
        return values[0] > self.threshold and values[1] > self.threshold
    
    def is_right_turn(self):
        """判断右转"""
        values = self.get_values()
        return values[3] > self.threshold and values[4] > self.threshold
