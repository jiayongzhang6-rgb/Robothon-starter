#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
传感器模块 - 巡线传感器
"""

from ..config import SENSOR


class LineSensor:
    """巡线传感器"""
    
    def __init__(self):
        self.values = [0, 0, 0]  # [左, 中, 右]
        self.calibrated = False
        
    def calibrate(self):
        """校准传感器"""
        print("[SENSOR] 校准中...")
        # 实际硬件时读取基准值
        self.calibrated = True
        print("[SENSOR] 校准完成")
        
    def get_values(self):
        """获取传感器值"""
        # 实际硬件时读取真实值
        # 模拟：返回 [左, 中, 右]
        return self.values
    
    def detect_line(self):
        """检测是否在黑线上"""
        values = self.get_values()
        return any(v > SENSOR['threshold'] for v in values)
    
    def is_at_position(self, target):
        """检查是否到达目标位置"""
        # 实际硬件时结合编码器/视觉判断
        return False
    
    def get_direction(self, target):
        """获取到目标的方向"""
        # 实际硬件时结合编码器/视觉计算
        return 0
