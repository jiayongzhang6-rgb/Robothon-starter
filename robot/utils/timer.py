#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
计时器工具
"""

import time


class Timer:
    """计时器"""
    
    def __init__(self):
        self.start_time = None
        self.laps = []
        
    def start(self):
        """开始计时"""
        self.start_time = time.time()
        
    def lap(self):
        """记录一圈"""
        if self.start_time:
            lap_time = time.time() - self.start_time
            self.laps.append(lap_time)
            self.start_time = time.time()
            return lap_time
        return 0
    
    def elapsed(self):
        """获取总时间"""
        if self.start_time:
            return time.time() - self.start_time
        return 0
    
    def reset(self):
        """重置"""
        self.start_time = None
        self.laps = []
