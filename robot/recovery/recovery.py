#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
冠军级恢复系统 - 三层恢复策略
"""

from ..config import RECOVERY_ANGLES, RECOVERY_BACKWARD


class RecoverySystem:
    """恢复系统"""
    
    def __init__(self, motor, sensor):
        self.motor = motor
        self.sensor = sensor
        
    def run(self):
        """执行恢复"""
        print("[RECOVERY] 开始恢复...")
        
        # 停止
        self.motor.stop()
        
        # 策略1：原地搜索
        if self._search_rotate():
            return True
        
        # 策略2：后退重进
        if self._backward_search():
            return True
        
        # 策略3：大角度旋转
        if self._wide_search():
            return True
        
        print("[RECOVERY] 恢复失败")
        return False
    
    def _search_rotate(self):
        """原地旋转搜索"""
        print("[RECOVERY] 策略1: 旋转搜索")
        for angle in RECOVERY_ANGLES:
            self.motor.rotate(angle)
            if self.sensor.detect_line():
                print(f"[RECOVERY] 找到线 (角度: {angle})")
                return True
        return False
    
    def _backward_search(self):
        """后退搜索"""
        print("[RECOVERY] 策略2: 后退搜索")
        self.motor.backward(RECOVERY_BACKWARD)
        if self.sensor.detect_line():
            print("[RECOVERY] 后退找到线")
            return True
        return False
    
    def _wide_search(self):
        """大角度搜索"""
        print("[RECOVERY] 策略3: 大角度搜索")
        self.motor.rotate(90)
        if self.sensor.detect_line():
            print("[RECOVERY] 大角度找到线")
            return True
        self.motor.rotate(-180)
        if self.sensor.detect_line():
            print("[RECOVERY] 反向找到线")
            return True
        return False
