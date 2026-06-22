#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
视觉模块 - 可选扩展（OpenMV/AprilTag）
"""


class VisionSystem:
    """视觉系统"""
    
    def __init__(self):
        self.camera = None
        self.aligned = False
        
    def init_camera(self):
        """初始化摄像头"""
        # 实际硬件时初始化OpenMV
        pass
    
    def detect_object(self):
        """检测物体"""
        # 返回物体位置偏移
        return 0
    
    def aligned(self):
        """检查是否对齐"""
        return self.aligned
    
    def align_to_task(self):
        """对齐到任务物体"""
        offset = self.detect_object()
        if abs(offset) < 2:
            self.aligned = True
        else:
            # 转动对齐
            pass
    
    def detect_apriltag(self):
        """检测AprilTag"""
        # 可选：用于精确定位
        return None
