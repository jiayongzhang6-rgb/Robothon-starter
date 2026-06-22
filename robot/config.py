#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FFAI Robothon 2026 - 机器人配置参数
所有可调参数集中管理
"""

# ============ PID参数 ============
# 巡线PID
LINE_PID = {
    'kp': 2.0,
    'ki': 0.01,
    'kd': 0.5,
}

# 任务PID（精细控制）
TASK_PID = {
    'kp': 1.5,
    'ki': 0.005,
    'kd': 0.3,
}

# ============ 速度参数 ============
SPEED = {
    'fast': 80,        # 快速段（>100cm）
    'medium': 50,      # 中速段（30-100cm）
    'slow': 30,        # 精细段（<30cm）
    'turn': 40,        # 转弯速度
    'recovery': 35,    # 恢复速度
}

# ============ 传感器阈值 ============
SENSOR = {
    'threshold': 500,       # 黑线检测阈值
    'lost_timeout': 100,    # 丢线超时（ms）
    'center': [0, 0, 1],    # 中心传感器配置
}

# ============ 任务参数 ============
TASK = {
    'align_threshold': 2,    # 对齐阈值（像素）
    'push_distance': 30,     # 推送距离（cm）
    'press_time': 0.5,       # 按压时间（秒）
    'max_retries': 3,        # 最大重试次数
}

# ============ 恢复参数 ============
RECOVERY = {
    'max_errors': 3,         # 最大连续错误次数
    'search_angles': [-30, 30, -60, 60],  # 搜索角度
    'backward_distance': 20, # 后退距离（cm）
    'overheat_wait': 2,      # 过热等待（秒）
}

# ============ 状态机定义 ============
from enum import Enum

class State(Enum):
    INIT = 0
    SEARCH_LINE = 1
    FOLLOW_LINE = 2
    NAVIGATE_TO_TASK = 3
    EXECUTE_TASK = 4
    RECOVER = 5
    FINISH = 6
