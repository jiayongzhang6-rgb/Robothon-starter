#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
冠军级巡线控制
"""


def follow(sensor, pid, motor):
    """巡线主函数"""
    # 获取加权误差
    error = sensor.weighted_error()
    
    # PID计算
    correction = pid.compute(error)
    
    # 动态速度
    base = motor.dynamic_speed(error)
    
    # 差速控制
    left = base - correction
    right = base + correction
    
    motor.set(left, right)


def weighted_error(values, weights=None):
    """计算加权误差"""
    if weights is None:
        weights = [-2, -1, 0, 1, 2]
    
    error = 0
    for i in range(min(len(values), len(weights))):
        error += weights[i] * values[i]
    
    return error
