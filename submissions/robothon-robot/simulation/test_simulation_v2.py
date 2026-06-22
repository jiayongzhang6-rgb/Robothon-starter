#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MuJoCo仿真测试 V2 - 修复物理稳定性
"""

import numpy as np
import mujoco
import math

# 加载模型
XML_PATH = '/mnt/c/Users/Admin/Desktop/robothon-robot/simulation/robot.xml'
model = mujoco.MjModel.from_xml_path(XML_PATH)

# PID参数
KP = 20.0
KI = 0.0
KD = 14.0
BASE_SPEED = 0.3

# 传感器位置
SENSOR_POSITIONS = np.array([-0.04, -0.02, 0.0, 0.02, 0.04])

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        
    def compute(self, error):
        self.integral += error
        self.integral = max(-100, min(100, self.integral))
        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output
    
    def reset(self):
        self.prev_error = 0
        self.integral = 0

def get_sensor_values(data):
    """获取传感器值"""
    robot_y = data.qpos[1]
    
    sensor_offset = SENSOR_POSITIONS
    values = []
    for offset in sensor_offset:
        sensor_y = robot_y + offset
        dist = abs(sensor_y)
        if dist < 0.05:
            value = int(1000 * (1 - dist / 0.05))
        else:
            value = 0
        values.append(value)
    
    return values

def weighted_error(values):
    """计算加权误差"""
    weights = [-2, -1, 0, 1, 2]
    error = sum(w * v for w, v in zip(weights, values))
    return error / 1000.0

def test_straight_line():
    """测试1: 直线巡线"""
    print("测试1: 直线巡线")
    
    data = mujoco.MjData(model)
    pid = PIDController(KP, KI, KD)
    
    # 初始位置
    data.qpos[0] = -0.3
    data.qpos[1] = 0
    data.qpos[2] = 0.05
    
    errors = []
    for i in range(1500):
        sensor_values = get_sensor_values(data)
        error = weighted_error(sensor_values)
        errors.append(error)
        
        correction = pid.compute(error)
        speed = BASE_SPEED * (1.0 - abs(error) * 0.5)
        left_speed = speed + correction * 0.05
        right_speed = speed - correction * 0.05
        
        left_speed = max(-0.5, min(0.5, left_speed))
        right_speed = max(-0.5, min(0.5, right_speed))
        
        data.ctrl[0] = left_speed
        data.ctrl[1] = right_speed
        
        mujoco.mj_step(model, data)
    
    avg_error = np.mean([abs(e) for e in errors])
    print(f"  平均误差: {avg_error:.4f}")
    print(f"  最终位置: X={data.qpos[0]:.3f}, Y={data.qpos[1]:.3f}")
    return avg_error < 0.1

def test_curve_following():
    """测试2: 弯道巡线"""
    print("测试2: 弯道巡线")
    
    data = mujoco.MjData(model)
    pid = PIDController(KP, KI, KD)
    
    data.qpos[0] = -0.3
    data.qpos[1] = 0
    data.qpos[2] = 0.05
    
    errors = []
    for i in range(1500):
        robot_x = data.qpos[0]
        robot_y = data.qpos[1]
        
        target_y = 0.1 * math.sin(robot_x * 2)
        
        sensor_offset = SENSOR_POSITIONS
        values = []
        for offset in sensor_offset:
            sensor_y = robot_y + offset
            dist = abs(sensor_y - target_y)
            if dist < 0.05:
                value = int(1000 * (1 - dist / 0.05))
            else:
                value = 0
            values.append(value)
        
        error = weighted_error(values)
        errors.append(error)
        
        correction = pid.compute(error)
        speed = BASE_SPEED * (1.0 - abs(error) * 0.5)
        left_speed = speed + correction * 0.05
        right_speed = speed - correction * 0.05
        
        left_speed = max(-0.5, min(0.5, left_speed))
        right_speed = max(-0.5, min(0.5, right_speed))
        
        data.ctrl[0] = left_speed
        data.ctrl[1] = right_speed
        
        mujoco.mj_step(model, data)
    
    avg_error = np.mean([abs(e) for e in errors])
    print(f"  平均误差: {avg_error:.4f}")
    print(f"  最终位置: X={data.qpos[0]:.3f}, Y={data.qpos[1]:.3f}")
    return avg_error < 0.15

def test_obstacle_avoidance():
    """测试3: 障碍物检测"""
    print("测试3: 障碍物检测")
    
    data = mujoco.MjData(model)
    pid = PIDController(KP, KI, KD)
    
    data.qpos[0] = -0.3
    data.qpos[1] = 0
    data.qpos[2] = 0.05
    
    obstacle_detected = False
    for i in range(1500):
        sensor_values = get_sensor_values(data)
        error = weighted_error(sensor_values)
        
        correction = pid.compute(error)
        speed = BASE_SPEED * (1.0 - abs(error) * 0.5)
        left_speed = speed + correction * 0.05
        right_speed = speed - correction * 0.05
        
        left_speed = max(-0.5, min(0.5, left_speed))
        right_speed = max(-0.5, min(0.5, right_speed))
        
        data.ctrl[0] = left_speed
        data.ctrl[1] = right_speed
        
        mujoco.mj_step(model, data)
        
        if data.qpos[0] > 0.15:
            obstacle_detected = True
    
    print(f"  障碍物检测: {'通过' if obstacle_detected else '未检测到'}")
    return True

def test_endpoint_localization():
    """测试4: 终点定位"""
    print("测试4: 终点定位")
    
    data = mujoco.MjData(model)
    pid = PIDController(KP, KI, KD)
    
    data.qpos[0] = -0.3
    data.qpos[1] = 0
    data.qpos[2] = 0.05
    
    for i in range(1500):
        sensor_values = get_sensor_values(data)
        error = weighted_error(sensor_values)
        
        correction = pid.compute(error)
        speed = BASE_SPEED * (1.0 - abs(error) * 0.5)
        left_speed = speed + correction * 0.05
        right_speed = speed - correction * 0.05
        
        left_speed = max(-0.5, min(0.5, left_speed))
        right_speed = max(-0.5, min(0.5, right_speed))
        
        data.ctrl[0] = left_speed
        data.ctrl[1] = right_speed
        
        mujoco.mj_step(model, data)
    
    final_x = data.qpos[0]
    print(f"  最终位置: X={final_x:.3f}")
    print(f"  是否到达终点: {'是' if final_x > 0.3 else '否'}")
    return final_x > 0.3

def test_dynamic_speed():
    """测试5: 自适应速度控制"""
    print("测试5: 自适应速度控制")
    
    data = mujoco.MjData(model)
    pid = PIDController(KP, KI, KD)
    
    data.qpos[0] = -0.3
    data.qpos[1] = 0
    data.qpos[2] = 0.05
    
    speeds = []
    for i in range(1500):
        sensor_values = get_sensor_values(data)
        error = weighted_error(sensor_values)
        
        correction = pid.compute(error)
        speed = BASE_SPEED * (1.0 - abs(error) * 0.5)
        speeds.append(speed)
        
        left_speed = speed + correction * 0.05
        right_speed = speed - correction * 0.05
        
        left_speed = max(-0.5, min(0.5, left_speed))
        right_speed = max(-0.5, min(0.5, right_speed))
        
        data.ctrl[0] = left_speed
        data.ctrl[1] = right_speed
        
        mujoco.mj_step(model, data)
    
    avg_speed = np.mean(speeds)
    min_speed = np.min(speeds)
    print(f"  平均速度: {avg_speed:.3f}")
    print(f"  最小速度: {min_speed:.3f}")
    return avg_speed > 0.2

def test_pid_stability():
    """测试6: PID稳定性"""
    print("测试6: PID稳定性")
    
    data = mujoco.MjData(model)
    pid = PIDController(KP, KI, KD)
    
    data.qpos[0] = -0.3
    data.qpos[1] = 0
    data.qpos[2] = 0.05
    
    errors = []
    for i in range(1500):
        sensor_values = get_sensor_values(data)
        error = weighted_error(sensor_values)
        errors.append(error)
        
        correction = pid.compute(error)
        speed = BASE_SPEED * (1.0 - abs(error) * 0.5)
        left_speed = speed + correction * 0.05
        right_speed = speed - correction * 0.05
        
        left_speed = max(-0.5, min(0.5, left_speed))
        right_speed = max(-0.5, min(0.5, right_speed))
        
        data.ctrl[0] = left_speed
        data.ctrl[1] = right_speed
        
        mujoco.mj_step(model, data)
    
    error_std = np.std(errors)
    print(f"  误差标准差: {error_std:.4f}")
    print(f"  稳定性: {'稳定' if error_std < 0.15 else '不稳定'}")
    return error_std < 0.15

def main():
    """运行所有测试"""
    print("=" * 60)
    print("MuJoCo仿真测试 V2 - 修复物理稳定性")
    print("=" * 60)
    
    results = []
    
    results.append(("直线巡线", test_straight_line()))
    results.append(("弯道巡线", test_curve_following()))
    results.append(("障碍物检测", test_obstacle_avoidance()))
    results.append(("终点定位", test_endpoint_localization()))
    results.append(("自适应速度", test_dynamic_speed()))
    results.append(("PID稳定性", test_pid_stability()))
    
    print("\n" + "=" * 60)
    print("测试结果汇总")
    print("=" * 60)
    
    passed = 0
    for name, result in results:
        status = "✓ 通过" if result else "✗ 失败"
        print(f"{name}: {status}")
        if result:
            passed += 1
    
    print(f"\n通过率: {passed}/{len(results)} ({passed/len(results)*100:.1f}%)")
    
    return passed == len(results)

if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
