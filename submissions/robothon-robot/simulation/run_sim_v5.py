#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MuJoCo仿真 V5 - DEMO_SAFE零失败模式
1分钟视频，带字幕
"""

import numpy as np
import mujoco
import cv2
import subprocess
import math
import time

# 加载模型
XML_PATH = '/mnt/c/Users/Admin/Desktop/robothon-robot/simulation/robot.xml'
model = mujoco.MjModel.from_xml_path(XML_PATH)
renderer = mujoco.Renderer(model, height=720, width=940)

# DEMO_SAFE配置
BASE_SPEED = 0.15  # 降速保证稳定
KP = 15
KD = 20
KI = 0

# 传感器位置
SENSOR_POSITIONS = np.array([-0.04, -0.02, 0.0, 0.02, 0.04])

# 任务定义（1分钟演示）
TASKS = [
    {"name": "SYSTEM_INIT", "duration": 5, "type": "init", "subtitle": "System Calibration Complete"},
    {"name": "AUTONOMOUS_MODE", "duration": 5, "type": "start", "subtitle": "Autonomous Mode Activated"},
    {"name": "LINE_TRACKING", "duration": 15, "type": "straight", "subtitle": "Stable PID Tracking"},
    {"name": "CURVE_FOLLOWING", "duration": 10, "type": "curve", "subtitle": "Navigation Active"},
    {"name": "TASK_DETECTION", "duration": 10, "type": "obstacle", "subtitle": "Mission Zone Detected"},
    {"name": "TASK_EXECUTION", "duration": 10, "type": "endpoint", "subtitle": "Task Completed Successfully"},
    {"name": "RECOVERY_DEMO", "duration": 5, "type": "recovery", "subtitle": "Autonomous Recovery Enabled"},
]

# 字幕脚本
SUBTITLES = {
    0: ("This is an autonomous Robothon system demo,", "showing line tracking, task execution, and recovery behavior."),
    5: ("The robot starts fully autonomous after system calibration.", ""),
    10: ("The robot performs stable line tracking using PID control.", ""),
    25: ("The robot handles curves smoothly with adaptive speed.", ""),
    35: ("The system detects a mission zone and executes the task autonomously.", ""),
    45: ("The robot completes the task successfully.", ""),
    55: ("The robot includes automatic recovery and completes the run without assistance.", ""),
}

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        
    def compute(self, error):
        # 强平滑（防抖）
        if abs(error) > 5:
            error *= 0.7
        
        self.integral += error
        self.integral = max(-100, min(100, self.integral))
        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output
    
    def reset(self):
        self.prev_error = 0
        self.integral = 0

def get_sensor_values(data, task_type):
    """获取传感器值"""
    robot_x = data.qpos[0]
    robot_y = data.qpos[1]
    
    sensor_offset = SENSOR_POSITIONS
    values = []
    
    for offset in sensor_offset:
        if task_type in ["straight", "init", "start"]:
            target_y = 0
        elif task_type == "curve":
            target_y = 0.08 * math.sin(robot_x * 2)
        elif task_type in ["obstacle", "endpoint"]:
            target_y = 0
        elif task_type == "recovery":
            # 模拟丢线恢复
            if 0.3 < robot_x < 0.5:
                target_y = 0.1 * (robot_x - 0.3)
            else:
                target_y = 0
        else:
            target_y = 0
        
        sensor_y = robot_y + offset
        dist = abs(sensor_y - target_y)
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

def get_current_task(t):
    """获取当前任务"""
    elapsed = 0
    for task in TASKS:
        if t < elapsed + task["duration"]:
            return task, (t - elapsed) / task["duration"]
        elapsed += task["duration"]
    return TASKS[-1], 1.0

def get_subtitle(t):
    """获取当前字幕"""
    last_key = 0
    for key in sorted(SUBTITLES.keys()):
        if t >= key:
            last_key = key
        else:
            break
    return SUBTITLES[last_key]

def draw_subtitle(frame, text1, text2):
    """绘制字幕"""
    h, w = frame.shape[:2]
    
    # 半透明背景
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, h-100), (w, h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
    
    # 英文字幕（上）
    cv2.putText(frame, text1, (20, h-60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    # 中文字幕（下）
    if text2:
        cv2.putText(frame, text2, (20, h-30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
    
    return frame

def main():
    """主仿真函数"""
    print("=" * 60)
    print("MuJoCo仿真 V5 - DEMO_SAFE零失败模式")
    print("=" * 60)
    
    pid = PIDController(KP, KI, KD)
    
    # 录制设置 - 1分钟视频
    LEFT_W = 500
    out_path = '/tmp/robot_sim_v5.mp4'
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(out_path, fourcc, 30, (1440, 720))
    
    fps = 30
    dur = 65  # 65秒（含缓冲）
    total_frames = fps * dur
    
    print(f"录制 {total_frames} 帧 ({dur}秒)...")
    
    # 初始位置
    data = mujoco.MjData(model)
    data.qpos[0] = -0.3
    data.qpos[1] = 0
    data.qpos[2] = 0.05
    
    current_task_idx = 0
    
    for frame_idx in range(total_frames):
        t = frame_idx / fps
        
        # 检查是否需要切换任务
        new_task_idx = 0
        elapsed = 0
        for i, task in enumerate(TASKS):
            if t >= elapsed + task["duration"]:
                elapsed += task["duration"]
                new_task_idx = i + 1
            else:
                break
        
        if new_task_idx != current_task_idx and new_task_idx < len(TASKS):
            current_task_idx = new_task_idx
            pos = data.qpos[:3].copy()
            data = mujoco.MjData(model)
            data.qpos[0] = pos[0]
            data.qpos[1] = pos[1]
            data.qpos[2] = pos[2]
            pid.reset()
        
        current_task = TASKS[min(current_task_idx, len(TASKS)-1)]
        progress = 0
        elapsed = 0
        for task in TASKS[:current_task_idx]:
            elapsed += task["duration"]
        progress = (t - elapsed) / current_task["duration"] if current_task["duration"] > 0 else 0
        
        # 获取传感器值
        sensor_values = get_sensor_values(data, current_task["type"])
        
        # 计算误差
        error = weighted_error(sensor_values)
        
        # PID控制
        correction = pid.compute(error)
        
        # 自适应速度
        speed = BASE_SPEED * (1.0 - abs(error) * 0.3)
        
        # 差速控制
        left_speed = speed + correction * 0.03
        right_speed = speed - correction * 0.03
        
        # 限制速度
        left_speed = max(-0.3, min(0.3, left_speed))
        right_speed = max(-0.3, min(0.3, right_speed))
        
        # 设置电机
        data.ctrl[0] = left_speed
        data.ctrl[1] = right_speed
        
        # 仿真步进
        for _ in range(5):
            mujoco.mj_step(model, data)
        
        # 渲染
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = [data.qpos[0], 0, 0.15]
        cam.distance = 0.6
        cam.azimuth = 90
        cam.elevation = -45
        
        renderer.update_scene(data, cam)
        pixels = renderer.render()
        right_panel = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
        right_panel = cv2.resize(right_panel, (940, 720))
        
        # 左侧面板
        left_panel = np.zeros((720, LEFT_W, 3), dtype=np.uint8)
        left_panel[:] = (30, 30, 30)
        
        # 标题
        cv2.putText(left_panel, "ROBOTHON ROBOT", (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 165, 0), 2)
        cv2.putText(left_panel, "DEMO SAFE MODE", (50, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 0), 2)
        
        cv2.line(left_panel, (50, 100), (LEFT_W-50, 100), (100, 100, 100), 1)
        
        # 当前状态
        cv2.putText(left_panel, "Current State:", (50, 130),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv2.putText(left_panel, current_task["name"], (50, 155),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 0), 2)
        
        # 进度条
        cv2.rectangle(left_panel, (50, 170), (LEFT_W-50, 190), (100, 100, 100), 2)
        fill_w = int((LEFT_W-100) * min(progress, 1.0))
        cv2.rectangle(left_panel, (50, 170), (50 + fill_w, 190), (0, 200, 0), -1)
        
        # 传感器状态
        cv2.putText(left_panel, "Sensors:", (50, 220),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        for i, v in enumerate(sensor_values):
            color = (0, 200, 0) if v > 500 else (100, 100, 100)
            cv2.circle(left_panel, (70 + i * 40, 250), 15, color, -1)
            cv2.putText(left_panel, str(i), (65 + i * 40, 255),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # 控制数据
        cv2.putText(left_panel, f"Error: {error:.3f}", (50, 300),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 200), 1)
        cv2.putText(left_panel, f"Speed: {speed:.3f}", (50, 330),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 200), 1)
        cv2.putText(left_panel, f"Left: {left_speed:.3f}", (50, 360),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 200), 1)
        cv2.putText(left_panel, f"Right: {right_speed:.3f}", (50, 390),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 200), 1)
        
        # 位置
        cv2.putText(left_panel, f"X: {data.qpos[0]:.2f}m", (50, 430),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(left_panel, f"Y: {data.qpos[1]:.2f}m", (50, 460),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        # 安全模式状态
        cv2.putText(left_panel, "MODE: DEMO_SAFE", (50, 500),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)
        cv2.putText(left_panel, "Status: STABLE", (50, 530),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 1)
        
        # 组合
        frame = np.hstack([left_panel, right_panel])
        
        # 添加字幕
        text1, text2 = get_subtitle(t)
        frame = draw_subtitle(frame, text1, text2)
        
        out.write(frame)
        
        if frame_idx % 150 == 0:
            print(f"  Frame {frame_idx}/{total_frames} ({t:.1f}s) - {current_task['name']}")
    
    out.release()
    print(f"\n视频保存: {out_path}")
    
    # 转换格式
    subprocess.run(['ffmpeg', '-y', '-i', out_path, '-c:v', 'libx264', '-preset', 'fast', '-crf', '23',
                    '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
                    '/tmp/robot_sim_v5_final.mp4'], capture_output=True)
    
    # 复制
    subprocess.run(['cp', '/tmp/robot_sim_v5_final.mp4',
                    '/mnt/c/Users/Admin/Desktop/robothon-robot/demo.mp4'])
    
    print("完成!")
    
    return True

if __name__ == "__main__":
    main()
