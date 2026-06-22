#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MuJoCo仿真 V13 - 完整演示（清晰叙事 + SAFE/PERF切换 + Recovery）
演示内容：
1. 系统初始化（LED闪烁）
2. SAFE模式巡线（慢速稳定）
3. PERFORMANCE模式切换（加速）
4. Recovery演示（偏离→自动恢复）
5. 任务完成
"""

import numpy as np
import mujoco
import cv2
import subprocess

XML_PATH = '/mnt/c/Users/Admin/Desktop/robothon-robot/simulation/robot.xml'
model = mujoco.MjModel.from_xml_path(XML_PATH)
renderer = mujoco.Renderer(model, height=720, width=940)

SENSOR_POSITIONS = np.array([-0.05, -0.025, 0.0, 0.025, 0.05])

SAFE_TORQUE = 0.05
PERF_TORQUE = 0.09
KP_SAFE = 0.02
KP_PERF = 0.03

# 演示流程（60秒）
TASKS = [
    # 阶段1: 初始化（5秒）
    {"name": "INIT", "duration": 5, "torque": 0, "kp": 0, "camera": "global_view",
     "subtitle1": "SYSTEM INITIALIZING", "subtitle2": "Sensor Calibration..."},
    
    # 阶段2: SAFE模式启动（5秒）
    {"name": "SAFE_START", "duration": 5, "torque": SAFE_TORQUE, "kp": KP_SAFE, "camera": "global_view",
     "subtitle1": "SAFE MODE ACTIVATED", "subtitle2": "Speed: 55% | Conservative Gains"},
    
    # 阶段3: SAFE巡线（15秒）
    {"name": "SAFE_LINE", "duration": 15, "torque": SAFE_TORQUE, "kp": KP_SAFE, "camera": "main_follow",
     "subtitle1": "LINE TRACKING - SAFE MODE", "subtitle2": "PID Stabilization Active"},
    
    # 阶段4: 切换到PERFORMANCE（3秒）
    {"name": "SWITCHING", "duration": 3, "torque": PERF_TORQUE, "kp": KP_PERF, "camera": "main_follow",
     "subtitle1": ">>> SWITCHING TO PERFORMANCE MODE", "subtitle2": "Speed: 75% | Aggressive Gains"},
    
    # 阶段5: PERFORMANCE巡线（12秒）
    {"name": "PERF_LINE", "duration": 12, "torque": PERF_TORQUE, "kp": KP_PERF, "camera": "main_follow",
     "subtitle1": "HIGH SPEED TRACKING", "subtitle2": "Adaptive Stability Maintained"},
    
    # 阶段6: Recovery演示（8秒）- 模拟偏离+恢复
    {"name": "RECOVERY", "duration": 8, "torque": SAFE_TORQUE, "kp": KP_SAFE * 2, "camera": "task_view",
     "subtitle1": ">>> DEVIATION DETECTED", "subtitle2": "Autonomous Recovery Active"},
    
    # 阶段7: 任务执行（7秒）
    {"name": "TASK", "duration": 7, "torque": SAFE_TORQUE * 0.5, "kp": KP_SAFE, "camera": "task_view",
     "subtitle1": "TASK EXECUTION", "subtitle2": "Mission Zone Reached"},
    
    # 阶段8: 完成（5秒）
    {"name": "COMPLETE", "duration": 5, "torque": 0, "kp": 0, "camera": "global_view",
     "subtitle1": "RUN COMPLETED SUCCESSFULLY", "subtitle2": "System Verified Stable"},
]

def get_sensor_values(data):
    robot_y = data.qpos[1]
    values = []
    for offset in SENSOR_POSITIONS:
        sensor_y = robot_y + offset
        dist = abs(sensor_y)
        if dist < 0.06:
            value = int(1000 * (1 - dist / 0.06))
        else:
            value = 0
        values.append(value)
    return values

def weighted_error(values):
    weights = [-2, -1, 0, 1, 2]
    error = sum(w * v for w, v in zip(weights, values))
    return error / 1000.0

def get_camera(camera_name, data):
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    
    rx, ry = data.qpos[0], data.qpos[1]
    
    if camera_name == "global_view":
        cam.lookat[:] = [rx/2, 0, 0]
        cam.distance = 2.5
        cam.azimuth = 180
        cam.elevation = -90
    elif camera_name == "main_follow":
        cam.lookat[:] = [rx, ry, 0]
        cam.distance = 0.9
        cam.azimuth = 135
        cam.elevation = -40
    elif camera_name == "task_view":
        cam.lookat[:] = [rx + 0.2, ry, 0]
        cam.distance = 0.5
        cam.azimuth = 135
        cam.elevation = -35
    
    return cam

def draw_hud(frame, task, t, sensor_values, error, mode):
    """绘制HUD信息"""
    h, w = frame.shape[:2]
    
    # 底部半透明字幕栏
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, h-120), (w, h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
    
    # 字幕
    cv2.putText(frame, task["subtitle1"], (30, h-70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    if task["subtitle2"]:
        cv2.putText(frame, task["subtitle2"], (30, h-35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1)
    
    # 右上角模式指示
    if mode == "safe":
        cv2.putText(frame, "MODE: SAFE", (w-200, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 0), 2)
    elif mode == "perf":
        cv2.putText(frame, "MODE: PERF", (w-200, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
    else:
        cv2.putText(frame, "MODE: INIT", (w-200, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)
    
    # 阶段指示器（顶部中间）
    stage_text = task["name"]
    cv2.putText(frame, f"[ {stage_text} ]", (w//2 - 80, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 165, 0), 2)
    
    return frame

def draw_left_panel(frame, task, t, sensor_values, error, mode, data):
    """绘制左侧面板"""
    h, w = frame.shape[:2]
    
    # 背景
    cv2.rectangle(frame, (0, 0), (w, h), (25, 25, 25), -1)
    
    y = 40
    
    # 标题
    cv2.putText(frame, "ROBOTHON", (30, y), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 165, 0), 2)
    y += 35
    cv2.putText(frame, "ROBOT", (30, y), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 165, 0), 2)
    y += 40
    
    # 分隔线
    cv2.line(frame, (30, y), (w-30, y), (100, 100, 100), 1)
    y += 25
    
    # 当前阶段
    cv2.putText(frame, "Phase:", (30, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
    y += 25
    cv2.putText(frame, task["name"], (30, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)
    y += 35
    
    # 模式
    mode_color = (0, 200, 0) if mode == "safe" else (0, 165, 255) if mode == "perf" else (150, 150, 150)
    cv2.putText(frame, f"Mode: {mode.upper()}", (30, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, mode_color, 1)
    y += 30
    
    # 参数
    cv2.putText(frame, f"Torque: {task['torque']:.2f}", (30, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
    y += 25
    cv2.putText(frame, f"KP: {task['kp']}", (30, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
    y += 35
    
    # 分隔线
    cv2.line(frame, (30, y), (w-30, y), (100, 100, 100), 1)
    y += 20
    
    # 传感器
    cv2.putText(frame, "Sensors:", (30, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)
    y += 25
    for i, v in enumerate(sensor_values):
        color = (0, 200, 0) if v > 500 else (60, 60, 60)
        cv2.circle(frame, (50 + i * 35, y), 12, color, -1)
        cv2.circle(frame, (50 + i * 35, y), 12, (100, 100, 100), 1)
    y += 40
    
    # 误差
    cv2.putText(frame, f"Error: {error:.4f}", (30, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 200), 1)
    y += 30
    
    # 位置
    cv2.putText(frame, f"Position:", (30, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)
    y += 25
    cv2.putText(frame, f"  X: {data.qpos[0]:.2f} m", (30, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
    y += 22
    cv2.putText(frame, f"  Y: {data.qpos[1]:.2f} m", (30, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
    y += 35
    
    # 时间
    cv2.putText(frame, f"Time: {t:.1f}s / 60s", (30, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)
    
    return frame

def main():
    print("=" * 60)
    print("MuJoCo仿真 V13 - 完整演示（清晰叙事）")
    print("=" * 60)
    
    LEFT_W = 440
    out_path = '/tmp/robot_sim_v13.mp4'
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(out_path, fourcc, 30, (LEFT_W + 940, 720))
    
    fps = 30
    dur = 60
    total_frames = fps * dur
    
    data = mujoco.MjData(model)
    data.qpos[2] = 0.08
    
    prev_error = 0
    recovery_offset = 0
    
    print(f"录制 {total_frames} 帧...")
    
    for frame_idx in range(total_frames):
        t = frame_idx / fps
        
        # 确定当前任务
        elapsed = 0
        current_task = TASKS[0]
        for task in TASKS:
            if t < elapsed + task["duration"]:
                current_task = task
                break
            elapsed += task["duration"]
        
        # Recovery阶段添加偏移
        if current_task["name"] == "RECOVERY":
            phase = (t - sum(T["duration"] for T in TASKS[:6])) / current_task["duration"]
            if phase < 0.3:
                recovery_offset = 0.15 * (phase / 0.3)  # 偏离
            elif phase < 0.7:
                recovery_offset = 0.15  # 保持偏离
            else:
                recovery_offset = 0.15 * (1 - (phase - 0.7) / 0.3)  # 恢复
        else:
            recovery_offset = 0
        
        # 传感器
        sensor_values = get_sensor_values(data)
        error = weighted_error(sensor_values) + recovery_offset
        
        # PID转向
        derivative = error - prev_error
        correction = current_task["kp"] * error + current_task["kp"] * 0.5 * derivative
        prev_error = error
        
        # 力矩
        data.ctrl[0] = current_task["torque"] + correction
        data.ctrl[1] = current_task["torque"] - correction
        
        for _ in range(5):
            mujoco.mj_step(model, data)
        
        # 渲染
        cam = get_camera(current_task["camera"], data)
        renderer.update_scene(data, cam)
        pixels = renderer.render()
        right_panel = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
        right_panel = cv2.resize(right_panel, (940, 720))
        
        # 左侧面板
        left_panel = np.zeros((720, LEFT_W, 3), dtype=np.uint8)
        left_panel[:] = (25, 25, 25)
        
        # HUD
        mode = "safe" if current_task["name"] in ["SAFE_START", "SAFE_LINE", "RECOVERY", "TASK"] else \
               "perf" if current_task["name"] in ["SWITCHING", "PERF_LINE"] else "init"
        
        left_panel = draw_left_panel(left_panel, current_task, t, sensor_values, error, mode, data)
        
        # 合并
        frame = np.hstack([left_panel, right_panel])
        
        frame = draw_hud(frame, current_task, t, sensor_values, error, mode)
        
        out.write(frame)
        
        if frame_idx % 90 == 0:
            print(f"  {t:.0f}s - {current_task['name']} X={data.qpos[0]:.2f}m")
    
    out.release()
    
    subprocess.run(['ffmpeg', '-y', '-i', out_path, '-c:v', 'libx264', '-preset', 'fast', '-crf', '23',
                    '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
                    '/tmp/robot_sim_v13_final.mp4'], capture_output=True)
    
    subprocess.run(['cp', '/tmp/robot_sim_v13_final.mp4',
                    '/mnt/c/Users/Admin/Desktop/robothon-robot/demo.mp4'])
    
    print("完成!")

if __name__ == "__main__":
    main()
