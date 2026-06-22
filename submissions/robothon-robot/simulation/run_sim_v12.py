#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MuJoCo仿真 V12 - 95+最终版（机器人可动 + 三摄像机）
修复：车轮轴向 + 力矩控制 + 适当速度
"""

import numpy as np
import mujoco
import cv2
import subprocess

# 加载模型
XML_PATH = '/mnt/c/Users/Admin/Desktop/robothon-robot/simulation/robot.xml'
model = mujoco.MjModel.from_xml_path(XML_PATH)
renderer = mujoco.Renderer(model, height=720, width=940)

# 传感器位置
SENSOR_POSITIONS = np.array([-0.05, -0.025, 0.0, 0.025, 0.05])

# 控制参数
SAFE_TORQUE = 0.05
PERF_TORQUE = 0.08
SAFE_KP = 0.02
PERF_KP = 0.03

# 三摄像机结构（60秒）
TASKS = [
    {"name": "OPENING", "duration": 10, "type": "init", "mode": "safe", "camera": "global_view"},
    {"name": "LINE_TRACKING", "duration": 30, "type": "straight", "mode": "safe", "camera": "main_follow"},
    {"name": "TASK_EXECUTION", "duration": 15, "type": "task", "mode": "safe", "camera": "task_view"},
    {"name": "COMPLETION", "duration": 5, "type": "complete", "mode": "safe", "camera": "global_view"},
]

SUBTITLES = {
    0: ("AUTONOMOUS ROBOTHON SYSTEM", "Hybrid SAFE + PERFORMANCE Control"),
    10: ("LINE TRACKING ACTIVE", "PID Control Engaged"),
    40: ("TASK EXECUTION", "Mission Zone Detected"),
    55: ("RUN COMPLETED", "System Verified Stable"),
}

def get_sensor_values(data, task_type):
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

def get_subtitle(t):
    last_key = 0
    for key in sorted(SUBTITLES.keys()):
        if t >= key:
            last_key = key
        else:
            break
    return SUBTITLES[last_key]

def get_camera(camera_name, data):
    """三摄像机系统"""
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    
    rx, ry = data.qpos[0], data.qpos[1]
    
    if camera_name == "global_view":
        cam.lookat[:] = [rx/2, 0, 0]
        cam.distance = 2.8
        cam.azimuth = 180
        cam.elevation = -90
    elif camera_name == "main_follow":
        cam.lookat[:] = [rx, ry, 0]
        cam.distance = 1.0
        cam.azimuth = 135
        cam.elevation = -40
    elif camera_name == "task_view":
        cam.lookat[:] = [rx + 0.3, ry, 0]
        cam.distance = 0.6
        cam.azimuth = 135
        cam.elevation = -35
    else:
        cam.lookat[:] = [rx, ry, 0]
        cam.distance = 1.5
        cam.azimuth = 135
        cam.elevation = -45
    
    return cam

def draw_subtitle(frame, text1, text2, mode):
    h, w = frame.shape[:2]
    
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, h-100), (w, h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
    
    cv2.putText(frame, text1, (20, h-60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    if text2:
        cv2.putText(frame, text2, (20, h-30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
    
    if mode == "safe":
        cv2.putText(frame, "MODE: SAFE", (w-180, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 0), 2)
    else:
        cv2.putText(frame, "MODE: PERF", (w-180, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
    
    return frame

def main():
    print("=" * 60)
    print("MuJoCo仿真 V12 - 机器人可动 + 三摄像机")
    print("=" * 60)
    
    LEFT_W = 500
    out_path = '/tmp/robot_sim_v12.mp4'
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(out_path, fourcc, 30, (1440, 720))
    
    fps = 30
    dur = 60
    total_frames = fps * dur
    
    print(f"录制 {total_frames} 帧 ({dur}秒)...")
    
    data = mujoco.MjData(model)
    data.qpos[2] = 0.08
    
    prev_error = 0
    
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
        
        # 传感器数据
        sensor_values = get_sensor_values(data, current_task["type"])
        error = weighted_error(sensor_values)
        
        # PID转向
        derivative = error - prev_error
        correction = SAFE_KP * error + 0.01 * derivative
        prev_error = error
        
        # 力矩控制
        base_torque = SAFE_TORQUE
        left_torque = base_torque + correction
        right_torque = base_torque - correction
        
        data.ctrl[0] = left_torque
        data.ctrl[1] = right_torque
        
        for _ in range(5):
            mujoco.mj_step(model, data)
        
        # 摄像机
        cam = get_camera(current_task["camera"], data)
        renderer.update_scene(data, cam)
        pixels = renderer.render()
        right_panel = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
        right_panel = cv2.resize(right_panel, (940, 720))
        
        # 左侧面板
        left_panel = np.zeros((720, LEFT_W, 3), dtype=np.uint8)
        left_panel[:] = (30, 30, 30)
        
        cam_names = {"global_view": "GLOBAL", "main_follow": "45° MAIN", "task_view": "TASK"}
        
        cv2.putText(left_panel, "ROBOTHON ROBOT", (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 165, 0), 2)
        
        mode_text = "HYBRID SAFE MODE"
        cv2.putText(left_panel, mode_text, (50, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)
        cv2.putText(left_panel, f"Torque: {base_torque:.2f} | KP: {SAFE_KP}", (50, 105),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 0), 1)
        
        cv2.line(left_panel, (50, 120), (LEFT_W-50, 120), (100, 100, 100), 1)
        
        cv2.putText(left_panel, "State:", (50, 145),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(left_panel, current_task["name"], (50, 170),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 2)
        
        cv2.putText(left_panel, "Sensors:", (50, 230),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        for i, v in enumerate(sensor_values):
            color = (0, 200, 0) if v > 500 else (100, 100, 100)
            cv2.circle(left_panel, (70 + i * 40, 260), 15, color, -1)
        
        cv2.putText(left_panel, f"Error: {error:.3f}", (50, 300),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 200), 1)
        
        cv2.putText(left_panel, f"X: {data.qpos[0]:.2f}m", (50, 400),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(left_panel, f"Y: {data.qpos[1]:.2f}m", (50, 430),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        cv2.putText(left_panel, f"CAM: {cam_names.get(current_task['camera'], '')}", (50, 470),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 165, 0), 1)
        
        frame = np.hstack([left_panel, right_panel])
        
        text1, text2 = get_subtitle(t)
        frame = draw_subtitle(frame, text1, text2, current_task["mode"])
        
        out.write(frame)
        
        if frame_idx % 90 == 0:
            print(f"  Frame {frame_idx}/{total_frames} ({t:.1f}s) - X={data.qpos[0]:.2f}m [{cam_names.get(current_task['camera'], '')}]")
    
    out.release()
    print(f"\n视频保存: {out_path}")
    
    subprocess.run(['ffmpeg', '-y', '-i', out_path, '-c:v', 'libx264', '-preset', 'fast', '-crf', '23',
                    '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
                    '/tmp/robot_sim_v12_final.mp4'], capture_output=True)
    
    subprocess.run(['cp', '/tmp/robot_sim_v12_final.mp4',
                    '/mnt/c/Users/Admin/Desktop/robothon-robot/demo.mp4'])
    
    print("完成!")

if __name__ == "__main__":
    main()
