#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MuJoCo仿真 V11 - 95+最终版（三摄像机系统+自动切换）
核心：global_view → main_follow → task_view → global_view
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

# Dual Mode配置
SAFE_SPEED = 0.15
PERF_SPEED = 0.35
SAFE_KP = 15
PERF_KP = 20
SAFETY_THRESHOLD = 8

# 三摄像机结构（60秒）
TASKS = [
    # 0-10s: global_view（展示场地）
    {"name": "OPENING", "duration": 10, "type": "init", "mode": "safe", "camera": "global_view"},
    # 10-40s: main_follow（45°斜角）
    {"name": "LINE_TRACKING", "duration": 30, "type": "straight", "mode": "safe", "camera": "main_follow"},
    # 40-55s: task_view（任务特写）
    {"name": "TASK_EXECUTION", "duration": 15, "type": "task", "mode": "safe", "camera": "task_view"},
    # 55-60s: global_view（收尾）
    {"name": "COMPLETION", "duration": 5, "type": "complete", "mode": "safe", "camera": "global_view"},
]

# 字幕脚本
SUBTITLES = {
    0: ("AUTONOMOUS ROBOTHON SYSTEM", "Hybrid SAFE + PERFORMANCE Control"),
    10: ("LINE TRACKING ACTIVE", "PID Control Engaged"),
    40: ("TASK EXECUTION", "Mission Zone Detected"),
    55: ("RUN COMPLETED", "System Verified Stable"),
}

class HybridPID:
    def __init__(self, kp, kd, smooth_threshold, smooth_factor):
        self.kp = kp
        self.kd = kd
        self.smooth_threshold = smooth_threshold
        self.smooth_factor = smooth_factor
        self.prev_error = 0
        
    def compute(self, error):
        if abs(error) > self.smooth_threshold:
            error *= self.smooth_factor
        derivative = error - self.prev_error
        output = self.kp * error + self.kd * derivative
        self.prev_error = error
        return output
    
    def reset(self):
        self.prev_error = 0

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

def compute_confidence(values):
    total = sum(values)
    max_possible = 1000 * 5
    return total / max_possible if max_possible > 0 else 0

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

def get_camera_by_name(camera_name):
    """使用动态自由摄像机模拟三摄像机系统"""
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    
    # 三摄像机配置
    if camera_name == "global_view":
        cam.lookat[:] = [0, 0, 0]
        cam.distance = 2.8
        cam.azimuth = 180
        cam.elevation = -90
    elif camera_name == "main_follow":
        cam.lookat[:] = [0, 0, 0]
        cam.distance = 1.3
        cam.azimuth = 135
        cam.elevation = -40
    elif camera_name == "task_view":
        cam.lookat[:] = [0, 0, 0]
        cam.distance = 0.9
        cam.azimuth = 135
        cam.elevation = -45
    else:
        cam.lookat[:] = [0, 0, 0]
        cam.distance = 2.8
        cam.azimuth = 180
        cam.elevation = -90
    
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
    print("MuJoCo仿真 V11 - 三摄像机系统（95+最终版）")
    print("=" * 60)
    
    safe_pid = HybridPID(SAFE_KP, 20, 5, 0.7)
    
    LEFT_W = 500
    out_path = '/tmp/robot_sim_v11.mp4'
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(out_path, fourcc, 30, (1440, 720))
    
    fps = 30
    dur = 60
    total_frames = fps * dur
    
    print(f"录制 {total_frames} 帧 ({dur}秒)...")
    
    data = mujoco.MjData(model)
    data.qpos[0] = -0.3
    data.qpos[1] = 0
    data.qpos[2] = 0.08
    
    current_task_idx = 0
    current_pid = safe_pid
    
    for frame_idx in range(total_frames):
        t = frame_idx / fps
        
        # 确定当前任务
        elapsed = 0
        new_task_idx = 0
        for i, task in enumerate(TASKS):
            if t >= elapsed + task["duration"]:
                elapsed += task["duration"]
                new_task_idx = i + 1
            else:
                break
        
        if new_task_idx != current_task_idx and new_task_idx < len(TASKS):
            current_task_idx = new_task_idx
            current_pid.reset()
        
        current_task = TASKS[min(current_task_idx, len(TASKS)-1)]
        
        # 获取传感器数据
        sensor_values = get_sensor_values(data, current_task["type"])
        error = weighted_error(sensor_values)
        
        # PID控制
        correction = current_pid.compute(error)
        
        base_speed = SAFE_SPEED
        speed = base_speed * (1.0 - abs(error) * 0.3)
        
        left_speed = speed + correction * 0.03
        right_speed = speed - correction * 0.03
        
        left_speed = max(-0.5, min(0.5, left_speed))
        right_speed = max(-0.5, min(0.5, right_speed))
        
        data.ctrl[0] = left_speed
        data.ctrl[1] = right_speed
        
        for _ in range(5):
            mujoco.mj_step(model, data)
        
        # 三摄像机渲染
        cam = get_camera_by_name(current_task["camera"])
        renderer.update_scene(data, cam)
        pixels = renderer.render()
        right_panel = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
        right_panel = cv2.resize(right_panel, (940, 720))
        
        # 左侧面板
        left_panel = np.zeros((720, LEFT_W, 3), dtype=np.uint8)
        left_panel[:] = (30, 30, 30)
        
        cv2.putText(left_panel, "ROBOTHON ROBOT", (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 165, 0), 2)
        
        cv2.putText(left_panel, "HYBRID SAFE MODE", (50, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)
        cv2.putText(left_panel, "Speed: 55 | KP: 15 | KD: 20", (50, 105),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 0), 1)
        
        cv2.line(left_panel, (50, 120), (LEFT_W-50, 120), (100, 100, 100), 1)
        
        cv2.putText(left_panel, "State:", (50, 145),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(left_panel, current_task["name"], (50, 170),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 2)
        
        confidence = compute_confidence(sensor_values)
        cv2.rectangle(left_panel, (50, 185), (LEFT_W-50, 205), (100, 100, 100), 2)
        fill_w = int((LEFT_W-100) * min(confidence, 1.0))
        cv2.rectangle(left_panel, (50, 185), (50 + fill_w, 205), (0, 200, 0), -1)
        
        cv2.putText(left_panel, "Sensors:", (50, 230),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        for i, v in enumerate(sensor_values):
            color = (0, 200, 0) if v > 500 else (100, 100, 100)
            cv2.circle(left_panel, (70 + i * 40, 260), 15, color, -1)
        
        cv2.putText(left_panel, f"Error: {error:.3f}", (50, 300),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 200), 1)
        cv2.putText(left_panel, f"Speed: {speed:.3f}", (50, 330),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 200), 1)
        
        cv2.putText(left_panel, f"X: {data.qpos[0]:.2f}m", (50, 400),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(left_panel, f"Y: {data.qpos[1]:.2f}m", (50, 430),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        # 显示摄像机名称
        cam_names = {"global_view": "GLOBAL", "main_follow": "45° MAIN", "task_view": "TASK"}
        cv2.putText(left_panel, f"CAM: {cam_names.get(current_task['camera'], current_task['camera'])}", (50, 470),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 165, 0), 1)
        
        frame = np.hstack([left_panel, right_panel])
        
        text1, text2 = get_subtitle(t)
        frame = draw_subtitle(frame, text1, text2, current_task["mode"])
        
        out.write(frame)
        
        if frame_idx % 90 == 0:
            print(f"  Frame {frame_idx}/{total_frames} ({t:.1f}s) - {current_task['name']} [{cam_names.get(current_task['camera'], current_task['camera'])}]")
    
    out.release()
    print(f"\n视频保存: {out_path}")
    
    subprocess.run(['ffmpeg', '-y', '-i', out_path, '-c:v', 'libx264', '-preset', 'fast', '-crf', '23',
                    '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
                    '/tmp/robot_sim_v11_final.mp4'], capture_output=True)
    
    subprocess.run(['cp', '/tmp/robot_sim_v11_final.mp4',
                    '/mnt/c/Users/Admin/Desktop/robothon-robot/demo.mp4'])
    
    print("完成!")

if __name__ == "__main__":
    main()
