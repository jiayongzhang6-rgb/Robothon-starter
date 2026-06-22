#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FFAI Robothon 2026 - 演示视频录制
展示状态机控制流程
"""

import numpy as np
import cv2
import time


def create_frame(frame_num, fps, total_frames):
    """创建视频帧"""
    t = frame_num / fps
    progress = frame_num / total_frames
    
    # 背景
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    frame[:] = (30, 30, 30)  # 深灰色背景
    
    # 标题
    cv2.putText(frame, "ROBOTHON ROBOT CONTROLLER", (50, 40), 
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 165, 0), 2)
    cv2.putText(frame, "Engineering-Grade State Machine", (50, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
    
    # 分隔线
    cv2.line(frame, (50, 90), (590, 90), (100, 100, 100), 1)
    
    # 状态机可视化
    states = ["INIT", "SEARCH", "FOLLOW", "NAVIGATE", "EXECUTE", "RECOVER", "FINISH"]
    current_state = min(int(t / 3), len(states) - 1)
    
    y = 130
    for i, state in enumerate(states):
        color = (0, 200, 0) if i == current_state else (100, 100, 100)
        cv2.putText(frame, f"[{state}]", (60, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        if i < len(states) - 1:
            cv2.arrowedLine(frame, (80, y + 5), (80, y + 25), (100, 100, 100), 1)
        y += 30
    
    # PID曲线
    cv2.putText(frame, "PID Control:", (350, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
    for i in range(100):
        x = 350 + i * 2
        y_val = int(200 + 30 * np.sin(i * 0.1 + t * 2))
        cv2.circle(frame, (x, y_val), 2, (0, 200, 0), -1)
    
    # 进度条
    cv2.putText(frame, "Progress:", (50, 420), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
    cv2.rectangle(frame, (150, 405), (550, 425), (100, 100, 100), 2)
    fill_w = int(400 * progress)
    cv2.rectangle(frame, (150, 405), (150 + fill_w, 425), (255, 165, 0), -1)
    
    # 技术参数
    cv2.putText(frame, "Kp=2.0  Ki=0.01  Kd=0.5", (50, 460),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
    
    return frame


def record_demo():
    """录制演示视频"""
    fps = 30
    duration = 21  # 21秒
    total_frames = fps * duration
    
    out_path = '/tmp/robot_demo.mp4'
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(out_path, fourcc, fps, (640, 480))
    
    print(f"Recording {total_frames} frames ({duration}s)...")
    
    for frame_idx in range(total_frames):
        frame = create_frame(frame_idx, fps, total_frames)
        out.write(frame)
        
        if frame_idx % 60 == 0:
            print(f"  Frame {frame_idx}/{total_frames}")
    
    out.release()
    print(f"Video saved: {out_path}")
    
    # 转换
    import subprocess
    subprocess.run(['ffmpeg', '-y', '-i', out_path, '-c:v', 'libx264', '-preset', 'fast', '-crf', '23',
                    '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
                    '/tmp/robot_demo_final.mp4'], capture_output=True)
    
    # 复制
    import shutil
    shutil.copy('/tmp/robot_demo_final.mp4',
                '/mnt/c/Users/Admin/Desktop/robothon-robot/demo.mp4')
    
    print("Done!")


if __name__ == "__main__":
    record_demo()
