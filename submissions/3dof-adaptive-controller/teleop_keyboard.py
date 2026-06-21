#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
键盘遥操作模式 - 3DOF Robot Controller
使用键盘控制机械臂末端执行器移动

控制方式:
  W/S: 前/后 (X轴)
  A/D: 左/右 (Y轴)
  Q/E: 上/下 (Z轴)
  J/K: 夹爪开/合
  R: 重置
  ESC: 退出

运行方式:
  python3 teleop_keyboard.py
"""

import numpy as np
import mujoco
import os
import sys
import tty
import termios

# 添加当前目录到路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot_controller import RobotController


def get_key():
    """获取单个按键（非阻塞）"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def main():
    print("=" * 60)
    print("3DOF Robot Teleoperation Mode")
    print("=" * 60)
    print()
    print("Controls:")
    print("  W/S  - Forward/Backward (X)")
    print("  A/D  - Left/Right (Y)")
    print("  Q/E  - Up/Down (Z)")
    print("  J/K  - Gripper Open/Close")
    print("  R    - Reset Position")
    print("  ESC  - Exit")
    print()
    print("Starting in 2 seconds...")
    
    robot = RobotController()
    renderer = mujoco.Renderer(robot.model, height=480, width=640)
    
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.1, 0, 0.4]
    cam.azimuth = 120
    cam.elevation = -30
    cam.distance = 1.2
    
    robot.reset()
    gripper_val = 0.0  # 0=闭合, 0.02=张开
    step_size = 0.02   # 2cm per keypress
    
    # 创建viewer
    viewer = mujoco.viewer.launch_passive(robot.model, robot.data)
    viewer.cam.lookat[:] = cam.lookat
    viewer.cam.azimuth = cam.azimuth
    viewer.cam.elevation = cam.elevation
    viewer.cam.distance = cam.distance
    
    print("\nReady! Press keys to control the robot.")
    
    try:
        while True:
            # 渲染
            viewer.sync()
            
            # 获取按键
            key = get_key()
            
            if key == '\x1b':  # ESC
                print("\nExiting...")
                break
            
            # 控制逻辑
            dx, dy, dz = 0, 0, 0
            action_desc = ""
            
            if key == 'w' or key == 'W':
                dx = step_size
                action_desc = f"Forward +{step_size*1000:.0f}mm"
            elif key == 's' or key == 'S':
                dx = -step_size
                action_desc = f"Backward -{step_size*1000:.0f}mm"
            elif key == 'a' or key == 'A':
                dy = step_size
                action_desc = f"Left +{step_size*1000:.0f}mm"
            elif key == 'd' or key == 'D':
                dy = -step_size
                action_desc = f"Right -{step_size*1000:.0f}mm"
            elif key == 'q' or key == 'Q':
                dz = step_size
                action_desc = f"Up +{step_size*1000:.0f}mm"
            elif key == 'e' or key == 'E':
                dz = -step_size
                action_desc = f"Down -{step_size*1000:.0f}mm"
            elif key == 'j' or key == 'J':
                gripper_val = 0.02
                action_desc = "Gripper OPEN"
            elif key == 'k' or key == 'K':
                gripper_val = 0.0
                action_desc = "Gripper CLOSE"
            elif key == 'r' or key == 'R':
                robot.reset()
                gripper_val = 0.0
                action_desc = "RESET"
            else:
                continue
            
            # 执行控制
            if dx != 0 or dy != 0 or dz != 0:
                ee, block, touch = robot.teleop_step(dx, dy, dz, gripper_val)
                print(f"  {action_desc} | EE: ({ee[0]:.3f}, {ee[1]:.3f}, {ee[2]:.3f}) | Touch: {touch:.3f}")
            elif "Gripper" in action_desc or "RESET" in action_desc:
                if "RESET" not in action_desc:
                    robot.data.ctrl[3] = gripper_val
                    robot.data.ctrl[4] = gripper_val
                print(f"  {action_desc}")
    
    finally:
        viewer.close()
        print("Teleoperation ended.")


if __name__ == "__main__":
    main()
