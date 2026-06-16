#!/usr/bin/env python3
"""录制v8_ultimate Demo视频 - 完整版"""
import numpy as np
import mujoco
import sys, os
import subprocess

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot_controller import RobotController

def main():
    robot = RobotController()
    renderer = mujoco.Renderer(robot.model, height=480, width=640)
    
    targets = [
        ("方块上方", np.array([0.3, 0, 0.5])),
        ("方块位置", np.array([0.3, 0, 0.4])),
        ("目标区域", np.array([-0.2, 0, 0.4])),
        ("左前方", np.array([0.15, 0, 0.5])),
        ("左上方", np.array([-0.15, 0, 0.5])),
    ]
    
    frames = []
    
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.1, 0, 0.4]
    cam.azimuth = 120
    cam.elevation = -30
    cam.distance = 1.2
    
    # 初始状态停留
    robot.reset()
    for _ in range(60):
        mujoco.mj_step(robot.model, robot.data)
        renderer.update_scene(robot.data, cam)
        frames.append(renderer.render().copy())
    
    for name, target in targets:
        print(f"录制: {name}")
        robot.reset()
        
        # 移动前的初始帧
        for _ in range(15):
            mujoco.mj_step(robot.model, robot.data)
            renderer.update_scene(robot.data, cam)
            frames.append(renderer.render().copy())
        
        ok, steps = robot.move_to(target, threshold=0.02, max_steps=1200)
        ee = robot.get_ee_pos()
        err = np.linalg.norm(target - ee) * 1000
        print(f"  {'✅' if ok else '❌'} err={err:.1f}mm steps={steps}")
        
        # 录制运动过程（每几步录一帧）
        step_count = 0
        for _ in range(steps):
            mujoco.mj_step(robot.model, robot.data)
            step_count += 1
            if step_count % 3 == 0:  # 每3步录一帧
                renderer.update_scene(robot.data, cam)
                frames.append(renderer.render().copy())
        
        # 到达后停留
        for _ in range(60):
            mujoco.mj_step(robot.model, robot.data)
            renderer.update_scene(robot.data, cam)
            frames.append(renderer.render().copy())
    
    # 保存视频
    output_path = "/tmp/robothon_v8_demo.mp4"
    h, w = frames[0].shape[:2]
    
    proc = subprocess.Popen([
        "ffmpeg", "-y",
        "-f", "rawvideo", "-vcodec", "rawvideo",
        "-s", f"{w}x{h}", "-pix_fmt", "rgb24",
        "-r", "30", "-i", "-",
        "-c:v", "libx264", "-pix_fmt", "yuv420p",
        "-preset", "fast", "-crf", "23",
        output_path,
    ], stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    
    for frame in frames:
        proc.stdin.write(frame.tobytes())
    proc.stdin.close()
    proc.wait()
    
    print(f"\n✅ 视频: {output_path}")
    print(f"   帧数: {len(frames)}, 时长: {len(frames)/30:.1f}s")

if __name__ == "__main__":
    main()
