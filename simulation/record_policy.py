#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MuJoCo 仿真 - PatrolPolicy 驱动
左面板: AI 决策日志 (behavior, Q-values, decision trace)
右面板: MuJoCo 3D 仿真
视频展示行为切换: explore → cautious → return_to_base → recover
"""

import os
import sys
import math
import numpy as np

# 项目路径
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import mujoco
import mujoco.viewer
import cv2
from robot.policy.patrol_policy import PatrolPolicy, Behavior

# ============ 配置 ============
WIDTH, HEIGHT = 1440, 720
PANEL_W = 400
SIM_W = WIDTH - PANEL_W
FPS = 30
DURATION = 30  # 秒
VIDEO_PATH = os.path.join(ROOT, "demo.mp4")

# ============ 颜色 ============
BG = (30, 30, 30)
WHITE = (255, 255, 255)
CYAN = (200, 220, 255)
GREEN = (80, 220, 120)
YELLOW = (80, 220, 255)
RED = (80, 80, 255)
ORANGE = (50, 180, 255)
GRAY = (120, 120, 120)
DARK = (50, 50, 50)

BEH_COLORS = {
    "explore": GREEN,
    "patrol": CYAN,
    "cautious_patrol": YELLOW,
    "obstacle_avoid": ORANGE,
    "return_to_base": RED,
    "recover": (100, 100, 255),
}


def draw_panel(frame, policy, step, t, fps_actual):
    """左面板: AI 决策信息"""
    panel = np.full((HEIGHT, PANEL_W, 3), 25, dtype=np.uint8)

    y = 30
    # 标题
    cv2.putText(panel, "ADAPTIVE PATROL AGENT", (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, GREEN, 1, cv2.LINE_AA)
    y += 20
    cv2.putText(panel, "Hierarchical Q-learning Policy", (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, GRAY, 1, cv2.LINE_AA)

    y += 35
    cv2.line(panel, (15, y), (PANEL_W - 15, y), DARK, 1)

    # 当前行为
    y += 25
    beh = policy._behavior.value
    beh_color = BEH_COLORS.get(beh, WHITE)
    cv2.putText(panel, "BEHAVIOR", (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, GRAY, 1, cv2.LINE_AA)
    y += 22
    cv2.putText(panel, beh.upper(), (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, beh_color, 2, cv2.LINE_AA)

    # 目标
    y += 30
    cv2.putText(panel, "TARGET", (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, GRAY, 1, cv2.LINE_AA)
    y += 18
    target = policy._target_wp or "none"
    cv2.putText(panel, target, (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, CYAN, 1, cv2.LINE_AA)

    # 位置
    y += 25
    pos = policy._robot_pos
    cv2.putText(panel, f"POS: ({pos[0]:.2f}, {pos[1]:.2f})", (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, WHITE, 1, cv2.LINE_AA)

    y += 35
    cv2.line(panel, (15, y), (PANEL_W - 15, y), DARK, 1)

    # Q-values
    y += 20
    cv2.putText(panel, "Q-VALUES", (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, GRAY, 1, cv2.LINE_AA)
    y += 5

    state = policy.behavior_selector._last_state
    if state and state in policy.behavior_selector._q_table:
        q_vals = policy.behavior_selector._q_table[state]
        sorted_q = sorted(q_vals.items(), key=lambda x: -x[1])
        for beh_name, q_val in sorted_q[:5]:
            y += 18
            bar_len = max(0, int(q_val * 25))
            color = BEH_COLORS.get(beh_name, WHITE)
            cv2.putText(panel, f"{beh_name[:12]:12s}", (15, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, color, 1, cv2.LINE_AA)
            cv2.rectangle(panel, (180, y - 8), (180 + bar_len, y - 2), color, -1)
            cv2.putText(panel, f"{q_val:.2f}", (190 + bar_len, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, color, 1, cv2.LINE_AA)

    y += 30
    cv2.line(panel, (15, y), (PANEL_W - 15, y), DARK, 1)

    # 最近决策 (Decision Trace)
    y += 18
    cv2.putText(panel, "DECISION TRACE", (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, GRAY, 1, cv2.LINE_AA)
    y += 5

    entries = policy.decision_log.entries[-4:]
    for e in entries:
        y += 16
        cand_str = ",".join([c[:6] for c in e.get("candidates", [])[:3]])
        cv2.putText(panel, f"s{e['step']}: {cand_str}", (15, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.28, CYAN, 1, cv2.LINE_AA)
        y += 14
        sel = e.get("selected_wp", "?")
        cost = e.get("selected_cost") or 0
        cv2.putText(panel, f"  -> {sel} ({cost:.3f})", (15, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.28, GREEN, 1, cv2.LINE_AA)

    y += 25
    cv2.line(panel, (15, y), (PANEL_W - 15, y), DARK, 1)

    # 统计
    y += 18
    cv2.putText(panel, "STATS", (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, GRAY, 1, cv2.LINE_AA)
    y += 18
    cv2.putText(panel, f"Step: {step}  Time: {t:.1f}s", (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, WHITE, 1, cv2.LINE_AA)
    y += 16
    unvisited = policy._count_unvisited()
    cv2.putText(panel, f"Unvisited: {unvisited}  Visits: {len(policy.route_sampler._visit_counts)}", (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, WHITE, 1, cv2.LINE_AA)
    y += 16
    cv2.putText(panel, f"FPS: {fps_actual:.0f}", (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, WHITE, 1, cv2.LINE_AA)

    return panel


def run_simulation():
    """主仿真循环"""
    model = mujoco.MjModel.from_xml_path(os.path.join(ROOT, "simulation", "robot.xml"))
    data = mujoco.MjData(model)

    # 初始状态
    mujoco.mj_resetData(model, data)
    data.qpos[0] = 0.0   # x
    data.qpos[1] = 0.0   # y
    data.qpos[2] = 0.08  # z
    data.qpos[3] = 1.0   # qw

    # 策略
    policy = PatrolPolicy(seed=42)
    policy.reset()

    dt_sim = model.opt.timestep
    steps_per_frame = int(1.0 / (FPS * dt_sim))
    total_steps = int(DURATION * FPS * steps_per_frame)

    # 视频写入
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(VIDEO_PATH, fourcc, FPS, (WIDTH, HEIGHT))

    print(f"Recording {DURATION}s video at {FPS}fps...")
    print(f"  Total frames: {DURATION * FPS}")
    print(f"  Steps per frame: {steps_per_frame}")

    frame_count = 0
    last_behavior = None
    behavior_changes = []

    for step in range(total_steps):
        t = step * dt_sim

        # 物理步进
        mujoco.mj_step(model, data)

        # 每帧处理
        if step % steps_per_frame == 0:
            frame_idx = step // steps_per_frame
            real_t = frame_idx / FPS

            # 获取机器人位置和朝向
            robot_pos = (float(data.qpos[0]), float(data.qpos[1]))
            qw, qx, qy, qz = data.qpos[3], data.qpos[4], data.qpos[5], data.qpos[6]
            angle = 2 * math.atan2(qz, qw)
            heading = (math.cos(angle), math.sin(angle))

            # 计算环境状态
            # 障碍密度
            obstacles = [(0.5, 0.15), (0.8, -0.1), (0.35, -0.2), (1.0, 0.2)]
            near_count = sum(1 for ox, oy in obstacles
                           if math.sqrt((robot_pos[0]-ox)**2 + (robot_pos[1]-oy)**2) < 0.25)
            obstacle_density = min(1.0, near_count / 2.0)

            # 电池
            battery = max(0, 1.0 - real_t / DURATION)

            # 策略观察
            obs = policy.observe(
                robot_pos=robot_pos,
                heading=heading,
                line_error=0.1 + obstacle_density * 0.3,
                obstacle_density=obstacle_density,
                battery_level=battery,
                has_task=False,
            )

            # 策略决策
            decision = policy.decide(obs)
            behavior = decision["behavior"]

            # 记录行为变化
            if behavior != last_behavior:
                behavior_changes.append((real_t, behavior.value))
                last_behavior = behavior

            # 导航控制
            target = decision["target"]
            dx = target[0] - robot_pos[0]
            dy = target[1] - robot_pos[1]
            dist = math.sqrt(dx*dx + dy*dy)

            if dist > 0.01:
                target_angle = math.atan2(dy, dx)
                angle_diff = target_angle - angle
                while angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2 * math.pi

                # 速度控制
                base_speed = 0.3 * decision["speed_factor"]
                turn_rate = 2.0 * decision["steering_gain"]

                if abs(angle_diff) > 0.3:
                    left_torque = -turn_rate * math.copysign(1, angle_diff) * 0.5
                    right_torque = turn_rate * math.copysign(1, angle_diff) * 0.5
                else:
                    left_torque = base_speed + turn_rate * angle_diff * 0.3
                    right_torque = base_speed - turn_rate * angle_diff * 0.3

                data.ctrl[0] = np.clip(left_torque * 20, -5, 5)
                data.ctrl[1] = np.clip(right_torque * 20, -5, 5)
            else:
                data.ctrl[0] = 0
                data.ctrl[1] = 0

            # 渲染
            renderer = mujoco.Renderer(model, height=HEIGHT, width=SIM_W)
            renderer.update_scene(data, camera="main_follow")
            sim_img = renderer.render()

            # 左面板
            panel = draw_panel(None, policy, frame_idx, real_t, FPS)

            # 合成
            combined = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
            combined[:, :PANEL_W] = panel
            combined[:, PANEL_W:] = sim_img

            # 行为变化指示条
            if len(behavior_changes) > 0:
                last_change = behavior_changes[-1]
                if real_t - last_change[0] < 1.5:
                    beh_name = last_change[1]
                    beh_color = BEH_COLORS.get(beh_name, WHITE)
                    cv2.rectangle(combined, (PANEL_W, 0), (WIDTH, 4), beh_color, -1)
                    cv2.putText(combined, f"-> {beh_name.upper()}",
                                (PANEL_W + 10, 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, beh_color, 2, cv2.LINE_AA)

            writer.write(combined)
            frame_count += 1

            if frame_idx % FPS == 0:
                print(f"  {real_t:.0f}s / {DURATION}s | "
                      f"behavior={behavior.value:20s} | "
                      f"pos=({robot_pos[0]:.2f},{robot_pos[1]:.2f}) | "
                      f"target={decision['info']['target_wp']}")

    writer.release()

    print(f"\nVideo saved: {VIDEO_PATH}")
    print(f"  Frames: {frame_count}")
    print(f"  Duration: {DURATION}s")

    # 行为变化统计
    print(f"\n  Behavior changes: {len(behavior_changes)}")
    for t, beh in behavior_changes:
        print(f"    t={t:.1f}s -> {beh}")

    # 策略摘要
    summary = policy.get_state_summary()
    print(f"\n  Visit counts: {summary['visit_counts']}")
    print(f"  Unvisited zones: {summary['unvisited']}")


if __name__ == "__main__":
    run_simulation()
