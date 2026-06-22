#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MuJoCo 仿真 v2 - PatrolPolicy 驱动 (物理稳定版)
"""

import os
import sys
import math
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import mujoco
import cv2
from robot.policy.patrol_policy import PatrolPolicy, Behavior

WIDTH, HEIGHT = 1440, 720
PANEL_W = 400
SIM_W = WIDTH - PANEL_W
FPS = 30
DURATION = 30
VIDEO_PATH = os.path.join(ROOT, "demo.mp4")

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
    panel = np.full((HEIGHT, PANEL_W, 3), 25, dtype=np.uint8)

    y = 30
    cv2.putText(panel, "ADAPTIVE PATROL AGENT", (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, GREEN, 1, cv2.LINE_AA)
    y += 20
    cv2.putText(panel, "Hierarchical Q-learning Policy", (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, GRAY, 1, cv2.LINE_AA)

    y += 35
    cv2.line(panel, (15, y), (PANEL_W - 15, y), DARK, 1)

    y += 25
    beh = policy._behavior.value
    beh_color = BEH_COLORS.get(beh, WHITE)
    cv2.putText(panel, "BEHAVIOR", (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, GRAY, 1, cv2.LINE_AA)
    y += 22
    cv2.putText(panel, beh.upper(), (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, beh_color, 2, cv2.LINE_AA)

    y += 30
    cv2.putText(panel, "TARGET", (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, GRAY, 1, cv2.LINE_AA)
    y += 18
    target = policy._target_wp or "none"
    cv2.putText(panel, target, (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, CYAN, 1, cv2.LINE_AA)

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

    # Decision Trace
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

    # Stats
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
    xml_path = os.path.join(ROOT, "simulation", "robot.xml")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # 物理稳定化: 降低速度限制
    model.opt.timestep = 0.005  # 更大的时间步 = 更稳定
    model.opt.iterations = 20   # 更多迭代 = 更稳定

    mujoco.mj_resetData(model, data)
    data.qpos[0] = 0.0
    data.qpos[1] = 0.0
    data.qpos[2] = 0.08
    data.qpos[3] = 1.0

    policy = PatrolPolicy(seed=42)
    policy.reset()

    dt_sim = model.opt.timestep
    steps_per_frame = int(1.0 / (FPS * dt_sim))
    total_steps = int(DURATION * FPS * steps_per_frame)

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(VIDEO_PATH, fourcc, FPS, (WIDTH, HEIGHT))

    print(f"Recording {DURATION}s at {FPS}fps (stable mode)...")
    print(f"  dt={dt_sim}, steps/frame={steps_per_frame}")

    # 障碍物位置 (与 XML 一致)
    obstacles = [(0.5, 0.15), (0.8, -0.1), (0.35, -0.2), (1.0, 0.2)]

    # 行为防抖: 需要连续 N 帧才切换
    behavior_buffer = []
    BUFFER_SIZE = 8
    current_stable_behavior = Behavior.PATROL
    last_behavior = None
    behavior_changes = []

    for step in range(total_steps):
        # 物理步进 (多步小步)
        for _ in range(2):
            mujoco.mj_step(model, data)

        # NaN 检测
        if np.any(np.isnan(data.qvel)) or np.any(np.abs(data.qvel) > 10):
            mujoco.mj_resetData(model, data)
            data.qpos[0] = 0.0
            data.qpos[1] = 0.0
            data.qpos[2] = 0.08
            data.qpos[3] = 1.0
            data.qvel[:] = 0

        if step % steps_per_frame == 0:
            frame_idx = step // steps_per_frame
            real_t = frame_idx / FPS

            robot_pos = (float(data.qpos[0]), float(data.qpos[1]))
            qw, qx, qy, qz = data.qpos[3], data.qpos[4], data.qpos[5], data.qpos[6]
            angle = 2 * math.atan2(qz, qw)
            heading = (math.cos(angle), math.sin(angle))

            # 障碍密度
            near_count = sum(1 for ox, oy in obstacles
                           if math.sqrt((robot_pos[0]-ox)**2 + (robot_pos[1]-oy)**2) < 0.25)
            obstacle_density = min(1.0, near_count / 2.0)

            battery = max(0, 1.0 - real_t / DURATION)

            obs = policy.observe(
                robot_pos=robot_pos,
                heading=heading,
                line_error=0.1 + obstacle_density * 0.3,
                obstacle_density=obstacle_density,
                battery_level=battery,
                has_task=False,
            )

            decision = policy.decide(obs)
            raw_behavior = decision["behavior"]

            # 行为防抖
            behavior_buffer.append(raw_behavior)
            if len(behavior_buffer) > BUFFER_SIZE:
                behavior_buffer.pop(0)

            # 只有连续 N 帧一致才切换
            if len(behavior_buffer) >= BUFFER_SIZE:
                from collections import Counter
                counts = Counter(behavior_buffer)
                most_common = counts.most_common(1)[0]
                if most_common[1] >= BUFFER_SIZE - 1:  # N-1 一致就切换
                    new_behavior = most_common[0]
                    if new_behavior != current_stable_behavior:
                        current_stable_behavior = new_behavior
                        behavior_changes.append((real_t, new_behavior.value))
                        behavior_buffer.clear()

            # 使用稳定后的行为
            stable_decision = dict(decision)
            stable_decision["behavior"] = current_stable_behavior
            stable_decision["speed_factor"] = policy.behavior_selector.get_speed_multiplier(current_stable_behavior)
            stable_decision["steering_gain"] = policy.behavior_selector.get_steering_gain(current_stable_behavior)

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

                base_speed = 0.2 * stable_decision["speed_factor"]
                turn_rate = 1.5 * stable_decision["steering_gain"]

                if abs(angle_diff) > 0.3:
                    left_torque = -turn_rate * math.copysign(1, angle_diff) * 0.3
                    right_torque = turn_rate * math.copysign(1, angle_diff) * 0.3
                else:
                    left_torque = base_speed + turn_rate * angle_diff * 0.2
                    right_torque = base_speed - turn_rate * angle_diff * 0.2

                data.ctrl[0] = np.clip(left_torque * 15, -3, 3)
                data.ctrl[1] = np.clip(right_torque * 15, -3, 3)
            else:
                data.ctrl[0] = 0
                data.ctrl[1] = 0

            # 渲染
            renderer = mujoco.Renderer(model, height=HEIGHT, width=SIM_W)
            renderer.update_scene(data, camera="main_follow")
            sim_img = renderer.render()

            # 检查渲染是否有效
            if sim_img is None or sim_img.size == 0:
                sim_img = np.full((HEIGHT, SIM_W, 3), 40, dtype=np.uint8)

            panel = draw_panel(None, policy, frame_idx, real_t, FPS)

            combined = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
            combined[:, :PANEL_W] = panel
            combined[:, PANEL_W:] = sim_img

            # 行为切换指示
            if len(behavior_changes) > 0:
                last_change = behavior_changes[-1]
                if real_t - last_change[0] < 2.0:
                    beh_name = last_change[1]
                    beh_color = BEH_COLORS.get(beh_name, WHITE)
                    cv2.rectangle(combined, (PANEL_W, 0), (WIDTH, 4), beh_color, -1)
                    cv2.putText(combined, f"-> {beh_name.upper()}",
                                (PANEL_W + 10, 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, beh_color, 2, cv2.LINE_AA)

            writer.write(combined)

            if frame_idx % FPS == 0:
                print(f"  {real_t:.0f}s / {DURATION}s | "
                      f"behavior={current_stable_behavior.value:20s} | "
                      f"pos=({robot_pos[0]:.2f},{robot_pos[1]:.2f})")

    writer.release()

    print(f"\nVideo saved: {VIDEO_PATH}")
    print(f"\nBehavior changes ({len(behavior_changes)}):")
    for t, beh in behavior_changes:
        print(f"  t={t:.1f}s -> {beh}")

    summary = policy.get_state_summary()
    print(f"\nVisit counts: {summary['visit_counts']}")
    print(f"Unvisited: {summary['unvisited']}")


if __name__ == "__main__":
    run_simulation()
