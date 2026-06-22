#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MuJoCo 仿真 v3 - 摄像机跟随 + 机器人可动 + 行为切换
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

WHITE = (255, 255, 255)
CYAN = (200, 220, 255)
GREEN = (80, 220, 120)
YELLOW = (80, 220, 255)
RED = (80, 80, 255)
ORANGE = (50, 180, 255)
GRAY = (120, 120, 120)
DARK = (50, 50, 50)

BEH_COLORS = {
    "explore": GREEN, "patrol": CYAN, "cautious_patrol": YELLOW,
    "obstacle_avoid": ORANGE, "return_to_base": RED, "recover": (100, 100, 255),
}


def draw_panel(policy, step, t):
    panel = np.full((HEIGHT, PANEL_W, 3), 25, dtype=np.uint8)
    y = 30
    cv2.putText(panel, "ADAPTIVE PATROL AGENT", (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, GREEN, 1, cv2.LINE_AA)
    y += 20
    cv2.putText(panel, "Hierarchical Q-learning Policy", (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, GRAY, 1, cv2.LINE_AA)
    y += 35; cv2.line(panel, (15, y), (PANEL_W-15, y), DARK, 1)

    y += 25
    beh = policy._behavior.value
    bc = BEH_COLORS.get(beh, WHITE)
    cv2.putText(panel, "BEHAVIOR", (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, GRAY, 1, cv2.LINE_AA)
    y += 22
    cv2.putText(panel, beh.upper(), (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, bc, 2, cv2.LINE_AA)

    y += 30
    cv2.putText(panel, "TARGET", (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, GRAY, 1, cv2.LINE_AA)
    y += 18
    cv2.putText(panel, policy._target_wp or "none", (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, CYAN, 1, cv2.LINE_AA)

    y += 25
    pos = policy._robot_pos
    cv2.putText(panel, f"POS: ({pos[0]:.2f}, {pos[1]:.2f})", (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, WHITE, 1, cv2.LINE_AA)
    y += 35; cv2.line(panel, (15, y), (PANEL_W-15, y), DARK, 1)

    y += 20
    cv2.putText(panel, "Q-VALUES", (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, GRAY, 1, cv2.LINE_AA)
    y += 5
    state = policy.behavior_selector._last_state
    if state and state in policy.behavior_selector._q_table:
        for beh_name, q_val in sorted(policy.behavior_selector._q_table[state].items(), key=lambda x: -x[1])[:5]:
            y += 18
            bar = max(0, int(q_val * 25))
            c = BEH_COLORS.get(beh_name, WHITE)
            cv2.putText(panel, f"{beh_name[:12]:12s}", (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.32, c, 1, cv2.LINE_AA)
            cv2.rectangle(panel, (180, y-8), (180+bar, y-2), c, -1)
            cv2.putText(panel, f"{q_val:.2f}", (190+bar, y), cv2.FONT_HERSHEY_SIMPLEX, 0.3, c, 1, cv2.LINE_AA)

    y += 30; cv2.line(panel, (15, y), (PANEL_W-15, y), DARK, 1)
    y += 18
    cv2.putText(panel, "DECISION TRACE", (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, GRAY, 1, cv2.LINE_AA)
    y += 5
    for e in policy.decision_log.entries[-4:]:
        y += 16
        cs = ",".join([c[:6] for c in e.get("candidates", [])[:3]])
        cv2.putText(panel, f"s{e['step']}: {cs}", (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.28, CYAN, 1, cv2.LINE_AA)
        y += 14
        sel = e.get("selected_wp", "?")
        cost = e.get("selected_cost") or 0
        cv2.putText(panel, f"  -> {sel} ({cost:.3f})", (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.28, GREEN, 1, cv2.LINE_AA)

    y += 25; cv2.line(panel, (15, y), (PANEL_W-15, y), DARK, 1)
    y += 18
    cv2.putText(panel, "STATS", (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, GRAY, 1, cv2.LINE_AA)
    y += 18
    cv2.putText(panel, f"Step: {step}  Time: {t:.1f}s", (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, WHITE, 1, cv2.LINE_AA)
    y += 16
    cv2.putText(panel, f"Unvisited: {policy._count_unvisited()}  Visits: {len(policy.route_sampler._visit_counts)}", (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, WHITE, 1, cv2.LINE_AA)
    return panel


def run():
    xml_path = os.path.join(ROOT, "simulation", "robot.xml")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    model.opt.timestep = 0.005
    model.opt.iterations = 20

    mujoco.mj_resetData(model, data)
    data.qpos[0] = 0.0; data.qpos[1] = 0.0; data.qpos[2] = 0.08; data.qpos[3] = 1.0

    policy = PatrolPolicy(seed=42)
    policy.reset()

    dt_sim = model.opt.timestep
    steps_per_frame = int(1.0 / (FPS * dt_sim))
    total_steps = int(DURATION * FPS * steps_per_frame)

    renderer = mujoco.Renderer(model, height=HEIGHT, width=SIM_W)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(VIDEO_PATH, fourcc, FPS, (WIDTH, HEIGHT))

    obstacles = [(0.5, 0.15), (0.8, -0.1), (0.35, -0.2), (1.0, 0.2)]
    behavior_buffer = []
    current_behavior = Behavior.PATROL
    behavior_changes = []

    print(f"Recording {DURATION}s...")

    for step in range(total_steps):
        mujoco.mj_step(model, data)

        if step % steps_per_frame == 0:
            fi = step // steps_per_frame
            t = fi / FPS

            # 获取机器人位置
            robot_x = float(data.qpos[0])
            robot_y = float(data.qpos[1])
            robot_pos = (robot_x, robot_y)

            qw, qx, qy, qz = data.qpos[3], data.qpos[4], data.qpos[5], data.qpos[6]
            angle = 2 * math.atan2(qz, qw)
            heading = (math.cos(angle), math.sin(angle))

            # 障碍密度
            near = sum(1 for ox, oy in obstacles
                      if math.sqrt((robot_x-ox)**2 + (robot_y-oy)**2) < 0.25)
            obs_density = min(1.0, near / 2.0)
            battery = max(0, 1.0 - t / DURATION)

            obs = policy.observe(
                robot_pos=robot_pos, heading=heading,
                line_error=0.1 + obs_density * 0.3,
                obstacle_density=obs_density,
                battery_level=battery, has_task=False,
            )
            decision = policy.decide(obs)
            raw_beh = decision["behavior"]

            # 防抖
            behavior_buffer.append(raw_beh)
            if len(behavior_buffer) > 8:
                behavior_buffer.pop(0)
            if len(behavior_buffer) >= 8:
                from collections import Counter
                mc = Counter(behavior_buffer).most_common(1)[0]
                if mc[1] >= 7 and mc[0] != current_behavior:
                    current_behavior = mc[0]
                    behavior_changes.append((t, current_behavior.value))
                    behavior_buffer.clear()

            # 导航
            target = decision["target"]
            dx = target[0] - robot_x
            dy = target[1] - robot_y
            dist = math.sqrt(dx*dx + dy*dy)

            spd = policy.behavior_selector.get_speed_multiplier(current_behavior)
            steer = policy.behavior_selector.get_steering_gain(current_behavior)

            if dist > 0.01:
                tgt_angle = math.atan2(dy, dx)
                adiff = tgt_angle - angle
                while adiff > math.pi: adiff -= 2*math.pi
                while adiff < -math.pi: adiff += 2*math.pi

                base = 0.4 * spd
                turn = 2.0 * steer

                if abs(adiff) > 0.3:
                    lt = -turn * math.copysign(1, adiff) * 0.5
                    rt = turn * math.copysign(1, adiff) * 0.5
                else:
                    lt = base + turn * adiff * 0.3
                    rt = base - turn * adiff * 0.3

                data.ctrl[0] = np.clip(lt * 30, -8, 8)
                data.ctrl[1] = np.clip(rt * 30, -8, 8)
            else:
                data.ctrl[0] = 0; data.ctrl[1] = 0

            # 渲染 - 自由摄像机跟随机器人
            cam = mujoco.MjvCamera()
            cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            cam.lookat[:] = [robot_x, robot_y, 0.05]
            cam.distance = 0.8
            cam.azimuth = 225
            cam.elevation = 40
            renderer.update_scene(data, camera=cam)
            sim_img = renderer.render()

            panel = draw_panel(policy, fi, t)
            combined = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
            combined[:, :PANEL_W] = panel
            combined[:, PANEL_W:] = sim_img

            if behavior_changes and t - behavior_changes[-1][0] < 2.0:
                bn = behavior_changes[-1][1]
                bc = BEH_COLORS.get(bn, WHITE)
                cv2.rectangle(combined, (PANEL_W, 0), (WIDTH, 4), bc, -1)
                cv2.putText(combined, f"-> {bn.upper()}", (PANEL_W+10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, bc, 2, cv2.LINE_AA)

            writer.write(combined)

            if fi % FPS == 0:
                print(f"  {t:.0f}s | {current_behavior.value:20s} | pos=({robot_x:.2f},{robot_y:.2f})")

    writer.release()
    print(f"\nSaved: {VIDEO_PATH}")
    print(f"Changes: {len(behavior_changes)}")
    for t, b in behavior_changes:
        print(f"  t={t:.1f}s -> {b}")


if __name__ == "__main__":
    run()
