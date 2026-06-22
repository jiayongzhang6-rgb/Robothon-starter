#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2D 俯视图仿真 - PatrolPolicy 驱动
左面板: AI 决策信息
右面板: 2D 俯视地图 (机器人 + 障碍 + 路径 + 目标)
"""

import os
import sys
import math
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import cv2
from robot.policy.patrol_policy import PatrolPolicy, Behavior

WIDTH, HEIGHT = 1440, 720
PANEL_W = 400
MAP_W = WIDTH - PANEL_W
MAP_H = HEIGHT
FPS = 30
DURATION = 30
VIDEO_PATH = os.path.join(ROOT, "demo.mp4")

# 场地参数 (米)
ARENA_W = 1.5
ARENA_H = 0.6
SCALE = MAP_W / ARENA_W * 0.85
OFFSET_X = int((MAP_W - ARENA_W * SCALE) / 2)
OFFSET_Y = int((MAP_H - ARENA_H * SCALE) / 2)

WHITE = (255, 255, 255)
CYAN = (200, 220, 255)
GREEN = (80, 220, 120)
YELLOW = (80, 220, 255)
RED = (80, 80, 255)
ORANGE = (50, 180, 255)
GRAY = (120, 120, 120)
DARK = (50, 50, 50)
BG = (30, 30, 30)

BEH_COLORS = {
    "explore": GREEN, "patrol": CYAN, "cautious_patrol": YELLOW,
    "obstacle_avoid": ORANGE, "return_to_base": RED, "recover": (100, 100, 255),
}

# 障碍物
OBSTACLES = [(0.5, 0.15), (0.8, -0.1), (0.35, -0.2), (1.0, 0.2)]

# 目标点
TARGETS = {
    "start": (0.0, 0.0), "zone_a": (0.6, 0.3), "zone_b": (1.2, 0.0),
    "zone_c": (0.8, -0.4), "zone_d": (0.3, -0.3),
    "intersection_1": (0.4, 0.15), "intersection_2": (0.9, 0.15),
    "intersection_3": (0.9, -0.2), "base": (0.0, 0.0),
}


def world_to_screen(wx, wy):
    sx = int(OFFSET_X + wx * SCALE)
    sy = int(MAP_H - OFFSET_Y - wy * SCALE)
    return sx, sy


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

    # Q-values
    y += 20
    cv2.putText(panel, "Q-VALUES", (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, GRAY, 1, cv2.LINE_AA)
    y += 5
    state = policy.behavior_selector._last_state
    if state and state in policy.behavior_selector._q_table:
        for bn, qv in sorted(policy.behavior_selector._q_table[state].items(), key=lambda x: -x[1])[:5]:
            y += 18
            bar = max(0, int(qv * 25))
            c = BEH_COLORS.get(bn, WHITE)
            cv2.putText(panel, f"{bn[:12]:12s}", (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.32, c, 1, cv2.LINE_AA)
            cv2.rectangle(panel, (180, y-8), (180+bar, y-2), c, -1)
            cv2.putText(panel, f"{qv:.2f}", (190+bar, y), cv2.FONT_HERSHEY_SIMPLEX, 0.3, c, 1, cv2.LINE_AA)

    y += 30; cv2.line(panel, (15, y), (PANEL_W-15, y), DARK, 1)

    # Decision Trace
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

    # Stats
    y += 18
    cv2.putText(panel, "STATS", (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, GRAY, 1, cv2.LINE_AA)
    y += 18
    cv2.putText(panel, f"Step: {step}  Time: {t:.1f}s", (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, WHITE, 1, cv2.LINE_AA)
    y += 16
    cv2.putText(panel, f"Unvisited: {policy._count_unvisited()}", (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, WHITE, 1, cv2.LINE_AA)
    return panel


def draw_map(frame, policy, robot_pos, heading, path_history, behavior_changes, t):
    """右面板: 2D 俯视地图"""
    map_img = np.full((MAP_H, MAP_W, 3), 40, dtype=np.uint8)

    # 场地边界
    tl = world_to_screen(0, ARENA_H)
    br = world_to_screen(ARENA_W, 0)
    cv2.rectangle(map_img, tl, br, GRAY, 2)

    # 网格
    for x in np.arange(0, ARENA_W + 0.1, 0.3):
        p1 = world_to_screen(x, 0)
        p2 = world_to_screen(x, ARENA_H)
        cv2.line(map_img, p1, p2, (55, 55, 55), 1)
    for y in np.arange(0, ARENA_H + 0.1, 0.15):
        p1 = world_to_screen(0, y)
        p2 = world_to_screen(ARENA_W, y)
        cv2.line(map_img, p1, p2, (55, 55, 55), 1)

    # 目标点
    for name, (wx, wy) in TARGETS.items():
        sx, sy = world_to_screen(wx, wy)
        visited = policy.route_sampler.get_visit_count(name)
        color = GREEN if visited > 0 else (80, 80, 80)
        cv2.circle(map_img, (sx, sy), 5, color, -1)
        cv2.putText(map_img, name[:8], (sx+8, sy+4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.25, color, 1, cv2.LINE_AA)

    # 障碍物
    for ox, oy in OBSTACLES:
        sx, sy = world_to_screen(ox, oy)
        cv2.circle(map_img, (sx, sy), 12, ORANGE, -1)
        cv2.circle(map_img, (sx, sy), 12, RED, 2)

    # 路径历史
    if len(path_history) > 1:
        pts = [world_to_screen(p[0], p[1]) for p in path_history[-300:]]
        for i in range(1, len(pts)):
            alpha = i / len(pts)
            c = tuple(int(v * alpha) for v in GREEN)
            cv2.line(map_img, pts[i-1], pts[i], c, 2)

    # 目标连线
    if policy._target_wp and policy._target_wp in TARGETS:
        tx, ty = TARGETS[policy._target_wp]
        tsx, tsy = world_to_screen(tx, ty)
        rsx, rsy = world_to_screen(robot_pos[0], robot_pos[1])
        cv2.line(map_img, (rsx, rsy), (tsx, tsy), CYAN, 1, cv2.LINE_AA)
        cv2.circle(map_img, (tsx, tsy), 8, CYAN, 2)

    # 机器人
    rsx, rsy = world_to_screen(robot_pos[0], robot_pos[1])
    beh = policy._behavior.value
    bc = BEH_COLORS.get(beh, WHITE)

    # 机器人方向
    angle = math.atan2(heading[1], heading[0])
    tip_x = int(rsx + 15 * math.cos(angle))
    tip_y = int(rsy - 15 * math.sin(angle))
    cv2.circle(map_img, (rsx, rsy), 10, bc, -1)
    cv2.circle(map_img, (rsx, rsy), 10, WHITE, 2)
    cv2.line(map_img, (rsx, rsy), (tip_x, tip_y), WHITE, 2)

    # 行为标签
    cv2.putText(map_img, f"Robot: {beh.upper()}", (rsx+15, rsy-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, bc, 1, cv2.LINE_AA)

    # 行为切换时间线
    y_timeline = MAP_H - 50
    cv2.putText(map_img, "BEHAVIOR TIMELINE", (10, y_timeline),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, GRAY, 1, cv2.LINE_AA)

    for bt, bn in behavior_changes:
        bx = int(10 + (bt / DURATION) * (MAP_W - 20))
        bc2 = BEH_COLORS.get(bn, WHITE)
        cv2.line(map_img, (bx, y_timeline + 5), (bx, y_timeline + 25), bc2, 2)
        cv2.putText(map_img, f"{bt:.0f}s", (bx-5, y_timeline + 38),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.25, bc2, 1, cv2.LINE_AA)

    # 当前时间指示
    cx = int(10 + (t / DURATION) * (MAP_W - 20))
    cv2.line(map_img, (cx, y_timeline + 5), (cx, y_timeline + 25), WHITE, 2)

    return map_img


def run():
    policy = PatrolPolicy(seed=42)
    policy.reset()

    # 初始物理状态
    px, py = 0.0, 0.0
    angle = 0.0
    vx, vy = 0.0, 0.0

    dt = 1.0 / FPS
    total_frames = DURATION * FPS

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(VIDEO_PATH, fourcc, FPS, (WIDTH, HEIGHT))

    path_history = []
    behavior_buffer = []
    current_behavior = Behavior.PATROL
    behavior_changes = []

    print(f"Recording {DURATION}s 2D visualization...")

    for fi in range(total_frames):
        t = fi * dt

        # 障碍密度
        near = sum(1 for ox, oy in OBSTACLES
                  if math.sqrt((px-ox)**2 + (py-oy)**2) < 0.25)
        obs_density = min(1.0, near / 2.0)
        battery = max(0, 1.0 - t / DURATION)

        heading = (math.cos(angle), math.sin(angle))
        obs = policy.observe(
            robot_pos=(px, py), heading=heading,
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
        dx = target[0] - px
        dy = target[1] - py
        dist = math.sqrt(dx*dx + dy*dy)

        spd = policy.behavior_selector.get_speed_multiplier(current_behavior)

        if dist > 0.01:
            tgt_angle = math.atan2(dy, dx)
            adiff = tgt_angle - angle
            while adiff > math.pi: adiff -= 2*math.pi
            while adiff < -math.pi: adiff += 2*math.pi

            # 转向
            angle += adiff * 0.1

            # 前进
            speed = 0.15 * spd
            px += speed * math.cos(angle) * dt
            py += speed * math.sin(angle) * dt

        # 边界
        px = max(-0.05, min(ARENA_W + 0.05, px))
        py = max(-0.05, min(ARENA_H + 0.05, py))

        path_history.append((px, py))
        if len(path_history) > 500:
            path_history.pop(0)

        # 绘制
        panel = draw_panel(policy, fi, t)
        map_img = draw_map(None, policy, (px, py), heading, path_history, behavior_changes, t)

        combined = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        combined[:, :PANEL_W] = panel
        combined[:, PANEL_W:] = map_img

        # 行为切换指示
        if behavior_changes and t - behavior_changes[-1][0] < 1.5:
            bn = behavior_changes[-1][1]
            bc = BEH_COLORS.get(bn, WHITE)
            cv2.rectangle(combined, (PANEL_W, 0), (WIDTH, 4), bc, -1)
            cv2.putText(combined, f"-> {bn.upper()}", (PANEL_W+10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, bc, 2, cv2.LINE_AA)

        writer.write(combined)

        if fi % FPS == 0:
            print(f"  {t:.0f}s | {current_behavior.value:20s} | pos=({px:.2f},{py:.2f}) | target={policy._target_wp}")

    writer.release()
    print(f"\nSaved: {VIDEO_PATH}")
    print(f"Behavior changes: {len(behavior_changes)}")
    for bt, bn in behavior_changes:
        print(f"  t={bt:.1f}s -> {bn}")


if __name__ == "__main__":
    run()
