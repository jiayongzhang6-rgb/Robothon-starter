#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2D 俯视图 v2 - 行为颜色轨迹 + 决策叠加 + 增强时间线
评审级可视化
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
PANEL_W = 420
MAP_W = WIDTH - PANEL_W
MAP_H = HEIGHT
FPS = 30
DURATION = 30
VIDEO_PATH = os.path.join(ROOT, "demo.mp4")

ARENA_W = 1.5
ARENA_H = 0.6
SCALE = MAP_W / ARENA_W * 0.82
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
MAGENTA = (200, 80, 200)

BEH_COLORS = {
    "explore": GREEN, "patrol": CYAN, "cautious_patrol": YELLOW,
    "obstacle_avoid": ORANGE, "return_to_base": RED, "recover": MAGENTA,
}

OBSTACLES = [(0.5, 0.15), (0.8, -0.1), (0.35, -0.2), (1.0, 0.2)]

TARGETS = {
    "start": (0.0, 0.0), "zone_a": (0.6, 0.3), "zone_b": (1.2, 0.0),
    "zone_c": (0.8, -0.4), "zone_d": (0.3, -0.3),
    "intersection_1": (0.4, 0.15), "intersection_2": (0.9, 0.15),
    "intersection_3": (0.9, -0.2), "base": (0.0, 0.0),
}


def w2s(wx, wy):
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

    # Behavior
    y += 25
    beh = policy._behavior.value
    bc = BEH_COLORS.get(beh, WHITE)
    cv2.putText(panel, "BEHAVIOR", (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, GRAY, 1, cv2.LINE_AA)
    y += 22
    cv2.putText(panel, beh.upper(), (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, bc, 2, cv2.LINE_AA)

    # Target
    y += 30
    cv2.putText(panel, "TARGET", (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, GRAY, 1, cv2.LINE_AA)
    y += 18
    cv2.putText(panel, policy._target_wp or "none", (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, CYAN, 1, cv2.LINE_AA)

    # Position
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
        for bn, qv in sorted(policy.behavior_selector._q_table[state].items(), key=lambda x: -x[1])[:6]:
            y += 18
            bar = max(0, int((qv + 2) * 15))  # offset for negative values
            c = BEH_COLORS.get(bn, WHITE)
            cv2.putText(panel, f"{bn[:14]:14s}", (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.30, c, 1, cv2.LINE_AA)
            cv2.rectangle(panel, (195, y-8), (195+bar, y-2), c, -1)
            cv2.putText(panel, f"{qv:.2f}", (200+bar, y), cv2.FONT_HERSHEY_SIMPLEX, 0.28, c, 1, cv2.LINE_AA)

    y += 30; cv2.line(panel, (15, y), (PANEL_W-15, y), DARK, 1)

    # Decision Trace
    y += 18
    cv2.putText(panel, "DECISION TRACE", (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, GRAY, 1, cv2.LINE_AA)
    y += 5
    for e in policy.decision_log.entries[-5:]:
        y += 15
        cs = ",".join([c[:5] for c in e.get("candidates", [])[:3]])
        beh_tag = e.get("behavior", "?")[:8]
        cv2.putText(panel, f"s{e['step']:3d}: {cs:20s} -> {beh_tag}", (15, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.26, CYAN, 1, cv2.LINE_AA)
        y += 13
        sel = e.get("selected_wp", "?")
        cost = e.get("selected_cost") or 0
        cv2.putText(panel, f"       {sel} (cost={cost:.3f})", (15, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.26, GREEN, 1, cv2.LINE_AA)

    y += 25; cv2.line(panel, (15, y), (PANEL_W-15, y), DARK, 1)

    # Stats
    y += 18
    cv2.putText(panel, "STATS", (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, GRAY, 1, cv2.LINE_AA)
    y += 18
    cv2.putText(panel, f"Step: {step}  Time: {t:.1f}s", (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, WHITE, 1, cv2.LINE_AA)
    y += 16
    unv = policy._count_unvisited()
    vis = len(policy.route_sampler._visit_counts)
    cv2.putText(panel, f"Zones: {vis} visited, {unv} unvisited", (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, WHITE, 1, cv2.LINE_AA)
    y += 16
    total = vis + unv
    cv2.putText(panel, f"Coverage: {vis}/{total} ({vis/total*100:.0f}%)", (15, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, GREEN, 1, cv2.LINE_AA)
    return panel


def draw_map(policy, robot_pos, heading, traj_with_behaviors, behavior_changes, t, last_decision):
    """2D 俯视地图 - 行为颜色轨迹 + 决策叠加"""
    img = np.full((MAP_H, MAP_W, 3), 40, dtype=np.uint8)

    # 场地边界
    tl = w2s(0, ARENA_H)
    br = w2s(ARENA_W, 0)
    cv2.rectangle(img, tl, br, GRAY, 2)

    # 网格
    for x in np.arange(0, ARENA_W + 0.1, 0.3):
        p1, p2 = w2s(x, 0), w2s(x, ARENA_H)
        cv2.line(img, p1, p2, (50, 50, 50), 1)
    for y in np.arange(0, ARENA_H + 0.1, 0.15):
        p1, p2 = w2s(0, y), w2s(ARENA_W, y)
        cv2.line(img, p1, p2, (50, 50, 50), 1)

    # 目标点
    for name, (wx, wy) in TARGETS.items():
        sx, sy = w2s(wx, wy)
        visited = policy.route_sampler.get_visit_count(name)
        color = GREEN if visited > 0 else (70, 70, 70)
        cv2.circle(img, (sx, sy), 5, color, -1)
        cv2.circle(img, (sx, sy), 6, color, 1)
        cv2.putText(img, name[:10], (sx+9, sy+4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.25, color, 1, cv2.LINE_AA)

    # 障碍物
    for ox, oy in OBSTACLES:
        sx, sy = w2s(ox, oy)
        cv2.circle(img, (sx, sy), 14, ORANGE, -1)
        cv2.circle(img, (sx, sy), 14, RED, 2)
        cv2.putText(img, "X", (sx-4, sy+5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, WHITE, 1, cv2.LINE_AA)

    # 行为颜色轨迹（核心加分点）
    if len(traj_with_behaviors) > 1:
        for i in range(1, len(traj_with_behaviors)):
            px1, py1, beh1 = traj_with_behaviors[i-1]
            px2, py2, beh2 = traj_with_behaviors[i]
            s1 = w2s(px1, py1)
            s2 = w2s(px2, py2)
            c = BEH_COLORS.get(beh2, GRAY)
            cv2.line(img, s1, s2, c, 2, cv2.LINE_AA)

    # 目标连线（虚线效果）
    if policy._target_wp and policy._target_wp in TARGETS:
        tx, ty = TARGETS[policy._target_wp]
        tsx, tsy = w2s(tx, ty)
        rsx, rsy = w2s(robot_pos[0], robot_pos[1])
        # 画虚线
        dx, dy = tsx - rsx, tsy - rsy
        length = math.sqrt(dx*dx + dy*dy)
        if length > 5:
            steps = int(length / 8)
            for i in range(0, steps, 2):
                x1 = int(rsx + dx * i / steps)
                y1 = int(rsy + dy * i / steps)
                x2 = int(rsx + dx * min(i+1, steps) / steps)
                y2 = int(rsy + dy * min(i+1, steps) / steps)
                cv2.line(img, (x1, y1), (x2, y2), CYAN, 1, cv2.LINE_AA)
        cv2.circle(img, (tsx, tsy), 8, CYAN, 2)

    # 决策叠加（sampled candidates）
    if last_decision:
        candidates = last_decision.get("candidates", [])
        for cand in candidates:
            if cand in TARGETS:
                cx, cy = TARGETS[cand]
                csx, csy = w2s(cx, cy)
                cv2.circle(img, (csx, csy), 10, YELLOW, 2)

    # 机器人
    rsx, rsy = w2s(robot_pos[0], robot_pos[1])
    beh = policy._behavior.value
    bc = BEH_COLORS.get(beh, WHITE)

    # 方向箭头
    angle = math.atan2(heading[1], heading[0])
    tip_x = int(rsx + 18 * math.cos(angle))
    tip_y = int(rsy - 18 * math.sin(angle))
    cv2.arrowedLine(img, (rsx, rsy), (tip_x, tip_y), WHITE, 3, tipLength=0.4)
    cv2.circle(img, (rsx, rsy), 12, bc, -1)
    cv2.circle(img, (rsx, rsy), 12, WHITE, 2)

    # 行为标签
    cv2.putText(img, f"ROBOT: {beh.upper()}", (rsx+18, rsy-12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, bc, 2, cv2.LINE_AA)

    # 行为时间线（底部）
    tl_y = MAP_H - 60
    cv2.putText(img, "BEHAVIOR TIMELINE", (10, tl_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, GRAY, 1, cv2.LINE_AA)

    # 时间线背景
    tl_x1, tl_x2 = 10, MAP_W - 10
    cv2.line(img, (tl_x1, tl_y + 8), (tl_x2, tl_y + 8), DARK, 3)

    # 行为切换标记
    for bt, bn in behavior_changes:
        bx = int(tl_x1 + (bt / DURATION) * (tl_x2 - tl_x1))
        bc2 = BEH_COLORS.get(bn, WHITE)
        cv2.line(img, (bx, tl_y + 2), (bx, tl_y + 18), bc2, 2)
        cv2.putText(img, f"{bn[:8]}", (bx-15, tl_y + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.25, bc2, 1, cv2.LINE_AA)
        cv2.putText(img, f"{bt:.0f}s", (bx-5, tl_y + 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.22, bc2, 1, cv2.LINE_AA)

    # 当前时间指针
    cx = int(tl_x1 + (t / DURATION) * (tl_x2 - tl_x1))
    cv2.line(img, (cx, tl_y + 2), (cx, tl_y + 18), WHITE, 2)

    # 图例
    legend_y = 20
    for beh_name, beh_color in BEH_COLORS.items():
        cv2.circle(img, (MAP_W - 100, legend_y), 5, beh_color, -1)
        cv2.putText(img, beh_name[:12], (MAP_W - 90, legend_y + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.28, beh_color, 1, cv2.LINE_AA)
        legend_y += 16

    return img


def run():
    policy = PatrolPolicy(seed=42)
    policy.reset()

    px, py, angle = 0.0, 0.0, 0.0
    dt = 1.0 / FPS
    total_frames = DURATION * FPS

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(VIDEO_PATH, fourcc, FPS, (WIDTH, HEIGHT))

    # 轨迹 + 行为历史
    traj_with_behaviors = []  # [(x, y, behavior_name), ...]
    behavior_buffer = []
    current_behavior = Behavior.PATROL
    behavior_changes = [(0.0, "patrol")]
    last_decision = None

    print(f"Recording {DURATION}s (behavior-colored trajectory)...")

    for fi in range(total_frames):
        t = fi * dt

        # 环境状态
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
        last_decision = decision.get("info", {})

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
            angle += adiff * 0.1
            speed = 0.15 * spd
            px += speed * math.cos(angle) * dt
            py += speed * math.sin(angle) * dt

        px = max(-0.05, min(ARENA_W + 0.05, px))
        py = max(-0.05, min(ARENA_H + 0.05, py))

        # 记录轨迹 + 行为
        traj_with_behaviors.append((px, py, current_behavior.value))
        if len(traj_with_behaviors) > 600:
            traj_with_behaviors.pop(0)

        # 绘制
        panel = draw_panel(policy, fi, t)
        map_img = draw_map(policy, (px, py), heading, traj_with_behaviors,
                           behavior_changes, t, last_decision)

        combined = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        combined[:, :PANEL_W] = panel
        combined[:, PANEL_W:] = map_img

        # 行为切换指示条
        if behavior_changes and t - behavior_changes[-1][0] < 1.5:
            bn = behavior_changes[-1][1]
            bc = BEH_COLORS.get(bn, WHITE)
            cv2.rectangle(combined, (PANEL_W, 0), (WIDTH, 5), bc, -1)
            cv2.putText(combined, f"-> {bn.upper()}", (PANEL_W+10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, bc, 2, cv2.LINE_AA)

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
