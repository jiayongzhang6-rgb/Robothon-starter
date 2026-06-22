#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Risk Estimator - 风险感知评分器
对候选路径进行多维度评分，选出最优目标
"""

import math
import numpy as np


class CostMap:
    """代价地图 - 表示环境中的风险分布"""

    def __init__(self, width=200, height=150, resolution=0.01):
        self.width = width
        self.height = height
        self.resolution = resolution
        # 静态障碍物代价（模拟场地中固定的障碍）
        self.static_cost = np.zeros((height, width))
        # 动态代价（运行时更新）
        self.dynamic_cost = np.zeros((height, width))

    def set_obstacle(self, x, y, radius=5, intensity=1.0):
        """设置一个圆形障碍区域"""
        cx = int(x / self.resolution)
        cy = int(y / self.resolution)
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dx*dx + dy*dy <= radius*radius:
                    px, py = cx + dx, cy + dy
                    if 0 <= px < self.width and 0 <= py < self.height:
                        dist = math.sqrt(dx*dx + dy*dy)
                        self.static_cost[py, px] = max(
                            self.static_cost[py, px],
                            intensity * (1 - dist / radius)
                        )

    def get_risk(self, world_x, world_y):
        """查询某世界坐标的碰撞风险值"""
        cx = int(world_x / self.resolution)
        cy = int(world_y / self.resolution)
        cx = max(0, min(self.width - 1, cx))
        cy = max(0, min(self.height - 1, cy))
        return float(self.static_cost[cy, cx] + self.dynamic_cost[cy, cx])


class RiskEstimator:
    """
    风险感知评分器
    对每个候选目标进行多维度打分，分越低越好（min-score选择）

    评分维度:
        1. distance_cost  — 距离代价（越近越好）
        2. obstacle_risk  — 碰撞风险（避开障碍）
        3. novelty_bonus  — 探索奖励（访问越少分越低 → 优先去）
        4. smoothness     — 转向代价（偏好直线，减少急转）
    """

    def __init__(self, cost_map=None):
        self.cost_map = cost_map or CostMap()

        # 权重（可调参）
        self.w_distance = 1.0
        self.w_obstacle = 2.5    # 障碍权重高，安全第一
        self.w_novelty  = -0.5   # 负号 = 新颖性高 → 总分低 → 优先选
        self.w_smooth   = 0.3    # 偏好与当前朝向一致

        # 历史信息
        self._prev_target = None
        self._prev_heading = (1.0, 0.0)

    def update_context(self, prev_target=None, heading=None):
        """每步更新上下文"""
        if prev_target is not None:
            self._prev_target = prev_target
        if heading is not None:
            self._prev_heading = heading

    def evaluate(self, waypoints, robot_pos, route_sampler):
        """
        对一组候选目标点评分

        返回: list of (wp_name, score)，按 score 升序（越小越好）
        """
        results = []

        for wp_name in waypoints:
            wp_pos = route_sampler.graph.position(wp_name)

            # 1. 距离代价
            dx = wp_pos[0] - robot_pos[0]
            dy = wp_pos[1] - robot_pos[1]
            dist = math.sqrt(dx*dx + dy*dy)
            distance_cost = dist * self.w_distance

            # 2. 障碍风险（目标点附近的碰撞概率）
            obs_risk = self.cost_map.get_risk(wp_pos[0], wp_pos[1])
            obstacle_cost = obs_risk * self.w_obstacle

            # 3. 新颖性奖励（访问越少 → cost 越低 → 越优先）
            novelty = route_sampler.novelty_score(wp_name)
            novelty_cost = novelty * self.w_novelty

            # 4. 平滑性（目标方向与当前朝向的夹角）
            if dist > 0.01:
                dir_x, dir_y = dx / dist, dy / dist
                hdg_x, hdg_y = self._prev_heading
                dot = dir_x * hdg_x + dir_y * hdg_y
                smooth_cost = (1.0 - dot) * self.w_smooth
            else:
                smooth_cost = 0.0

            total = distance_cost + obstacle_cost + novelty_cost + smooth_cost

            results.append((wp_name, total))

        results.sort(key=lambda x: x[1])
        return results

    def get_best(self, waypoints, robot_pos, route_sampler):
        """直接返回最优目标"""
        scored = self.evaluate(waypoints, robot_pos, route_sampler)
        if scored:
            return scored[0][0]
        return waypoints[0] if waypoints else "start"
