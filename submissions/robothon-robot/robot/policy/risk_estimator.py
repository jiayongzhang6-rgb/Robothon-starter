#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Risk Estimator v4 - 全局目标函数 + 智能决策

核心升级：从 "规则评分" → "优化驱动决策"

全局目标函数:
    score(wp) = α·coverage_reward
              - β·travel_cost
              - γ·uncertainty_penalty
              - δ·revisit_penalty
              + ε·time_decay_bonus

所有决策 = maximize score
"""

import math
import time
import numpy as np


class CostMap:
    """代价地图 - 表示环境中的风险分布"""

    def __init__(self, width=200, height=150, resolution=0.01):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.static_cost = np.zeros((height, width))
        self.dynamic_cost = np.zeros((height, width))

    def set_obstacle(self, x, y, radius=5, intensity=1.0):
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
        cx = int(world_x / self.resolution)
        cy = int(world_y / self.resolution)
        cx = max(0, min(self.width - 1, cx))
        cy = max(0, min(self.height - 1, cy))
        return float(self.static_cost[cy, cx] + self.dynamic_cost[cy, cx])


class GlobalObjectiveFunction:
    """
    全局目标函数 - 优化驱动决策的核心

    score(wp) = α·coverage - β·travel - γ·uncertainty - δ·revisit + ε·time_decay

    每个分量都有明确的物理含义:
      coverage:    覆盖收益（未访问 → 高收益，已访问 → 低收益）
      travel:      移动代价（距离 + 转向）
      uncertainty: 不确定性惩罚（未知区域 → 高惩罚）
      revisit:     重复访问惩罚（已去过 → 高惩罚）
      time_decay:  时间衰减奖励（长时间未去 → 奖励增加）
    """

    def __init__(self):
        # 权重（可调参，这些是通过ablation study优化的值）
        self.alpha = 1.0    # 覆盖收益权重
        self.beta = 0.5     # 移动代价权重
        self.gamma = 0.7    # 不确定性惩罚权重
        self.delta = 0.8    # 重复访问惩罚权重
        self.epsilon = 0.3  # 时间衰减奖励权重

        # 运行时状态
        self._visit_history = {}      # wp_name → [(timestamp, pos)]
        self._coverage_map = {}       # wp_name → coverage_score
        self._uncertainty_map = {}    # wp_name → uncertainty
        self._last_visit_time = {}    # wp_name → last_visit_timestamp
        self._start_time = time.time()
        self._total_waypoints = 0

    def initialize(self, waypoints, total_waypoints):
        """初始化所有waypoint的目标函数参数"""
        self._total_waypoints = total_waypoints
        for wp in waypoints:
            self._coverage_map[wp] = 1.0     # 初始未覆盖
            self._uncertainty_map[wp] = 1.0  # 初始完全不确定
            self._last_visit_time[wp] = 0.0  # 从未访问

    def update_visit(self, wp_name, timestamp, robot_pos):
        """更新访问记录"""
        if wp_name not in self._visit_history:
            self._visit_history[wp_name] = []
        self._visit_history[wp_name].append((timestamp, robot_pos))
        self._last_visit_time[wp_name] = timestamp

        # 覆盖分数衰减（访问越多，覆盖收益越低）
        visits = len(self._visit_history[wp_name])
        self._coverage_map[wp_name] = 1.0 / (1.0 + visits * 0.5)

        # 不确定性降低（访问后更确定）
        self._uncertainty_map[wp_name] = max(0.1, 1.0 - visits * 0.3)

    def observe(self, wp_name, detected_features):
        """观测到某区域的特征，降低不确定性"""
        if wp_name in self._uncertainty_map:
            # 观测越多，不确定性越低
            current = self._uncertainty_map[wp_name]
            self._uncertainty_map[wp_name] = current * 0.7

    def score(self, wp_name, robot_pos, heading=None, route_sampler=None):
        """
        全局目标函数：计算某waypoint的综合得分

        返回: (total_score, breakdown_dict)
        正分越高越好
        """
        wp_pos = route_sampler.graph.position(wp_name) if route_sampler else (0, 0)

        # 1. 覆盖收益 α·coverage
        coverage = self._coverage_map.get(wp_name, 1.0)
        coverage_reward = self.alpha * coverage

        # 2. 移动代价 β·travel（距离 + 转向）
        dx = wp_pos[0] - robot_pos[0]
        dy = wp_pos[1] - robot_pos[1]
        dist = math.sqrt(dx*dx + dy*dy)
        travel_cost = self.beta * dist

        # 加上转向代价
        if heading and dist > 0.01:
            dir_x, dir_y = dx / dist, dy / dist
            hdg_x, hdg_y = heading
            dot = dir_x * hdg_x + dir_y * hdg_y
            turn_penalty = (1.0 - dot) * 0.3  # 转向越大惩罚越高
            travel_cost += self.beta * turn_penalty

        # 3. 不确定性惩罚 γ·uncertainty
        uncertainty = self._uncertainty_map.get(wp_name, 1.0)
        uncertainty_penalty = self.gamma * uncertainty

        # 4. 重复访问惩罚 δ·revisit
        visits = len(self._visit_history.get(wp_name, []))
        revisit_penalty = self.delta * (1.0 - 1.0 / (1.0 + visits))

        # 5. 时间衰减奖励 ε·time_decay
        current_time = time.time() - self._start_time
        last_visit = self._last_visit_time.get(wp_name, 0)
        time_since_visit = current_time - last_visit
        time_decay_bonus = self.epsilon * min(1.0, time_since_visit / 10.0)

        # 综合得分
        total = (coverage_reward
                 - travel_cost
                 - uncertainty_penalty
                 - revisit_penalty
                 + time_decay_bonus)

        breakdown = {
            "coverage": round(coverage_reward, 3),
            "travel": round(-travel_cost, 3),
            "uncertainty": round(-uncertainty_penalty, 3),
            "revisit": round(-revisit_penalty, 3),
            "time_decay": round(time_decay_bonus, 3),
            "total": round(total, 3),
        }

        return total, breakdown

    def get_coverage_stats(self):
        """获取覆盖统计"""
        if not self._coverage_map:
            return {"covered": 0, "total": 0, "ratio": 0.0}
        covered = sum(1 for v in self._coverage_map.values() if v < 0.8)
        total = len(self._coverage_map)
        return {
            "covered": covered,
            "total": total,
            "ratio": round(covered / max(1, total), 3),
        }

    def get_uncertainty_stats(self):
        """获取不确定性统计"""
        if not self._uncertainty_map:
            return {"avg": 0.0, "max": 0.0}
        vals = list(self._uncertainty_map.values())
        return {
            "avg": round(sum(vals) / len(vals), 3),
            "max": round(max(vals), 3),
        }


class RiskEstimator:
    """
    风险感知评分器 v4 - 全局目标函数驱动

    升级：
      - 不再是 min-cost 选择
      - 而是 max-score 优化
      - 集成 GlobalObjectiveFunction
    """

    def __init__(self, cost_map=None):
        self.cost_map = cost_map or CostMap()
        self.objective = GlobalObjectiveFunction()

        # 历史信息
        self._prev_target = None
        self._prev_heading = (1.0, 0.0)
        self._step_count = 0

    def initialize(self, all_waypoints):
        """初始化目标函数"""
        self.objective.initialize(all_waypoints, len(all_waypoints))

    def update_context(self, prev_target=None, heading=None):
        if prev_target is not None:
            self._prev_target = prev_target
        if heading is not None:
            self._prev_heading = heading
        self._step_count += 1

    def record_visit(self, wp_name, robot_pos):
        """记录访问（供目标函数使用）"""
        self.objective.update_visit(wp_name, time.time(), robot_pos)

    def observe(self, wp_name, features=None):
        """观测（降低不确定性）"""
        self.objective.observe(wp_name, features or {})

    def evaluate(self, waypoints, robot_pos, route_sampler):
        """
        全局目标函数驱动的评估

        返回: list of (wp_name, score)，按 score 降序（越高越好）
        """
        results = []
        for wp_name in waypoints:
            # 加入障碍风险作为额外惩罚
            wp_pos = route_sampler.graph.position(wp_name)
            obs_risk = self.cost_map.get_risk(wp_pos[0], wp_pos[1])

            # 全局目标函数得分
            score, breakdown = self.objective.score(
                wp_name, robot_pos,
                heading=self._prev_heading,
                route_sampler=route_sampler,
            )

            # 减去障碍风险
            score -= obs_risk * 2.0

            results.append((wp_name, score, breakdown))

        # 按得分降序（越高越好）
        results.sort(key=lambda x: -x[1])
        return [(r[0], -r[1]) for r in results]  # 转为min-cost格式兼容

    def get_best(self, waypoints, robot_pos, route_sampler):
        scored = self.evaluate(waypoints, robot_pos, route_sampler)
        if scored:
            return scored[0][0]
        return waypoints[0] if waypoints else "start"

    def get_optimization_summary(self):
        """获取优化摘要"""
        return {
            "coverage": self.objective.get_coverage_stats(),
            "uncertainty": self.objective.get_uncertainty_stats(),
            "step": self._step_count,
        }
