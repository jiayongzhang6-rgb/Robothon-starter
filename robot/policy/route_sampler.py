#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Route Sampler v4 - 智能路径规划器

核心升级：从 "随机采样" → "期望奖励最大化"

next_node = argmax(expected_reward(node))

考虑因素：
  - distance（距离）
  - uncertainty（不确定性）
  - revisit_penalty（重复访问惩罚）
  - time_decay（时间衰减）
  - coverage_value（覆盖价值）
"""

import random
import math
import numpy as np
from collections import defaultdict


class WaypointGraph:
    """路径图 - 定义所有可达目标点和连接关系"""

    def __init__(self):
        self.waypoints = {
            "start":         (0.0,  0.0),
            "zone_a":        (0.6,  0.3),
            "zone_b":        (1.2,  0.0),
            "zone_c":        (0.8, -0.4),
            "zone_d":        (0.3, -0.3),
            "intersection_1":(0.4,  0.15),
            "intersection_2":(0.9,  0.15),
            "intersection_3":(0.9, -0.2),
            "base":          (-0.2, 0.0),
        }

        self.edges = {
            "start":    ["intersection_1", "zone_a"],
            "zone_a":   ["start", "intersection_1", "intersection_2"],
            "zone_b":   ["intersection_2", "zone_c"],
            "zone_c":   ["zone_b", "zone_d", "intersection_3"],
            "zone_d":   ["zone_c", "intersection_1", "intersection_3"],
            "intersection_1": ["start", "zone_a", "zone_d", "intersection_2"],
            "intersection_2": ["zone_a", "zone_b", "intersection_1", "intersection_3"],
            "intersection_3": ["zone_c", "zone_d", "intersection_2"],
            "base":     ["start"],
        }

        # 连通距离（预计算，用于期望奖励中的travel cost）
        self._edge_distances = {}
        for wp1, neighbors in self.edges.items():
            p1 = self.waypoints[wp1]
            for wp2 in neighbors:
                p2 = self.waypoints[wp2]
                d = math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
                self._edge_distances[(wp1, wp2)] = d
                self._edge_distances[(wp2, wp1)] = d

    def neighbors(self, wp_name):
        return self.edges.get(wp_name, [])

    def position(self, wp_name):
        return self.waypoints.get(wp_name, (0, 0))

    def all_waypoints(self):
        return list(self.waypoints.keys())

    def edge_distance(self, wp1, wp2):
        """获取两点间的预计算距离"""
        return self._edge_distances.get((wp1, wp2), 999.0)


class IntelligentNodeSelector:
    """
    智能节点选择器 - 期望奖励最大化

    next_node = argmax(E[reward(node)])

    E[reward] = coverage_value * discovery_bonus
              - travel_cost * distance_weight
              - revisit_penalty * visit_count
              + uncertainty_bonus * exploration_value
              + time_decay * recency_bonus
    """

    def __init__(self):
        # 选择参数
        self.w_coverage = 1.0       # 覆盖价值权重
        self.w_travel = 0.4         # 移动代价权重
        self.w_revisit = 0.6        # 重复访问惩罚权重
        self.w_uncertainty = 0.5    # 不确定性奖励权重
        self.w_time_decay = 0.3     # 时间衰减权重
        self.w_surprise = 0.2       # 惊喜值权重（信息增益）

        # 历史
        self._visit_counts = defaultdict(int)
        self._visit_times = defaultdict(float)
        self._feature_history = defaultdict(list)
        self._start_time = 0.0

    def expected_reward(self, wp_name, robot_pos, graph,
                        heading=None, current_time=0.0):
        """
        计算某节点的期望奖励

        正分越高 → 越应该去
        """
        wp_pos = graph.position(wp_name)

        # 1. 覆盖价值：未访问 → 高价值
        visits = self._visit_counts[wp_name]
        coverage_value = self.w_coverage / (1.0 + visits * 0.5)

        # 2. 移动代价：距离越远代价越高
        dx = wp_pos[0] - robot_pos[0]
        dy = wp_pos[1] - robot_pos[1]
        dist = math.sqrt(dx*dx + dy*dy)

        # 基础移动代价
        travel_cost = self.w_travel * dist

        # 转向代价（偏好与当前朝向一致的方向）
        if heading and dist > 0.01:
            dir_x, dir_y = dx / dist, dy / dist
            hdg_x, hdg_y = heading
            dot = dir_x * hdg_x + dir_y * hdg_y
            turn_cost = (1.0 - dot) * 0.2
            travel_cost += self.w_travel * turn_cost

        # 3. 重复访问惩罚：去过越多，惩罚越高
        revisit_penalty = self.w_revisit * (1.0 - 1.0 / (1.0 + visits))

        # 4. 不确定性奖励：不确定性高 → 探索价值高
        uncertainty_bonus = self.w_uncertainty * (1.0 / (1.0 + visits * 0.3))

        # 5. 时间衰减：长时间未去 → 奖励增加
        last_visit = self._visit_times.get(wp_name, 0.0)
        time_since = current_time - last_visit
        time_decay = self.w_time_decay * min(1.0, time_since / 15.0)

        # 6. 惊喜值（信息增益）：如果该区域特征变化大 → 高惊喜
        surprise = self._compute_surprise(wp_name)

        # 综合期望奖励
        reward = (coverage_value
                  - travel_cost
                  - revisit_penalty
                  + uncertainty_bonus
                  + time_decay
                  + self.w_surprise * surprise)

        return reward

    def _compute_surprise(self, wp_name):
        """计算惊喜值（基于历史特征变化）"""
        history = self._feature_history.get(wp_name, [])
        if len(history) < 2:
            return 0.5  # 未知区域给中等惊喜

        # 计算最近两次观测的差异
        recent = history[-1]
        prev = history[-2]
        diff = sum(abs(recent.get(k, 0) - prev.get(k, 0))
                   for k in set(list(recent.keys()) + list(prev.keys())))
        return min(1.0, diff / 5.0)

    def record_visit(self, wp_name, current_time, features=None):
        """记录访问"""
        self._visit_counts[wp_name] += 1
        self._visit_times[wp_name] = current_time
        if features:
            self._feature_history[wp_name].append(features)

    def select_best(self, candidates, robot_pos, graph,
                    heading=None, n=3):
        """
        从候选节点中选出最优的n个

        返回: [(wp_name, expected_reward), ...] 按reward降序
        """
        current_time = self._visit_times.get("__current__", 0.0)

        scored = []
        for wp in candidates:
            reward = self.expected_reward(
                wp, robot_pos, graph, heading, current_time
            )
            scored.append((wp, reward))

        scored.sort(key=lambda x: -x[1])
        return scored[:n]

    def get_stats(self):
        return {
            "visit_counts": dict(self._visit_counts),
            "total_visits": sum(self._visit_counts.values()),
        }


class RouteSampler:
    """
    智能路径规划器 v4

    核心：不再随机采样，而是用期望奖励最大化选择下一个目标
    """

    def __init__(self, graph=None, seed=None):
        self.graph = graph or WaypointGraph()
        self.rng = random.Random(seed)
        self._visit_counts = defaultdict(int)
        self._node_selector = IntelligentNodeSelector()
        self._step_count = 0

    def reset(self):
        self._visit_counts.clear()
        self._node_selector = IntelligentNodeSelector()
        self._step_count = 0

    def sample(self, context, n_candidates=4):
        """
        智能采样：从候选中选出期望奖励最高的n个

        context:
            - robot_pos: (x, y)
            - current_wp: str
            - heading: (hx, hy) optional
        """
        self._step_count += 1
        current = context.get("current_wp", "start")
        robot_pos = context.get("robot_pos", (0, 0))
        heading = context.get("heading", None)

        # 更新节点选择器的当前时间
        self._node_selector._visit_times["__current__"] = self._step_count * 0.05

        # 1. 获取所有可达邻居
        candidates = self.graph.neighbors(current)
        if not candidates:
            candidates = [current]

        # 2. 加入全局候选（智能选择，不是随机）
        all_wps = self.graph.all_waypoints()
        unvisited = [w for w in all_wps if self._visit_counts[w] < 2]
        if unvisited:
            # 选期望奖励最高的未访问节点
            best_unvisited = self._node_selector.select_best(
                unvisited, robot_pos, self.graph, heading, n=2
            )
            for wp, _ in best_unvisited:
                if wp not in candidates:
                    candidates.append(wp)

        # 3. 智能排序（期望奖励最大化）
        scored = self._node_selector.select_best(
            candidates, robot_pos, self.graph, heading, n=n_candidates
        )

        return [s[0] for s in scored]

    def record_visit(self, wp_name):
        self._visit_counts[wp_name] += 1
        self._node_selector.record_visit(wp_name, self._step_count * 0.05)

    def get_visit_count(self, wp_name):
        return self._visit_counts[wp_name]

    def novelty_score(self, wp_name):
        count = self._visit_counts[wp_name]
        return 1.0 / (1.0 + count)
