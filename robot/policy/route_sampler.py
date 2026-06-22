#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Route Sampler - 动态路径选择器
不再走固定路径，而是根据当前上下文动态采样候选目标
"""

import random
import math
import numpy as np
from collections import defaultdict


class WaypointGraph:
    """路径图 - 定义所有可达目标点和连接关系"""

    def __init__(self):
        # 巡逻目标点（比赛场地关键位置）
        self.waypoints = {
            "start":         (0.0,  0.0),
            "zone_a":        (0.6,  0.3),
            "zone_b":        (1.2,  0.0),
            "zone_c":        (0.8, -0.4),
            "zone_d":        (0.3, -0.3),
            "intersection_1":(0.4,  0.15),
            "intersection_2":(0.9,  0.15),
            "intersection_3":(0.9, -0.2),
            "base":          (0.0,  0.0),
        }

        # 邻接关系（哪些点之间有路径相连）
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

    def neighbors(self, wp_name):
        """获取某点的邻居节点"""
        return self.edges.get(wp_name, [])

    def position(self, wp_name):
        """获取某点坐标"""
        return self.waypoints.get(wp_name, (0, 0))

    def all_waypoints(self):
        return list(self.waypoints.keys())


class RouteSampler:
    """
    动态路径采样器
    核心思想：不再固定 A→B→C→D，而是根据上下文动态选择下一个目标
    """

    def __init__(self, graph=None, seed=None):
        self.graph = graph or WaypointGraph()
        self.rng = random.Random(seed)
        self._visit_counts = defaultdict(int)

    def reset(self):
        self._visit_counts.clear()

    def sample(self, context, n_candidates=3):
        """
        根据当前上下文采样 n 个候选目标点

        context dict 必须包含:
            - robot_pos: (x, y) 当前位置
            - current_wp: str 当前所在目标点名称
            - cost_map: CostMap 对象（可选）
        """
        current = context.get("current_wp", "start")
        robot_pos = context.get("robot_pos", (0, 0))

        # 1. 获取所有可达邻居
        candidates = self.graph.neighbors(current)
        if not candidates:
            candidates = [current]

        # 2. 加入全局探索候选（概率性跳转到未频繁访问的区域）
        all_wps = self.graph.all_waypoints()
        unvisited = [w for w in all_wps if self._visit_counts[w] < 2]
        if unvisited and self.rng.random() < 0.3:
            candidates.extend(self.rng.sample(
                unvisited, min(2, len(unvisited))
            ))

        # 3. 去重并限制数量
        candidates = list(dict.fromkeys(candidates))  # 保持顺序去重
        self.rng.shuffle(candidates)
        candidates = candidates[:n_candidates]

        return candidates

    def record_visit(self, wp_name):
        """记录访问过该点"""
        self._visit_counts[wp_name] += 1

    def get_visit_count(self, wp_name):
        return self._visit_counts[wp_name]

    def novelty_score(self, wp_name):
        """新颖性分数：访问越少，分数越高"""
        count = self._visit_counts[wp_name]
        return 1.0 / (1.0 + count)
