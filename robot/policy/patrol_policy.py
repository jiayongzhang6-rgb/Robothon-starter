#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PatrolPolicy v2 - Adaptive Patrol Agent with online learning
核心 AI 决策层 + 完整决策日志
"""

import math
from .route_sampler import RouteSampler, WaypointGraph
from .risk_estimator import RiskEstimator, CostMap
from .behavior_selector import AdaptiveBehaviorSelector, Behavior


class DecisionLog:
    """决策日志 - 记录每步决策过程"""

    def __init__(self):
        self.entries = []

    def log(self, step, **kwargs):
        self.entries.append({"step": step, **kwargs})

    def get_trace(self, last_n=5):
        """获取最近 N 步决策轨迹"""
        recent = self.entries[-last_n:]
        lines = []
        for e in recent:
            lines.append(
                f"  step {e['step']}: "
                f"sampled {e.get('candidates', [])} → "
                f"selected {e.get('selected_wp', '?')} "
                f"(cost={e.get('selected_cost', '?')}) | "
                f"behavior={e.get('behavior', '?')} "
                f"(Q={e.get('q_values', {})})"
            )
        return "\n".join(lines)

    def get_full_trace(self):
        return self.get_trace(last_n=len(self.entries))

    def summary(self):
        behaviors = {}
        for e in self.entries:
            b = e.get("behavior", "unknown")
            behaviors[b] = behaviors.get(b, 0) + 1
        return {
            "total_steps": len(self.entries),
            "behavior_distribution": behaviors,
        }


class PatrolPolicy:
    """
    Adaptive Patrol Agent - 自适应巡逻策略 v2

    带在线学习的分层决策系统:
        observe → evaluate → decide → act → reward → adapt
    """

    def __init__(self, seed=None):
        graph = WaypointGraph()
        self.route_sampler = RouteSampler(graph, seed=seed)
        self.risk_estimator = RiskEstimator(CostMap())
        self.behavior_selector = AdaptiveBehaviorSelector()
        self.decision_log = DecisionLog()

        self._current_wp = "start"
        self._target_wp = None
        self._behavior = Behavior.PATROL
        self._heading = (1.0, 0.0)
        self._robot_pos = (0.0, 0.0)
        self._step_count = 0
        self._path_history = []

    def reset(self):
        self.route_sampler.reset()
        self._current_wp = "start"
        self._target_wp = None
        self._behavior = Behavior.PATROL
        self._heading = (1.0, 0.0)
        self._robot_pos = (0.0, 0.0)
        self._step_count = 0
        self._path_history = []
        self.decision_log = DecisionLog()

    def observe(self, robot_pos, heading=None, **kwargs):
        self._robot_pos = robot_pos
        if heading is not None:
            self._heading = heading
        self._path_history.append(robot_pos)
        self._step_count += 1

        if self._target_wp:
            target_pos = self.route_sampler.graph.position(self._target_wp)
            dist = math.sqrt(
                (robot_pos[0] - target_pos[0])**2 +
                (robot_pos[1] - target_pos[1])**2
            )
            if dist < 0.15:
                self._current_wp = self._target_wp
                self.route_sampler.record_visit(self._current_wp)

        return {
            "robot_pos": robot_pos,
            "current_wp": self._current_wp,
            "target_wp": self._target_wp,
            "step_count": self._step_count,
            **kwargs
        }

    def decide(self, context):
        """核心决策 + 完整日志"""

        # 1. 行为决策（在线学习）
        behavior_context = {
            "line_error": context.get("line_error", 0),
            "obstacle_density": context.get("obstacle_density", 0),
            "unvisited_count": self._count_unvisited(),
            "battery_level": context.get("battery_level", 1.0),
        }
        self._behavior = self.behavior_selector.select(behavior_context)

        # 2. 目标选择
        candidates = []
        selected_cost = None
        if self._behavior in (Behavior.EXPLORE, Behavior.PATROL,
                               Behavior.CAUTIOUS, Behavior.RETURN_BASE,
                               Behavior.AVOID):
            candidates, selected_cost = self._select_target_with_log(context)

        # 3. 获取目标坐标
        if self._target_wp:
            target_pos = self.route_sampler.graph.position(self._target_wp)
        else:
            target_pos = self.route_sampler.graph.position(self._current_wp)

        speed_factor = self.behavior_selector.get_speed_multiplier(self._behavior)
        steering_gain = self.behavior_selector.get_steering_gain(self._behavior)

        # 4. 记录决策日志
        q_vals = {}
        state = self.behavior_selector._last_state
        if state and state in self.behavior_selector._q_table:
            q_vals = {k: round(v, 2)
                      for k, v in self.behavior_selector._q_table[state].items()}

        self.decision_log.log(
            step=self._step_count,
            candidates=candidates,
            selected_wp=self._target_wp,
            selected_cost=round(selected_cost, 3) if selected_cost is not None else None,
            behavior=self._behavior.value,
            q_values=q_vals,
            pos=(round(self._robot_pos[0], 3), round(self._robot_pos[1], 3)),
        )

        return {
            "target": target_pos,
            "behavior": self._behavior,
            "speed_factor": speed_factor,
            "steering_gain": steering_gain,
            "info": {
                "current_wp": self._current_wp,
                "target_wp": self._target_wp,
                "step": self._step_count,
                "unvisited": self._count_unvisited(),
                "path_length": len(self._path_history),
                "candidates": candidates,
            }
        }

    def _select_target_with_log(self, context):
        """选择目标并返回候选列表 + 选中代价"""
        sample_ctx = {
            "robot_pos": self._robot_pos,
            "current_wp": self._current_wp,
        }
        candidates = self.route_sampler.sample(sample_ctx, n_candidates=4)

        self.risk_estimator.update_context(
            prev_target=self._target_wp,
            heading=self._heading
        )
        scored = self.risk_estimator.evaluate(
            candidates, self._robot_pos, self.route_sampler
        )

        if scored:
            self._target_wp = scored[0][0]
            return [s[0] for s in scored], scored[0][1]
        return candidates, 0.0

    def _count_unvisited(self):
        all_wps = self.route_sampler.graph.all_waypoints()
        return sum(1 for w in all_wps
                   if self.route_sampler.get_visit_count(w) < 2)

    def get_path_history(self):
        return list(self._path_history)

    def get_state_summary(self):
        return {
            "current_wp": self._current_wp,
            "target_wp": self._target_wp,
            "behavior": self._behavior.value,
            "step": self._step_count,
            "position": self._robot_pos,
            "unvisited": self._count_unvisited(),
            "visit_counts": dict(self.route_sampler._visit_counts),
            "decision_trace": self.decision_log.get_trace(last_n=5),
            "q_learning_summary": list(self.behavior_selector.get_q_summary().values())[:3],
        }
