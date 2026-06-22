#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Adaptive Behavior Selector with online learning
从规则选择 → 在线学习选择
"""

import math
import random
from enum import Enum


class Behavior(Enum):
    EXPLORE = "explore"
    PATROL = "patrol"
    CAUTIOUS = "cautious_patrol"
    AVOID = "obstacle_avoid"
    RETURN_BASE = "return_to_base"
    RECOVER = "recover"


class AdaptiveBehaviorSelector:
    """
    行为决策器 - 带在线学习

    核心：不是 if-else 规则，而是 Q-learning 风格的行为选择
    状态 → 行为 → 获得 reward → 更新策略
    """

    def __init__(self):
        # 状态离散化参数
        self._obstacle_bins = [0.2, 0.5, 0.8]  # 低/中/高
        self._battery_bins = [0.2, 0.5, 0.8]
        self._exploration_bins = [2, 5]  # 少/中/多未访问

        # Q-table: (obstacle_level, battery_level, exploration_level) → {behavior: Q值}
        self._q_table = {}
        self._init_q_values()

        # 学习参数
        self.alpha = 0.3    # 学习率
        self.gamma = 0.9    # 折扣因子
        self.epsilon = 0.2  # 探索率（20%随机选择）

        # 历史
        self._last_state = None
        self._last_behavior = None
        self._step_count = 0

    def _init_q_values(self):
        """初始化 Q 值（启发式先验）"""
        behaviors = [b.value for b in Behavior]
        for obs_l in range(3):
            for bat_l in range(3):
                for exp_l in range(3):
                    state = (obs_l, bat_l, exp_l)
                    q = {}
                    for beh in behaviors:
                        # 启发式先验：高障碍→avoid高，低电量→return高，多未访问→explore高
                        prior = 0.0
                        if beh == "obstacle_avoid" and obs_l >= 2:
                            prior = 1.5
                        elif beh == "return_to_base" and bat_l <= 0:
                            prior = 2.0
                        elif beh == "explore" and exp_l >= 2:
                            prior = 1.2
                        elif beh == "cautious_patrol" and obs_l == 1:
                            prior = 0.8
                        elif beh == "patrol":
                            prior = 0.5
                        elif beh == "recover":
                            prior = 0.3
                        q[beh] = prior
                    self._q_table[state] = q

    def _discretize(self, obstacle_density, battery_level, unvisited_count):
        """连续状态 → 离散状态"""
        obs_l = sum(1 for b in self._obstacle_bins if obstacle_density > b)
        bat_l = sum(1 for b in self._battery_bins if battery_level > b)
        exp_l = sum(1 for b in self._exploration_bins if unvisited_count > b)
        return (min(obs_l, 2), min(bat_l, 2), min(exp_l, 2))

    def _reward(self, obstacle_density, battery_level, line_error):
        """
        奖励函数 - 定义什么是"好的行为"

        reward > 0: 好的行为（安全巡逻、探索新区域）
        reward < 0: 差的行为（撞障碍、丢线、浪费电量）
        """
        reward = 0.0

        # 安全奖励：低障碍密度时巡逻 = 好
        if obstacle_density < 0.3:
            reward += 0.5

        # 电量惩罚：低电量时不巡逻 = 好
        if battery_level < 0.2:
            reward += 1.0  # return_base 好
        else:
            reward -= 0.3  # 浪费电量差

        # 丢线惩罚
        reward -= line_error * 2.0

        # 探索奖励：未访问多时探索 = 好
        reward += 0.3

        return reward

    def _epsilon_greedy(self, q_values):
        """ε-greedy 行为选择"""
        if random.random() < self.epsilon:
            return random.choice(list(q_values.keys()))
        return max(q_values, key=q_values.get)

    def select(self, context):
        """
        选择行为（在线学习版本）

        context:
            - obstacle_density: float [0,1]
            - battery_level: float [0,1]
            - unvisited_count: int
            - line_error: float
        """
        obstacle_density = context.get("obstacle_density", 0)
        battery_level = context.get("battery_level", 1.0)
        unvisited_count = context.get("unvisited_count", 5)
        line_error = context.get("line_error", 0)

        # 离散化状态
        state = self._discretize(obstacle_density, battery_level, unvisited_count)

        # 获取 Q 值
        q_values = self._q_table.get(state, {})

        # 更新上一步的 Q 值（如果有的话）
        if self._last_state is not None and self._last_behavior is not None:
            reward = self._reward(obstacle_density, battery_level, line_error)
            old_q = self._q_table[self._last_state][self._last_behavior]
            max_next_q = max(self._q_table[state].values()) if q_values else 0
            new_q = old_q + self.alpha * (reward + self.gamma * max_next_q - old_q)
            self._q_table[self._last_state][self._last_behavior] = new_q

        # 选择行为
        if not q_values:
            behavior_name = "patrol"
        else:
            behavior_name = self._epsilon_greedy(q_values)

        # 记录
        self._last_state = state
        self._last_behavior = behavior_name
        self._step_count += 1

        return Behavior(behavior_name)

    def get_speed_multiplier(self, behavior):
        multipliers = {
            Behavior.EXPLORE: 1.0,
            Behavior.PATROL: 0.85,
            Behavior.CAUTIOUS: 0.55,
            Behavior.AVOID: 0.4,
            Behavior.RETURN_BASE: 0.9,
            Behavior.RECOVER: 0.5,
        }
        return multipliers.get(behavior, 0.8)

    def get_steering_gain(self, behavior):
        gains = {
            Behavior.EXPLORE: 1.0,
            Behavior.PATROL: 1.2,
            Behavior.CAUTIOUS: 1.8,
            Behavior.AVOID: 2.5,
            Behavior.RETURN_BASE: 1.0,
            Behavior.RECOVER: 1.5,
        }
        return gains.get(behavior, 1.0)

    def get_q_summary(self):
        """获取 Q 值学习摘要"""
        summary = {}
        for state, q_vals in self._q_table.items():
            best = max(q_vals, key=q_vals.get)
            summary[f"obs={state[0]}_bat={state[1]}_exp={state[2]}"] = {
                "best_behavior": best,
                "q_values": {k: round(v, 2) for k, v in q_vals.items()}
            }
        return summary
