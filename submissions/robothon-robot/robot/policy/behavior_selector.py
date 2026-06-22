#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Adaptive Behavior Selector v3 - 概率决策 + 在线学习

核心升级：epsilon-greedy → softmax probabilistic transitions
不再是 "if obstacle > threshold then avoid"，而是：
  P(next_state | belief, uncertainty) = softmax(Q / temperature)
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
    行为决策器 v3 - 概率决策版

    升级点：
    1. Softmax选择（概率分布，不是硬切换）
    2. 意图感知（TaskIntent层影响行为概率）
    3. 不确定性驱动（belief uncertainty影响探索/利用权衡）
    4. 温度自适应（根据任务进度调整决策随机性）
    """

    def __init__(self):
        # 状态离散化参数
        self._obstacle_bins = [0.2, 0.5, 0.8]
        self._battery_bins = [0.2, 0.5, 0.8]
        self._exploration_bins = [2, 5]

        # Q-table: (obstacle, battery, exploration) → {behavior: Q值}
        self._q_table = {}
        self._init_q_values()

        # 学习参数
        self.alpha = 0.3    # 学习率
        self.gamma = 0.9    # 折扣因子

        # === 概率决策参数 ===
        self.temperature = 1.0     # softmax温度（低=确定性高，高=随机性高）
        self.temp_min = 0.3        # 最低温度（最确定）
        self.temp_max = 2.0        # 最高温度（最随机）
        self.temp_decay = 0.995    # 每步温度衰减（逐渐趋于确定）

        # 意图偏置（TaskIntent层可以注入）
        self._intent_bias = {}     # {behavior_name: bonus}

        # 历史
        self._last_state = None
        self._last_behavior = None
        self._step_count = 0

        # 概率分布缓存（供UI显示）
        self._last_probs = {}

    def _init_q_values(self):
        """初始化 Q 值（启发式先验）"""
        behaviors = [b.value for b in Behavior]
        for obs_l in range(3):
            for bat_l in range(3):
                for exp_l in range(3):
                    state = (obs_l, bat_l, exp_l)
                    q = {}
                    for beh in behaviors:
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
        obs_l = sum(1 for b in self._obstacle_bins if obstacle_density > b)
        bat_l = sum(1 for b in self._battery_bins if battery_level > b)
        exp_l = sum(1 for b in self._exploration_bins if unvisited_count > b)
        return (min(obs_l, 2), min(bat_l, 2), min(exp_l, 2))

    def _softmax(self, q_values, temperature=None):
        """
        概率选择核心：softmax(Q / temperature)

        返回: {behavior_name: probability}
        """
        temp = temperature or self.temperature

        # 加上意图偏置
        biased_q = {}
        for beh, q in q_values.items():
            bias = self._intent_bias.get(beh, 0.0)
            biased_q[beh] = q + bias

        # 数值稳定softmax
        max_q = max(biased_q.values())
        exp_values = {}
        exp_sum = 0.0
        for beh, q in biased_q.items():
            exp_val = math.exp((q - max_q) / max(temp, 0.01))
            exp_values[beh] = exp_val
            exp_sum += exp_val

        probs = {}
        for beh, exp_val in exp_values.items():
            probs[beh] = exp_val / exp_sum if exp_sum > 0 else 1.0 / len(biased_q)

        return probs

    def _sample_from_probs(self, probs):
        """从概率分布中采样"""
        r = random.random()
        cumulative = 0.0
        for beh, prob in sorted(probs.items()):
            cumulative += prob
            if r <= cumulative:
                return beh
        return list(probs.keys())[-1]

    def _reward(self, obstacle_density, battery_level, line_error):
        reward = 0.0
        if obstacle_density < 0.3:
            reward += 0.5
        if battery_level < 0.2:
            reward += 1.0
        else:
            reward -= 0.3
        reward -= line_error * 2.0
        reward += 0.3
        return reward

    def set_intent_bias(self, intent_type):
        """
        根据TaskIntent层的意图类型设置行为偏置

        意图 → 行为概率偏移
        """
        self._intent_bias = {}

        if intent_type is None:
            return

        intent_name = intent_type.name if hasattr(intent_type, 'name') else str(intent_type)

        if intent_name == "AVOID_DANGER":
            self._intent_bias["obstacle_avoid"] = 2.0
            self._intent_bias["cautious_patrol"] = 1.0
        elif intent_name == "RETURN_BASE":
            self._intent_bias["return_to_base"] = 2.5
        elif intent_name == "RECOVER":
            self._intent_bias["recover"] = 2.0
            self._intent_bias["explore"] = 0.5
        elif intent_name == "INVESTIGATE":
            self._intent_bias["explore"] = 1.5
            self._intent_bias["cautious_patrol"] = 1.0
        elif intent_name == "PATROL_AREA":
            self._intent_bias["patrol"] = 1.0
            self._intent_bias["explore"] = 0.8

    def select(self, context):
        """
        概率行为选择

        context:
            - obstacle_density: float [0,1]
            - battery_level: float [0,1]
            - unvisited_count: int
            - line_error: float
            - intent_type: IntentType (optional, from TaskIntentLayer)
            - uncertainty: float [0,1] (optional, belief uncertainty)
        """
        obstacle_density = context.get("obstacle_density", 0)
        battery_level = context.get("battery_level", 1.0)
        unvisited_count = context.get("unvisited_count", 5)
        line_error = context.get("line_error", 0)
        uncertainty = context.get("uncertainty", 0.5)

        # 离散化状态
        state = self._discretize(obstacle_density, battery_level, unvisited_count)
        q_values = self._q_table.get(state, {})

        # 更新上一步Q值
        if self._last_state is not None and self._last_behavior is not None:
            reward = self._reward(obstacle_density, battery_level, line_error)
            old_q = self._q_table[self._last_state][self._last_behavior]
            max_next_q = max(self._q_table[state].values()) if q_values else 0
            new_q = old_q + self.alpha * (reward + self.gamma * max_next_q - old_q)
            self._q_table[self._last_state][self._last_behavior] = new_q

        # === 概率决策 ===
        # 不确定性高 → 温度升高 → 更多探索
        adaptive_temp = self.temperature * (1.0 + uncertainty * 0.5)

        if not q_values:
            behavior_name = "patrol"
            self._last_probs = {"patrol": 1.0}
        else:
            probs = self._softmax(q_values, adaptive_temp)
            self._last_probs = probs
            behavior_name = self._sample_from_probs(probs)

        # 温度自适应衰减
        self.temperature = max(self.temp_min,
                               self.temperature * self.temp_decay)

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
        summary = {}
        for state, q_vals in self._q_table.items():
            best = max(q_vals, key=q_vals.get)
            summary[f"obs={state[0]}_bat={state[1]}_exp={state[2]}"] = {
                "best_behavior": best,
                "q_values": {k: round(v, 2) for k, v in q_vals.items()}
            }
        return summary

    def get_decision_info(self):
        """获取当前决策信息（供UI/日志）"""
        return {
            "temperature": round(self.temperature, 3),
            "last_probs": {k: round(v, 3) for k, v in self._last_probs.items()},
            "intent_bias": self._intent_bias.copy(),
        }
