#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Prediction Layer - 未来状态预测器 v3

核心升级：从 reactive → predictive
  - 预测未来 N 步的状态
  - 用于 early recovery（提前规避）
  - 用于 smoother transitions（平滑切换）
  - 用于 intent预判（提前切换意图）
"""
import math
from collections import deque


class StatePredictor:
    """
    基于历史轨迹的简单线性+惯性预测器

    不依赖外部模型，用最近 K 步的速度和加速度做外推
    """

    def __init__(self, history_size=10, prediction_horizon=5):
        self.history_size = history_size
        self.prediction_horizon = prediction_horizon
        self._pos_history = deque(maxlen=history_size)
        self._vel_history = deque(maxlen=history_size)
        self._behavior_history = deque(maxlen=history_size)

    def update(self, robot_pos, behavior=None):
        """每步更新历史"""
        self._pos_history.append(robot_pos)

        # 计算速度
        if len(self._pos_history) >= 2:
            p1 = self._pos_history[-2]
            p2 = self._pos_history[-1]
            vx = p2[0] - p1[0]
            vy = p2[1] - p1[1]
            self._vel_history.append((vx, vy))
        else:
            self._vel_history.append((0.0, 0.0))

        if behavior is not None:
            self._behavior_history.append(behavior)

    def predict_position(self, steps_ahead=5):
        """
        预测 steps_ahead 步后的位置

        方法：线性外推 + 惯性衰减
        """
        if len(self._pos_history) < 2:
            return self._pos_history[-1] if self._pos_history else (0.0, 0.0)

        # 最近的平均速度
        recent_vels = list(self._vel_history)[-5:]
        if not recent_vels:
            return self._pos_history[-1]

        avg_vx = sum(v[0] for v in recent_vels) / len(recent_vels)
        avg_vy = sum(v[1] for v in recent_vels) / len(recent_vels)

        # 惯性衰减（越远的预测越不确定）
        decay = 0.85 ** steps_ahead  # 指数衰减

        current = self._pos_history[-1]
        pred_x = current[0] + avg_vx * steps_ahead * decay
        pred_y = current[1] + avg_vy * steps_ahead * decay

        return (pred_x, pred_y)

    def predict_trajectory(self, horizon=None):
        """预测未来多个时间步的轨迹"""
        horizon = horizon or self.prediction_horizon
        trajectory = []
        for i in range(1, horizon + 1):
            pos = self.predict_position(i)
            trajectory.append(pos)
        return trajectory

    def predict_behavior_transition(self, current_behavior):
        """
        预测行为是否即将切换

        基于历史行为变化频率
        """
        if len(self._behavior_history) < 3:
            return current_behavior, 0.5  # 置信度低

        behaviors = list(self._behavior_history)
        # 计算最近的行为变化率
        changes = 0
        for i in range(1, len(behaviors)):
            if behaviors[i] != behaviors[i-1]:
                changes += 1
        change_rate = changes / (len(behaviors) - 1)

        # 高变化率 → 预测可能再次切换
        confidence = 1.0 - change_rate

        return current_behavior, confidence

    def get_future_error(self, target_pos, steps_ahead=5):
        """
        预测 steps_ahead 步后的到目标距离

        用于 early recovery：如果预测未来会偏离太多，提前调整
        """
        predicted_pos = self.predict_position(steps_ahead)
        dx = target_pos[0] - predicted_pos[0]
        dy = target_pos[1] - predicted_pos[1]
        return math.sqrt(dx*dx + dy*dy)

    def is_heading_toward_obstacle(self, obstacles, lookahead=3):
        """
        预测是否正朝障碍物前进

        用于 early obstacle avoidance
        """
        predicted = self.predict_position(lookahead)
        for obs in obstacles:
            dx = predicted[0] - obs[0]
            dy = predicted[1] - obs[1]
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < 0.2:  # 预测距离障碍 < 0.2m
                return True, obs
        return False, None

    def get_prediction_confidence(self):
        """预测置信度：基于历史数据量"""
        n = len(self._pos_history)
        if n < 3:
            return 0.3
        elif n < 8:
            return 0.6
        else:
            return 0.9


class RiskPredictor:
    """
    风险预测器 - 结合 RiskEstimator 和 StatePredictor

    不只看当前风险，还预测未来风险趋势
    """

    def __init__(self, state_predictor):
        self.state_predictor = state_predictor

    def predict_risk_trend(self, cost_map, steps=5):
        """
        预测未来 N 步的风险趋势

        返回: (current_risk, predicted_risk, trend)
            trend: "increasing" / "stable" / "decreasing"
        """
        current_pos = self.state_predictor._pos_history[-1] \
            if self.state_predictor._pos_history else (0, 0)
        current_risk = cost_map.get_risk(current_pos[0], current_pos[1])

        predicted_pos = self.state_predictor.predict_position(steps)
        predicted_risk = cost_map.get_risk(predicted_pos[0], predicted_pos[1])

        if predicted_risk > current_risk * 1.2:
            trend = "increasing"
        elif predicted_risk < current_risk * 0.8:
            trend = "decreasing"
        else:
            trend = "stable"

        return current_risk, predicted_risk, trend

    def should_early_recover(self, cost_map, threshold=0.5):
        """
        判断是否需要提前恢复

        如果预测未来风险 > threshold，即使当前风险低也提前行动
        """
        current_risk, predicted_risk, trend = self.predict_risk_trend(cost_map)
        if predicted_risk > threshold and trend == "increasing":
            return True, predicted_risk
        return False, current_risk
