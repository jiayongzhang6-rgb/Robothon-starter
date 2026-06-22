#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PatrolPolicy v3 - Adaptive Patrol Agent with Full AI Stack

架构升级：
    Observation → Belief → TaskIntent → Planning → ProbabilisticFSM → Control
                                  ↑                        ↓
                          Prediction ←──→ Recovery ←────────┘

核心新增：
    1. TaskIntent Layer: 目标驱动规划
    2. Probabilistic FSM: softmax概率决策
    3. Prediction Layer: 未来状态预测
    4. Adaptive Recovery: re-localize + re-plan
"""
import math
from .route_sampler import RouteSampler, WaypointGraph
from .risk_estimator import RiskEstimator, CostMap
from .behavior_selector import AdaptiveBehaviorSelector, Behavior
from .task_intent import TaskIntentLayer, IntentType
from .prediction import StatePredictor, RiskPredictor
from .recovery import AdaptiveRecovery, RecoveryPhase


class DecisionLog:
    """决策日志 - 完整记录每步决策过程（含新层信息）"""

    def __init__(self):
        self.entries = []

    def log(self, step, **kwargs):
        self.entries.append({"step": step, **kwargs})

    def get_trace(self, last_n=5):
        recent = self.entries[-last_n:]
        lines = []
        for e in recent:
            lines.append(
                f"  step {e['step']}: "
                f"intent={e.get('intent', '?')} | "
                f"behavior={e.get('behavior', '?')} "
                f"(Q={e.get('q_values', {})}) | "
                f"subgoal={e.get('subgoal', '?')} | "
                f"prediction={e.get('prediction_confidence', '?')}"
            )
        return "\n".join(lines)

    def get_full_trace(self):
        return self.get_trace(last_n=len(self.entries))

    def summary(self):
        behaviors = {}
        intents = {}
        for e in self.entries:
            b = e.get("behavior", "unknown")
            i = e.get("intent", "unknown")
            behaviors[b] = behaviors.get(b, 0) + 1
            intents[i] = intents.get(i, 0) + 1
        return {
            "total_steps": len(self.entries),
            "behavior_distribution": behaviors,
            "intent_distribution": intents,
        }


class PatrolPolicy:
    """
    Adaptive Patrol Agent - 分层策略系统 v3

    完整AI栈：
        observe → believe → plan → decide → predict → act → recover → learn
    """

    def __init__(self, seed=None):
        # === 核心模块 ===
        graph = WaypointGraph()
        self.route_sampler = RouteSampler(graph, seed=seed)
        self.risk_estimator = RiskEstimator(CostMap())
        self.behavior_selector = AdaptiveBehaviorSelector()
        self.decision_log = DecisionLog()

        # === 新增层 ===
        self.task_intent = TaskIntentLayer()
        self.state_predictor = StatePredictor(history_size=10,
                                               prediction_horizon=5)
        self.risk_predictor = RiskPredictor(self.state_predictor)
        self.recovery = AdaptiveRecovery()

        # === 状态 ===
        self._current_wp = "start"
        self._target_wp = None
        self._behavior = Behavior.PATROL
        self._heading = (1.0, 0.0)
        self._robot_pos = (0.0, 0.0)
        self._step_count = 0
        self._path_history = []
        self._belief_state = {}
        self._uncertainty = 0.5

    def reset(self):
        self.route_sampler.reset()
        self.task_intent = TaskIntentLayer()
        self.state_predictor = StatePredictor()
        self.risk_predictor = RiskPredictor(self.state_predictor)
        self.recovery = AdaptiveRecovery()
        self._current_wp = "start"
        self._target_wp = None
        self._behavior = Behavior.PATROL
        self._heading = (1.0, 0.0)
        self._robot_pos = (0.0, 0.0)
        self._step_count = 0
        self._path_history = []
        self._belief_state = {}
        self._uncertainty = 0.5
        self.decision_log = DecisionLog()

    def observe(self, robot_pos, heading=None, **kwargs):
        """观测层 - 收集环境信息 + 更新预测器"""
        self._robot_pos = robot_pos
        if heading is not None:
            self._heading = heading
        self._path_history.append(robot_pos)
        self._step_count += 1

        # 更新预测器
        self.state_predictor.update(robot_pos, self._behavior.value)

        # 检查到达
        if self._target_wp:
            target_pos = self.route_sampler.graph.position(self._target_wp)
            dist = math.sqrt(
                (robot_pos[0] - target_pos[0])**2 +
                (robot_pos[1] - target_pos[1])**2
            )
            if dist < 0.15:
                self._current_wp = self._target_wp
                self.route_sampler.record_visit(self._current_wp)
                # 推进子目标
                self.task_intent.advance_subgoal()

        # 构建belief state
        self._belief_state = {
            "robot_pos": robot_pos,
            "current_wp": self._current_wp,
            "target_wp": self._target_wp,
            "step_count": self._step_count,
            "obstacle_density": kwargs.get("obstacle_density", 0),
            "battery_level": kwargs.get("battery_level", 1.0),
            "line_error": kwargs.get("line_error", 0),
            "confidence": 1.0 - self._uncertainty,
            "available_waypoints": self.route_sampler.graph.all_waypoints(),
            **kwargs
        }

        return self._belief_state

    def decide(self, context):
        """核心决策 - 集成所有AI层"""

        # === 1. Intent层：评估当前意图 ===
        self.task_intent.evaluate_intent(self._belief_state)
        current_intent = self.task_intent.current_intent

        # 确保有子目标
        if not self.task_intent._subgoals:
            self.task_intent.generate_subgoals(
                current_intent,
                self.route_sampler.graph.all_waypoints(),
                self._robot_pos,
            )

        # === 2. Recovery检查 ===
        if self.recovery.phase != RecoveryPhase.IDLE:
            # 恢复进行中
            recovery_result = self.recovery.update(
                self._robot_pos,
                self.risk_estimator.cost_map,
                self.route_sampler.graph,
                self.route_sampler,
            )
            if recovery_result["target"]:
                # 恢复有目标
                target_pos = recovery_result["target"]
                if isinstance(target_pos, str):
                    target_pos = self.route_sampler.graph.position(target_pos)
                self._behavior = Behavior.RECOVER
                self.decision_log.log(
                    step=self._step_count,
                    intent=current_intent.name,
                    behavior="recover",
                    subgoal=f"recovery:{recovery_result['phase']}",
                    prediction_confidence=round(
                        self.state_predictor.get_prediction_confidence(), 2),
                    pos=(round(self._robot_pos[0], 3),
                         round(self._robot_pos[1], 3)),
                    recovery=recovery_result["recovery_plan"],
                )
                return {
                    "target": target_pos,
                    "behavior": Behavior.RECOVER,
                    "speed_factor": 0.4,
                    "steering_gain": 1.5,
                    "info": {
                        "current_wp": self._current_wp,
                        "target_wp": "recovery",
                        "step": self._step_count,
                        "intent": current_intent.name,
                        "recovery_phase": recovery_result["phase"],
                        "prediction_confidence": round(
                            self.state_predictor.get_prediction_confidence(), 2),
                    }
                }
            else:
                # 恢复完成，继续正常流程
                pass

        # === 3. 行为决策（概率版，意图感知）===
        self.behavior_selector.set_intent_bias(current_intent)

        behavior_context = {
            "line_error": context.get("line_error", 0),
            "obstacle_density": context.get("obstacle_density", 0),
            "unvisited_count": self._count_unvisited(),
            "battery_level": context.get("battery_level", 1.0),
            "intent_type": current_intent,
            "uncertainty": self._uncertainty,
        }
        self._behavior = self.behavior_selector.select(behavior_context)

        # === 4. 目标选择 ===
        candidates = []
        selected_cost = None
        if self._behavior in (Behavior.EXPLORE, Behavior.PATROL,
                               Behavior.CAUTIOUS, Behavior.RETURN_BASE,
                               Behavior.AVOID):
            candidates, selected_cost = self._select_target_with_log(context)

        # === 5. Prediction层：预测未来 + early recovery ===
        prediction_confidence = self.state_predictor.get_prediction_confidence()
        if self._target_wp:
            target_pos = self.route_sampler.graph.position(self._target_wp)
            future_error = self.state_predictor.get_future_error(target_pos, 5)
            # 预测信息用于UI显示，不自动触发恢复
            # recovery只由stagnation触发
        else:
            target_pos = self.route_sampler.graph.position(self._current_wp)

        # === 6. Risk预测 ===
        current_risk, predicted_risk, trend = \
            self.risk_predictor.predict_risk_trend(
                self.risk_estimator.cost_map
            )

        speed_factor = self.behavior_selector.get_speed_multiplier(
            self._behavior)
        steering_gain = self.behavior_selector.get_steering_gain(
            self._behavior)

        # === 7. 记录完整决策日志 ===
        q_vals = {}
        state = self.behavior_selector._last_state
        if state and state in self.behavior_selector._q_table:
            q_vals = {k: round(v, 2)
                      for k, v in self.behavior_selector._q_table[state].items()}

        self.decision_log.log(
            step=self._step_count,
            intent=current_intent.name,
            candidates=candidates,
            selected_wp=self._target_wp,
            selected_cost=round(selected_cost, 3) if selected_cost else None,
            behavior=self._behavior.value,
            q_values=q_vals,
            subgoal=str(self.task_intent.current_subgoal),
            prediction_confidence=round(prediction_confidence, 2),
            risk_trend=trend,
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
                "intent": current_intent.name,
                "subgoal": str(self.task_intent.current_subgoal),
                "unvisited": self._count_unvisited(),
                "path_length": len(self._path_history),
                "candidates": candidates,
                "prediction_confidence": round(prediction_confidence, 2),
                "risk_trend": trend,
                "temperature": round(
                    self.behavior_selector.temperature, 3),
            }
        }

    def _select_target_with_log(self, context):
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
            "intent": self.task_intent.current_intent.name,
            "step": self._step_count,
            "position": self._robot_pos,
            "unvisited": self._count_unvisited(),
            "visit_counts": dict(self.route_sampler._visit_counts),
            "decision_trace": self.decision_log.get_trace(last_n=5),
            "q_learning_summary": list(
                self.behavior_selector.get_q_summary().values())[:3],
            "intent_summary": self.task_intent.get_intent_summary(),
            "prediction_confidence": round(
                self.state_predictor.get_prediction_confidence(), 2),
            "recovery_status": self.recovery.get_status(),
            "decision_info": self.behavior_selector.get_decision_info(),
        }
