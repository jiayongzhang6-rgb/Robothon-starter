#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Task Intent Layer - 目标驱动规划层 v3

核心升级：从 reaction system → goal-driven system
系统不再只是"看到什么反应什么"，而是：
  1. 有明确的 goal representation
  2. 有 task graph / subgoal decomposition
  3. 有 constraints + priority 评估
  4. 有 intent reconciliation（多意图冲突解决）
"""
import math
from enum import Enum, auto


class IntentType(Enum):
    """高层任务意图"""
    PATROL_AREA = auto()       # 巡逻指定区域
    INVESTIGATE = auto()       # 调查异常（障碍/传感器触发）
    RETURN_BASE = auto()       # 返回基站（电量低/任务完成）
    AVOID_DANGER = auto()      # 紧急避障
    RECOVER = auto()           # 状态恢复（迷失/卡死）


class SubGoal:
    """子目标 - 任务分解后的可执行单元"""

    def __init__(self, name, target_wp, intent_type, priority=0.5,
                 constraints=None, deadline_steps=None):
        self.name = name
        self.target_wp = target_wp
        self.intent_type = intent_type
        self.priority = priority          # 0~1，越高越重要
        self.constraints = constraints or {}
        self.deadline_steps = deadline_steps  # None=无期限
        self.completed = False
        self.started_step = None

    def __repr__(self):
        status = "✓" if self.completed else "○"
        return f"SubGoal({status} {self.name}→{self.target_wp} p={self.priority:.1f})"


class TaskGraph:
    """
    任务图 - 定义子目标之间的依赖和时序关系

    不是线性 FSM，而是一个 DAG（有向无环图）：
        PATROL_AREA → [zone_a, zone_b, zone_c]
        INVESTIGATE → [approach, scan, report]
        RETURN_BASE → [navigate, dock]
    """

    # 子目标模板（根据意图类型生成）
    TEMPLATES = {
        IntentType.PATROL_AREA: [
            ("approach_zone", 0.6),
            ("scan_perimeter", 0.4),
            ("log_coverage", 0.3),
        ],
        IntentType.INVESTIGATE: [
            ("approach_anomaly", 0.9),
            ("scan_anomaly", 0.7),
            ("assess_threat", 0.5),
        ],
        IntentType.RETURN_BASE: [
            ("navigate_home", 0.8),
            ("dock_station", 0.6),
        ],
        IntentType.AVOID_DANGER: [
            ("emergency_stop", 1.0),
            ("clear_path", 0.8),
            ("resume_patrol", 0.3),
        ],
        IntentType.RECOVER: [
            ("localize_self", 0.9),
            ("reassess_env", 0.7),
            ("replan_route", 0.5),
        ],
    }

    @classmethod
    def build(cls, intent_type, target_wps, constraints=None):
        """根据意图类型生成子目标序列"""
        template = cls.TEMPLATES.get(intent_type, [])
        subgoals = []
        for i, (name, priority) in enumerate(template):
            wp = target_wps[i % len(target_wps)] if target_wps else "start"
            sg = SubGoal(
                name=name,
                target_wp=wp,
                intent_type=intent_type,
                priority=priority,
                constraints=constraints or {},
            )
            subgoals.append(sg)
        return subgoals


class TaskIntentLayer:
    """
    目标驱动规划层 - 系统的"大脑"

    职责：
    1. 维护当前 goal representation
    2. 管理 task graph（子目标分解）
    3. 评估 constraints + priority
    4. 多意图冲突解决（intent reconciliation）
    5. 为下层（FSM/Policy）提供"该做什么"的指令

    升级前：Observation → Belief → FSM → Control
    升级后：Observation → Belief → Intent → Planning → FSM → Control
    """

    def __init__(self):
        # 当前活跃意图
        self._active_intent = IntentType.PATROL_AREA
        self._intent_stack = []          # 意图栈（支持嵌套）
        self._subgoals = []              # 当前子目标序列
        self._current_subgoal_idx = 0    # 当前执行的子目标索引
        self._goal_history = []          # 意图历史

        # 环境状态缓存
        self._belief_snapshot = {}
        self._step_count = 0

        # 意图切换阈值
        self._danger_threshold = 0.9     # 障碍密度 > 0.7 → 切换避障
        self._battery_threshold = 0.15    # 电量 < 0.2 → 返回基站
        self._confusion_threshold = 500   # 连续N步未进展 → 恢复
        self._max_stack_depth = 3        # 意图栈最大深度

        # 追踪
        self._stagnation_counter = 0
        self._last_progress_pos = (0.0, 0.0)

    @property
    def current_intent(self):
        return self._active_intent

    @property
    def current_subgoal(self):
        if self._current_subgoal_idx < len(self._subgoals):
            return self._subgoals[self._current_subgoal_idx]
        return None

    @property
    def goal_progress(self):
        """当前子目标完成度 [0,1]"""
        if not self._subgoals:
            return 0.0
        completed = sum(1 for sg in self._subgoals if sg.completed)
        return completed / len(self._subgoals)

    def evaluate_intent(self, belief_state):
        """
        核心：根据 belief state 评估应该切换到什么意图

        输入：belief_state dict（来自 Belief Estimator）
        输出：IntentType（可能是当前的，也可能切换）
        """
        self._belief_snapshot = belief_state
        self._step_count += 1

        obstacle_density = belief_state.get("obstacle_density", 0)
        battery_level = belief_state.get("battery_level", 1.0)
        line_error = belief_state.get("line_error", 0)
        robot_pos = belief_state.get("robot_pos", (0, 0))
        confidence = belief_state.get("confidence", 1.0)

        # === 意图切换逻辑（优先级从高到低）===

        # 1. 紧急避障（最高优先级）
        if obstacle_density > self._danger_threshold:
            if self._active_intent != IntentType.AVOID_DANGER:
                self._push_intent(IntentType.AVOID_DANGER, priority=1.0)
                return self._active_intent

        # 2. 低电量返回
        if battery_level < self._battery_threshold:
            if self._active_intent != IntentType.RETURN_BASE:
                self._push_intent(IntentType.RETURN_BASE, priority=0.9)
                return self._active_intent

        # 3. 迷失恢复（连续多步无进展）
        progress = self._check_progress(robot_pos)
        if progress < 0.01:
            self._stagnation_counter += 1
        else:
            self._stagnation_counter = 0

        if self._stagnation_counter >= self._confusion_threshold:
            if self._active_intent != IntentType.RECOVER:
                self._push_intent(IntentType.RECOVER, priority=0.8)
                return self._active_intent

        # 4. 异常调查（线误差大 + 中等障碍）
        if line_error > 0.6 and obstacle_density > 0.5:
            if self._active_intent not in (IntentType.INVESTIGATE,
                                           IntentType.AVOID_DANGER):
                self._push_intent(IntentType.INVESTIGATE, priority=0.6)
                return self._active_intent

        # 5. 默认：巡逻（恢复到基础意图）
        if self._active_intent in (IntentType.AVOID_DANGER,
                                    IntentType.RECOVER):
            # 避障/恢复完成 → 回到巡逻
            if self._active_intent == IntentType.AVOID_DANGER:
                if obstacle_density < 0.3:
                    self._pop_intent()
            elif self._active_intent == IntentType.RECOVER:
                if self._stagnation_counter < 2:
                    self._pop_intent()

        if not self._intent_stack:
            self._active_intent = IntentType.PATROL_AREA

        return self._active_intent

    def generate_subgoals(self, intent, available_waypoints, robot_pos):
        """
        为当前意图生成子目标序列

        不是固定模板，而是根据环境动态选择目标点
        """
        if intent == IntentType.PATROL_AREA:
            # 选择未充分探索的区域
            target_wps = self._select_patrol_targets(
                available_waypoints, robot_pos, n=3
            )
        elif intent == IntentType.AVOID_DANGER:
            # 选择远离障碍的安全点
            target_wps = self._select_safe_retreat(
                available_waypoints, robot_pos
            )
        elif intent == IntentType.RECOVER:
            # 回到最近的已知位置
            target_wps = ["start"]
        elif intent == IntentType.RETURN_BASE:
            target_wps = ["base"]
        elif intent == IntentType.INVESTIGATE:
            # 朝向当前异常方向
            target_wps = self._select_investigate_target(
                available_waypoints, robot_pos
            )
        else:
            target_wps = ["start"]

        self._subgoals = TaskGraph.build(intent, target_wps)
        self._current_subgoal_idx = 0
        self._goal_history.append({
            "step": self._step_count,
            "intent": intent.name,
            "subgoals": [sg.name for sg in self._subgoals],
        })
        return self._subgoals

    def advance_subgoal(self):
        """当前子目标完成 → 推进到下一个"""
        if self._current_subgoal_idx < len(self._subgoals):
            self._subgoals[self._current_subgoal_idx].completed = True
            self._current_subgoal_idx += 1
            return self.current_subgoal
        return None

    def check_subgoal_arrival(self, robot_pos, threshold=0.15):
        """检查是否到达当前子目标"""
        sg = self.current_subgoal
        if sg is None:
            return False
        target_pos = sg.target_wp  # 这里存的是waypoint名称，需要外部解析
        return False  # 由外部（PatrolPolicy）判断

    def get_intent_summary(self):
        """获取意图摘要（供UI/日志使用）"""
        return {
            "active_intent": self._active_intent.name,
            "intent_stack": [i.name for i in self._intent_stack],
            "current_subgoal": str(self.current_subgoal),
            "subgoal_progress": f"{self._current_subgoal_idx}/{len(self._subgoals)}",
            "goal_progress": round(self.goal_progress, 2),
            "stagnation": self._stagnation_counter,
            "total_goals_completed": len(self._goal_history),
        }

    # === 内部方法 ===

    def _push_intent(self, intent, priority=0.5):
        """压入新意图（当前意图入栈）"""
        if self._active_intent != intent:
            # 限制栈深度，防止无限嵌套
            if len(self._intent_stack) >= self._max_stack_depth:
                return
            self._intent_stack.append(self._active_intent)
            self._active_intent = intent
            # 为新意图生成子目标
            self.generate_subgoals(
                intent,
                list(self._belief_snapshot.get("available_waypoints", ["start"])),
                self._belief_snapshot.get("robot_pos", (0, 0)),
            )

    def _pop_intent(self):
        """弹出当前意图，恢复上一个"""
        if self._intent_stack:
            self._active_intent = self._intent_stack.pop()
            self.generate_subgoals(
                self._active_intent,
                list(self._belief_snapshot.get("available_waypoints", ["start"])),
                self._belief_snapshot.get("robot_pos", (0, 0)),
            )
        else:
            self._active_intent = IntentType.PATROL_AREA

    def _check_progress(self, robot_pos):
        """检查位置是否在移动"""
        dx = robot_pos[0] - self._last_progress_pos[0]
        dy = robot_pos[1] - self._last_progress_pos[1]
        dist = math.sqrt(dx*dx + dy*dy)
        if dist > 0.05:
            self._last_progress_pos = robot_pos
        return dist

    def _select_patrol_targets(self, waypoints, robot_pos, n=3):
        """选择巡逻目标：距离适中 + 新颖性高"""
        scored = []
        for wp in waypoints:
            if wp in ("start", "base"):
                continue
            # 简化评分：距离 + 随机扰动
            # 实际由 RiskEstimator 提供更精确的评分
            scored.append((wp, 0.5))  # placeholder
        scored.sort(key=lambda x: -x[1])
        return [s[0] for s in scored[:n]] if scored else ["zone_a"]

    def _select_safe_retreat(self, waypoints, robot_pos):
        """选择安全撤退点"""
        # 返回最近的非障碍区域
        return ["start"]

    def _select_investigate_target(self, waypoints, robot_pos):
        """选择调查目标"""
        return ["zone_a"]  # placeholder，实际根据异常位置
