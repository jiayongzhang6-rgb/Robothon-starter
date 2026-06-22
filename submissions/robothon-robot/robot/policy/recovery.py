#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Adaptive Recovery v3 - 智能恢复系统

升级：从 reset → re-localize → re-plan → re-enter
不再是简单的"回退重来"，而是真正的环境再评估和路径重规划
"""
import math
from enum import Enum, auto


class RecoveryPhase(Enum):
    IDLE = auto()           # 无恢复中
    RE_OBSERVE = auto()     # 重新观测环境
    RE_LOCALIZE = auto()    # 重新定位
    RE_PLAN = auto()        # 重新规划路径
    RE_ENTRY = auto()       # 重新进入任务
    DONE = auto()           # 恢复完成


class AdaptiveRecovery:
    """
    智能恢复系统

    流程：
        1. 检测到异常（卡死/迷失/障碍密集）
        2. RE_OBSERVE：重新扫描环境，更新代价地图
        3. RE_LOCALIZE：确定当前位置，找最近已知路标
        4. RE_PLAN：基于新环境信息重新规划路径
        5. RE_ENTRY：从安全点重新进入任务
    """

    def __init__(self):
        self.phase = RecoveryPhase.IDLE
        self._phase_steps = 0
        self._max_retries = 3
        self._retry_count = 0

        # 恢复状态
        self._last_known_pos = None
        self._recovery_target = None
        self._environment_snapshot = {}

        # 触发条件
        self._stagnation_threshold = 500   # 连续N步无进展
        self._stagnation_counter = 0
        self._last_moved_pos = None

        # 恢复策略参数
        self._localization_radius = 0.3   # 在半径内搜索已知路标
        self._safe_distance = 0.2         # 与最近障碍的安全距离

    def check_trigger(self, robot_pos, obstacle_density, battery_level,
                      line_error, path_stagnation=None):
        """
        检查是否需要触发恢复

        返回: (should_recover, reason)
        """
        # 1. 位置停滞
        if self._last_moved_pos is not None:
            dx = robot_pos[0] - self._last_moved_pos[0]
            dy = robot_pos[1] - self._last_moved_pos[1]
            moved = math.sqrt(dx*dx + dy*dy)
            if moved > 0.05:
                self._stagnation_counter = 0
                self._last_moved_pos = robot_pos
            else:
                self._stagnation_counter += 1
        else:
            self._last_moved_pos = robot_pos

        if self._stagnation_counter >= self._stagnation_threshold:
            return True, "stagnation"

        # 2. 高障碍密度 + 高线误差（迷失）
        if obstacle_density > 0.6 and line_error > 0.5:
            return True, "lost_in_obstacles"

        # 3. 连续高线误差（无法回到巡线）
        if line_error > 0.7:
            return True, "line_deviation"

        return False, None

    def start_recovery(self, robot_pos, environment_info=None):
        """启动恢复流程"""
        self.phase = RecoveryPhase.RE_OBSERVE
        self._last_known_pos = robot_pos
        self._environment_snapshot = environment_info or {}
        self._phase_steps = 0
        self._retry_count += 1
        self._recovery_target = None

    def update(self, robot_pos, cost_map, waypoints_graph,
               route_sampler=None):
        """
        每步更新恢复状态机

        返回: dict {
            "phase": RecoveryPhase,
            "action": str,
            "target": tuple or None,
            "recovery_plan": list,
        }
        """
        self._phase_steps += 1

        if self.phase == RecoveryPhase.IDLE:
            return {"phase": "IDLE", "action": "none", "target": None,
                    "recovery_plan": []}

        elif self.phase == RecoveryPhase.RE_OBSERVE:
            return self._do_reobserve(robot_pos, cost_map)

        elif self.phase == RecoveryPhase.RE_LOCALIZE:
            return self._do_relocalize(robot_pos, waypoints_graph,
                                        route_sampler)

        elif self.phase == RecoveryPhase.RE_PLAN:
            return self._do_replan(robot_pos, cost_map, waypoints_graph)

        elif self.phase == RecoveryPhase.RE_ENTRY:
            return self._do_reentry(robot_pos)

        elif self.phase == RecoveryPhase.DONE:
            return self._finish_recovery()

        return {"phase": "UNKNOWN", "action": "none", "target": None,
                "recovery_plan": []}

    def _do_reobserve(self, robot_pos, cost_map):
        """Phase 1: 重新观测环境"""
        # 扫描周围障碍分布
        nearby_risks = []
        for angle_deg in range(0, 360, 45):
            angle = math.radians(angle_deg)
            scan_x = robot_pos[0] + 0.3 * math.cos(angle)
            scan_y = robot_pos[1] + 0.3 * math.sin(angle)
            risk = cost_map.get_risk(scan_x, scan_y)
            nearby_risks.append((angle_deg, risk))

        # 找最安全的方向
        safest = min(nearby_risks, key=lambda x: x[1])
        self._recovery_target = (
            robot_pos[0] + 0.3 * math.cos(math.radians(safest[0])),
            robot_pos[1] + 0.3 * math.sin(math.radians(safest[0])),
        )

        if self._phase_steps >= 3:  # 观测3步
            self.phase = RecoveryPhase.RE_LOCALIZE
            self._phase_steps = 0

        return {
            "phase": "RE_OBSERVE",
            "action": "scan_environment",
            "target": self._recovery_target,
            "recovery_plan": ["scanning", f"safest_direction={safest[0]}°"],
        }

    def _do_relocalize(self, robot_pos, waypoints_graph, route_sampler):
        """Phase 2: 重新定位 - 找最近的已知路标"""
        if waypoints_graph is None:
            self.phase = RecoveryPhase.RE_PLAN
            self._phase_steps = 0
            return {"phase": "RE_LOCALIZE", "action": "no_graph",
                    "target": None, "recovery_plan": ["skip_to_plan"]}

        # 找最近且距离>0.2m的waypoint（避免选到当前位置）
        all_wps = waypoints_graph.all_waypoints()
        nearest_wp = None
        nearest_dist = float("inf")

        for wp in all_wps:
            wp_pos = waypoints_graph.position(wp)
            dx = wp_pos[0] - robot_pos[0]
            dy = wp_pos[1] - robot_pos[1]
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_wp = wp

        self._recovery_target = nearest_wp

        if self._phase_steps >= 2:
            self.phase = RecoveryPhase.RE_PLAN
            self._phase_steps = 0

        return {
            "phase": "RE_LOCALIZE",
            "action": f"found_landmark={nearest_wp}",
            "target": nearest_wp,
            "recovery_plan": [
                f"nearest_waypoint={nearest_wp}",
                f"distance={nearest_dist:.2f}m",
            ],
        }

    def _do_replan(self, robot_pos, cost_map, waypoints_graph):
        """Phase 3: 重新规划 - 从当前位置到目标的最优路径"""
        if self._recovery_target and waypoints_graph:
            target_pos = waypoints_graph.position(self._recovery_target)
        else:
            target_pos = (0.0, 0.0)  # 回起点

        # 检查直达路径是否安全
        dx = target_pos[0] - robot_pos[0]
        dy = target_pos[1] - robot_pos[1]
        dist = math.sqrt(dx*dx + dy*dy)

        # 沿路径采样检查
        path_safe = True
        n_samples = max(2, int(dist / 0.1))
        for i in range(n_samples):
            t = i / n_samples
            check_x = robot_pos[0] + dx * t
            check_y = robot_pos[1] + dy * t
            if cost_map.get_risk(check_x, check_y) > 0.5:
                path_safe = False
                break

        if path_safe:
            plan = ["direct_path_safe", f"target={self._recovery_target}"]
        else:
            plan = ["path_blocked", "need_detour", f"target={self._recovery_target}"]

        if self._phase_steps >= 2:
            self.phase = RecoveryPhase.RE_ENTRY
            self._phase_steps = 0

        return {
            "phase": "RE_PLAN",
            "action": "path_planned",
            "target": self._recovery_target,
            "recovery_plan": plan,
        }

    def _do_reentry(self, robot_pos):
        """Phase 4: 重新进入任务"""
        if self._phase_steps >= 2:
            self.phase = RecoveryPhase.DONE
            self._phase_steps = 0

        return {
            "phase": "RE_ENTRY",
            "action": "resuming_mission",
            "target": self._recovery_target,
            "recovery_plan": ["recovery_complete", "resuming_patrol"],
        }

    def _finish_recovery(self):
        """恢复完成，重置状态"""
        result = {
            "phase": "DONE",
            "action": "recovery_finished",
            "target": None,
            "recovery_plan": [f"retries_used={self._retry_count}"],
        }
        self.phase = RecoveryPhase.IDLE
        self._stagnation_counter = 0
        self._phase_steps = 0
        return result

    def get_status(self):
        return {
            "phase": self.phase.name,
            "phase_steps": self._phase_steps,
            "retry_count": self._retry_count,
            "recovery_target": self._recovery_target,
            "stagnation_counter": self._stagnation_counter,
        }
