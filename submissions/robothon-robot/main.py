#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FFAI Robothon 2026 - Adaptive Patrol Agent
分层策略系统主程序 v3

架构（v3 - 95+ 目标）:
    Observation → Belief → TaskIntent → Planning → ProbabilisticFSM → Control
                                  ↑                        ↓
                          Prediction ←──→ Recovery ←────────┘

新增层：
    1. TaskIntent Layer: goal representation + task graph + intent reconciliation
    2. Probabilistic FSM: softmax(Q/temperature) + uncertainty-driven
    3. Prediction Layer: future state prediction + early recovery
    4. Adaptive Recovery: re-localize + re-plan + re-enter
"""

import math
import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot.policy.patrol_policy import PatrolPolicy, Behavior


class PatrolSimulator:
    """仿真器 - 验证完整AI栈"""

    def __init__(self, duration=30.0, dt=0.05):
        self.duration = duration
        self.dt = dt
        self.steps = int(duration / dt)

        # 场地障碍物
        self.obstacles = [
            (0.3, 0.2), (0.7, -0.1), (1.0, 0.3),
        ]

        # 机器人
        self.pos = [0.0, 0.0]
        self.heading = 0.0

        # 策略
        self.policy = PatrolPolicy(seed=42)

        # 控制参数
        self.base_speed = 0.3
        self.turn_rate = 2.0

        # 日志
        self.log = []
        self.behavior_ticks = {}
        self.intent_ticks = {}

    def _navigate_to(self, target):
        dx = target[0] - self.pos[0]
        dy = target[1] - self.pos[1]
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < 0.01:
            return 0.0, 0.0

        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.heading
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        if abs(angle_diff) > 0.3:
            angular = self.turn_rate * math.copysign(1, angle_diff)
            linear = 0.05
        elif abs(angle_diff) > 0.1:
            angular = self.turn_rate * angle_diff * 0.8
            linear = self.base_speed * 0.6
        else:
            angular = self.turn_rate * angle_diff * 0.5
            linear = self.base_speed

        if dist > 0.3:
            linear *= 1.2
        elif dist < 0.15:
            linear *= 0.4

        return linear, angular

    def _step_physics(self, linear, angular):
        self.heading += angular * self.dt
        self.pos[0] += linear * math.cos(self.heading) * self.dt
        self.pos[1] += linear * math.sin(self.heading) * self.dt
        self.pos[0] = max(-0.05, min(1.5, self.pos[0]))
        self.pos[1] = max(-0.45, min(0.45, self.pos[1]))

    def _obstacle_density(self):
        count = 0
        for obs in self.obstacles:
            d = math.sqrt((self.pos[0]-obs[0])**2 + (self.pos[1]-obs[1])**2)
            if d < 0.25:
                count += 1
        return min(1.0, count / 2.0)

    def _line_error(self):
        if self.policy._target_wp:
            tp = self.policy.route_sampler.graph.position(self.policy._target_wp)
            dx = tp[0] - self.pos[0]
            dy = tp[1] - self.pos[1]
            dist = math.sqrt(dx*dx + dy*dy)
            if dist > 0.01:
                target_angle = math.atan2(dy, dx)
                angle_diff = abs(target_angle - self.heading)
                while angle_diff > math.pi:
                    angle_diff = 2*math.pi - angle_diff
                return angle_diff / math.pi
        return 0.0

    def run(self):
        print("=" * 70)
        print("  Adaptive Patrol Agent - 分层策略系统 v3")
        print("  Architecture: Intent + Prediction + Probabilistic FSM + Recovery")
        print("=" * 70)
        print(f"  Duration: {self.duration}s | Steps: {self.steps} | dt: {self.dt}s")
        print()

        self.policy.reset()
        t0 = time.time()

        for step in range(self.steps):
            t = step * self.dt

            # 1. 观测
            obs_density = self._obstacle_density()
            context = self.policy.observe(
                robot_pos=(self.pos[0], self.pos[1]),
                heading=(math.cos(self.heading), math.sin(self.heading)),
                line_error=self._line_error(),
                obstacle_density=obs_density,
                battery_level=max(0, 1.0 - t / self.duration),
                has_task=False,
            )

            # 2. 决策（含 Intent + Prediction + Probabilistic FSM）
            decision = self.policy.decide(context)
            behavior = decision["behavior"]
            intent = decision["info"].get("intent", "?")

            self.behavior_ticks[behavior.value] = \
                self.behavior_ticks.get(behavior.value, 0) + 1
            self.intent_ticks[intent] = self.intent_ticks.get(intent, 0) + 1

            # 3. 导航控制
            target = decision["target"]
            linear, angular = self._navigate_to(target)
            linear *= decision["speed_factor"]
            angular *= decision["steering_gain"]

            # 4. 物理步进
            self._step_physics(linear, angular)

            # 5. 日志
            if step % int(2.0 / self.dt) == 0:
                entry = {
                    "t": round(t, 1),
                    "pos": (round(self.pos[0], 3), round(self.pos[1], 3)),
                    "behavior": behavior.value,
                    "intent": intent,
                    "target": decision["info"].get("target_wp", "?"),
                    "subgoal": decision["info"].get("subgoal", "?"),
                    "prediction": decision["info"].get(
                        "prediction_confidence", "?"),
                    "risk_trend": decision["info"].get("risk_trend", "?"),
                }
                self.log.append(entry)
                print(f"  t={t:5.1f}s | "
                      f"intent={intent:15s} | "
                      f"behavior={behavior.value:20s} | "
                      f"target={str(entry['target']):15s} | "
                      f"pred_conf={entry['prediction']} | "
                      f"risk={entry['risk_trend']}")

        elapsed = time.time() - t0
        print(f"\n  Simulation complete! Elapsed: {elapsed:.2f}s")

    def print_summary(self):
        s = self.policy.get_state_summary()
        total = sum(self.behavior_ticks.values())
        print("\n" + "=" * 70)
        print("  Strategy Summary (v3)")
        print("=" * 70)
        print(f"  Total steps: {s['step']}")
        print(f"  Final position: ({self.pos[0]:.3f}, {self.pos[1]:.3f})")
        print(f"  Path length: {len(self.policy.get_path_history())} steps")
        print(f"  Prediction confidence: {s['prediction_confidence']}")
        print(f"  Temperature: {s['decision_info']['temperature']}")

        print(f"\n  Behavior Distribution:")
        for beh, cnt in sorted(self.behavior_ticks.items(),
                                key=lambda x: -x[1]):
            pct = cnt / total * 100
            bar = "█" * int(pct / 2)
            print(f"    {beh:20s}: {cnt:4d} ({pct:5.1f}%) {bar}")

        print(f"\n  Intent Distribution:")
        intent_total = sum(self.intent_ticks.values())
        for intent, cnt in sorted(self.intent_ticks.items(),
                                   key=lambda x: -x[1]):
            pct = cnt / intent_total * 100
            bar = "█" * int(pct / 2)
            print(f"    {intent:20s}: {cnt:4d} ({pct:5.1f}%) {bar}")

        print(f"\n  Visit counts: {s['visit_counts']}")
        print(f"\n  Intent summary: {s['intent_summary']}")
        print(f"  Recovery status: {s['recovery_status']}")
        print()


def main():
    sim = PatrolSimulator(duration=30.0, dt=0.05)
    sim.run()
    sim.print_summary()


if __name__ == "__main__":
    main()
