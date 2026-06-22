#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FFAI Robothon 2026 - Adaptive Patrol Agent
分层策略系统主程序

架构:
    PatrolPolicy (AI决策层)
      → RouteSampler (动态路径)
      → RiskEstimator (风险评估)
      → BehaviorSelector (行为选择)
    Low-Level Controller (导航 + PID)
"""

import math
import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot.policy.patrol_policy import PatrolPolicy, Behavior


# ============ 仿真环境 ============

class PatrolSimulator:
    """仿真器 - 验证分层策略逻辑"""

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
        self.heading = 0.0  # 弧度

        # 策略
        self.policy = PatrolPolicy(seed=42)

        # 控制参数
        self.base_speed = 0.3     # m/s
        self.turn_rate = 2.0      # rad/s
        self.arrival_threshold = 0.12  # m

        # 日志
        self.log = []
        self.behavior_ticks = {}

    def _navigate_to(self, target):
        """
        导航控制 - 让机器人转向并前进到目标
        返回 (linear_vel, angular_vel)
        """
        dx = target[0] - self.pos[0]
        dy = target[1] - self.pos[1]
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < 0.01:
            return 0.0, 0.0

        # 目标方向角
        target_angle = math.atan2(dy, dx)

        # 角度差（-π 到 π）
        angle_diff = target_angle - self.heading
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # 控制律：先转向，再前进
        if abs(angle_diff) > 0.3:
            # 大角度：原地转向
            angular = self.turn_rate * math.copysign(1, angle_diff)
            linear = 0.05
        elif abs(angle_diff) > 0.1:
            # 中角度：边转边走
            angular = self.turn_rate * angle_diff * 0.8
            linear = self.base_speed * 0.6
        else:
            # 小角度：直走
            angular = self.turn_rate * angle_diff * 0.5
            linear = self.base_speed

        # 距离远时加速，接近时减速
        if dist > 0.3:
            linear *= 1.2
        elif dist < 0.15:
            linear *= 0.4

        return linear, angular

    def _step_physics(self, linear, angular):
        """差速驱动运动学"""
        self.heading += angular * self.dt
        self.pos[0] += linear * math.cos(self.heading) * self.dt
        self.pos[1] += linear * math.sin(self.heading) * self.dt

        # 边界
        self.pos[0] = max(-0.05, min(1.5, self.pos[0]))
        self.pos[1] = max(-0.45, min(0.45, self.pos[1]))

    def _obstacle_density(self):
        """当前区域障碍密度"""
        count = 0
        for obs in self.obstacles:
            d = math.sqrt((self.pos[0]-obs[0])**2 + (self.pos[1]-obs[1])**2)
            if d < 0.25:
                count += 1
        return min(1.0, count / 2.0)

    def _line_error(self):
        """模拟巡线误差（偏离目标方向的程度）"""
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
                return angle_diff / math.pi  # 归一化到 [0,1]
        return 0.0

    def run(self):
        print("=" * 60)
        print("  Adaptive Patrol Agent - 分层策略仿真 v2")
        print("  架构: Hierarchical Policy (AI决策层)")
        print("=" * 60)
        print(f"  时长: {self.duration}s | 步数: {self.steps} | dt: {self.dt}s")
        print()

        self.policy.reset()
        t0 = time.time()

        for step in range(self.steps):
            t = step * self.dt

            # 1. 观测
            context = self.policy.observe(
                robot_pos=(self.pos[0], self.pos[1]),
                heading=(math.cos(self.heading), math.sin(self.heading)),
                line_error=self._line_error(),
                obstacle_density=self._obstacle_density(),
                battery_level=max(0, 1.0 - t / self.duration),
                has_task=False,
            )

            # 2. 决策
            decision = self.policy.decide(context)
            behavior = decision["behavior"]

            # 统计行为分布
            self.behavior_ticks[behavior.value] = \
                self.behavior_ticks.get(behavior.value, 0) + 1

            # 3. 导航控制
            target = decision["target"]
            linear, angular = self._navigate_to(target)

            # 应用策略的速度/转向增益
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
                    "target": decision["info"]["target_wp"],
                    "unvisited": decision["info"]["unvisited"],
                }
                self.log.append(entry)
                print(f"  t={t:5.1f}s | "
                      f"pos={entry['pos']} | "
                      f"behavior={behavior.value:20s} | "
                      f"target={entry['target']:15s} | "
                      f"unvisited={entry['unvisited']}")

        elapsed = time.time() - t0
        print(f"\n  仿真完成! 耗时: {elapsed:.2f}s")

    def print_summary(self):
        s = self.policy.get_state_summary()
        total = sum(self.behavior_ticks.values())
        print("\n" + "=" * 60)
        print("  策略摘要")
        print("=" * 60)
        print(f"  总步数: {s['step']}")
        print(f"  最终位置: ({self.pos[0]:.3f}, {self.pos[1]:.3f})")
        print(f"  路径长度: {len(self.policy.get_path_history())} 步")
        print(f"\n  行为分布:")
        for beh, cnt in sorted(self.behavior_ticks.items(),
                                key=lambda x: -x[1]):
            pct = cnt / total * 100
            bar = "█" * int(pct / 2)
            print(f"    {beh:20s}: {cnt:4d} ({pct:5.1f}%) {bar}")
        print(f"\n  访问统计: {s['visit_counts']}")
        print()


def main():
    sim = PatrolSimulator(duration=30.0, dt=0.05)
    sim.run()
    sim.print_summary()


if __name__ == "__main__":
    main()
