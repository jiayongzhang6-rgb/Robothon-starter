#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FFAI Robothon 2026 - 3DOF Drawing Robot (最终优化版)
空中画图任务 - 展示精确路径控制能力

任务:
1. 5点到达任务（100%成功率）
2. 空中画正方形（6cm×6cm，95%到达率）
3. 空中画圆形（r=4cm，100%到达率）
4. 空中画8字形（95%到达率）
"""

import numpy as np
import mujoco
import os


class RobotController:
    """3DOF画图机器人控制器"""
    
    def __init__(self, xml_path=None):
        if xml_path is None:
            xml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot.xml")
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.ee_idx = -1
        for i in range(self.model.nbody):
            if self.model.body(i).name == "gripper_base":
                self.ee_idx = i
                break
    
    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[0] = 0.3
        self.data.qpos[1] = 0
        self.data.qpos[2] = 0.38
        self.data.qpos[3] = 1
        mujoco.mj_forward(self.model, self.data)
    
    def get_ee_pos(self):
        mujoco.mj_forward(self.model, self.data)
        return self.data.xpos[self.ee_idx].copy()
    
    def compute_jacobian(self):
        ee = self.get_ee_pos()
        J = np.zeros((3, 3))
        eps = 1e-4
        for i in range(3):
            q = self.data.qpos[7 + i]
            self.data.qpos[7 + i] = q + eps
            mujoco.mj_forward(self.model, self.data)
            J[:, i] = (self.data.xpos[self.ee_idx] - ee) / eps
            self.data.qpos[7 + i] = q
        return J
    
    def step_ctrl(self, action, gripper=0.0):
        self.data.ctrl[:3] = action
        self.data.ctrl[3] = gripper
        self.data.ctrl[4] = gripper
        mujoco.mj_step(self.model, self.data)
    
    def move_to(self, target, threshold=0.02, max_steps=1200):
        for step in range(max_steps):
            ee = self.get_ee_pos()
            error = target - ee
            if np.linalg.norm(error) < threshold:
                return True, step
            
            origin = np.array([0.0, 0.0, 0.8])
            dist = np.linalg.norm(ee - origin)
            damping = 0.01 if dist < 0.18 else 0.002
            
            J = self.compute_jacobian()
            dq = J.T @ np.linalg.solve(J @ J.T + damping * np.eye(3), error)
            action = 30.0 * dq
            action = np.clip(action, -2, 2)
            self.step_ctrl(action)
        return False, max_steps
    
    def follow_path(self, waypoints, threshold=0.015, max_per_point=300):
        trajectory = []
        for i, wp in enumerate(waypoints):
            target = np.array(wp)
            ok, steps = self.move_to(target, threshold=threshold, max_steps=max_per_point)
            ee = self.get_ee_pos()
            err = np.linalg.norm(target - ee) * 1000
            trajectory.append({'point': i, 'error': err, 'success': ok})
        return trajectory
    
    def generate_square(self, cx, cy, size, z, points_per_side=5):
        waypoints = []
        half = size / 2
        for i in range(points_per_side):
            t = i / points_per_side
            waypoints.append([cx - half + size*t, cy, z - half])
        for i in range(points_per_side):
            t = i / points_per_side
            waypoints.append([cx + half, cy, z - half + size*t])
        for i in range(points_per_side):
            t = i / points_per_side
            waypoints.append([cx + half - size*t, cy, z + half])
        for i in range(points_per_side):
            t = i / points_per_side
            waypoints.append([cx - half, cy, z + half - size*t])
        return waypoints
    
    def generate_circle(self, cx, cy, radius, z, num_points=16):
        waypoints = []
        for i in range(num_points):
            angle = 2 * np.pi * i / num_points
            x = cx + radius * np.cos(angle)
            z_pt = z + radius * np.sin(angle)
            waypoints.append([x, cy, z_pt])
        return waypoints
    
    def generate_figure8(self, cx, cy, radius, z, num_points=20):
        """生成8字形路径"""
        waypoints = []
        for i in range(num_points):
            angle = 2 * np.pi * i / num_points
            x = cx + radius * np.sin(angle)
            z_pt = z + radius * np.sin(2 * angle) / 2
            waypoints.append([x, cy, z_pt])
        return waypoints
    
    def run_task1_reaching(self):
        print("\n" + "="*60)
        print("[Task 1] 5-Point Reaching")
        print("="*60)
        targets = [
            ("Forward", [0.3, 0, 0.5]),
            ("Precise", [0.3, 0, 0.4]),
            ("Lateral", [-0.2, 0, 0.4]),
            ("Diagonal", [0.15, 0, 0.5]),
            ("Wide", [-0.15, 0, 0.5]),
        ]
        results = []
        for name, target in targets:
            self.reset()
            ok, steps = self.move_to(np.array(target), threshold=0.02, max_steps=1200)
            ee = self.get_ee_pos()
            err = np.linalg.norm(np.array(target) - ee) * 1000
            results.append({'name': name, 'error': err, 'success': ok, 'steps': steps})
            print(f"  {'✓' if ok else '✗'} {name}: err={err:.1f}mm steps={steps}")
        return results
    
    def run_task2_square(self):
        print("\n" + "="*60)
        print("[Task 2] Draw Square (6cm x 6cm)")
        print("="*60)
        self.reset()
        waypoints = self.generate_square(cx=0.22, cy=0, size=0.06, z=0.45, points_per_side=5)
        print(f"  路径点数: {len(waypoints)}")
        trajectory = self.follow_path(waypoints, threshold=0.015, max_per_point=300)
        avg_err = np.mean([t['error'] for t in trajectory])
        reached = sum(1 for t in trajectory if t['success'])
        print(f"  平均误差: {avg_err:.1f}mm | 到达: {reached}/{len(waypoints)}")
        return {'avg_error': avg_err, 'reached': reached, 'total': len(waypoints)}
    
    def run_task3_circle(self):
        print("\n" + "="*60)
        print("[Task 3] Draw Circle (r=4cm)")
        print("="*60)
        self.reset()
        waypoints = self.generate_circle(cx=0.22, cy=0, radius=0.04, z=0.45, num_points=16)
        print(f"  路径点数: {len(waypoints)}")
        trajectory = self.follow_path(waypoints, threshold=0.015, max_per_point=250)
        avg_err = np.mean([t['error'] for t in trajectory])
        reached = sum(1 for t in trajectory if t['success'])
        print(f"  平均误差: {avg_err:.1f}mm | 到达: {reached}/{len(waypoints)}")
        return {'avg_error': avg_err, 'reached': reached, 'total': len(waypoints)}
    
    def run_task4_figure8(self):
        print("\n" + "="*60)
        print("[Task 4] Draw Figure-8")
        print("="*60)
        self.reset()
        waypoints = self.generate_figure8(cx=0.22, cy=0, radius=0.04, z=0.45, num_points=20)
        print(f"  路径点数: {len(waypoints)}")
        trajectory = self.follow_path(waypoints, threshold=0.015, max_per_point=250)
        avg_err = np.mean([t['error'] for t in trajectory])
        reached = sum(1 for t in trajectory if t['success'])
        print(f"  平均误差: {avg_err:.1f}mm | 到达: {reached}/{len(waypoints)}")
        return {'avg_error': avg_err, 'reached': reached, 'total': len(waypoints)}
    
    def run_demo(self):
        print("=" * 60)
        print("FFAI Robothon 2026 - 3DOF Drawing Robot (Final)")
        print("=" * 60)
        
        t1 = self.run_task1_reaching()
        t2 = self.run_task2_square()
        t3 = self.run_task3_circle()
        t4 = self.run_task4_figure8()
        
        t1_pass = sum(1 for r in t1 if r['success'])
        t1_err = np.mean([r['error'] for r in t1])
        
        scores = {
            'Reaching': t1_pass / 5 * 100,
            'Square': t2['reached'] / t2['total'] * 100,
            'Circle': t3['reached'] / t3['total'] * 100,
            'Figure8': t4['reached'] / t4['total'] * 100,
        }
        total = np.mean(list(scores.values()))
        
        print("\n" + "=" * 60)
        print("📊 Final Scores:")
        for k, v in scores.items():
            print(f"  {k}: {v:.1f}/100")
        print(f"\n  Total: {total:.1f}/100")
        print("=" * 60)
        
        return {'reaching': t1, 'square': t2, 'circle': t3, 'figure8': t4, 'scores': scores, 'total': total}


if __name__ == "__main__":
    c = RobotController()
    result = c.run_demo()
