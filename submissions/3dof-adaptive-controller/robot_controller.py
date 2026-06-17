#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FFAI Robothon 2026 - 3DOF Confined-Space Precision Manipulator

核心创新: Safe Zone实时奇异点规避算法
- 距离工作空间中心<0.18m时，阻尼×3防止发散
- 距离≥0.18m时，阻尼=0.002最大化收敛
- 无需预计算，完全自适应

任务:
1. 5点到达（精确点位控制）
2. 正方形6cm（直线路径跟踪）
3. 圆形r=4cm（曲线平滑控制）
4. 8字形（双向曲线）
5. 螺旋线（渐变半径）
6. 五角星（锐角转向）
7. 心形（非凸曲线）
8. 螺旋星（复合轨迹）
"""

import numpy as np
import mujoco
import os
from typing import List, Tuple, Optional, Dict


class RobotController:
    """3DOF受限空间精密操作控制器
    
    集成Safe Zone奇异点规避和自适应阻尼控制。
    
    算法流程:
    1. 计算末端执行器到目标的误差向量
    2. 计算Jacobian矩阵
    3. 根据Safe Zone算法选择阻尼系数
    4. 使用DLS (Damped Least Squares) 求解关节速度
    5. 应用控制增益并执行一步仿真
    """
    
    # Safe Zone参数
    SINGULARITY_DISTANCE = 0.18  # 安全距离阈值(m)
    DAMPING_NEAR = 0.01          # 近奇异点阻尼
    DAMPING_FAR = 0.002          # 远离奇异点阻尼
    WORKSPACE_CENTER = np.array([0.0, 0.0, 0.8])
    
    # 控制参数
    GAIN = 30.0                  # 控制增益
    CLIP_RANGE = 2.0             # 控制量裁剪范围
    
    def __init__(self, xml_path: Optional[str] = None):
        if xml_path is None:
            xml_path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)), "robot.xml"
            )
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.ee_idx = -1
        for i in range(self.model.nbody):
            if self.model.body(i).name == "gripper_base":
                self.ee_idx = i
                break
    
    def reset(self):
        """重置机器人到初始状态"""
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[0] = 0.3
        self.data.qpos[1] = 0
        self.data.qpos[2] = 0.38
        self.data.qpos[3] = 1
        mujoco.mj_forward(self.model, self.data)
    
    def get_ee_pos(self) -> np.ndarray:
        """获取末端执行器位置"""
        mujoco.mj_forward(self.model, self.data)
        return self.data.xpos[self.ee_idx].copy()
    
    def compute_jacobian(self) -> np.ndarray:
        """计算3×3数值Jacobian矩阵
        
        使用前向差分法，eps=1e-4。
        """
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
    
    def safe_zone_damping(self, ee_pos: np.ndarray) -> float:
        """Safe Zone奇异点规避阻尼计算
        
        核心创新: 根据末端执行器到工作空间中心的距离，
        动态调整DLS阻尼系数。
        """
        dist = np.linalg.norm(ee_pos - self.WORKSPACE_CENTER)
        if dist < self.SINGULARITY_DISTANCE:
            return self.DAMPING_NEAR  # 接近奇异点，增加阻尼
        else:
            return self.DAMPING_FAR   # 远离奇异点，减小阻尼
    
    def step_ctrl(self, action: np.ndarray, gripper: float = 0.0):
        """执行一步控制"""
        self.data.ctrl[:3] = action
        self.data.ctrl[3] = gripper
        self.data.ctrl[4] = gripper
        mujoco.mj_step(self.model, self.data)
    
    def move_to(self, target: np.ndarray, threshold: float = 0.015,
                max_steps: int = 1200) -> Tuple[bool, int]:
        """移动到目标位置
        
        使用Safe Zone DLS逆运动学。
        """
        for step in range(max_steps):
            ee = self.get_ee_pos()
            error = target - ee
            if np.linalg.norm(error) < threshold:
                return True, step
            
            # Safe Zone阻尼
            damping = self.safe_zone_damping(ee)
            
            # DLS逆运动学
            J = self.compute_jacobian()
            dq = J.T @ np.linalg.solve(J @ J.T + damping * np.eye(3), error)
            action = np.clip(self.GAIN * dq, -self.CLIP_RANGE, self.CLIP_RANGE)
            self.step_ctrl(action)
        
        return False, max_steps
    
    def follow_path(self, waypoints: List, threshold: float = 0.015,
                   max_per_point: int = 1200) -> List[Dict]:
        """跟踪路径"""
        trajectory = []
        for i, wp in enumerate(waypoints):
            target = np.array(wp)
            ok, steps = self.move_to(
                target, threshold=threshold, max_steps=max_per_point
            )
            ee = self.get_ee_pos()
            err = np.linalg.norm(target - ee) * 1000
            trajectory.append({
                'point': i, 'error': err, 'success': ok, 'steps': steps
            })
        return trajectory
    
    # ========== 路径生成器 ==========
    
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
        waypoints = []
        for i in range(num_points):
            angle = 2 * np.pi * i / num_points
            x = cx + radius * np.sin(angle)
            z_pt = z + radius * np.sin(2 * angle) / 2
            waypoints.append([x, cy, z_pt])
        return waypoints
    
    def generate_spiral(self, cx, cy, radius, z, num_points=24, turns=2):
        waypoints = []
        for i in range(num_points):
            t = i / num_points
            angle = 2 * np.pi * turns * t
            r = radius * t
            x = cx + r * np.cos(angle)
            z_pt = z + r * np.sin(angle)
            waypoints.append([x, cy, z_pt])
        return waypoints
    
    def generate_star(self, cx, cy, radius, z, num_points=25):
        waypoints = []
        outer_angle_offset = np.pi / 2
        outer_points = []
        for i in range(5):
            angle = outer_angle_offset + 2 * np.pi * i / 5
            x = cx + radius * np.cos(angle)
            z_pt = z + radius * np.sin(angle)
            outer_points.append([x, cy, z_pt])
        inner_points = []
        for i in range(5):
            angle = outer_angle_offset + np.pi / 5 + 2 * np.pi * i / 5
            r_inner = radius * 0.382
            x = cx + r_inner * np.cos(angle)
            z_pt = z + r_inner * np.sin(angle)
            inner_points.append([x, cy, z_pt])
        points_per_line = num_points // 10
        for i in range(5):
            for j in range(points_per_line):
                t = j / points_per_line
                x = outer_points[i][0] + t * (inner_points[i][0] - outer_points[i][0])
                z_pt = outer_points[i][2] + t * (inner_points[i][2] - outer_points[i][2])
                waypoints.append([x, cy, z_pt])
            next_outer = outer_points[(i + 1) % 5]
            for j in range(points_per_line):
                t = j / points_per_line
                x = inner_points[i][0] + t * (next_outer[0] - inner_points[i][0])
                z_pt = inner_points[i][2] + t * (next_outer[2] - inner_points[i][2])
                waypoints.append([x, cy, z_pt])
        return waypoints
    
    def generate_heart(self, cx, cy, size, z, num_points=30):
        waypoints = []
        for i in range(num_points):
            t = 2 * np.pi * i / num_points
            x = size * (16 * np.sin(t)**3) / 16
            z_pt = size * (13 * np.cos(t) - 5 * np.cos(2*t) 
                          - 2 * np.cos(3*t) - np.cos(4*t)) / 16
            waypoints.append([cx + x, cy, z + z_pt])
        return waypoints
    
    def generate_spiral_star(self, cx, cy, radius, z, num_points=30, arms=5):
        waypoints = []
        for i in range(num_points):
            t = i / num_points
            angle = 2 * np.pi * t
            r = radius * t
            arm_mod = 1 + 0.3 * np.sin(arms * angle)
            x = cx + r * arm_mod * np.cos(angle)
            z_pt = z + r * arm_mod * np.sin(angle)
            waypoints.append([x, cy, z_pt])
        return waypoints
    
    # ========== 任务执行器 ==========
    
    def run_task1_reaching(self) -> List[Dict]:
        print("\n" + "="*60)
        print("[Task 1] 5-Point Reaching (Precise Positioning)")
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
            ok, steps = self.move_to(
                np.array(target), threshold=0.01, max_steps=1200
            )
            ee = self.get_ee_pos()
            err = np.linalg.norm(np.array(target) - ee) * 1000
            results.append({
                'name': name, 'error': err, 'success': ok, 'steps': steps
            })
            print(f"  {'✓' if ok else '✗'} {name}: err={err:.1f}mm steps={steps}")
        return results
    
    def _run_path_task(self, name: str, waypoints: List,
                      desc: str = "") -> Dict:
        print(f"\n{'='*60}")
        print(f"[{name}] {desc}")
        print(f"{'='*60}")
        self.reset()
        print(f"  路径点数: {len(waypoints)}")
        trajectory = self.follow_path(
            waypoints, threshold=0.015, max_per_point=1200
        )
        avg_err = np.mean([t['error'] for t in trajectory])
        reached = sum(1 for t in trajectory if t['success'])
        print(f"  平均误差: {avg_err:.1f}mm | 到达: {reached}/{len(waypoints)}")
        return {
            'avg_error': avg_err, 'reached': reached, 'total': len(waypoints)
        }
    
    def run_task2_square(self) -> Dict:
        wp = self.generate_square(cx=0.22, cy=0, size=0.06, z=0.45)
        return self._run_path_task("Task 2", wp, "Draw Square (6cm × 6cm)")
    
    def run_task3_circle(self) -> Dict:
        wp = self.generate_circle(cx=0.22, cy=0, radius=0.04, z=0.45)
        return self._run_path_task("Task 3", wp, "Draw Circle (r=4cm)")
    
    def run_task4_figure8(self) -> Dict:
        wp = self.generate_figure8(cx=0.22, cy=0, radius=0.04, z=0.45)
        return self._run_path_task("Task 4", wp, "Draw Figure-8")
    
    def run_task5_spiral(self) -> Dict:
        wp = self.generate_spiral(cx=0.22, cy=0, radius=0.04, z=0.45)
        return self._run_path_task("Task 5", wp, "Draw Spiral (2 turns)")
    
    def run_task6_star(self) -> Dict:
        wp = self.generate_star(cx=0.22, cy=0, radius=0.04, z=0.45)
        return self._run_path_task("Task 6", wp, "Draw Star (5-point)")
    
    def run_task7_heart(self) -> Dict:
        wp = self.generate_heart(cx=0.22, cy=0, size=0.04, z=0.45)
        return self._run_path_task("Task 7", wp, "Draw Heart")
    
    def run_task8_spiral_star(self) -> Dict:
        wp = self.generate_spiral_star(cx=0.22, cy=0, radius=0.04, z=0.45)
        return self._run_path_task("Task 8", wp, "Draw Spiral Star (5-arm)")
    
    def run_task9_grasp(self) -> Dict:
        """Task 9: Grasp and transport the red block
        
        Demonstrates MuJoCo depth: gripper physics, collision detection,
        object manipulation, and force-based grasping.
        """
        print(f"\n{'='*60}")
        print("[Task 9] Grasp & Transport Red Block")
        print(f"{'='*60}")
        self.reset()
        
        block_init = np.array([0.3, 0, 0.38])
        block_target = np.array([-0.2, 0, 0.38])
        
        # Step 1: Open gripper and approach block
        print("  1. Approaching block...")
        self.data.ctrl[3] = 1.0  # Open gripper
        self.data.ctrl[4] = 1.0
        ok, _ = self.move_to(block_init + np.array([0, 0, 0.02]),
                            threshold=0.015, max_steps=800)
        if not ok:
            print("  ✗ Failed to approach block")
            return {'success': False, 'error': 999}
        
        # Step 2: Lower to grasp height
        print("  2. Lowering to grasp...")
        ok, _ = self.move_to(block_init + np.array([0, 0, -0.01]),
                            threshold=0.01, max_steps=800)
        
        # Step 3: Close gripper
        print("  3. Closing gripper...")
        for _ in range(50):
            self.data.ctrl[3] = -1.0  # Close gripper
            self.data.ctrl[4] = -1.0
            mujoco.mj_step(self.model, self.data)
        
        # Step 4: Lift block
        print("  4. Lifting block...")
        lift_pos = block_init + np.array([0, 0, 0.08])
        ok, _ = self.move_to(lift_pos, threshold=0.02, max_steps=800)
        
        # Check if block was lifted (using touch sensor or position)
        block_pos = self.data.xpos[
            next(i for i in range(self.model.nbody) 
                 if self.model.body(i).name == "block")
        ].copy()
        lift_height = block_pos[2] - 0.38
        
        # Step 5: Transport to target
        print("  5. Transporting to target...")
        ok, _ = self.move_to(block_target + np.array([0, 0, 0.08]),
                            threshold=0.03, max_steps=1000)
        
        # Step 6: Place block
        print("  6. Placing block...")
        ok, _ = self.move_to(block_target + np.array([0, 0, -0.01]),
                            threshold=0.02, max_steps=800)
        
        # Step 7: Release
        for _ in range(50):
            self.data.ctrl[3] = 1.0
            self.data.ctrl[4] = 1.0
            mujoco.mj_step(self.model, self.data)
        
        # Final check
        block_final = self.data.xpos[
            next(i for i in range(self.model.nbody)
                 if self.model.body(i).name == "block")
        ].copy()
        transport_err = np.linalg.norm(
            block_final[:2] - block_target[:2]
        ) * 1000
        lifted = lift_height > 0.02
        
        print(f"  {'✓' if lifted and transport_err < 30 else '✗'} "
              f"Block lifted: {lifted} ({lift_height*1000:.1f}mm), "
              f"Transport error: {transport_err:.1f}mm")
        
        return {
            'success': lifted and transport_err < 30,
            'lifted': lifted,
            'transport_error': transport_err
        }
    
    def run_demo(self) -> Dict:
        print("=" * 60)
        print("FFAI Robothon 2026 - 3DOF Confined-Space Manipulator")
        print("Safe Zone Adaptive Controller")
        print("=" * 60)
        
        t1 = self.run_task1_reaching()
        t2 = self.run_task2_square()
        t3 = self.run_task3_circle()
        t4 = self.run_task4_figure8()
        t5 = self.run_task5_spiral()
        t6 = self.run_task6_star()
        t7 = self.run_task7_heart()
        t8 = self.run_task8_spiral_star()
        
        t1_pass = sum(1 for r in t1 if r['success'])
        
        scores = {
            'Reaching': t1_pass / 5 * 100,
            'Square': t2['reached'] / t2['total'] * 100,
            'Circle': t3['reached'] / t3['total'] * 100,
            'Figure8': t4['reached'] / t4['total'] * 100,
            'Spiral': t5['reached'] / t5['total'] * 100,
            'Star': t6['reached'] / t6['total'] * 100,
            'Heart': t7['reached'] / t7['total'] * 100,
            'SpiralStar': t8['reached'] / t8['total'] * 100,
        }
        total = np.mean(list(scores.values()))
        
        print("\n" + "=" * 60)
        print("📊 Final Scores:")
        for k, v in scores.items():
            print(f"  {k}: {v:.1f}/100")
        print(f"\n  Total: {total:.1f}/100")
        print("=" * 60)
        
        return {
            'reaching': t1, 'square': t2, 'circle': t3, 'figure8': t4,
            'spiral': t5, 'star': t6, 'heart': t7, 'spiral_star': t8,
            'scores': scores, 'total': total
        }


if __name__ == "__main__":
    c = RobotController()
    result = c.run_demo()
