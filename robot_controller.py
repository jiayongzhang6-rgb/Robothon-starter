#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FFAI Robothon 2026 - v8 终极优化版
目标分数: 85+
"""

import numpy as np
import mujoco
from typing import Dict, Any, Tuple, List


class RobotController:
    """v8终极优化版 - 超高增益 + 极小阻尼"""
    
    JOINT_ADDR = [7, 8, 9]
    
    XML = """
    <mujoco model="robothon_v8_ultimate">
        <option timestep="0.002" gravity="0 0 -9.81">
            <flag contact="enable"/>
        </option>
        
        <visual>
            <global offwidth="1920" offheight="1080"/>
        </visual>
        
        <asset>
            <texture name="texplane" type="2d" builtin="checker" rgb1="0.85 0.85 0.85" rgb2="0.7 0.7 0.7" width="512" height="512"/>
            <material name="matplane" texture="texplane" texrepeat="5 5" reflectance="0.1"/>
            <material name="red" rgba="0.85 0.2 0.1 1"/>
            <material name="blue" rgba="0.1 0.3 0.85 1"/>
            <material name="green" rgba="0.1 0.75 0.3 1"/>
            <material name="yellow" rgba="0.9 0.8 0.1 1"/>
            <material name="metal" rgba="0.5 0.5 0.55 1" specular="0.6"/>
            <material name="wood" rgba="0.65 0.45 0.25 1"/>
        </asset>
        
        <worldbody>
            <geom name="floor" type="plane" size="2 2 0.1" material="matplane"/>
            <light pos="0 0 3" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
            
            <!-- 桌子 -->
            <body name="table" pos="0.3 0 0.35">
                <geom name="table_top" type="box" size="0.15 0.15 0.012" material="wood"/>
                <geom name="leg1" type="cylinder" size="0.012 0.175" pos="0.13 0.13 -0.187" material="wood"/>
                <geom name="leg2" type="cylinder" size="0.012 0.175" pos="-0.13 0.13 -0.187" material="wood"/>
                <geom name="leg3" type="cylinder" size="0.012 0.175" pos="0.13 -0.13 -0.187" material="wood"/>
                <geom name="leg4" type="cylinder" size="0.012 0.175" pos="-0.13 -0.13 -0.187" material="wood"/>
            </body>
            
            <!-- 红色方块 -->
            <body name="block" pos="0.3 0 0.38">
                <freejoint name="block_joint"/>
                <geom name="block_geom" type="box" size="0.02 0.02 0.02" material="red" mass="0.05"/>
            </body>
            
            <!-- 蓝色目标区域 -->
            <body name="target_zone" pos="-0.2 0 0.35">
                <geom name="target_geom" type="cylinder" size="0.05 0.001" material="blue"/>
            </body>
            
            <!-- 机械臂 -->
            <body name="base" pos="0 0 0.3">
                <geom name="base_geom" type="cylinder" size="0.08 0.03" material="metal"/>
                
                <body name="link1" pos="0 0 0.03">
                    <joint name="joint1" type="hinge" axis="0 0 1" range="-180 180" damping="0.5"/>
                    <geom name="link1_geom" type="capsule" fromto="0 0 0 0 0 0.18" size="0.035" material="blue"/>
                    
                    <body name="link2" pos="0 0 0.18">
                        <joint name="joint2" type="hinge" axis="0 1 0" range="-135 135" damping="0.5"/>
                        <geom name="link2_geom" type="capsule" fromto="0 0 0 0 0 0.22" size="0.03" material="green"/>
                        
                        <body name="link3" pos="0 0 0.22">
                            <joint name="joint3" type="hinge" axis="0 1 0" range="-150 150" damping="0.5"/>
                            <geom name="link3_geom" type="capsule" fromto="0 0 0 0 0 0.18" size="0.022" material="yellow"/>
                            
                            <!-- 夹爪 -->
                            <body name="gripper_base" pos="0 0 0.18">
                                <geom name="gripper_base_geom" type="box" size="0.012 0.03 0.012" material="metal"/>
                                
                                <body name="left_finger" pos="0 0.03 0.012">
                                    <joint name="finger_left" type="slide" axis="0 1 0" range="0 0.02" damping="0.3"/>
                                    <geom name="left_finger_geom" type="box" size="0.006 0.01 0.02" material="metal" friction="2.0"/>
                                </body>
                                
                                <body name="right_finger" pos="0 -0.03 0.012">
                                    <joint name="finger_right" type="slide" axis="0 -1 0" range="0 0.02" damping="0.3"/>
                                    <geom name="right_finger_geom" type="box" size="0.006 0.01 0.02" material="metal" friction="2.0"/>
                                </body>
                                
                                <site name="touch_site" pos="0 0 0.03" size="0.006"/>
                            </body>
                        </body>
                    </body>
                </body>
            </body>
        </worldbody>
        
        <sensor>
            <touch name="touch_sensor" site="touch_site"/>
            <framepos name="ee_pos" objtype="body" objname="gripper_base"/>
            <framepos name="block_pos" objtype="body" objname="block"/>
        </sensor>
        
        <actuator>
            <motor name="m1" joint="joint1" ctrlrange="-2 2"/>
            <motor name="m2" joint="joint2" ctrlrange="-2 2"/>
            <motor name="m3" joint="joint3" ctrlrange="-2 2"/>
            <position name="fingers" joint="finger_left" ctrlrange="0 0.02" kp="60"/>
            <position name="fingers_r" joint="finger_right" ctrlrange="0 0.02" kp="60"/>
        </actuator>
    </mujoco>
    """
    
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_string(self.XML)
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
    
    def get_block_pos(self):
        mujoco.mj_forward(self.model, self.data)
        for i in range(self.model.nbody):
            if self.model.body(i).name == "block":
                return self.data.xpos[i].copy()
        return np.array([0.3, 0, 0.38])
    
    def get_touch(self):
        mujoco.mj_forward(self.model, self.data)
        return self.data.sensordata[0]
    
    def compute_jacobian(self):
        """计算Jacobian矩阵"""
        ee = self.get_ee_pos()
        J = np.zeros((3, 3))
        eps = 1e-4
        for i in range(3):
            q = self.data.qpos[7 + i]
            self.data.qpos[7 + i] = q + eps
            mujoco.mj_forward(self.model, self.data)
            ee_plus = self.data.xpos[self.ee_idx].copy()
            J[:, i] = (ee_plus - ee) / eps
            self.data.qpos[7 + i] = q
        return J
    
    def step(self, action, gripper=0.0):
        self.data.ctrl[:3] = action
        self.data.ctrl[3] = gripper
        self.data.ctrl[4] = gripper
        mujoco.mj_step(self.model, self.data)
    
    def move_to(self, target, threshold=0.02, max_steps=1200, gripper=0.0):
        """核心控制算法 - 终极优化版"""
        for step in range(max_steps):
            ee = self.get_ee_pos()
            error = target - ee
            error_mag = np.linalg.norm(error)
            
            if error_mag < threshold:
                return True, step
            
            # Safe Zone检测
            origin = np.array([0.0, 0.0, 0.8])
            dist_to_origin = np.linalg.norm(ee - origin)
            is_safe_zone = dist_to_origin < 0.18
            
            # 极小阻尼
            damping = 0.01 if is_safe_zone else 0.002
            
            # Jacobian伪逆
            J = self.compute_jacobian()
            dq = J.T @ np.linalg.solve(J @ J.T + damping * np.eye(3), error)
            
            # 极高增益
            gain = 30.0
            
            action = gain * dq
            action = np.clip(action, -2, 2)
            self.step(action, gripper)
        
        return False, max_steps
    
    def run_demo(self):
        """运行完整演示"""
        print("=" * 60)
        print("FFAI Robothon 2026 - v8 终极优化版")
        print("超高增益(30.0) + 极小阻尼(0.01/0.002)")
        print("=" * 60)
        
        self.reset()
        
        print("\n[1] 初始状态:")
        print(f"    EE: {self.get_ee_pos()}")
        print(f"    方块: {self.get_block_pos()}")
        
        targets = [
            ("方块上方", np.array([0.3, 0, 0.5])),
            ("方块位置", np.array([0.3, 0, 0.4])),
            ("目标区域", np.array([-0.2, 0, 0.4])),
            ("左前方", np.array([0.15, 0, 0.5])),
            ("左上方", np.array([-0.15, 0, 0.5])),
        ]
        
        results = []
        for name, target in targets:
            self.reset()
            ok, steps = self.move_to(target, threshold=0.02, max_steps=1200)
            ee = self.get_ee_pos()
            err = np.linalg.norm(target - ee)
            results.append({
                'name': name,
                'target': target.tolist(),
                'actual': ee.tolist(),
                'error': err,
                'success': ok,
                'steps': steps,
            })
            status = '✓' if ok else '✗'
            print(f"  {status} {name}: err={err:.4f}, steps={steps}")
        
        # 计算分数
        successes = sum(1 for r in results if r['success'])
        avg_error = np.mean([r['error'] for r in results])
        avg_steps = np.mean([r['steps'] for r in results])
        
        scores = {
            '到达成功率': successes / len(targets) * 100,
            '平均精度': max(0, 100 - avg_error * 1000),
            '效率': max(0, 100 - avg_steps / 12),
        }
        total = sum(scores.values()) / len(scores)
        
        print("\n" + "=" * 60)
        print("📊 评分:")
        for k, v in scores.items():
            print(f"  {k}: {v:.1f}/100")
        print(f"\n  综合: {total:.1f}/100")
        print("=" * 60)
        
        return {'results': results, 'scores': scores, 'total': total}


if __name__ == "__main__":
    c = RobotController()
    result = c.run_demo()
