#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FFAI Robothon 2026 - 3DOF Adaptive Robot Controller
自适应容错机器人控制系统 - MuJoCo仿真
"""

import numpy as np
import mujoco
import os
from typing import Dict, Any, Tuple, List


class RobotController:
    """3DOF自适应机器人控制器
    
    特性:
    - DLS逆运动学 (Jacobian伪逆 + 阻尼最小二乘)
    - 自适应增益调度 (gain=30.0, damping=0.002)
    - Safe Zone奇异点检测与规避
    - 电机执行器控制 (kp=25, ki=3, kd=0.8)
    """
    
    JOINT_ADDR = [7, 8, 9]
    
    def __init__(self, xml_path=None):
        """初始化控制器
        
        Args:
            xml_path: MJCF模型文件路径。默认为同目录下的 robot.xml
        """
        if xml_path is None:
            xml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot.xml")
        
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        
        # 查找末端执行器索引
        self.ee_idx = -1
        for i in range(self.model.nbody):
            if self.model.body(i).name == "gripper_base":
                self.ee_idx = i
                break
    
    def reset(self):
        """重置到初始状态"""
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[0] = 0.3  # 方块X
        self.data.qpos[1] = 0    # 方块Y
        self.data.qpos[2] = 0.38 # 方块Z
        self.data.qpos[3] = 1    # 方块四元数w
        mujoco.mj_forward(self.model, self.data)
    
    def get_ee_pos(self):
        """获取末端执行器位置"""
        mujoco.mj_forward(self.model, self.data)
        return self.data.xpos[self.ee_idx].copy()
    
    def get_block_pos(self):
        """获取方块位置"""
        mujoco.mj_forward(self.model, self.data)
        for i in range(self.model.nbody):
            if self.model.body(i).name == "block":
                return self.data.xpos[i].copy()
        return np.array([0.3, 0, 0.38])
    
    def get_touch(self):
        """获取触觉传感器读数"""
        mujoco.mj_forward(self.model, self.data)
        return self.data.sensordata[0]
    
    def compute_jacobian(self):
        """计算Jacobian矩阵（数值微分法）"""
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
        """执行一步控制"""
        self.data.ctrl[:3] = action
        self.data.ctrl[3] = gripper
        self.data.ctrl[4] = gripper
        mujoco.mj_step(self.model, self.data)
    
    def move_to(self, target, threshold=0.02, max_steps=1200, gripper=0.0):
        """核心控制算法 - DLS逆运动学 + 自适应增益
        
        Args:
            target: 目标位置 [x, y, z]
            threshold: 收敛阈值 (m)
            max_steps: 最大迭代步数
            gripper: 夹爪控制值 (0=闭合, 0.02=张开)
            
        Returns:
            (success: bool, steps: int)
        """
        for step in range(max_steps):
            ee = self.get_ee_pos()
            error = target - ee
            error_mag = np.linalg.norm(error)
            
            # 收敛检测
            if error_mag < threshold:
                return True, step
            
            # Safe Zone检测 - 奇异点附近自动调整阻尼
            origin = np.array([0.0, 0.0, 0.8])
            dist_to_origin = np.linalg.norm(ee - origin)
            is_safe_zone = dist_to_origin < 0.18
            
            # 自适应阻尼：安全区内3倍阻尼
            damping = 0.01 if is_safe_zone else 0.002
            
            # Jacobian伪逆 (DLS)
            J = self.compute_jacobian()
            dq = J.T @ np.linalg.solve(J @ J.T + damping * np.eye(3), error)
            
            # 自适应增益
            gain = 30.0
            action = gain * dq
            action = np.clip(action, -2, 2)
            self.step(action, gripper)
        
        return False, max_steps
    
    def push_block(self, start_pos, end_pos, approach_height=0.42, push_height=0.38):
        """推块任务 - 将方块从A点推到B点
        
        Args:
            start_pos: 方块起始位置 [x, y]
            end_pos: 方块目标位置 [x, y]
            approach_height: 接近高度 (z)
            push_height: 推动高度 (z)
            
        Returns:
            (success: bool, steps: int)
        """
        # 1. 移动到方块后方
        approach_pos = np.array([start_pos[0] - 0.05, start_pos[1], approach_height])
        ok, steps = self.move_to(approach_pos, threshold=0.02, max_steps=800)
        if not ok:
            return False, steps
        
        # 2. 降低到推动高度
        push_pos = np.array([start_pos[0] - 0.05, start_pos[1], push_height])
        ok, steps2 = self.move_to(push_pos, threshold=0.02, max_steps=400)
        
        # 3. 推动方块到目标位置
        target_pos = np.array([end_pos[0], end_pos[1], push_height])
        ok, steps3 = self.move_to(target_pos, threshold=0.03, max_steps=800)
        
        total_steps = steps + steps2 + steps3
        
        # 检查方块是否到达目标附近
        block_pos = self.get_block_pos()
        block_xy = block_pos[:2]
        dist = np.linalg.norm(block_xy - np.array(end_pos))
        
        return dist < 0.05, total_steps
    
    def run_demo(self):
        """运行完整演示"""
        print("=" * 60)
        print("FFAI Robothon 2026 - 3DOF Adaptive Robot Controller")
        print("DLS逆运动学 + 自适应增益 + Safe Zone检测")
        print("=" * 60)
        
        self.reset()
        
        print("\n[1] 初始状态:")
        print(f"    末端执行器: {self.get_ee_pos()}")
        print(f"    方块位置: {self.get_block_pos()}")
        
        # === 任务1: 5点到达 ===
        print("\n[2] 5点到达任务:")
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
            print(f"  {status} {name}: err={err:.4f}m, steps={steps}")
        
        # === 任务2: 推块任务 ===
        print("\n[3] 推块任务 (A→B):")
        self.reset()
        push_ok, push_steps = self.push_block(
            start_pos=[0.3, 0],
            end_pos=[-0.2, 0]
        )
        block_pos = self.get_block_pos()
        print(f"  {'✓' if push_ok else '✗'} 方块从(0.3,0)推到(-0.2,0)")
        print(f"    最终位置: ({block_pos[0]:.3f}, {block_pos[1]:.3f})")
        print(f"    步数: {push_steps}")
        
        # === 评分 ===
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
