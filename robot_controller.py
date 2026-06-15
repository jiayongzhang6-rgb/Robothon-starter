#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FFAI Robothon 2026 - 3自由度版本
增加第三个关节解决运动学限制
"""

import numpy as np
import mujoco
from typing import Dict, Any


class NumericalIK:
    """3自由度数值IK求解器"""
    
    def __init__(self, model, data):
        self.model = model
        self.data = data
    
    def solve(self, target: np.ndarray, max_iter: int = 200) -> np.ndarray:
        """使用雅可比逆解求IK"""
        q = np.array([0.0, 0.0, 0.0])
        
        for _ in range(max_iter):
            # 计算当前末端位置
            self.data.qpos[:3] = q
            mujoco.mj_forward(self.model, self.data)
            current_pos = self.data.xpos[-1].copy()
            
            # 计算误差
            error = target - current_pos
            
            # 计算雅可比矩阵
            J = np.zeros((3, 3))
            eps = 0.001
            for i in range(3):
                q_plus = q.copy()
                q_plus[i] += eps
                self.data.qpos[:3] = q_plus
                mujoco.mj_forward(self.model, self.data)
                pos_plus = self.data.xpos[-1].copy()
                J[:, i] = (pos_plus - current_pos) / eps
            
            # 使用伪逆求解
            JT = J.T
            JJT = J @ JT
            dq = JT @ np.linalg.solve(JJT + 0.0001 * np.eye(3), error)
            
            q = q + 0.3 * dq
            
            # 限制角度范围
            q[0] = np.clip(q[0], -np.pi, np.pi)
            q[1] = np.clip(q[1], -np.pi/2, np.pi/2)
            q[2] = np.clip(q[2], -np.pi/2, np.pi/2)
        
        return q


class RobotController:
    """3自由度机器人控制器"""
    
    # 3自由度模型：基座旋转 + 肩关节 + 肘关节
    XML = """
    <mujoco model="robothon_robot_3dof">
        <option timestep="0.01" gravity="0 0 -9.81"/>
        <worldbody>
            <geom name="floor" type="plane" size="1 1 0.1" rgba="0.8 0.8 0.8 1"/>
            <body name="base" pos="0 0 0.4">
                <geom name="base_geom" type="box" size="0.2 0.2 0.1" rgba="0.2 0.6 1 1"/>
                <body name="link1" pos="0 0 0.1">
                    <joint name="joint1" type="hinge" axis="0 0 1" range="-180 180"/>
                    <geom name="link1_geom" type="cylinder" size="0.05" fromto="0 0 0 0 0 0.15" rgba="1 0.5 0 1"/>
                    <body name="link2" pos="0 0 0.15">
                        <joint name="joint2" type="hinge" axis="0 1 0" range="-90 90"/>
                        <geom name="link2_geom" type="cylinder" size="0.04" fromto="0 0 0 0 0 0.2" rgba="0 1 0.5 1"/>
                        <body name="link3" pos="0 0 0.2">
                            <joint name="joint3" type="hinge" axis="0 1 0" range="-90 90"/>
                            <geom name="link3_geom" type="cylinder" size="0.03" fromto="0 0 0 0 0 0.15" rgba="0.5 0 1 1"/>
                            <body name="end_effector" pos="0 0 0.15">
                                <geom name="ee_geom" type="sphere" size="0.03" rgba="1 0 0 1"/>
                            </body>
                        </body>
                    </body>
                </body>
            </body>
        </worldbody>
        <actuator>
            <motor name="motor1" joint="joint1" ctrlrange="-1 1"/>
            <motor name="motor2" joint="joint2" ctrlrange="-1 1"/>
            <motor name="motor3" joint="joint3" ctrlrange="-1 1"/>
        </actuator>
    </mujoco>
    """
    
    def __init__(self, kp: float = 15.0, ki: float = 2.0, kd: float = 0.6):
        self.model = mujoco.MjModel.from_xml_string(self.XML)
        self.data = mujoco.MjData(self.model)
        self.ik = NumericalIK(self.model, self.data)
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = np.zeros(3)
        self.prev_error = np.zeros(3)
    
    def reset(self) -> Dict[str, Any]:
        mujoco.mj_resetData(self.model, self.data)
        self.integral = np.zeros(3)
        self.prev_error = np.zeros(3)
        return self.get_status()
    
    def step(self, action: np.ndarray) -> Dict[str, Any]:
        self.data.ctrl[:len(action)] = action
        mujoco.mj_step(self.model, self.data)
        return self.get_status()
    
    def pid_step(self, target_pos: np.ndarray, dt: float = 0.01) -> Dict[str, Any]:
        current_angles = self.data.qpos[:3].copy()
        target_angles = self.ik.solve(target_pos)
        error = target_angles - current_angles
        
        # 处理角度环绕
        error = (error + np.pi) % (2 * np.pi) - np.pi
        
        self.integral = np.clip(self.integral + error * dt, -15, 15)
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        
        action = self.kp * error + self.ki * self.integral + self.kd * derivative
        action = np.clip(action, -1, 1)
        
        self.prev_error = error
        return self.step(action)
    
    def get_status(self) -> Dict[str, Any]:
        return {
            "joint_angles": np.degrees(self.data.qpos[:3]).tolist(),
            "end_effector_pos": self.data.xpos[-1].tolist(),
            "time": self.data.time
        }
    
    def run_control(self, target_pos: np.ndarray, max_steps: int = 300, threshold: float = 0.005) -> list:
        trajectory = []
        for i in range(max_steps):
            state = self.pid_step(target_pos)
            trajectory.append(state)
            
            current_pos = np.array(state["end_effector_pos"])
            error = np.linalg.norm(target_pos - current_pos)
            
            if error < threshold:
                break
        
        return trajectory
