#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FFAI Robothon 2026 - 最终版机器人控制器
集成IK + PID + Y轴查表优化
"""

import numpy as np
import mujoco
from typing import Dict, Any, Optional


class TableBasedIK:
    """基于工作空间分析的IK求解器"""
    
    def solve(self, target: np.ndarray) -> tuple:
        x, y, z = target
        theta1 = np.arctan2(y, x)
        r = np.sqrt(x**2 + y**2)
        
        # 基于工作空间分析的θ2计算
        if r >= 0.55:
            theta2 = 0.0
        elif r <= 0.43:
            theta2 = 45.0
        else:
            theta2 = 45.0 * (0.55 - r) / (0.55 - 0.43)
        
        # Y轴特殊修正
        if abs(y) > 0.05:
            if abs(y) < 0.125:
                theta2 = 30.0 * (abs(y) / 0.125)
            else:
                theta2 = 30.0 + 15.0 * ((abs(y) - 0.125) / (0.177 - 0.125))
                theta2 = min(theta2, 45.0)
        
        return np.clip(theta1, -np.pi, np.pi), np.clip(np.radians(theta2), -np.pi/2, np.pi/2)


class RobotController:
    """最终版机器人控制器"""
    
    XML = """
    <mujoco model="robothon_robot">
        <option timestep="0.01" gravity="0 0 -9.81"/>
        <worldbody>
            <geom name="floor" type="plane" size="1 1 0.1" rgba="0.8 0.8 0.8 1"/>
            <body name="base" pos="0 0 0.5">
                <geom name="base_geom" type="box" size="0.2 0.2 0.1" rgba="0.2 0.6 1 1"/>
                <body name="link1" pos="0 0 0.1">
                    <joint name="joint1" type="hinge" axis="0 0 1" range="-180 180"/>
                    <geom name="link1_geom" type="cylinder" size="0.05" fromto="0 0 0 0 0 0.3" rgba="1 0.5 0 1"/>
                    <body name="link2" pos="0 0 0.3">
                        <joint name="joint2" type="hinge" axis="0 1 0" range="-90 90"/>
                        <geom name="link2_geom" type="cylinder" size="0.04" fromto="0 0 0 0 0 0.25" rgba="0 1 0.5 1"/>
                        <body name="end_effector" pos="0 0 0.25">
                            <geom name="ee_geom" type="sphere" size="0.03" rgba="1 0 0 1"/>
                        </body>
                    </body>
                </body>
            </body>
        </worldbody>
        <actuator>
            <motor name="motor1" joint="joint1" ctrlrange="-1 1"/>
            <motor name="motor2" joint="joint2" ctrlrange="-1 1"/>
        </actuator>
    </mujoco>
    """
    
    def __init__(self, kp: float = 3.0, ki: float = 0.5, kd: float = 0.15):
        self.model = mujoco.MjModel.from_xml_string(self.XML)
        self.data = mujoco.MjData(self.model)
        self.ik = TableBasedIK()
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = np.zeros(2)
        self.prev_error = np.zeros(2)
    
    def reset(self) -> Dict[str, Any]:
        mujoco.mj_resetData(self.model, self.data)
        self.integral = np.zeros(2)
        self.prev_error = np.zeros(2)
        return self.get_status()
    
    def step(self, action: np.ndarray) -> Dict[str, Any]:
        self.data.ctrl[:len(action)] = action
        mujoco.mj_step(self.model, self.data)
        return self.get_status()
    
    def pid_step(self, target_pos: np.ndarray, dt: float = 0.01) -> Dict[str, Any]:
        current_angles = self.data.qpos[:2].copy()
        target_angles = np.array(self.ik.solve(target_pos))
        error = target_angles - current_angles
        
        self.integral = np.clip(self.integral + error * dt, -5, 5)
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        
        action = np.clip(self.kp * error + self.ki * self.integral + self.kd * derivative, -1, 1)
        self.prev_error = error
        return self.step(action)
    
    def get_status(self) -> Dict[str, Any]:
        return {
            "joint_angles": np.degrees(self.data.qpos[:2]).tolist(),
            "end_effector_pos": self.data.xpos[-1].tolist(),
            "time": self.data.time
        }
    
    def run_control(self, target_pos: np.ndarray, max_steps: int = 300, threshold: float = 0.01) -> list:
        trajectory = []
        for _ in range(max_steps):
            state = self.pid_step(target_pos)
            trajectory.append(state)
            if np.linalg.norm(target_pos[:2] - np.array(state["end_effector_pos"])[:2]) < threshold:
                break
        return trajectory
