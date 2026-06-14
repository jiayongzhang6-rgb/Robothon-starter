#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FFAI Robothon 2026 - 机器人控制器
封装 MuJoCo 仿真环境，提供简洁的控制接口
"""

import mujoco
import numpy as np
from typing import Optional, Tuple, Dict, Any


class RobotController:
    """
    机器人控制器类
    封装 MuJoCo 仿真，提供 reset/step/get_status 接口
    """
    
    # 默认机器人模型 XML
    DEFAULT_XML = """
    <mujoco model="robothon_robot">
        <option timestep="0.01" gravity="0 0 -9.81"/>
        
        <worldbody>
            <!-- 地面 -->
            <geom name="floor" type="plane" size="1 1 0.1" rgba="0.8 0.8 0.8 1"/>
            
            <!-- 机器人基座 -->
            <body name="base" pos="0 0 0.5">
                <geom name="base_geom" type="box" size="0.2 0.2 0.1" rgba="0.2 0.6 1 1"/>
                
                <!-- 机械臂第一段 -->
                <body name="link1" pos="0 0 0.1">
                    <joint name="joint1" type="hinge" axis="0 0 1" range="-180 180"/>
                    <geom name="link1_geom" type="cylinder" size="0.05" fromto="0 0 0 0 0 0.3" rgba="1 0.5 0 1"/>
                    
                    <!-- 机械臂第二段 -->
                    <body name="link2" pos="0 0 0.3">
                        <joint name="joint2" type="hinge" axis="0 1 0" range="-90 90"/>
                        <geom name="link2_geom" type="cylinder" size="0.04" fromto="0 0 0 0 0 0.25" rgba="0 1 0.5 1"/>
                        
                        <!-- 末端执行器 -->
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
    
    def __init__(self, xml: Optional[str] = None):
        """
        初始化控制器
        
        Args:
            xml: MuJoCo 模型 XML 字符串，None 则使用默认模型
        """
        self.xml = xml or self.DEFAULT_XML
        self.model: Optional[mujoco.MjModel] = None
        self.data: Optional[mujoco.MjData] = None
        self.renderer: Optional[mujoco.Renderer] = None
        
        # 初始化模型
        self._load_model()
        
    def _load_model(self):
        """加载 MuJoCo 模型"""
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        
    def reset(self) -> Dict[str, Any]:
        """
        重置仿真环境到初始状态
        
        Returns:
            初始状态字典
        """
        mujoco.mj_resetData(self.model, self.data)
        return self.get_status()
    
    def step(self, action: np.ndarray) -> Dict[str, Any]:
        """
        执行一步仿真
        
        Args:
            action: 控制信号数组，形状 (n_actuators,)
            
        Returns:
            执行后的状态字典
        """
        # 设置控制信号
        self.data.ctrl[:len(action)] = action
        
        # 执行一步仿真
        mujoco.mj_step(self.model, self.data)
        
        return self.get_status()
    
    def get_status(self) -> Dict[str, Any]:
        """
        获取当前仿真状态
        
        Returns:
            包含关节角度和末端位置的状态字典
        """
        # 获取关节角度（转换为角度制）
        joint_angles = np.degrees(self.data.qpos[:2]).tolist()
        
        # 获取末端执行器位置
        end_effector_pos = self.data.xpos[-1].tolist()
        
        # 获取末端执行器姿态（旋转矩阵）
        end_effector_mat = self.data.xmat[-1].reshape(3, 3).tolist()
        
        return {
            "joint_angles": joint_angles,
            "end_effector_pos": end_effector_pos,
            "end_effector_orientation": end_effector_mat,
            "time": self.data.time
        }
    
    def get_camera_image(self, width: int = 640, height: int = 480) -> np.ndarray:
        """
        获取当前视角的渲染图像
        
        Args:
            width: 图像宽度
            height: 图像高度
            
        Returns:
            RGB 图像数组，形状 (height, width, 3)
        """
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, height=height, width=width)
        
        self.renderer.update_scene(self.data)
        return self.renderer.render()
    
    def set_joint_positions(self, positions: np.ndarray):
        """
        直接设置关节位置（用于运动规划）
        
        Args:
            positions: 目标关节角度（弧度）
        """
        self.data.qpos[:len(positions)] = positions
        mujoco.mj_forward(self.model, self.data)
        
    def get_joint_positions(self) -> np.ndarray:
        """
        获取当前关节位置（弧度）
        
        Returns:
            关节角度数组
        """
        return self.data.qpos[:2].copy()
    
    def get_end_effector_position(self) -> np.ndarray:
        """
        获取末端执行器位置
        
        Returns:
            位置数组 [x, y, z]
        """
        return self.data.xpos[-1].copy()
    
    def run_simulation(self, duration: float = 1.0) -> list:
        """
        运行指定时长的仿真
        
        Args:
            duration: 仿真时长（秒）
            
        Returns:
            状态轨迹列表
        """
        steps = int(duration / self.model.opt.timestep)
        trajectory = []
        
        for _ in range(steps):
            state = self.step(np.zeros(self.model.nu))
            trajectory.append(state)
            
        return trajectory


# 使用示例
if __name__ == "__main__":
    print("FFAI Robothon 2026 - RobotController 测试")
    print("=" * 50)
    
    # 初始化控制器
    robot = RobotController()
    print("✓ 控制器初始化成功")
    
    # 重置环境
    state = robot.reset()
    print(f"✓ 环境重置完成")
    print(f"  初始末端位置: {state['end_effector_pos']}")
    
    # 执行控制动作
    action = np.array([0.5, 0.3])
    state = robot.step(action)
    print(f"\n✓ 执行动作: {action}")
    print(f"  关节角度: {state['joint_angles']}")
    print(f"  末端位置: {state['end_effector_pos']}")
    
    # 运行一段仿真
    print("\n正在运行仿真...")
    robot.reset()
    trajectory = robot.run_simulation(duration=0.5)
    final_state = trajectory[-1]
    print(f"✓ 仿真完成 (共 {len(trajectory)} 步)")
    print(f"  最终末端位置: {final_state['end_effector_pos']}")
    
    print("\n" + "=" * 50)
    print("✓ RobotController 测试通过！")
