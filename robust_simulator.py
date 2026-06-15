#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FFAI Robothon 2026 - 3DOF容错仿真系统
模型几何重构版本
"""

import json
import logging
import traceback
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, Optional
from enum import Enum
import numpy as np


class ErrorType(Enum):
    MUJOCO_FAILED = "mujoco_failed"
    SIMULATION_CRASH = "simulation_crash"
    TIMEOUT = "timeout"
    UNKNOWN = "unknown"


class LogManager:
    """日志管理器"""
    
    def __init__(self, log_dir: str = "logs"):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(exist_ok=True)
        self.logger = logging.getLogger("Robothon")
        self.error_history = []
        
    def log_error(self, error_type: ErrorType, message: str, context: Dict = None):
        timestamp = datetime.now().isoformat()
        self.error_history.append({
            "timestamp": timestamp,
            "type": error_type.value,
            "message": message,
            "context": context or {}
        })
        # 保存到文件
        with open(self.log_dir / "errors.json", "w") as f:
            json.dump(self.error_history, f, indent=2)


class BasicPhysicsEngine:
    """基础物理引擎（MuJoCo不可用时的降级方案）"""
    
    def __init__(self):
        self.reset()
    
    def reset(self):
        self.pos = np.array([0.0, 0.0, 0.85])  # 调整初始位置
        return {"end_effector_pos": self.pos.tolist(), "engine": "basic"}
    
    def step(self, action: np.ndarray):
        self.pos[:2] += action[:2] * 0.01
        return {"end_effector_pos": self.pos.tolist(), "engine": "basic"}


class RobustController:
    """3DOF容错控制器"""
    
    def __init__(self):
        self.log = LogManager()
        self.engine = None
        self.engine_type = "unknown"
        self._init_mujoco()
    
    def _init_mujoco(self):
        try:
            from robot_controller import RobotController
            self.engine = RobotController()
            self.engine_type = "mujoco_3dof"
        except Exception as e:
            self.log.log_error(ErrorType.MUJOCO_FAILED, str(e))
            self.engine = BasicPhysicsEngine()
            self.engine_type = "basic"
    
    def reset(self):
        return self.engine.reset()
    
    def step(self, action: np.ndarray):
        try:
            return self.engine.step(action)
        except Exception as e:
            self.log.log_error(ErrorType.SIMULATION_CRASH, str(e))
            self.engine = BasicPhysicsEngine()
            self.engine_type = "basic"
            return self.engine.step(action)
    
    def pid_step(self, target_pos: np.ndarray, dt: float = 0.01):
        if hasattr(self.engine, 'pid_step'):
            return self.engine.pid_step(target_pos, dt)
        return self.step(target_pos[:3] * 0.1)
    
    def get_status(self):
        return self.engine.get_status() if hasattr(self.engine, 'get_status') else {"engine": self.engine_type}
