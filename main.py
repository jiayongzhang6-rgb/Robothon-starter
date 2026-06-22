#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FFAI Robothon 2026 - 主程序
工程级机器人控制系统
"""

import time
from robot.config import State, LINE_PID, TASK_PID, SPEED, TASK
from robot.state_machine import RobotStateMachine
from robot.motion.pid import PID
from robot.motion.motor import MotorController
from robot.sensors.line_sensor import LineSensor
from robot.tasks.task_manager import TaskManager, Task
from robot.tasks.task_executor import TaskExecutor


class RobotContext:
    """机器人上下文 - 所有模块的容器"""
    
    def __init__(self):
        self.motors = MotorController()
        self.sensors = LineSensor()
        self.pid = PID(**LINE_PID)
        self.task_pid = PID(**TASK_PID)
        self.task_manager = None
        self.task_executor = None


def create_tasks():
    """创建任务列表"""
    return [
        Task("推送方块", "PUSH", position=[50, 0], params={'distance': 30}),
        Task("按压按钮", "PRESS", position=[100, 50]),
        Task("配送物体", "DELIVER", position=[150, 0]),
    ]


def main():
    """主程序入口"""
    print("=" * 60)
    print("FFAI Robothon 2026 - 机器人控制系统 v1.0")
    print("工程级架构: 状态机 + PID + 模块化")
    print("=" * 60)
    
    # 初始化
    context = RobotContext()
    context.task_manager = TaskManager(create_tasks())
    context.task_executor = TaskExecutor(context.motors, context.sensors)
    
    state_machine = RobotStateMachine()
    
    # 主循环
    print("\n[MAIN] 启动机器人...")
    while not state_machine.finished:
        try:
            state_machine.update(context)
            time.sleep(0.01)  # 100Hz控制频率
        except KeyboardInterrupt:
            print("\n[MAIN] 用户中断")
            context.motors.stop()
            break
    
    print("\n[MAIN] 程序结束")


if __name__ == "__main__":
    main()
