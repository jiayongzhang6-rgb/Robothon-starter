#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FFAI Robothon 2026 - 冠军级主程序
"""

from robot.config import State, PID as PID_CONFIG, BASE_SPEED
from robot.state_machine import StateMachine
from robot.controller.pid import PID
from robot.controller.motor import MotorController
from robot.sensors.line_sensor import LineSensor
from robot.sensors.vision import VisionSystem
from robot.tasks.task_manager import TaskManager, Task
from robot.tasks.task_executor import TaskExecutor
from robot.recovery.recovery import RecoverySystem
from robot.utils.timer import Timer
from robot.utils.logger import Logger


class RobotContext:
    """机器人上下文"""
    
    def __init__(self):
        self.motor = MotorController()
        self.sensor = LineSensor()
        self.vision = VisionSystem()
        self.pid = PID(**PID_CONFIG)
        self.task_manager = None
        self.task_executor = None
        self.recovery = None
        self.timer = Timer()
        self.logger = Logger()


def create_tasks():
    """创建任务列表"""
    return [
        Task("推送方块", "PUSH", position=[50, 0], params={'distance': 30}),
        Task("按压按钮", "PRESS", position=[100, 50]),
        Task("配送物体", "DELIVER", position=[150, 0]),
    ]


def main():
    """主程序"""
    print("=" * 60)
    print("FFAI Robothon 2026 - 冠军级机器人控制系统")
    print("工程结构: 状态机 + PID + 三层恢复")
    print("=" * 60)
    
    # 初始化
    ctx = RobotContext()
    ctx.task_manager = TaskManager(create_tasks())
    ctx.task_executor = TaskExecutor(ctx.motor, ctx.vision)
    ctx.recovery = RecoverySystem(ctx.motor, ctx.sensor)
    
    sm = StateMachine()
    
    # 主循环
    print("\n[MAIN] 启动...")
    ctx.timer.start()
    
    while not sm.finished:
        try:
            sm.update(ctx)
            
            # 状态输出
            if sm.state == State.LINE_FOLLOW:
                error = ctx.sensor.weighted_error()
                speed = ctx.motor.dynamic_speed(error)
                ctx.logger.info(f"State: FOLLOW | Error: {error:.2f} | Speed: {speed}")
            
        except KeyboardInterrupt:
            print("\n[MAIN] 用户中断")
            ctx.motor.stop()
            break
    
    # 结束
    elapsed = ctx.timer.elapsed()
    print(f"\n[MAIN] 完成! 用时: {elapsed:.2f}秒")
    ctx.logger.save("robot.log")


if __name__ == "__main__":
    main()
