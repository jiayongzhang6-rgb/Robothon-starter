#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
状态机模块 - 机器人核心控制逻辑
"""

from .config import State, RECOVERY


class RobotStateMachine:
    """机器人状态机"""
    
    def __init__(self):
        self.state = State.INIT
        self.current_task = None
        self.error_count = 0
        self.finished = False
        self.task_index = 0
        
    def update(self, context):
        """状态机主更新循环"""
        if self.state == State.INIT:
            self._init(context)
        elif self.state == State.SEARCH_LINE:
            self._search_line(context)
        elif self.state == State.FOLLOW_LINE:
            self._follow_line(context)
        elif self.state == State.NAVIGATE_TO_TASK:
            self._navigate(context)
        elif self.state == State.EXECUTE_TASK:
            self._execute(context)
        elif self.state == State.RECOVER:
            self._recover(context)
        elif self.state == State.FINISH:
            self._finish(context)
    
    def _init(self, context):
        """初始化状态"""
        print("[INIT] 系统初始化...")
        context.motors.stop()
        context.sensors.calibrate()
        self.state = State.SEARCH_LINE
        print("[INIT] 初始化完成，开始搜索线")
    
    def _search_line(self, context):
        """搜索黑线"""
        print("[SEARCH] 搜索黑线...")
        if context.sensors.detect_line():
            self.state = State.FOLLOW_LINE
            self.error_count = 0
            print("[SEARCH] 检测到黑线")
        else:
            # 原地旋转搜索
            context.motors.rotate(30)
    
    def _follow_line(self, context):
        """巡线模式"""
        if not context.sensors.detect_line():
            self.error_count += 1
            if self.error_count > RECOVERY['max_errors']:
                self.state = State.RECOVER
                print("[FOLLOW] 丢线过多，进入恢复")
            return
        
        self.error_count = 0
        # 获取传感器数据并计算误差
        sensor_values = context.sensors.get_values()
        error = context.pid.compute(sensor_values)
        
        # 动态速度控制
        distance = context.task_manager.get_distance_to_task()
        speed = self._dynamic_speed(distance)
        
        context.motors.set_speed(speed - error, speed + error)
    
    def _navigate(self, context):
        """导航到任务点"""
        task = context.task_manager.get_current()
        if task is None:
            self.state = State.FINISH
            return
        
        # 检查是否到达
        if context.sensors.is_at_position(task.position):
            self.state = State.EXECUTE_TASK
            print(f"[NAVIGATE] 到达任务点: {task.name}")
        else:
            # 继续导航
            direction = context.sensors.get_direction(task.position)
            context.motors.move_toward(direction)
    
    def _execute(self, context):
        """执行任务"""
        task = context.task_manager.get_current()
        if task is None:
            self.state = State.FINISH
            return
        
        success = context.task_executor.execute(task)
        if success:
            print(f"[EXECUTE] 任务完成: {task.name}")
            context.task_manager.next()
            self.state = State.NAVIGATE_TO_TASK
        else:
            self.error_count += 1
            if self.error_count > TASK['max_retries']:
                print(f"[EXECUTE] 任务失败: {task.name}")
                context.task_manager.skip()
                self.state = State.NAVIGATE_TO_TASK
    
    def _recover(self, context):
        """恢复模式"""
        print("[RECOVER] 执行恢复...")
        context.motors.stop()
        
        # 尝试搜索黑线
        for angle in RECOVERY['search_angles']:
            context.motors.rotate(angle)
            if context.sensors.detect_line():
                self.state = State.FOLLOW_LINE
                self.error_count = 0
                print("[RECOVER] 恢复成功")
                return
        
        # 后退再找
        context.motors.backward(RECOVERY['backward_distance'])
        if context.sensors.detect_line():
            self.state = State.FOLLOW_LINE
            self.error_count = 0
        else:
            self.state = State.SEARCH_LINE
    
    def _finish(self, context):
        """完成状态"""
        print("[FINISH] 所有任务完成!")
        context.motors.stop()
        self.finished = True
    
    def _dynamic_speed(self, distance):
        """动态速度控制"""
        if distance > 100:
            return SPEED['fast']
        elif distance > 30:
            return SPEED['medium']
        else:
            return SPEED['slow']
