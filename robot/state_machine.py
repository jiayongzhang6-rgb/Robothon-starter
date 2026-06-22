#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
冠军级状态机
"""

from .config import State


class StateMachine:
    """比赛级状态机"""
    
    def __init__(self):
        self.state = State.INIT
        self.error_count = 0
        self.task = None
        self.finished = False
        
    def update(self, ctx):
        """状态机主循环"""
        
        if self.state == State.INIT:
            print("[INIT] 系统启动")
            self.state = State.CALIBRATION
            
        elif self.state == State.CALIBRATION:
            print("[CALIB] 传感器校准...")
            ctx.sensor.calibrate()
            self.state = State.LINE_FOLLOW
            print("[CALIB] 校准完成")
            
        elif self.state == State.LINE_FOLLOW:
            if ctx.sensor.line_lost():
                self.error_count += 1
                self.state = State.RECOVERY
                print("[FOLLOW] 丢线，进入恢复")
            elif ctx.sensor.is_intersection():
                self.state = State.INTERSECTION
                print("[FOLLOW] 检测到十字路口")
            else:
                # 正常巡线
                from .controller.line_follow import follow
                follow(ctx.sensor, ctx.pid, ctx.motor)
                
        elif self.state == State.INTERSECTION:
            self.task = ctx.task_manager.get_next()
            if self.task is None:
                self.state = State.FINISH
                print("[INTER] 所有任务完成")
            else:
                self.state = State.TASK_ALIGN
                print(f"[INTER] 获取任务: {self.task.name}")
                
        elif self.state == State.TASK_ALIGN:
            if ctx.vision.aligned():
                self.state = State.TASK_EXECUTE
                print("[ALIGN] 对齐完成")
            else:
                ctx.vision.align_to_task()
                
        elif self.state == State.TASK_EXECUTE:
            success = ctx.task_executor.execute(self.task)
            if success:
                print(f"[EXEC] 任务完成: {self.task.name}")
            else:
                print(f"[EXEC] 任务失败: {self.task.name}")
            self.state = State.LINE_FOLLOW
            
        elif self.state == State.RECOVERY:
            ctx.recovery.run()
            self.error_count = 0
            self.state = State.LINE_FOLLOW
            print("[RECOVER] 恢复完成")
            
        elif self.state == State.FINISH:
            print("[FINISH] 比赛结束!")
            ctx.motor.stop()
            self.finished = True
