#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务管理器
"""


class Task:
    """任务定义"""
    
    def __init__(self, name, task_type, position=None, params=None):
        self.name = name
        self.type = task_type  # PUSH, PRESS, DELIVER
        self.position = position or [0, 0]
        self.params = params or {}
        self.completed = False


class TaskManager:
    """任务管理器"""
    
    def __init__(self, task_list):
        self.tasks = task_list
        self.index = 0
        self.total = len(task_list)
        
    def get_current(self):
        """获取当前任务"""
        if self.index >= self.total:
            return None
        return self.tasks[self.index]
    
    def get_distance_to_task(self):
        """获取到当前任务的距离"""
        task = self.get_current()
        if task is None:
            return 0
        # 实际硬件时计算真实距离
        return 100
    
    def next(self):
        """完成当前任务，进入下一个"""
        if self.index < self.total:
            self.tasks[self.index].completed = True
            self.index += 1
            
    def skip(self):
        """跳过当前任务"""
        self.index += 1
        
    def get_progress(self):
        """获取进度"""
        completed = sum(1 for t in self.tasks if t.completed)
        return completed / self.total if self.total > 0 else 0
