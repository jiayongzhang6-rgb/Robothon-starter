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
        
    def get_next(self):
        """获取下一个任务"""
        if self.index >= self.total:
            return None
        task = self.tasks[self.index]
        self.index += 1
        return task
    
    def get_current(self):
        """获取当前任务"""
        if self.index >= self.total:
            return None
        return self.tasks[self.index - 1] if self.index > 0 else None
    
    def get_progress(self):
        """获取进度"""
        return self.index / self.total if self.total > 0 else 0
    
    def reset(self):
        """重置"""
        self.index = 0
        for task in self.tasks:
            task.completed = False
