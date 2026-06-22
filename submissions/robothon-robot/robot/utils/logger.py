#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
日志工具
"""

import time


class Logger:
    """日志记录器"""
    
    def __init__(self, name="robot"):
        self.name = name
        self.logs = []
        
    def info(self, msg):
        """信息日志"""
        entry = f"[{time.strftime('%H:%M:%S')}] [INFO] {msg}"
        print(entry)
        self.logs.append(entry)
        
    def warning(self, msg):
        """警告日志"""
        entry = f"[{time.strftime('%H:%M:%S')}] [WARN] {msg}"
        print(entry)
        self.logs.append(entry)
        
    def error(self, msg):
        """错误日志"""
        entry = f"[{time.strftime('%H:%M:%S')}] [ERROR] {msg}"
        print(entry)
        self.logs.append(entry)
        
    def save(self, filename="robot.log"):
        """保存日志"""
        with open(filename, 'w') as f:
            f.write('\n'.join(self.logs))
