#!/bin/bash
# FFAI Robothon 2026 - 3DOF Confined-Space Manipulator
# 一键运行脚本
set -e

echo "============================================================"
echo "FFAI Robothon 2026 - 3DOF Confined-Space Manipulator"
echo "Safe Zone Adaptive Controller"
echo "============================================================"

# 检查Python
if ! command -v python3 &> /dev/null; then
    echo "❌ Python3 未安装"
    exit 1
fi

# 安装依赖
echo "📦 安装依赖..."
pip install -q -r requirements.txt 2>/dev/null || pip3 install -q -r requirements.txt 2>/dev/null

# 运行控制器
echo ""
echo "🚀 启动控制器..."
echo ""
python3 robot_controller.py
