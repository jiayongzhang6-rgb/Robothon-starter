#!/bin/bash
# FFAI Robothon 2026 - 一键提交脚本（3DOF版本）
set -e

CONFIG="config.json"
REPORT="evaluation_report.json"

echo "════════════════════════════════════════════════════════════"
echo "   FFAI Robothon 2026 - 自动提交系统 (3DOF)"
echo "════════════════════════════════════════════════════════════"

# 读取参赛ID
ID=$(python3 -c "import json; print(json.load(open('$CONFIG'))['participant_id'])")
echo "参赛ID: $ID"

# 运行评估
echo ""
echo "▸ 运行评估测试..."
python3 - <<'EOF'
import sys, json, numpy as np
sys.path.insert(0, '.')
from robot_controller import RobotController

robot = RobotController()
tests = [
    ("基础定位", [0.1, 0.0, 0.8]),
    ("横向移动", [0.0, 0.15, 0.8]),
    ("对角移动", [0.1, 0.1, 0.85]),
]

results = []
for name, target in tests:
    robot.reset()
    traj = robot.run_control(np.array(target), max_steps=300, threshold=0.005)
    final = np.array(traj[-1]["end_effector_pos"])
    error = np.linalg.norm(np.array(target) - final)
    results.append({"name": name, "target": target, "final_pos": final.tolist(), "error": float(error), "passed": bool(error < 0.1)})
    status = "✅" if error < 0.1 else "❌"
    print(f"  {status} {name}: 误差={error:.4f}m | 目标={target} | 实际={final.tolist()}")

avg_error = np.mean([r["error"] for r in results])
pass_rate = sum(1 for r in results if r["passed"]) / len(results) * 100

print(f"\n  平均误差: {avg_error:.4f}m")
print(f"  通过率: {pass_rate:.0f}%")

with open("evaluation_report.json", "w") as f:
    json.dump({"participant_id": "d2f04863-5683-4e20-bd39-32f0cf339dc2", "model_version": "3DOF_几何重构版本", "avg_error": avg_error, "pass_rate": pass_rate, "tests": results}, f, indent=2, ensure_ascii=False)
EOF

# Git操作
echo ""
echo "▸ Git提交..."
git init 2>/dev/null || true
git add -A
git commit -m "🤖 Robothon Submission [freestyle] - 3DOF版本

Participant: d2f04863-5683-4e20-bd39-32f0cf339dc2
Model: 3DOF with numerical IK
Key changes: 3rd joint + base height 0.4m

Co-Authored-By: Hermes Agent" 2>/dev/null || echo "  无新更改"

# 推送到GitHub（如果配置了remote）
if git remote -v | grep -q origin; then
    echo ""
    echo "▸ 推送到GitHub..."
    git push origin master 2>&1 || echo "  ⚠️ 推送失败，可能需要手动推送"
fi

echo ""
echo "════════════════════════════════════════════════════════════"
echo "✅ 提交流程完成!"
echo "════════════════════════════════════════════════════════════"
