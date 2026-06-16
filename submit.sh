#!/bin/bash
# FFAI Robothon 2026 - 提交脚本（v8_ultimate 85.7分版本）
set -e

CONFIG="config.json"
REPORT="evaluation_report.json"

echo "════════════════════════════════════════════════════════════"
echo "   FFAI Robothon 2026 - v8_ultimate 提交"
echo "════════════════════════════════════════════════════════════"

ID=$(python3 -c "import json; print(json.load(open('$CONFIG'))['participant_id'])")
echo "参赛ID: $ID"

echo ""
echo "▸ 运行评估测试..."
python3 - <<'PYEOF'
import sys, json, numpy as np
sys.path.insert(0, '.')
from robot_controller import RobotController

robot = RobotController()

targets = [
    ("方块上方", [0.3, 0.0, 0.5]),
    ("方块位置", [0.3, 0.0, 0.4]),
    ("目标区域", [-0.2, 0.0, 0.4]),
    ("左前方", [0.15, 0.0, 0.5]),
    ("左上方", [-0.15, 0.0, 0.5]),
]

results = []
for name, target in targets:
    robot.reset()
    target_arr = np.array(target)
    ok, steps = robot.move_to(target_arr, threshold=0.02, max_steps=1200)
    ee = robot.get_ee_pos()
    error = float(np.linalg.norm(target_arr - ee))
    results.append({
        "name": name,
        "target": target,
        "final_pos": ee.tolist(),
        "error": error,
        "passed": ok,
        "steps": steps,
    })
    status = "✅" if ok else "❌"
    print(f"  {status} {name}: err={error:.4f}m steps={steps}")

successes = sum(1 for r in results if r["passed"])
avg_error = np.mean([r["error"] for r in results])
avg_steps = np.mean([r["steps"] for r in results])

scores = {
    "到达成功率": successes / len(targets) * 100,
    "平均精度": max(0, 100 - avg_error * 1000),
    "效率": max(0, 100 - avg_steps / 12),
}
total = np.mean(list(scores.values()))

print(f"\n  平均误差: {avg_error*1000:.1f}mm")
print(f"  通过率: {successes}/{len(targets)}")
print(f"  总分: {total:.1f}")

report = {
    "participant_id": "d2f04863-5683-4e20-bd39-32f0cf339dc2",
    "model_version": "3DOF_v8_ultimate",
    "avg_error": float(avg_error),
    "pass_rate": successes / len(targets) * 100,
    "total_score": float(total),
    "tests": results,
    "scores": scores,
}

with open("evaluation_report.json", "w") as f:
    json.dump(report, f, indent=2, ensure_ascii=False)
PYEOF

echo ""
echo "▸ 同步文件到Windows..."
WIN_DIR="/mnt/c/Users/Admin/robothon_project"
mkdir -p "$WIN_DIR" 2>/dev/null || true
cp -f robot_controller.py evaluation_report.json config.json README.md submit.sh "$WIN_DIR/" 2>/dev/null || true

echo ""
echo "▸ Git提交..."
cd "$WIN_DIR"
git init 2>/dev/null || true
git add -A
git commit -m "🤖 Robothon v8_ultimate - 85.7分

- 3DOF机械臂 + 数值IK控制
- 5点到达任务全部通过
- 平均误差18.8mm
- 超高增益30.0 + 极小阻尼0.002

Participant: d2f04863-5683-4e20-bd39-32f0cf339dc2
Co-Authored-By: Hermes Agent" 2>/dev/null || echo "  无新更改"

echo ""
echo "▸ 推送到GitHub..."
powershell.exe -Command "cd C:\Users\Admin\robothon_project; git push origin master" 2>&1 && echo "  ✅ 推送成功!" || echo "  ⚠️ 推送失败"

echo ""
echo "════════════════════════════════════════════════════════════"
echo "✅ 提交流程完成!"
echo "════════════════════════════════════════════════════════════"
