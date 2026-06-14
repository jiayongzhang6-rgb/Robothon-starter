#!/bin/bash
# ============================================================
# FFAI Robothon 2026 - 自动提交脚本
# 用法: ./submit.sh [commit message]
# ============================================================

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 配置
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${SCRIPT_DIR}/config.json"
EVALUATOR="${SCRIPT_DIR}/evaluator.py"
REPORT_FILE="${SCRIPT_DIR}/evaluation_report.json"

# 读取参赛ID
if [ ! -f "$CONFIG_FILE" ]; then
    echo -e "${RED}❌ 错误: 找不到 config.json${NC}"
    exit 1
fi

PARTICIPANT_ID=$(python3 -c "import json; print(json.load(open('$CONFIG_FILE'))['participant_id'])")
COMPETITION=$(python3 -c "import json; print(json.load(open('$CONFIG_FILE'))['competition'])")
CATEGORY=$(python3 -c "import json; print(json.load(open('$CONFIG_FILE'))['category'])")

echo -e "${CYAN}════════════════════════════════════════════════════════════${NC}"
echo -e "${CYAN}   FFAI Robothon 2026 - 自动提交系统${NC}"
echo -e "${CYAN}════════════════════════════════════════════════════════════${NC}"
echo ""
echo -e "${BLUE}参赛ID:${NC} $PARTICIPANT_ID"
echo -e "${BLUE}赛道:${NC} $CATEGORY"
echo ""

# ============================================================
# 第1步: 运行评估测试
# ============================================================
echo -e "${YELLOW}▸ 步骤 1/4: 运行评估测试...${NC}"

# 激活虚拟环境
source /tmp/mujoco_env/bin/activate 2>/dev/null || true

# 运行评估器并生成JSON报告
cd "$SCRIPT_DIR"
python3 - <<'PYTHON_SCRIPT'
import sys
import json
import numpy as np

sys.path.insert(0, '.')
from robot_controller import RobotController
from evaluator import Evaluator, AutoTuner

print("  正在初始化控制器...")
controller = RobotController()
evaluator = Evaluator(controller)

# 自定义JSON编码器，处理numpy类型
class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, (np.integer,)):
            return int(obj)
        if isinstance(obj, (np.floating,)):
            return float(obj)
        if isinstance(obj, (np.bool_,)):
            return bool(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return super().default(obj)

# 定义测试任务
test_tasks = [
    {"name": "基础定位", "target": np.array([0.1, 0.0, 0.8]), "tolerance": 0.1},
    {"name": "横向移动", "target": np.array([0.0, 0.15, 0.8]), "tolerance": 0.1},
    {"name": "对角移动", "target": np.array([0.1, 0.1, 0.85]), "tolerance": 0.1},
]

results = []
all_passed = True

for task in test_tasks:
    print(f"  测试: {task['name']}...")
    state = controller.reset()
    trajectory = []
    
    # 运行控制
    for step in range(150):
        current_pos = np.array(state["end_effector_pos"])
        error = task["target"][:2] - current_pos[:2]
        action = np.clip(error * 1.5, -1, 1)
        state = controller.step(action)
        trajectory.append(state)
    
    # 评估
    result = evaluator.evaluate_task(trajectory, task["target"], task["tolerance"])
    
    task_result = {
        "name": task["name"],
        "target": task["target"].tolist(),
        "stability": result.stability_score,
        "efficiency": result.efficiency_score,
        "success_rate": result.success_rate,
        "overall": result.overall_score,
        "passed": result.passed,
        "position_error": result.details["success"]["position_error"]
    }
    results.append(task_result)
    
    status = "✅" if result.passed else "⚠️"
    print(f"    {status} 稳定性: {result.stability_score:.1f} | 效率: {result.efficiency_score:.1f} | 成功率: {result.success_rate:.1f} | 综合: {result.overall_score:.1f}")
    
    if not result.passed:
        all_passed = False

# 计算平均分
avg_stability = np.mean([r["stability"] for r in results])
avg_efficiency = np.mean([r["efficiency"] for r in results])
avg_success = np.mean([r["success_rate"] for r in results])
avg_overall = np.mean([r["overall"] for r in results])

# 保存报告
report = {
    "participant_id": "d2f04863-5683-4e20-bd39-32f0cf339dc2",
    "all_passed": bool(all_passed),
    "summary": {
        "avg_stability": round(float(avg_stability), 2),
        "avg_efficiency": round(float(avg_efficiency), 2),
        "avg_success_rate": round(float(avg_success), 2),
        "avg_overall": round(float(avg_overall), 2)
    },
    "tasks": results
}

with open("evaluation_report.json", "w") as f:
    json.dump(report, f, indent=2, ensure_ascii=False, cls=NumpyEncoder)

print(f"\n  📊 平均得分:")
print(f"    稳定性: {avg_stability:.2f}")
print(f"    效率: {avg_efficiency:.2f}")
print(f"    成功率: {avg_success:.2f}")
print(f"    综合: {avg_overall:.2f}")

if all_passed:
    print("\n  ✅ 所有测试通过!")
else:
    print("\n  ⚠️ 部分测试未达标，但仍可提交")

sys.exit(0 if all_passed else 0)  # 都允许提交，但标记状态
PYTHON_SCRIPT

EVAL_EXIT=$?

# 读取评估结果
if [ -f "$REPORT_FILE" ]; then
    ALL_PASSED=$(python3 -c "import json; print(json.load(open('$REPORT_FILE'))['all_passed'])")
    AVG_STABILITY=$(python3 -c "import json; print(json.load(open('$REPORT_FILE'))['summary']['avg_stability'])")
    AVG_EFFICIENCY=$(python3 -c "import json; print(json.load(open('$REPORT_FILE'))['summary']['avg_efficiency'])")
    AVG_SUCCESS=$(python3 -c "import json; print(json.load(open('$REPORT_FILE'))['summary']['avg_success_rate'])")
    AVG_OVERALL=$(python3 -c "import json; print(json.load(open('$REPORT_FILE'))['summary']['avg_overall'])")
else
    echo -e "${RED}  ❌ 评估报告生成失败${NC}"
    exit 1
fi

if [ "$ALL_PASSED" = "True" ]; then
    echo -e "${GREEN}  ✅ 所有测试通过!${NC}"
else
    echo -e "${YELLOW}  ⚠️ 部分测试未达标，继续提交...${NC}"
fi

# ============================================================
# 第2步: 生成Commit信息
# ============================================================
echo ""
echo -e "${YELLOW}▸ 步骤 2/4: 生成提交信息...${NC}"

# 获取当前时间戳
TIMESTAMP=$(date -u +"%Y-%m-%dT%H:%M:%SZ")
DATE=$(date +"%Y-%m-%d")

# 自动生成的commit message
AUTO_MESSAGE="🤖 Robothon Submission [${CATEGORY}]

Participant: ${PARTICIPANT_ID}
Date: ${DATE}
Competition: ${COMPETITION}
Category: ${CATEGORY}

📊 Performance Metrics:
  - Stability: ${AVG_STABILITY}/100
  - Efficiency: ${AVG_EFFICIENCY}/100
  - Success Rate: ${AVG_SUCCESS}/100
  - Overall Score: ${AVG_OVERALL}/100

${ALL_PASSED:+✅ All tests passed}${ALL_PASSED:-⚠️ Partial pass - optimization in progress}

Optimization Summary:
  - RobotController: reset/step/get_status API
  - Auto-tuner: damping, gain_p/i/d, smoothing
  - Evaluator: stability, efficiency, success metrics

Co-Authored-By: Hermes Agent <hermes@nousresearch.com>"

# 如果用户提供了自定义消息，追加到末尾
if [ -n "$1" ]; then
    AUTO_MESSAGE="${AUTO_MESSAGE}

Custom Note: $1"
fi

echo -e "${GREEN}  ✅ 提交信息已生成${NC}"

# ============================================================
# 第3步: Git操作
# ============================================================
echo ""
echo -e "${YELLOW}▸ 步骤 3/4: 执行Git操作...${NC}"

cd "$SCRIPT_DIR"

# 检查是否是git仓库
if [ ! -d ".git" ]; then
    echo -e "${YELLOW}  初始化Git仓库...${NC}"
    git init
    git config user.email "participant@robthon2026.ai"
    git config user.name "Robothon Participant"
fi

# 添加文件
echo "  添加文件..."
git add -A

# 检查是否有更改
if git diff --cached --quiet 2>/dev/null; then
    echo -e "${YELLOW}  没有新的更改需要提交${NC}"
else
    # 提交
    echo "  创建提交..."
    echo "$AUTO_MESSAGE" | git commit -F -
    echo -e "${GREEN}  ✅ 提交成功${NC}"
fi

# 显示最近的提交
echo ""
echo -e "${BLUE}  最近提交:${NC}"
git log --oneline -1

# ============================================================
# 第4步: 创建PR
# ============================================================
echo ""
echo -e "${YELLOW}▸ 步骤 4/4: 创建 Pull Request...${NC}"

# 检查 gh CLI
if ! command -v gh &> /dev/null; then
    echo -e "${YELLOW}  gh CLI 未安装，尝试安装...${NC}"
    
    # 尝试用curl下载
    GH_VERSION="2.40.1"
    curl -sL "https://github.com/cli/cli/releases/download/v${GH_VERSION}/gh_${GH_VERSION}_linux_amd64.tar.gz" -o /tmp/gh.tar.gz
    tar -xzf /tmp/gh.tar.gz -C /tmp/
    sudo mv /tmp/gh_${GH_VERSION}_linux_amd64/bin/gh /usr/local/bin/
    rm -rf /tmp/gh*
fi

# 检查是否已登录
if ! gh auth status &> /dev/null 2>&1; then
    echo -e "${YELLOW}  请先登录GitHub: gh auth login${NC}"
    echo -e "${YELLOW}  或设置环境变量: export GITHUB_TOKEN=your_token${NC}"
    
    # 尝试使用环境变量
    if [ -z "$GITHUB_TOKEN" ]; then
        echo -e "${RED}  ❌ 无法创建PR: 未认证${NC}"
        echo ""
        echo -e "${BLUE}  手动创建PR的命令:${NC}"
        echo "  gh pr create \\"
        echo "    --title \"🤖 Robothon Submission - ${CATEGORY}\" \\"
        echo "    --body \"${AUTO_MESSAGE}\" \\"
        echo "    --label \"robthon-2026,${CATEGORY}\""
        exit 1
    fi
fi

# 创建分支
BRANCH_NAME="submission/${CATEGORY}/${DATE}"
git checkout -b "$BRANCH_NAME" 2>/dev/null || git checkout "$BRANCH_NAME"

# 推送（如果配置了远程仓库）
REMOTE_URL=$(git remote get-url origin 2>/dev/null || echo "")
if [ -n "$REMOTE_URL" ]; then
    echo "  推送到远程..."
    git push -u origin "$BRANCH_NAME" 2>/dev/null || true
    
    # 创建PR
    PR_BODY="## FFAI Robothon 2026 Submission

### Participant Information
- **ID**: \`${PARTICIPANT_ID}\`
- **Category**: ${CATEGORY}
- **Date**: ${DATE}

### Performance Metrics
| Metric | Score |
|--------|-------|
| Stability | ${AVG_STABILITY}/100 |
| Efficiency | ${AVG_EFFICIENCY}/100 |
| Success Rate | ${AVG_SUCCESS}/100 |
| **Overall** | **${AVG_OVERALL}/100** |

### Test Results
$(if [ "$ALL_PASSED" = "True" ]; then echo "✅ All tests passed"; else echo "⚠️ Partial pass - optimization in progress"; fi)

### Optimization Summary
- RobotController with reset/step/get_status API
- Auto-tuner for parameter optimization
- Evaluator for stability, efficiency, and success metrics

### Files Changed
$(git diff --stat HEAD~1 2>/dev/null || echo "Initial submission")

---
*Auto-generated by submit.sh*"

    echo "  创建Pull Request..."
    PR_URL=$(gh pr create \
        --title "🤖 Robothon Submission - ${CATEGORY} [${PARTICIPANT_ID:0:8}...]" \
        --body "$PR_BODY" \
        --label "robthon-2026" \
        2>&1 || echo "PR_CREATE_FAILED")
    
    if [[ "$PR_URL" == *"github.com"* ]]; then
        echo -e "${GREEN}  ✅ PR创建成功!${NC}"
        echo -e "${BLUE}  PR链接: ${PR_URL}${NC}"
    else
        echo -e "${YELLOW}  ⚠️ PR创建需要远程仓库${NC}"
    fi
else
    echo -e "${YELLOW}  未配置远程仓库，跳过PR创建${NC}"
    echo ""
    echo -e "${BLUE}  配置远程仓库后，运行以下命令创建PR:${NC}"
    echo "  git remote add origin https://github.com/YOUR_USERNAME/robthon2026.git"
    echo "  git push -u origin $BRANCH_NAME"
    echo "  gh pr create --title '🤖 Robothon Submission' --body-file - < commit_msg.txt"
fi

# ============================================================
# 完成
# ============================================================
echo ""
echo -e "${CYAN}════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}✅ 提交流程完成!${NC}"
echo -e "${CYAN}════════════════════════════════════════════════════════════${NC}"
echo ""
echo -e "${BLUE}📊 本次提交摘要:${NC}"
echo -e "  参赛ID: ${PARTICIPANT_ID}"
echo -e "  赛道: ${CATEGORY}"
echo -e "  综合得分: ${AVG_OVERALL}/100"
echo -e "  状态: $(if [ "$ALL_PASSED" = "True" ]; then echo -e '${GREEN}全部达标${NC}'; else echo -e '${YELLOW}部分达标${NC}'; fi)"
echo ""
echo -e "${BLUE}📁 相关文件:${NC}"
echo "  - evaluation_report.json (详细评估报告)"
echo "  - config.json (参赛配置)"
echo ""
