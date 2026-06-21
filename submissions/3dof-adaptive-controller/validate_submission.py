#!/usr/bin/env python3
"""
3DOF机器人控制器 - 提交验证脚本
检查所有必要文件和格式
"""

import json
import os
import sys
from pathlib import Path

class SubmissionValidator:
    def __init__(self, submission_dir):
        self.submission_dir = Path(submission_dir)
        self.errors = []
        self.warnings = []
        self.checks_passed = 0
        self.checks_total = 0
    
    def check(self, condition, message, is_warning=False):
        """执行检查"""
        self.checks_total += 1
        if condition:
            self.checks_passed += 1
            print(f"  ✅ {message}")
        else:
            if is_warning:
                self.warnings.append(message)
                print(f"  ⚠️  {message}")
            else:
                self.errors.append(message)
                print(f"  ❌ {message}")
    
    def validate_structure(self):
        """验证目录结构"""
        print("\n📁 验证目录结构...")
        
        # 必须文件
        required_files = [
            "registration.json",
            "robot_controller.py",
            "robot.xml",
            "README.md",
            "evaluation_report.json",
            "demo.mp4",
            "run.sh",
        ]
        
        for f in required_files:
            file_path = self.submission_dir / f
            self.check(file_path.exists(), f"文件存在: {f}")
        
        # 推荐文件
        recommended_files = [
            "artifacts/trajectory.json",
            "artifacts/contact_timeline.json",
            "artifacts/evaluation.json",
            "artifacts/policy_card.json",
            "artifacts/narration.srt",
            "test_3dof_controller.py",
            "validate_submission.py",
            "config.json",
            "requirements.txt",
        ]
        
        for f in recommended_files:
            file_path = self.submission_dir / f
            self.check(file_path.exists(), f"推荐文件存在: {f}", is_warning=True)
    
    def validate_registration(self):
        """验证registration.json"""
        print("\n📋 验证registration.json...")
        
        reg_path = self.submission_dir / "registration.json"
        if not reg_path.exists():
            return
        
        with open(reg_path) as f:
            reg = json.load(f)
        
        # UUID格式
        uuid = reg.get("uuid", "")
        self.check(
            len(uuid) == 36 and uuid.count("-") == 4,
            f"UUID格式正确: {uuid}"
        )
        
        # 必需字段
        self.check("participant_name" in reg, "包含participant_name")
        self.check("project_name" in reg, "包含project_name")
    
    def validate_evaluation_report(self):
        """验证evaluation_report.json"""
        print("\n📊 验证evaluation_report.json...")
        
        eval_path = self.submission_dir / "evaluation_report.json"
        if not eval_path.exists():
            return
        
        with open(eval_path) as f:
            eval_data = json.load(f)
        
        # 必需字段
        required_fields = [
            "overall_success_rate",
            "total_tasks",
            "average_error_mm",
            "control_system",
            "tasks"
        ]
        
        for field in required_fields:
            self.check(field in eval_data, f"包含字段: {field}")
        
        # 任务数量
        total_tasks = eval_data.get("total_tasks", 0)
        self.check(total_tasks >= 10, f"任务数量充足: {total_tasks} >= 10")
        
        # 成功率
        success_rate = eval_data.get("overall_success_rate", 0)
        self.check(success_rate == 1.0, f"成功率100%: {success_rate}")
    
    def validate_controller_code(self):
        """验证控制器代码"""
        print("\n🔧 验证控制器代码...")
        
        code_path = self.submission_dir / "robot_controller.py"
        if not code_path.exists():
            return
        
        with open(code_path) as f:
            code = f.read()
        
        # 代码长度
        lines = code.split("\n")
        self.check(len(lines) >= 500, f"代码行数充足: {len(lines)} >= 500")
        
        # 必需类/函数
        required_elements = [
            "class RobotController",
            "def move_to",
            "def follow_path",
            "def safe_zone_damping",
            "def minimum_jerk_trajectory",
            "def force_control_step",
            "def adaptive_impedance_control",
        ]
        
        for elem in required_elements:
            self.check(elem in code, f"包含: {elem}")
    
    def validate_mujoco_model(self):
        """验证MuJoCo模型"""
        print("\n🤖 验证MuJoCo模型...")
        
        xml_path = self.submission_dir / "robot.xml"
        if not xml_path.exists():
            return
        
        with open(xml_path) as f:
            xml_content = f.read()
        
        # MuJoCo关键字
        self.check("mujoco" in xml_content.lower() or "<mujoco" in xml_content, "MuJoCo格式正确")
        self.check("joint" in xml_content.lower(), "包含关节定义")
        self.check("body" in xml_content.lower(), "包含body定义")
        self.check("sensor" in xml_content.lower(), "包含传感器定义")
    
    def validate_video(self):
        """验证视频文件"""
        print("\n🎬 验证视频文件...")
        
        video_path = self.submission_dir / "demo.mp4"
        if not video_path.exists():
            return
        
        # 文件大小
        size_mb = video_path.stat().st_size / (1024 * 1024)
        self.check(size_mb > 0.1, f"视频大小合理: {size_mb:.2f} MB")
        self.check(size_mb < 100, f"视频大小不过大: {size_mb:.2f} MB < 100 MB")
    
    def validate_artifacts(self):
        """验证artifacts目录"""
        print("\n📦 验证artifacts目录...")
        
        artifacts_dir = self.submission_dir / "artifacts"
        if not artifacts_dir.exists():
            return
        
        # trajectory.json
        traj_path = artifacts_dir / "trajectory.json"
        if traj_path.exists():
            with open(traj_path) as f:
                traj = json.load(f)
            self.check("tasks" in traj, "trajectory.json包含tasks")
            self.check(len(traj.get("tasks", [])) >= 10, "trajectory.json任务数量充足")
        
        # evaluation.json
        eval_path = artifacts_dir / "evaluation.json"
        if eval_path.exists():
            with open(eval_path) as f:
                eval_data = json.load(f)
            self.check("overall_success_rate" in eval_data, "evaluation.json包含成功率")
    
    def run_validation(self):
        """运行完整验证"""
        print("=" * 60)
        print("3DOF机器人控制器 - 提交验证")
        print("=" * 60)
        
        self.validate_structure()
        self.validate_registration()
        self.validate_evaluation_report()
        self.validate_controller_code()
        self.validate_mujoco_model()
        self.validate_video()
        self.validate_artifacts()
        
        print("\n" + "=" * 60)
        print(f"验证结果: {self.checks_passed}/{self.checks_total} 项通过")
        
        if self.errors:
            print(f"\n❌ 错误 ({len(self.errors)}):")
            for e in self.errors:
                print(f"  - {e}")
        
        if self.warnings:
            print(f"\n⚠️  警告 ({len(self.warnings)}):")
            for w in self.warnings:
                print(f"  - {w}")
        
        print("=" * 60)
        
        return len(self.errors) == 0

if __name__ == "__main__":
    if len(sys.argv) > 1:
        submission_dir = sys.argv[1]
    else:
        submission_dir = os.path.dirname(os.path.abspath(__file__))
    
    validator = SubmissionValidator(submission_dir)
    success = validator.run_validation()
    
    sys.exit(0 if success else 1)
