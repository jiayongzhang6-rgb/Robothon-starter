#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FFAI Robothon 2026 - 任务评估器
自动计算稳定性、效率、成功率指标，并触发参数重调
"""

import numpy as np
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
from robot_controller import RobotController
import json
import time


@dataclass
class EvaluationResult:
    """评估结果数据类"""
    stability_score: float      # 稳定性得分 (0-100)
    efficiency_score: float     # 效率得分 (0-100)
    success_rate: float         # 成功率 (0-100)
    overall_score: float        # 综合得分
    passed: bool                # 是否达标
    details: Dict[str, Any]     # 详细信息
    recommendations: List[str]  # 调优建议


class Evaluator:
    """
    任务评估器
    自动计算各项指标并判断是否需要参数重调
    """
    
    # 达标阈值
    THRESHOLDS = {
        "stability_min": 70.0,      # 稳定性最低要求
        "efficiency_min": 60.0,     # 效率最低要求
        "success_rate_min": 80.0,   # 成功率最低要求
        "overall_min": 70.0,        # 综合得分最低要求
        "position_error_max": 0.1,  # 位置误差最大容忍（米）
        "oscillation_freq_max": 2.0,# 震荡频率最大容忍（Hz）
        "torque_anomaly_max": 0.3,  # 力矩异常最大容忍
    }
    
    def __init__(self, controller: RobotController):
        """
        初始化评估器
        
        Args:
            controller: RobotController 实例
        """
        self.controller = controller
        self.history: List[EvaluationResult] = []
        
    def evaluate_task(
        self,
        trajectory: List[Dict[str, Any]],
        target_pos: np.ndarray,
        target_tolerance: float = 0.05
    ) -> EvaluationResult:
        """
        评估一次任务执行
        
        Args:
            trajectory: 状态轨迹列表（来自 step() 的返回值）
            target_pos: 目标位置 [x, y, z]
            target_tolerance: 目标容忍误差（米）
            
        Returns:
            EvaluationResult 评估结果
        """
        if len(trajectory) < 2:
            return self._create_failed_result("轨迹数据不足")
        
        # 1. 计算稳定性指标
        stability_score, stability_details = self._calculate_stability(trajectory)
        
        # 2. 计算执行效率
        efficiency_score, efficiency_details = self._calculate_efficiency(trajectory)
        
        # 3. 计算任务成功率
        success_rate, success_details = self._calculate_success_rate(
            trajectory, target_pos, target_tolerance
        )
        
        # 4. 计算综合得分
        overall_score = self._calculate_overall_score(
            stability_score, efficiency_score, success_rate
        )
        
        # 5. 生成调优建议
        recommendations = self._generate_recommendations(
            stability_score, efficiency_score, success_rate,
            stability_details, efficiency_details, success_details
        )
        
        # 6. 判断是否达标
        passed = self._check_passed(overall_score, stability_score, 
                                     efficiency_score, success_rate)
        
        result = EvaluationResult(
            stability_score=stability_score,
            efficiency_score=efficiency_score,
            success_rate=success_rate,
            overall_score=overall_score,
            passed=passed,
            details={
                "stability": stability_details,
                "efficiency": efficiency_details,
                "success": success_details
            },
            recommendations=recommendations
        )
        
        self.history.append(result)
        return result
    
    def _calculate_stability(
        self, 
        trajectory: List[Dict[str, Any]]
    ) -> Tuple[float, Dict[str, Any]]:
        """
        计算稳定性指标
        - 震荡频率：关节角度变化的频率分析
        - 力矩异常值：控制信号的突变检测
        """
        # 提取关节角度序列
        joint_angles = np.array([s["joint_angles"] for s in trajectory])
        
        # 提取时间序列
        times = np.array([s["time"] for s in trajectory])
        
        # 1. 计算震荡频率（使用FFT）
        dt = times[1] - times[0] if len(times) > 1 else 0.01
        oscillation_freqs = []
        
        for joint_idx in range(joint_angles.shape[1]):
            signal = joint_angles[:, joint_idx]
            if len(signal) > 10:
                # FFT分析
                fft_vals = np.fft.rfft(signal - np.mean(signal))
                freqs = np.fft.rfftfreq(len(signal), d=dt)
                magnitudes = np.abs(fft_vals)
                
                # 找主要频率
                if np.max(magnitudes) > 0:
                    dominant_freq_idx = np.argmax(magnitudes[1:]) + 1
                    dominant_freq = freqs[dominant_freq_idx]
                    oscillation_freqs.append(dominant_freq)
        
        avg_oscillation_freq = np.mean(oscillation_freqs) if oscillation_freqs else 0
        
        # 2. 计算力矩异常值（关节角度变化率的突变）
        joint_vel = np.diff(joint_angles, axis=0) / dt
        if len(joint_vel) > 0:
            # 计算加速度（力矩的代理）
            joint_acc = np.diff(joint_vel, axis=0) / dt
            # 异常值：超过2倍标准差的点
            if joint_acc.size > 0:
                mean_acc = np.mean(joint_acc, axis=0)
                std_acc = np.std(joint_acc, axis=0)
                anomalies = np.sum(np.abs(joint_acc) > 2 * (std_acc + 1e-6), axis=0)
                torque_anomaly_ratio = np.mean(anomalies / len(joint_acc))
            else:
                torque_anomaly_ratio = 0
        else:
            torque_anomaly_ratio = 0
        
        # 3. 计算稳定性得分
        # 震荡频率越低越好，异常值越少越好
        freq_penalty = min(avg_oscillation_freq / self.THRESHOLDS["oscillation_freq_max"], 1.0)
        anomaly_penalty = min(torque_anomaly_ratio / self.THRESHOLDS["torque_anomaly_max"], 1.0)
        
        stability_score = max(0, 100 - freq_penalty * 50 - anomaly_penalty * 50)
        
        details = {
            "oscillation_freq_hz": round(avg_oscillation_freq, 3),
            "torque_anomaly_ratio": round(torque_anomaly_ratio, 4),
            "joint_angle_range": {
                f"joint_{i}": {
                    "min": round(float(np.min(joint_angles[:, i])), 3),
                    "max": round(float(np.max(joint_angles[:, i])), 3),
                    "std": round(float(np.std(joint_angles[:, i])), 3)
                }
                for i in range(joint_angles.shape[1])
            }
        }
        
        return stability_score, details
    
    def _calculate_efficiency(
        self, 
        trajectory: List[Dict[str, Any]]
    ) -> Tuple[float, Dict[str, Any]]:
        """
        计算执行效率
        - 路径平滑度：末端执行器轨迹的平滑程度
        - 路径长度比：实际路径与直线距离的比值
        """
        # 提取末端位置序列
        positions = np.array([s["end_effector_pos"] for s in trajectory])
        
        if len(positions) < 2:
            return 50.0, {"error": "轨迹数据不足"}
        
        # 1. 计算路径长度
        path_diffs = np.diff(positions, axis=0)
        path_lengths = np.linalg.norm(path_diffs, axis=1)
        total_path_length = np.sum(path_lengths)
        
        # 2. 计算直线距离
        straight_distance = np.linalg.norm(positions[-1] - positions[0])
        
        # 3. 路径长度比（越接近1越好）
        if straight_distance > 0:
            path_ratio = total_path_length / straight_distance
        else:
            path_ratio = 1.0
        
        # 4. 计算平滑度（使用加速度的Jerk指标）
        if len(positions) > 2:
            velocities = np.diff(positions, axis=0)
            accelerations = np.diff(velocities, axis=0)
            jerks = np.diff(accelerations, axis=0)
            
            # Jerk的RMS值，越小越平滑
            jerk_rms = np.sqrt(np.mean(np.sum(jerks**2, axis=1)))
            # 归一化
            smoothness = max(0, 100 - jerk_rms * 1000)
        else:
            smoothness = 100.0
        
        # 5. 计算效率得分
        # 路径比越接近1越好，平滑度越高越好
        path_efficiency = max(0, 100 - (path_ratio - 1) * 50)
        efficiency_score = 0.6 * smoothness + 0.4 * path_efficiency
        
        details = {
            "total_path_length": round(float(total_path_length), 4),
            "straight_distance": round(float(straight_distance), 4),
            "path_ratio": round(float(path_ratio), 3),
            "smoothness_jerk_rms": round(float(jerk_rms), 6) if len(positions) > 2 else 0,
            "efficiency_score_smoothness": round(float(smoothness), 2),
            "efficiency_score_path": round(float(path_efficiency), 2)
        }
        
        return efficiency_score, details
    
    def _calculate_success_rate(
        self,
        trajectory: List[Dict[str, Any]],
        target_pos: np.ndarray,
        tolerance: float
    ) -> Tuple[float, Dict[str, Any]]:
        """
        计算任务成功率
        - 最终位置与目标点的误差
        """
        # 获取最终位置
        final_pos = np.array(trajectory[-1]["end_effector_pos"])
        
        # 计算位置误差
        position_error = np.linalg.norm(final_pos - target_pos)
        
        # 计算各轴误差
        axis_errors = np.abs(final_pos - target_pos)
        
        # 计算成功率
        if position_error <= tolerance:
            success_rate = 100.0
        elif position_error <= tolerance * 3:
            # 线性衰减
            success_rate = 100.0 * (1 - (position_error - tolerance) / (tolerance * 2))
        else:
            success_rate = max(0, 50.0 - position_error * 100)
        
        details = {
            "target_position": target_pos.tolist(),
            "final_position": final_pos.tolist(),
            "position_error": round(float(position_error), 4),
            "axis_errors": {
                "x": round(float(axis_errors[0]), 4),
                "y": round(float(axis_errors[1]), 4),
                "z": round(float(axis_errors[2]), 4)
            },
            "tolerance": tolerance,
            "within_tolerance": position_error <= tolerance
        }
        
        return success_rate, details
    
    def _calculate_overall_score(
        self,
        stability: float,
        efficiency: float,
        success: float
    ) -> float:
        """计算综合得分"""
        # 加权平均：稳定性40%，效率30%，成功率30%
        weights = {"stability": 0.4, "efficiency": 0.3, "success": 0.3}
        overall = (
            stability * weights["stability"] +
            efficiency * weights["efficiency"] +
            success * weights["success"]
        )
        return round(overall, 2)
    
    def _generate_recommendations(
        self,
        stability: float,
        efficiency: float,
        success: float,
        stability_details: Dict,
        efficiency_details: Dict,
        success_details: Dict
    ) -> List[str]:
        """生成调优建议"""
        recommendations = []
        
        # 稳定性建议
        if stability < self.THRESHOLDS["stability_min"]:
            freq = stability_details.get("oscillation_freq_hz", 0)
            anomaly = stability_details.get("torque_anomaly_ratio", 0)
            
            if freq > self.THRESHOLDS["oscillation_freq_max"]:
                recommendations.append(
                    f"⚠️ 震荡频率过高 ({freq:.2f} Hz)，建议：增加阻尼系数或降低控制增益"
                )
            if anomaly > self.THRESHOLDS["torque_anomaly_max"]:
                recommendations.append(
                    f"⚠️ 力矩异常值过高 ({anomaly:.4f})，建议：添加力矩平滑滤波器"
                )
        
        # 效率建议
        if efficiency < self.THRESHOLDS["efficiency_min"]:
            path_ratio = efficiency_details.get("path_ratio", 1)
            if path_ratio > 1.5:
                recommendations.append(
                    f"⚠️ 路径过长 (比值: {path_ratio:.2f})，建议：优化轨迹规划算法"
                )
            if efficiency_details.get("smoothness_jerk_rms", 0) > 0.01:
                recommendations.append(
                    "⚠️ 运动不够平滑，建议：使用五次多项式插值或S曲线规划"
                )
        
        # 成功率建议
        if success < self.THRESHOLDS["success_rate_min"]:
            error = success_details.get("position_error", 0)
            tolerance = success_details.get("tolerance", 0.05)
            recommendations.append(
                f"⚠️ 位置误差过大 ({error:.4f} m)，建议：增加PID控制精度或调整目标点"
            )
        
        # 如果全部达标
        if not recommendations:
            recommendations.append("✅ 所有指标达标，当前参数表现良好")
        
        return recommendations
    
    def _check_passed(
        self,
        overall: float,
        stability: float,
        efficiency: float,
        success: float
    ) -> bool:
        """检查是否达标"""
        return (
            overall >= self.THRESHOLDS["overall_min"] and
            stability >= self.THRESHOLDS["stability_min"] and
            efficiency >= self.THRESHOLDS["efficiency_min"] and
            success >= self.THRESHOLDS["success_rate_min"]
        )
    
    def _create_failed_result(self, reason: str) -> EvaluationResult:
        """创建失败的评估结果"""
        return EvaluationResult(
            stability_score=0,
            efficiency_score=0,
            success_rate=0,
            overall_score=0,
            passed=False,
            details={"error": reason},
            recommendations=[f"❌ 评估失败: {reason}"]
        )


class AutoTuner:
    """
    自动参数调优器
    根据评估结果自动调整控制器参数
    """
    
    def __init__(self, controller: RobotController):
        self.controller = controller
        self.current_params = {
            "damping": 0.1,
            "gain_p": 1.0,
            "gain_i": 0.01,
            "gain_d": 0.1,
            "smoothing": 0.5
        }
        self.param_history = [self.current_params.copy()]
        self.best_params = self.current_params.copy()
        self.best_score = 0
        
    def auto_tune(self, result: EvaluationResult) -> Dict[str, float]:
        """
        根据评估结果自动调整参数
        
        Returns:
            调整后的参数
        """
        print("\n🔧 自动参数调优中...")
        
        details = result.details
        
        # 1. 处理震荡问题
        stability_details = details.get("stability", {})
        freq = stability_details.get("oscillation_freq_hz", 0)
        anomaly = stability_details.get("torque_anomaly_ratio", 0)
        
        if freq > 1.5:
            # 增加阻尼
            self.current_params["damping"] *= 1.2
            self.current_params["gain_d"] *= 1.1
            print(f"  → 增加阻尼: damping={self.current_params['damping']:.3f}")
        
        if anomaly > 0.2:
            # 降低增益，增加平滑
            self.current_params["gain_p"] *= 0.9
            self.current_params["smoothing"] = min(0.9, self.current_params["smoothing"] + 0.1)
            print(f"  → 增加平滑: smoothing={self.current_params['smoothing']:.3f}")
        
        # 2. 处理效率问题
        efficiency_details = details.get("efficiency", {})
        path_ratio = efficiency_details.get("path_ratio", 1)
        
        if path_ratio > 1.3:
            # 优化路径规划
            self.current_params["gain_i"] *= 0.8
            print(f"  → 调整积分增益: gain_i={self.current_params['gain_i']:.4f}")
        
        # 3. 处理精度问题
        success_details = details.get("success", {})
        error = success_details.get("position_error", 0)
        
        if error > 0.05:
            # 提高精度
            self.current_params["gain_p"] *= 1.1
            print(f"  → 提高精度: gain_p={self.current_params['gain_p']:.3f}")
        
        # 保存历史
        self.param_history.append(self.current_params.copy())
        
        # 更新最佳参数
        if result.overall_score > self.best_score:
            self.best_score = result.overall_score
            self.best_params = self.current_params.copy()
            print(f"  ✓ 新的最佳得分: {self.best_score:.2f}")
        
        return self.current_params
    
    def get_best_params(self) -> Dict[str, float]:
        """获取历史最佳参数"""
        return self.best_params.copy()
    
    def reset_to_best(self):
        """重置到最佳参数"""
        self.current_params = self.best_params.copy()
        print(f"  ↩️ 重置到最佳参数: {self.best_params}")


def run_evaluation_loop(
    target_pos: np.ndarray,
    max_iterations: int = 10,
    verbose: bool = True
) -> Tuple[bool, Dict[str, Any]]:
    """
    运行评估-调优循环
    
    Args:
        target_pos: 目标位置
        max_iterations: 最大迭代次数
        verbose: 是否打印详细信息
        
    Returns:
        (是否成功, 最终结果)
    """
    print("\n" + "="*60)
    print("🚀 FFAI Robothon 2026 - 评估与调优循环")
    print("="*60)
    
    # 初始化
    controller = RobotController()
    evaluator = Evaluator(controller)
    tuner = AutoTuner(controller)
    
    final_result = None
    
    for iteration in range(max_iterations):
        print(f"\n{'─'*60}")
        print(f"📊 迭代 {iteration + 1}/{max_iterations}")
        print(f"{'─'*60}")
        
        # 重置环境
        state = controller.reset()
        
        # 运行仿真（使用当前参数）
        trajectory = []
        action_scale = tuner.current_params["gain_p"]
        
        for step in range(100):
            # 简单的P控制
            current_pos = np.array(state["end_effector_pos"])
            error = target_pos - current_pos
            
            # 计算动作（简化版，使用2个关节）
            error_2d = error[:2]  # 只使用xy平面
            action = np.clip(error_2d * action_scale, -1, 1)
            
            state = controller.step(action)
            trajectory.append(state)
        
        # 评估
        result = evaluator.evaluate_task(trajectory, target_pos)
        final_result = result
        
        if verbose:
            print(f"\n📈 评估结果:")
            print(f"  稳定性: {result.stability_score:.2f}/100")
            print(f"  效率:   {result.efficiency_score:.2f}/100")
            print(f"  成功率: {result.success_rate:.2f}/100")
            print(f"  综合:   {result.overall_score:.2f}/100")
            print(f"  {'✅ 达标' if result.passed else '❌ 未达标'}")
        
        # 打印建议
        if result.recommendations:
            print(f"\n💡 建议:")
            for rec in result.recommendations:
                print(f"  {rec}")
        
        # 检查是否达标
        if result.passed:
            print(f"\n🎉 在第 {iteration + 1} 次迭代达标！")
            break
        
        # 参数调优
        if iteration < max_iterations - 1:
            new_params = tuner.auto_tune(result)
            if verbose:
                print(f"\n📋 当前参数:")
                for k, v in new_params.items():
                    print(f"  {k}: {v:.4f}")
    
    # 最终报告
    print("\n" + "="*60)
    print("📊 最终报告")
    print("="*60)
    
    if final_result and final_result.passed:
        print("✅ 任务达标！")
    else:
        print("❌ 未达标，建议继续优化")
    
    print(f"\n🏆 最佳参数:")
    for k, v in tuner.get_best_params().items():
        print(f"  {k}: {v:.4f}")
    
    print(f"\n📈 历史最佳得分: {tuner.best_score:.2f}")
    
    return final_result.passed if final_result else False, {
        "iterations": len(evaluator.history),
        "best_params": tuner.get_best_params(),
        "final_result": final_result
    }


# 主函数
if __name__ == "__main__":
    print("FFAI Robothon 2026 - Evaluator 测试")
    
    # 定义目标点
    target = np.array([0.2, 0.0, 0.8])
    
    # 运行评估循环
    success, report = run_evaluation_loop(
        target_pos=target,
        max_iterations=5,
        verbose=True
    )
    
    print("\n" + "="*60)
    print("✅ Evaluator 测试完成！")
    print("="*60)
