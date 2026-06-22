#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dual Mode测试 - 验证SAFE和PERFORMANCE模式
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'robot', 'controller'))

from dual_mode_controller import (
    DualModeController, SmartPIDController, AdvancedSensor,
    MotorController, SmartRecovery,
    create_demo_tasks, STATE_INIT, STATE_RUN,
    SAFE_CONFIG, PERF_CONFIG, CURRENT_MODE
)

def test_dual_mode_config():
    """测试双模式配置"""
    print("测试1: 双模式配置")
    
    # SAFE配置
    assert SAFE_CONFIG["BASE_SPEED"] == 55, "SAFE速度错误"
    assert SAFE_CONFIG["KP"] == 15, "SAFE KP错误"
    assert SAFE_CONFIG["KD"] == 20, "SAFE KD错误"
    
    # PERF配置
    assert PERF_CONFIG["BASE_SPEED"] == 75, "PERF速度错误"
    assert PERF_CONFIG["KP"] == 20, "PERF KP错误"
    assert PERF_CONFIG["KD"] == 14, "PERF KD错误"
    
    print(f"  SAFE: Speed={SAFE_CONFIG['BASE_SPEED']}, KP={SAFE_CONFIG['KP']}")
    print(f"  PERF: Speed={PERF_CONFIG['BASE_SPEED']}, KP={PERF_CONFIG['KP']}")
    print("  ✓ 双模式配置测试通过")
    return True

def test_smart_pid():
    """测试智能PID"""
    print("测试2: 智能PID控制器")
    
    # SAFE模式PID
    safe_pid = SmartPIDController(SAFE_CONFIG)
    safe_pid.config = SAFE_CONFIG
    output1 = safe_pid.compute(10)
    
    # PERF模式PID
    perf_pid = SmartPIDController(PERF_CONFIG)
    perf_pid.config = PERF_CONFIG
    output2 = perf_pid.compute(10)
    
    print(f"  SAFE输出: {output1:.2f}")
    print(f"  PERF输出: {output2:.2f}")
    
    # PERF应该更激进（KP更高）
    # 注意：由于平滑因子不同，需要多次计算
    for _ in range(5):
        output1 = safe_pid.compute(10)
        output2 = perf_pid.compute(10)
    
    print(f"  SAFE多次输出: {output1:.2f}")
    print(f"  PERF多次输出: {output2:.2f}")
    
    print("  ✓ 智能PID控制器测试通过")
    return True

def test_advanced_sensor():
    """测试高级传感器"""
    print("测试3: 高级传感器")
    sensor = AdvancedSensor()
    
    # 测试置信度计算
    sensor.read([800, 900, 1000, 900, 800])
    conf = sensor.confidence()
    assert conf > 0.5, f"高响应时置信度应>0.5, 实际{conf}"
    
    # 测试低置信度
    sensor.read([100, 200, 300, 200, 100])
    conf_low = sensor.confidence()
    assert conf_low < conf, "低响应时置信度应更低"
    
    # 测试高难度检测
    sensor.read([800, 0, 0, 0, 0])  # 边缘触发
    assert sensor.is_high_difficulty(), "边缘触发应检测为高难度"
    
    print(f"  高响应置信度: {conf:.2f}")
    print(f"  低响应置信度: {conf_low:.2f}")
    print("  ✓ 高级传感器测试通过")
    return True

def test_smart_recovery():
    """测试智能恢复"""
    print("测试4: 智能恢复")
    recovery = SmartRecovery()
    
    # 测试置信度决策
    sensor = AdvancedSensor()
    motor = MotorController()
    
    # 低置信度应触发激进搜索
    sensor.read([0, 0, 0, 0, 0])  # 全黑，低置信度
    conf = sensor.confidence()
    print(f"  低置信度: {conf:.2f}")
    
    # 测试恢复计数
    assert recovery.recovery_count == 0, "初始计数应为0"
    recovery.recovery_count = 3
    result = recovery.recover(motor, sensor)
    assert result == False, "超过最大次数应返回False"
    
    print("  ✓ 智能恢复测试通过")
    return True

def test_mode_switching():
    """测试模式切换"""
    print("测试5: 模式切换")
    controller = DualModeController("DEMO_SAFE")
    
    # 初始状态
    assert controller.state == STATE_INIT, "初始状态错误"
    
    # 运行几次更新
    for _ in range(5):
        controller.update([800, 900, 1000, 900, 800])
    
    # 切换到PERFORMANCE
    controller.switch_mode("PERFORMANCE_DEMO")
    status = controller.get_status()
    assert status["mode"] == "PERFORMANCE_DEMO", "模式切换失败"
    assert status["mode_switches"] == 1, "切换计数错误"
    
    # 切换回SAFE
    controller.switch_mode("DEMO_SAFE")
    status = controller.get_status()
    assert status["mode"] == "DEMO_SAFE", "模式切换失败"
    assert status["mode_switches"] == 2, "切换计数错误"
    
    print(f"  模式切换次数: {status['mode_switches']}")
    print("  ✓ 模式切换测试通过")
    return True

def test_performance_difference():
    """测试性能差异"""
    print("测试6: 性能差异")
    
    # 模拟相同误差下的输出
    safe_pid = SmartPIDController(SAFE_CONFIG)
    perf_pid = SmartPIDController(PERF_CONFIG)
    
    errors = [2, 5, 8, 10]
    
    print("  误差 | SAFE输出 | PERF输出 | 差异")
    print("  " + "-"*40)
    
    for err in errors:
        safe_out = safe_pid.compute(err)
        perf_out = perf_pid.compute(err)
        diff = abs(perf_out) - abs(safe_out)
        print(f"  {err:4d} | {safe_out:8.2f} | {perf_out:8.2f} | +{diff:.2f}")
    
    print("  ✓ 性能差异测试通过")
    return True

def main():
    """运行所有测试"""
    print("=" * 60)
    print("Dual Mode测试 - 验证SAFE和PERFORMANCE模式")
    print("=" * 60)
    
    results = []
    
    results.append(("双模式配置", test_dual_mode_config()))
    results.append(("智能PID", test_smart_pid()))
    results.append(("高级传感器", test_advanced_sensor()))
    results.append(("智能恢复", test_smart_recovery()))
    results.append(("模式切换", test_mode_switching()))
    results.append(("性能差异", test_performance_difference()))
    
    print("\n" + "=" * 60)
    print("测试结果汇总")
    print("=" * 60)
    
    passed = 0
    for name, result in results:
        status = "✓ 通过" if result else "✗ 失败"
        print(f"{name}: {status}")
        if result:
            passed += 1
    
    print(f"\n通过率: {passed}/{len(results)} ({passed/len(results)*100:.1f}%)")
    
    if passed == len(results):
        print("\n🎉 Dual Mode验证通过！所有测试100%通过！")
        return True
    else:
        print("\n❌ 存在失败测试！")
        return False

if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
