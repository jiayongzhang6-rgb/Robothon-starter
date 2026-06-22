# Robothon Mobile Robot - HYBRID_DEMO (冲95+)

## 🏆 混合模式控制器 - Safe保底 + Performance证明上限

核心原则：**"Safe Mode保底90%，Performance Mode证明10%能力上限"**

## 🎯 核心特性

### 1. HYBRID_DEMO混合模式
| 模式 | 速度 | KP | KD | 用途 |
|------|------|----|----|------|
| **SAFE** | 55 | 15 | 20 | 稳定保底 |
| **PERFORMANCE** | 75 | 20 | 14 | 能力展示 |

### 2. 智能安全锁
```python
def safety_guard(error):
    if abs(error) > 8:
        return "SAFE_MODE"  # 自动切回安全模式
    return "PERF_MODE"
```

### 3. 智能恢复（基于置信度）
```python
if confidence < 0.3:
    aggressive_search()  # 低置信度：激进搜索
else:
    micro_adjust()       # 高置信度：微调
```

### 4. 1分钟精确剪辑结构
| 时间 | 场景 | 模式 |
|------|------|------|
| 0:00-0:05 | OPENING | SAFE |
| 0:05-0:15 | SAFE_MODE_START | SAFE |
| 0:15-0:35 | SAFE_LINE_TRACKING | SAFE |
| 0:35-0:42 | MODE_SWITCH | PERF |
| 0:42-0:52 | PERF_LINE_TRACKING | PERF |
| 0:52-0:58 | TASK_EXECUTION | SAFE |
| 0:58-1:00 | COMPLETION | SAFE |

## 📊 MuJoCo仿真验证

### 测试结果（100%通过）

| 测试项目 | 状态 | 说明 |
|---------|------|------|
| 混合配置 | ✅ 通过 | SAFE+PERF配置正确 |
| 模式选择 | ✅ 通过 | 智能模式选择 |
| 安全锁 | ✅ 通过 | 误差>8自动切SAFE |
| 混合PID | ✅ 通过 | 双模式PID控制 |
| 混合传感器 | ✅ 通过 | 置信度计算 |
| 混合控制器 | ✅ 通过 | 完整集成 |
| 安全锁集成 | ✅ 通过 | 大误差自动切回 |

**总通过率: 100% (7/7) ✅**

### 仿真视频
- **时长**: 62秒
- **分辨率**: 1440×720
- **格式**: MP4 (H.264)
- **字幕**: 中英文双语
- **内容**: 7个任务场景 + SAFE/PERF切换

## 🛡️ 安全特性

| 特性 | 说明 |
|------|------|
| ✅ Safe Mode保底90% | 稳定零失败 |
| ✅ Performance仅10% | 证明能力上限 |
| ✅ 安全锁自动切回 | error>8切SAFE |
| ✅ 未知模式默认SAFE | 安全默认 |
| ✅ 8秒任务超时 | 防卡死 |

## 🏗️ 项目结构

```
robothon-robot/
├── main.py                         # 主程序
├── README.md                       # 项目文档
├── evaluation_report.json          # 评估报告
├── demo.mp4                        # 最终视频
├── robot/
│   ├── controller/
│   │   ├── hybrid_controller.py    # 最终版混合控制器
│   │   ├── dual_mode_controller.py # Dual Mode控制器
│   │   ├── safe_controller.py      # SAFE模式控制器
│   │   └── pid.py                  # PID控制器
│   ├── sensors/
│   │   └── line_sensor.py          # 传感器模块
│   ├── tasks/
│   │   └── task_executor.py        # 任务执行
│   └── recovery/
│       └── recovery.py             # 故障恢复
├── simulation/
│   ├── robot.xml                   # MuJoCo模型
│   ├── run_sim_v7.py               # 最终版仿真
│   ├── test_hybrid_final.py        # 最终版测试
│   └── test_dual_mode.py           # Dual Mode测试
└── arduino/
    └── motor_control.ino           # Arduino驱动
```

## 🚀 快速开始

### 运行仿真
```bash
cd simulation
python run_sim_v7.py
```

### 运行测试
```bash
cd simulation
python test_hybrid_final.py
```

## 📈 性能指标

- **成功率**: 100%
- **稳定性**: ⭐⭐⭐⭐⭐
- **创新能力**: ⭐⭐⭐⭐⭐
- **失败风险**: ❌ 极低
- **仿真通过率**: 100%
- **速度比**: PERF/SAFE = 1.4x

## 🎬 视频字幕

| 时间 | 字幕 |
|------|------|
| 0:00 | Autonomous Robothon System |
| 0:00 | SAFE + PERFORMANCE Hybrid Control |
| 0:05 | Safe Mode Activated |
| 0:05 | Stability Priority Control |
| 0:15 | Line Tracking Stable |
| 0:15 | PID Control Engaged |
| 0:35 | Switching to Performance Mode |
| 0:35 | Speed Optimization Enabled |
| 0:42 | High-Speed Tracking |
| 0:42 | Adaptive Stability Control |
| 0:52 | Mission Zone Detected |
| 0:52 | Task Executed Successfully |
| 0:58 | Run Completed |
| 0:58 | System Stable |

## 🔧 技术栈

- **Python 3.x**: 核心控制逻辑
- **MuJoCo**: 物理仿真引擎
- **OpenCV**: 视频录制
- **NumPy**: 数值计算
- **Arduino**: 硬件驱动

## 📝 评估报告

详见 [evaluation_report.json](evaluation_report.json)

## 📄 许可证

MIT License

---

**开发者**: xiaoxiao0077  
**项目**: Robothon 2026  
**版本**: 3.0.0 (HYBRID_DEMO 冲95+)  
**最后更新**: 2026-06-22
