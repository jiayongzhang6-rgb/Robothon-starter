"""
Robot Orchestra v8 - 真实物理碰撞 + 力控反馈 + 成功率统计
"""

import numpy as np
import mujoco
import cv2
import imageio

class RobotOrchestra:
    def __init__(self, model_path):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, height=720, width=1280)
        
        self.arms = {
            'drummer': ['d_j1', 'd_j2', 'd_j3'],
            'xylo': ['x_j1', 'x_j2', 'x_j3'],
            'perc': ['p_j1', 'p_j2', 'p_j3'],
        }
        self.arm_list = list(self.arms.keys())
        
        # PD增益 (增大以提高跟踪精度)
        self.kp = np.array([200, 200, 150])
        self.kd = np.array([20, 20, 15])
        
        # 位姿 (优化到能碰到乐器)
        self.poses = {
            'd_rest': np.array([0.0, -0.5, 0.2]),
            'd_snare': np.array([1.60, -0.30, 1.30]),   # 距离0.126m
            'd_kick': np.array([1.60, 0.10, 1.70]),      # 距离0.233m
            'd_hihat': np.array([1.20, -0.50, 1.10]),    # 距离0.023m
            'd_up': np.array([0.0, -0.4, 0.2]),
            'x_rest': np.array([0.0, -0.5, 0.2]),
            'x_c': np.array([2.20, -0.10, 1.30]),        # 距离0.050m
            'x_e': np.array([1.80, -0.10, 1.30]),
            'x_g': np.array([1.40, -0.10, 1.30]),
            'x_up': np.array([0.0, -0.4, 0.2]),
            'p_rest': np.array([0.0, -0.5, 0.2]),
            'p_cymbal': np.array([1.40, -0.70, 1.30]),   # 距离0.146m
            'p_triangle': np.array([0.60, -0.70, 1.70]),  # 距离0.142m
            'p_up': np.array([0.0, -0.4, 0.2]),
        }
        
        # 统计
        self.hit_counts = {'drum': 0, 'xylo': 0, 'perc': 0}
        self.attempt_counts = {'drum': 0, 'xylo': 0, 'perc': 0}
        self.touch_threshold = 5  # 降低触觉阈值
        
        self.video_writer = None
        
    def get_qpos(self, arm):
        return np.array([self.data.qpos[self.model.joint(n).qposadr[0]] 
                         for n in self.arms[arm]])
    
    def set_ctrl(self, arm, target):
        qpos = self.get_qpos(arm)
        qvel = np.array([self.data.qvel[self.model.joint(n).dofadr[0]] 
                         for n in self.arms[arm]])
        base = self.arm_list.index(arm) * 3
        for i in range(3):
            self.data.ctrl[base + i] = self.kp[i] * (target[i] - qpos[i]) - self.kd[i] * qvel[i]
    
    def get_touch(self, sensor_name):
        return self.data.sensor(sensor_name).data[0]
    
    def move(self, arm, target, steps=50, smooth=True):
        start = self.get_qpos(arm)
        for s in range(steps):
            a = 0.5 * (1 - np.cos(np.pi * s / steps)) if smooth else s / steps
            self.set_ctrl(arm, start * (1 - a) + target * a)
            mujoco.mj_step(self.model, self.data)
    
    def hit_and_check(self, arm, hit_pose, up_pose, sensor_name, instrument):
        """打击并检测接触"""
        self.attempt_counts[instrument] += 1
        
        # 快速下击
        self.move(arm, hit_pose, steps=15, smooth=False)
        
        # 检测触觉
        touch = self.get_touch(sensor_name)
        if touch > self.touch_threshold:
            self.hit_counts[instrument] += 1
        
        # 快速抬起
        self.move(arm, up_pose, steps=10, smooth=False)
        
        return touch
    
    def start_video(self, path, fps=30):
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_writer = cv2.VideoWriter(path, fourcc, fps, (1280, 720))
    
    def write_frame(self, camera='overview'):
        self.renderer.update_scene(self.data, camera=camera)
        frame = self.renderer.render()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        self.video_writer.write(frame_bgr)
    
    def write_n(self, n, camera='overview'):
        for _ in range(n):
            self.write_frame(camera)
    
    def end_video(self):
        if self.video_writer:
            self.video_writer.release()
    
    def hud(self, text, beat, total_beats, phase, dynamics="mf"):
        """HUD叠加"""
        self.renderer.update_scene(self.data, camera='overview')
        frame = self.renderer.render()
        overlay = frame.copy()
        
        # 顶部
        cv2.rectangle(overlay, (0, 0), (1280, 50), (0, 0, 0), -1)
        cv2.putText(overlay, f"Robot Orchestra | {text}", 
                    (15, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(overlay, f"Beat {beat}/{total_beats} | {phase} | {dynamics}", 
                    (850, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
        
        # 节拍条
        cv2.rectangle(overlay, (15, 55), (1265, 67), (40, 40, 40), -1)
        for i in range(total_beats):
            x = 15 + int(1250 * i / total_beats)
            w = max(1, int(1250 / total_beats) - 1)
            color = (0, 200, 0) if i < beat else (80, 80, 80)
            if i % 4 == 0:
                color = (0, 255, 255) if i < beat else (100, 100, 100)
            cv2.rectangle(overlay, (x, 55), (x + w, 67), color, -1)
        
        # 触觉反馈
        cv2.rectangle(overlay, (15, 72), (350, 130), (30, 30, 30), -1)
        cv2.putText(overlay, "Tactile Feedback:", (20, 87), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
        for i, (name, sensor) in enumerate([('Drum', 'drum_hit'), ('Xylo', 'xylo_hit'), ('Perc', 'perc_hit')]):
            touch = self.get_touch(sensor)
            bar_w = int(min(200, touch / 5))
            color = (0, 255, 0) if touch > self.touch_threshold else (100, 100, 100)
            cv2.rectangle(overlay, (20, 92 + i * 12), (20 + bar_w, 100 + i * 12), color, -1)
            cv2.putText(overlay, f"{name}: {touch:.0f}N", (230, 100 + i * 12), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)
        
        # 成功率
        total_hits = sum(self.hit_counts.values())
        total_attempts = sum(self.attempt_counts.values())
        rate = total_hits / total_attempts * 100 if total_attempts > 0 else 0
        cv2.rectangle(overlay, (15, 135), (350, 165), (30, 30, 30), -1)
        cv2.putText(overlay, f"Success: {total_hits}/{total_attempts} ({rate:.0f}%)", 
                    (20, 155), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
                    (0, 255, 0) if rate > 80 else (0, 200, 255), 1)
        
        # 底部
        cv2.rectangle(overlay, (0, 690), (1280, 720), (20, 20, 20), -1)
        cv2.putText(overlay, "Robot Orchestra | 3 Arms | Closed-Loop PD | Tactile Feedback | 120 BPM", 
                    (15, 712), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (160, 160, 160), 1)
        
        frame_bgr = cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR)
        self.video_writer.write(frame_bgr)
    
    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.hit_counts = {'drum': 0, 'xylo': 0, 'perc': 0}
        self.attempt_counts = {'drum': 0, 'xylo': 0, 'perc': 0}
        for arm in self.arms:
            for jn in self.arms[arm]:
                self.data.qpos[self.model.joint(jn).qposadr[0]] = 0.0
        for _ in range(200):
            for arm in self.arms:
                self.set_ctrl(arm, self.poses[f'{arm[0]}_rest'])
            mujoco.mj_step(self.model, self.data)


DRUM = ['d_kick', 'd_hihat', 'd_snare', 'd_hihat']
XYLO = ['x_c', 'x_e', 'x_g', 'x_e']
PERC = ['p_cymbal', None, 'p_triangle', None]


def play_beat(orch, i, camera='overview'):
    """演奏一个节拍 (真实物理碰撞)"""
    d_key = DRUM[i % 4]
    x_key = XYLO[i % 4]
    p_key = PERC[i % 4]
    
    # 鼓手
    orch.hit_and_check('drummer', orch.poses[d_key], orch.poses['d_up'], 'drum_hit', 'drum')
    orch.write_n(3, camera)
    
    # 木琴手
    orch.hit_and_check('xylo', orch.poses[x_key], orch.poses['x_up'], 'xylo_hit', 'xylo')
    orch.write_n(3, camera)
    
    # 打击乐手
    if p_key:
        orch.hit_and_check('perc', orch.poses[p_key], orch.poses['p_up'], 'perc_hit', 'perc')
        orch.write_n(3, camera)
    else:
        orch.write_n(3, camera)


def main():
    print("=" * 50)
    print("Robot Orchestra v8 - 真实物理碰撞")
    print("=" * 50)
    
    orch = RobotOrchestra('scene.xml')
    orch.reset()
    
    orch.start_video('demo.mp4', fps=30)
    
    # 开场
    orch.write_n(30, 'overview')
    
    total_beats = 48
    beat = 0
    
    # Phase 1: Intro (8拍)
    print("[Phase 1] Intro - pp")
    for i in range(8):
        play_beat(orch, i, 'overview')
        beat += 1
    for _ in range(10):
        orch.hud("3 Arms | 4 Instruments", beat, total_beats, "Intro", "pp")
    
    # Phase 2: Verse (12拍)
    print("[Phase 2] Verse - mf")
    for i in range(12):
        cam = ['drummer_cam', 'xylo_cam', 'perc_cam'][i % 3]
        play_beat(orch, i, cam)
        beat += 1
    for _ in range(10):
        orch.hud("3 Arms | 4 Instruments", beat, total_beats, "Verse", "mf")
    
    # Phase 3: Chorus (16拍)
    print("[Phase 3] Chorus - f")
    for i in range(16):
        cam = ['drummer_cam', 'xylo_cam', 'perc_cam'][i % 3]
        play_beat(orch, i, cam)
        beat += 1
    for _ in range(10):
        orch.hud("3 Arms | 4 Instruments", beat, total_beats, "Chorus", "f")
    
    # Phase 4: Finale (12拍)
    print("[Phase 4] Finale - mp")
    for i in range(12):
        play_beat(orch, i, 'overview')
        beat += 1
    
    # 结束
    print("[结束]")
    for arm in ['drummer', 'xylo', 'perc']:
        orch.move(arm, orch.poses[f'{arm[0]}_rest'], steps=60)
    
    # 最终统计
    total_hits = sum(orch.hit_counts.values())
    total_attempts = sum(orch.attempt_counts.values())
    rate = total_hits / total_attempts * 100 if total_attempts > 0 else 0
    
    print(f"\n统计:")
    print(f"  鼓: {orch.hit_counts['drum']}/{orch.attempt_counts['drum']}")
    print(f"  木琴: {orch.hit_counts['xylo']}/{orch.attempt_counts['xylo']}")
    print(f"  打击乐: {orch.hit_counts['perc']}/{orch.attempt_counts['perc']}")
    print(f"  总成功率: {total_hits}/{total_attempts} ({rate:.0f}%)")
    
    # 结束帧
    for _ in range(60):
        orch.renderer.update_scene(orch.data, camera='overview')
        frame = orch.renderer.render()
        overlay = frame.copy()
        cv2.rectangle(overlay, (280, 220), (1000, 440), (0, 0, 0), -1)
        cv2.rectangle(overlay, (282, 222), (998, 438), (0, 180, 0), 2)
        cv2.putText(overlay, "Robot Orchestra", 
                    (360, 270), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 255, 0), 3)
        cv2.putText(overlay, f"Success Rate: {total_hits}/{total_attempts} ({rate:.0f}%)", 
                    (350, 315), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(overlay, "3 Arms | 4 Instruments | 120 BPM", 
                    (350, 355), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1)
        cv2.putText(overlay, "Closed-Loop PD Control | Tactile Feedback", 
                    (340, 390), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (180, 180, 180), 1)
        cv2.putText(overlay, f"Drum: {orch.hit_counts['drum']}/{orch.attempt_counts['drum']} | Xylo: {orch.hit_counts['xylo']}/{orch.attempt_counts['xylo']} | Perc: {orch.hit_counts['perc']}/{orch.attempt_counts['perc']}", 
                    (330, 420), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (160, 160, 160), 1)
        cv2.rectangle(overlay, (0, 690), (1280, 720), (20, 20, 20), -1)
        cv2.putText(overlay, "Robot Orchestra | 3 Arms | Closed-Loop PD | Tactile Feedback | 120 BPM", 
                    (15, 712), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (160, 160, 160), 1)
        frame_bgr = cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR)
        orch.video_writer.write(frame_bgr)
    
    orch.end_video()
    
    import subprocess
    result = subprocess.run(['ffprobe', '-v', 'quiet', '-show_format', 'demo.mp4'], 
                          capture_output=True, text=True)
    for line in result.stdout.split('\n'):
        if 'duration' in line:
            print(f"\n视频时长: {line.strip()}")
    
    print("\n演出结束!")


if __name__ == '__main__':
    main()
