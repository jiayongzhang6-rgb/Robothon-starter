"""
Robot Orchestra v7 - 统一overview相机
"""

import numpy as np
import mujoco
import cv2

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
        
        self.poses = {
            'd_rest': np.array([0.0, -0.5, 0.2]),
            'd_snare': np.array([-0.15, -0.35, 0.45]),
            'd_kick': np.array([0.05, -0.45, 0.4]),
            'd_hihat': np.array([-0.3, -0.3, 0.35]),
            'd_up': np.array([0.0, -0.3, 0.0]),
            'x_rest': np.array([0.0, -0.5, 0.2]),
            'x_c': np.array([0.25, -0.3, 0.4]),
            'x_e': np.array([0.05, -0.35, 0.35]),
            'x_g': np.array([-0.2, -0.3, 0.4]),
            'x_up': np.array([0.0, -0.3, 0.0]),
            'p_rest': np.array([0.0, -0.5, 0.2]),
            'p_cymbal': np.array([-0.2, -0.2, 0.4]),
            'p_triangle': np.array([0.2, -0.35, 0.35]),
            'p_up': np.array([0.0, -0.3, 0.0]),
        }
        
        self.video_writer = None
        
    def set_pose(self, arm, pose):
        for i, jn in enumerate(self.arms[arm]):
            self.data.qpos[self.model.joint(jn).qposadr[0]] = pose[i]
    
    def set_all(self, d, x, p):
        self.set_pose('drummer', d)
        self.set_pose('xylo', x)
        self.set_pose('perc', p)
        mujoco.mj_forward(self.model, self.data)
    
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
    
    def hud(self, text, beat, total_beats, phase):
        self.renderer.update_scene(self.data, camera='overview')
        frame = self.renderer.render()
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (1280, 45), (0, 0, 0), -1)
        cv2.putText(overlay, f"Robot Orchestra | {text}", 
                    (15, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(overlay, f"Beat {beat}/{total_beats} | {phase}", 
                    (900, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv2.rectangle(overlay, (15, 50), (1265, 62), (40, 40, 40), -1)
        for i in range(total_beats):
            x = 15 + int(1250 * i / total_beats)
            w = max(1, int(1250 / total_beats) - 1)
            color = (0, 200, 0) if i < beat else (80, 80, 80)
            if i % 4 == 0:
                color = (0, 255, 255) if i < beat else (100, 100, 100)
            cv2.rectangle(overlay, (x, 50), (x + w, 62), color, -1)
        cv2.rectangle(overlay, (0, 690), (1280, 720), (20, 20, 20), -1)
        cv2.putText(overlay, "Robot Orchestra | 3 Arms | 4 Instruments | Closed-Loop PD Control | 120 BPM", 
                    (15, 712), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (160, 160, 160), 1)
        frame_bgr = cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR)
        self.video_writer.write(frame_bgr)


DRUM = ['d_kick', 'd_hihat', 'd_snare', 'd_hihat']
XYLO = ['x_c', 'x_e', 'x_g', 'x_e']
PERC = ['p_cymbal', None, 'p_triangle', None]


def play_beat(orch, i):
    d_key = DRUM[i % 4]
    x_key = XYLO[i % 4]
    p_key = PERC[i % 4]
    
    # 下击
    orch.set_all(orch.poses[d_key], orch.poses[x_key], 
                 orch.poses[p_key] if p_key else orch.poses['p_rest'])
    orch.write_n(6, 'overview')  # 增加到6帧
    
    # 抬起
    orch.set_all(orch.poses['d_up'], orch.poses['x_up'], orch.poses['p_up'])
    orch.write_n(6, 'overview')  # 增加到6帧


def main():
    print("=" * 50)
    print("Robot Orchestra v7 - 统一overview相机")
    print("=" * 50)
    
    orch = RobotOrchestra('scene.xml')
    orch.set_all(orch.poses['d_rest'], orch.poses['x_rest'], orch.poses['p_rest'])
    
    orch.start_video('demo.mp4', fps=30)
    
    # 开场 (2秒)
    orch.write_n(30, 'overview')
    
    total_beats = 48
    beat = 0
    
    # Phase 1: Intro (8拍)
    print("[Phase 1] Intro")
    for i in range(8):
        play_beat(orch, i)
        beat += 1
    
    for _ in range(15):
        orch.hud("3 Arms | 4 Instruments", beat, total_beats, "Intro")
    
    # Phase 2: Verse (12拍)
    print("[Phase 2] Verse")
    for i in range(12):
        play_beat(orch, i)
        beat += 1
    
    for _ in range(15):
        orch.hud("3 Arms | 4 Instruments", beat, total_beats, "Verse")
    
    # Phase 3: Chorus (16拍)
    print("[Phase 3] Chorus")
    for i in range(16):
        play_beat(orch, i)
        beat += 1
    
    for _ in range(15):
        orch.hud("3 Arms | 4 Instruments", beat, total_beats, "Chorus")
    
    # Phase 4: Finale (12拍)
    print("[Phase 4] Finale")
    for i in range(12):
        play_beat(orch, i)
        beat += 1
    
    # 结束亮相
    print("[结束]")
    orch.set_all(orch.poses['d_rest'], orch.poses['x_rest'], orch.poses['p_rest'])
    
    for _ in range(60):
        orch.renderer.update_scene(orch.data, camera='overview')
        frame = orch.renderer.render()
        overlay = frame.copy()
        cv2.rectangle(overlay, (280, 240), (1000, 420), (0, 0, 0), -1)
        cv2.rectangle(overlay, (282, 242), (998, 418), (0, 180, 0), 2)
        cv2.putText(overlay, "Robot Orchestra", 
                    (360, 290), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 255, 0), 3)
        cv2.putText(overlay, "3 Arms | 4 Instruments | 120 BPM", 
                    (350, 335), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
        cv2.putText(overlay, "Closed-Loop PD Control | Multi-Agent Coordination", 
                    (320, 370), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (200, 200, 200), 1)
        cv2.putText(overlay, "Drums | Xylophone | Cymbal | Triangle", 
                    (340, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1)
        cv2.rectangle(overlay, (0, 690), (1280, 720), (20, 20, 20), -1)
        cv2.putText(overlay, "Robot Orchestra | 3 Arms | 4 Instruments | Closed-Loop PD Control | 120 BPM", 
                    (15, 712), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (160, 160, 160), 1)
        frame_bgr = cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR)
        orch.video_writer.write(frame_bgr)
    
    orch.end_video()
    
    import subprocess
    result = subprocess.run(['ffprobe', '-v', 'quiet', '-show_format', 'demo.mp4'], 
                          capture_output=True, text=True)
    for line in result.stdout.split('\n'):
        if 'duration' in line:
            print(f"视频时长: {line.strip()}")
    
    print("\n演出结束!")


if __name__ == '__main__':
    main()
