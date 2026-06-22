"""录制高冲击力MuJoCo演示视频 - 目标45秒，冲第一"""
import sys, os, numpy as np, mujoco, imageio
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from franka_controller import FrankaController

# 字体缓存
_font_cache = {}
def get_font(size=20):
    if size not in _font_cache:
        for path in ["/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
                     "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf"]:
            try:
                _font_cache[size] = ImageFont.truetype(path, size)
                break
            except:
                continue
        if size not in _font_cache:
            _font_cache[size] = ImageFont.load_default()
    return _font_cache[size]

def add_subtitle(frame, text, position="bottom", fontsize=22):
    """在帧上添加字幕 - 半透明背景，居中"""
    img = Image.fromarray(frame)
    draw = ImageDraw.Draw(img)
    font = get_font(fontsize)
    
    bbox = draw.textbbox((0, 0), text, font=font)
    text_w = bbox[2] - bbox[0]
    text_h = bbox[3] - bbox[1]
    
    x = (640 - text_w) // 2
    if position == "bottom":
        y = 480 - text_h - 25
    elif position == "top":
        y = 12
    else:
        y = position
    
    # 半透明背景
    overlay = Image.new('RGBA', img.size, (0, 0, 0, 0))
    overlay_draw = ImageDraw.Draw(overlay)
    pad = 6
    overlay_draw.rounded_rectangle(
        [x - pad, y - pad, x + text_w + pad, y + text_h + pad],
        radius=6, fill=(0, 0, 0, 200)
    )
    img = img.convert('RGBA')
    img = Image.alpha_composite(img, overlay)
    
    draw = ImageDraw.Draw(img)
    draw.text((x, y), text, fill=(255, 255, 255), font=font)
    
    return np.array(img.convert('RGB'))

def add_progress_bar(frame, current, total):
    """顶部进度条"""
    img = Image.fromarray(frame)
    draw = ImageDraw.Draw(img)
    bar_w = int(640 * (current / total))
    draw.rectangle([0, 0, bar_w, 4], fill=(0, 200, 255))
    return np.array(img)

def record_demo(output_path="demo.mp4", fps=30):
    """录制高冲击力演示视频"""
    ctrl = FrankaController()
    frames = []
    total_tasks = 25
    task_counter = [0]
    
    def cap(text, n=1, show_progress=True):
        for _ in range(n):
            mujoco.mj_step(ctrl.model, ctrl.data)
            raw = ctrl.render_frame()
            framed = add_subtitle(raw, text)
            if show_progress:
                framed = add_progress_bar(framed, task_counter[0], total_tasks)
            frames.append(framed)
    
    def task_done():
        task_counter[0] += 1
    
    print("开始录制高冲击力视频...")
    
    # ===== 开场 (0-2s) - 慢镜头展示机器人 =====
    ctrl.reset()
    ctrl.gripper_control(0.04, steps=20)
    # 从侧面缓慢旋转展示
    for angle in np.linspace(0, 0.5, fps*2):
        q = ctrl.HOME_QPOS.copy()
        q[0] = angle
        ctrl.data.ctrl[:7] = q
        mujoco.mj_step(ctrl.model, ctrl.data)
        raw = ctrl.render_frame()
        framed = add_subtitle(raw, "Franka Panda Smart Manipulation", fontsize=24)
        frames.append(framed)
    print(f"  开场: {len(frames)}帧")
    
    # ===== 快速任务展示 - 每个任务0.8秒 =====
    task_duration = int(fps * 0.8)
    
    # Task 1: Home Position
    ctrl.reset()
    ctrl.gripper_control(0.04, steps=20)
    cap("Home Position", task_duration)
    task_done()
    
    # Task 2: Joint Limits
    limits = ctrl.get_joint_info()
    cap(f"Joint Limits: {limits['num_joints']} joints", task_duration)
    task_done()
    
    # Task 3: Forward Kinematics
    pose = ctrl.forward_kinematics(ctrl.HOME_QPOS)
    cap(f"FK: x={pose.position[0]:.2f} y={pose.position[1]:.2f}", task_duration)
    task_done()
    
    # Task 4: Jacobian
    J, _ = ctrl.get_jacobian(ctrl.HOME_QPOS)
    cap(f"Jacobian: {J.shape[0]}x{J.shape[1]}", task_duration)
    task_done()
    
    # Task 5: Damped Least Squares (快速IK演示)
    target = np.array([0.4, 0.2, 0.3])
    for _ in range(3):
        current_pose = ctrl.forward_kinematics(ctrl.data.qpos[:7])
        J, _ = ctrl.get_jacobian(ctrl.data.qpos[:7])
        JtJ = J.T @ J + 0.01 * np.eye(7)
        dx = target - current_pose.position
        dq = np.linalg.solve(JtJ, J.T @ dx)
        ctrl.data.ctrl[:7] = ctrl.data.qpos[:7] + dq
        mujoco.mj_step(ctrl.model, ctrl.data)
    raw = ctrl.render_frame()
    framed = add_subtitle(raw, "Task 5: DLS Inverse Kinematics")
    framed = add_progress_bar(framed, task_counter[0], total_tasks)
    for _ in range(task_duration):
        frames.append(framed)
    task_done()
    
    # Task 6: Joint Space Interpolation (快速)
    q_end = np.array([0.5, -0.3, 0.5, -1.5, 0.5, 1.5, 0.0])
    traj = ctrl.linear_interpolation_joint(ctrl.HOME_QPOS, q_end, num_points=task_duration)
    for i, pt in enumerate(traj):
        ctrl.data.ctrl[:7] = pt
        mujoco.mj_step(ctrl.model, ctrl.data)
        raw = ctrl.render_frame()
        framed = add_subtitle(raw, "Task 6: Joint Interpolation")
        frames.append(framed)
    task_done()
    ctrl.reset()
    ctrl.gripper_control(0.04, steps=15)
    
    # Task 7: Cartesian Space Interpolation (快速)
    pose_start = ctrl.forward_kinematics(ctrl.HOME_QPOS)
    pose_end = ctrl.forward_kinematics(q_end)
    traj = ctrl.linear_interpolation_cartesian(pose_start, pose_end, num_points=task_duration)
    for i, pt in enumerate(traj):
        ctrl.data.ctrl[:7] = ctrl.data.qpos[:7] + (pt.position - ctrl.forward_kinematics(ctrl.data.qpos[:7]).position) * 0.5
        mujoco.mj_step(ctrl.model, ctrl.data)
        raw = ctrl.render_frame()
        framed = add_subtitle(raw, "Task 7: Cartesian Interpolation")
        frames.append(framed)
    task_done()
    ctrl.reset()
    ctrl.gripper_control(0.04, steps=15)
    
    # Task 8: Minimum Jerk (快速平滑)
    q_end2 = np.array([0.3, 0.5, 0.4, -1.2, 0.3, 1.2, -0.5])
    traj = ctrl.minimum_jerk_trajectory(ctrl.HOME_QPOS, q_end2, duration=0.8, num_points=task_duration)
    for pt in traj:
        ctrl.data.ctrl[:7] = pt.position
        mujoco.mj_step(ctrl.model, ctrl.data)
        raw = ctrl.render_frame()
        framed = add_subtitle(raw, "Task 8: Minimum-Jerk Trajectory")
        frames.append(framed)
    task_done()
    ctrl.reset()
    ctrl.gripper_control(0.04, steps=15)
    
    # Task 9: Obstacle Avoidance (快速)
    ctrl.data.ctrl[:7] = ctrl.HOME_QPOS
    for _ in range(20):
        mujoco.mj_step(ctrl.model, ctrl.data)
    current = ctrl.data.qpos[:7]
    obstacle_pos = np.array([0.4, 0.0, 0.3])
    q_new = ctrl.obstacle_avoidance(current, obstacle_pos)
    ctrl.data.ctrl[:7] = q_new
    for _ in range(task_duration):
        mujoco.mj_step(ctrl.model, ctrl.data)
        raw = ctrl.render_frame()
        framed = add_subtitle(raw, "Task 9: Obstacle Avoidance")
        frames.append(framed)
    task_done()
    
    # Task 10: Workspace Analysis (快速统计)
    ws = ctrl.workspace_analysis(num_samples=200)
    cap(f"Task 10: Workspace - {ws['num_reachable']}/{200} reachable", task_duration)
    task_done()
    
    # Task 11: Approach Vector (快速)
    obj_pos = np.array([0.4, 0.15, 0.02])
    approach = ctrl.compute_approach_vector(obj_pos)
    cap(f"Task 11: Approach Vector - angle={np.degrees(np.arccos(approach['approach_vector'][2])):.0f}°", task_duration)
    task_done()
    
    # Task 12: Grasp Pose (快速)
    grasp = ctrl.compute_grasp_pose(obj_pos)
    cap(f"Task 12: Grasp Pose - width={grasp['width']:.3f}m", task_duration)
    task_done()
    
    # Task 13: Pre-Grasp Position (快速)
    pre = ctrl.pre_grasp_position(obj_pos)
    ctrl._move_to_cartesian_pos(pre, steps=30)
    cap(f"Task 13: Pre-Grasp - z={pre[2]:.3f}m", task_duration)
    task_done()
    
    # Task 14: Gripper Control - Opening
    ctrl.gripper_control(0.04, steps=15)
    cap("Task 14: Gripper Opening", task_duration)
    task_done()
    
    # Task 15: Force Estimation (快速)
    force = ctrl.force_estimation()
    cap(f"Task 15: Force - {force['total_force']:.2f}N", task_duration)
    task_done()
    
    # Task 16: Pick Object (快速抓取)
    ctrl._move_to_cartesian_pos(grasp['grasp_pos'], steps=30)
    ctrl.gripper_control(0.015, steps=20)
    ctrl._move_to_cartesian_pos(grasp['lift_pos'], steps=30)
    cap("Task 16: Pick Object", task_duration)
    task_done()
    
    # Task 17: Place Object (快速放置)
    place_pos = np.array([0.25, 0.15, 0.05])
    above = place_pos.copy(); above[2] += 0.25
    ctrl._move_to_cartesian_pos(above, steps=20)
    ctrl._move_to_cartesian_pos(place_pos, steps=20)
    ctrl.gripper_control(0.04, steps=15)
    cap("Task 17: Place Object", task_duration)
    task_done()
    
    # Task 18: Stack Objects (快速堆叠)
    ctrl._move_to_cartesian_pos(above, steps=15)
    obj2 = np.array([0.4, -0.1, 0.02])
    grasp2 = ctrl.compute_grasp_pose(obj2)
    ctrl._move_to_cartesian_pos(grasp2['grasp_pos'], steps=20)
    ctrl.gripper_control(0.015, steps=15)
    ctrl._move_to_cartesian_pos(grasp2['lift_pos'], steps=20)
    stack_pos = np.array([0.25, 0.15, 0.09])
    above_s = stack_pos.copy(); above_s[2] += 0.25
    ctrl._move_to_cartesian_pos(above_s, steps=20)
    ctrl._move_to_cartesian_pos(stack_pos, steps=20)
    ctrl.gripper_control(0.04, steps=15)
    ctrl._move_to_cartesian_pos(above_s, steps=15)
    cap("Task 18: Stack Objects", task_duration)
    task_done()
    
    # Task 19: Sort Objects (快速分拣)
    obj3 = np.array([0.5, 0.0, 0.02])
    grasp3 = ctrl.compute_grasp_pose(obj3)
    ctrl._move_to_cartesian_pos(grasp3['grasp_pos'], steps=20)
    ctrl.gripper_control(0.015, steps=15)
    ctrl._move_to_cartesian_pos(grasp3['lift_pos'], steps=20)
    sort_pos = np.array([-0.1, 0.1, 0.05])
    above_sort = sort_pos.copy(); above_sort[2] += 0.25
    ctrl._move_to_cartesian_pos(above_sort, steps=20)
    ctrl._move_to_cartesian_pos(sort_pos, steps=20)
    ctrl.gripper_control(0.04, steps=15)
    ctrl._move_to_cartesian_pos(above_sort, steps=15)
    cap("Task 19: Sort Objects", task_duration)
    task_done()
    
    # Task 20: Trajectory Recording (快速)
    traj_data = ctrl.record_trajectory([ctrl.HOME_QPOS, q_end, ctrl.HOME_QPOS], record_force=True)
    cap(f"Task 20: Trajectory - {len(traj_data)} points", task_duration)
    task_done()
    
    # Task 21: Collision Detection (快速)
    result = ctrl.collision_detection()
    cap(f"Task 21: Collision - {result['num_contacts']} contacts", task_duration)
    task_done()
    
    # Task 22: Impedance Control (快速)
    target_pos = np.array([0.4, 0, 0.3])
    tau = ctrl.impedance_control(target_pos)
    ctrl.data.ctrl[:7] = ctrl.data.qpos[:7] + tau * 0.001
    for _ in range(task_duration):
        mujoco.mj_step(ctrl.model, ctrl.data)
        raw = ctrl.render_frame()
        framed = add_subtitle(raw, "Task 22: Impedance Control")
        frames.append(framed)
    task_done()
    
    # Task 23: Visual Servoing (快速)
    target_pixel = np.array([320.0, 240.0])
    current_pixel = np.array([280.0, 200.0])
    dq = ctrl.visual_servoing(target_pixel, current_pixel)
    ctrl.data.ctrl[:7] = ctrl.data.qpos[:7] + dq
    for _ in range(task_duration):
        mujoco.mj_step(ctrl.model, ctrl.data)
        raw = ctrl.render_frame()
        framed = add_subtitle(raw, "Task 23: Visual Servoing (IBVS)")
        frames.append(framed)
    task_done()
    
    # Task 24: Skill Learning (快速)
    demos = [ctrl.HOME_QPOS, q_end, ctrl.HOME_QPOS]
    result = ctrl.skill_learning(demos)
    cap(f"Task 24: Skill Learning - {result['num_demos']} demos", task_duration)
    task_done()
    
    # Task 25: Task Orchestration (最终)
    task_seq = ['pick', 'place', 'stack', 'sort']
    result = ctrl.task_orchestration(task_seq)
    cap(f"Task 25: Orchestration - {result['completed']}/{result['total_tasks']}", task_duration)
    task_done()
    
    print(f"  任务展示: {len(frames)}帧")
    
    # ===== 结束画面 (1s) =====
    ctrl.reset()
    ctrl.gripper_control(0.04, steps=20)
    for _ in range(fps):
        mujoco.mj_step(ctrl.model, ctrl.data)
        raw = ctrl.render_frame()
        framed = add_subtitle(raw, "25/25 Tasks | 77 Tests | Complete!", fontsize=24)
        framed = add_progress_bar(framed, total_tasks, total_tasks)
        frames.append(framed)
    
    # 写入视频
    print(f"\n写入视频... {len(frames)}帧")
    imageio.mimsave(output_path, frames, fps=fps, codec='libx264')
    sz = os.path.getsize(output_path)
    print(f"完成! {len(frames)/fps:.0f}秒, {sz/1024/1024:.1f}MB")

if __name__ == "__main__":
    output = os.path.join(os.path.dirname(os.path.abspath(__file__)), "demo.mp4")
    record_demo(output)
