#!/usr/bin/env python3
"""太空舱视频 v7 - 修复相机角度，地板在底部"""
import mujoco, numpy as np, os, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt, io
from PIL import Image

BASE = os.path.dirname(os.path.abspath(__file__))
XML = os.path.join(BASE, "scene_dual_v5.xml")
OUT = os.path.join(BASE, "dual_arm_v7.mp4")
FPS = 24; SEC = 18; NFRAMES = FPS * SEC

model = mujoco.MjModel.from_xml_path(XML)
data = mujoco.MjData(model)
W, H = 960, 540
rend = mujoco.Renderer(model, height=H, width=W)

hL = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand_L")
hR = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand_R")
mA = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "module_a")
mB = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "module_b")
mC = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "module_c")

def qa(n): return model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]
def da(n): return model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]

L_Q = [qa(f"joint{i}_L") for i in range(1,8)]
R_Q = [qa(f"joint{i}_R") for i in range(1,8)]
L_D = [da(f"joint{i}_L") for i in range(1,8)]
R_D = [da(f"joint{i}_R") for i in range(1,8)]
L_FQ = [qa("finger_joint1_L"), qa("finger_joint2_L")]
R_FQ = [qa("finger_joint1_R"), qa("finger_joint2_R")]
L_CI = list(range(0,7)); R_CI = list(range(8,15))
L_FI = [7]; R_FI = [15]
mA_q = qa("module_a_free"); mB_q = qa("module_b_free"); mC_q = qa("module_c_free")

def solve_ik(target, qi, di, bid, iters=500):
    for _ in range(iters):
        mujoco.mj_forward(model, data)
        err = target - data.xpos[bid]
        if np.linalg.norm(err) < 0.005: return True
        J = np.zeros((3, model.nv)); mujoco.mj_jac(model, data, J, None, data.xpos[bid].copy(), bid)
        Ja = np.zeros((3, 7))
        for i in range(7): Ja[:, i] = J[:, di[i]]
        dq = np.linalg.solve(Ja.T @ Ja + 0.01 * np.eye(7), Ja.T @ err)
        for i in range(7): data.qpos[qi[i]] += dq[i] * 0.5
    return False

def get_ik(tL, tR):
    mujoco.mj_resetData(model, data); mujoco.mj_forward(model, data)
    solve_ik(tL, L_Q, L_D, hL); qL = [data.qpos[L_Q[i]] for i in range(7)]
    solve_ik(tR, R_Q, R_D, hR); qR = [data.qpos[R_Q[i]] for i in range(7)]
    return qL, qR

mujoco.mj_resetData(model, data); mujoco.mj_forward(model, data)
ML = data.xpos[mA].copy(); MB = data.xpos[mB].copy(); MC = data.xpos[mC].copy()

print("Computing IK...")
qL_home, qR_home = get_ik(np.array([-0.3, 0.2, 0.9]), np.array([0.3, 0.2, 0.9]))
print("  home OK")
qL_reach, qR_idle = get_ik(ML + [0,0,0.15], np.array([0.3, 0.2, 0.9]))
print("  reach OK")
qL_grasp, qR_idle2 = get_ik(ML + [0,0,0.08], np.array([0.3, 0.2, 0.9]))
print("  grasp OK")
qL_lift, qR_reach2 = get_ik(np.array([-0.2, 0, 1.1]), MB + [0,0,0.15])
print("  lift OK")
qL_ho, qR_ho = get_ik(np.array([-0.08, 0.2, 1.0]), np.array([0.08, 0.2, 1.0]))
print("  handoff OK")
qL_asm, qR_asm = get_ik(np.array([-0.04, 0.2, 1.05]), np.array([0.04, 0.2, 1.05]))
print("  assemble OK")
qL_hold, qR_gc = get_ik(np.array([-0.04, 0.2, 1.05]), MC + [0,0,0.15])
print("  grab_C OK")
qL_fin, qR_fin = get_ik(np.array([-0.06, 0.2, 1.15]), np.array([0.06, 0.2, 1.15]))
print("  final OK")

STEPS = [
    ("home",     qL_home,  qR_home,  0.04, 0.04, 1.5, None),
    ("reach",    qL_reach, qR_idle,  0.04, 0.04, 1.5, None),
    ("grasp",    qL_grasp, qR_idle2, 0.0,  0.04, 1.0, "L"),
    ("lift",     qL_lift,  qR_reach2,0.0,  0.04, 2.0, "L"),
    ("handoff",  qL_ho,    qR_ho,    0.0,  0.0,  1.5, "B"),
    ("assemble", qL_asm,   qR_asm,   0.0,  0.0,  1.5, "B"),
    ("grab_C",   qL_hold,  qR_gc,    0.0,  0.04, 1.5, "L"),
    ("final",    qL_fin,   qR_fin,   0.0,  0.0,  2.0, "B"),
]

LABELS = ["Home", "Reach Blue", "Grasp Blue", "Lift + Reach Red",
          "Handoff", "Assembly", "Grab Green", "Final"]

def make_cam(az, el):
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [0, 0.1, 0.8]
    cam.distance = 5.0
    cam.azimuth = az
    cam.elevation = el
    return cam

# 相机配置 - elevation用负数从上往下看
CAMS = [
    (30, -25),    # home
    (30, -25),    # reach
    (10, -30),    # grasp
    (90, -25),    # lift - 侧面
    (30, -30),    # handoff
    (30, -50),    # assemble - 俯视
    (50, -25),    # grab_C
    (30, -25),    # final
]

def overlay(pi, t, fL, fR, gL, gR):
    fig = plt.figure(figsize=(9.6, 5.4), dpi=100); fig.patch.set_alpha(0)
    ax = fig.add_axes([0.02, 0.72, 0.25, 0.26])
    ax.set_xlim(0,1); ax.set_ylim(0,1); ax.set_facecolor((0.05,0.08,0.14,0.92))
    ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values(): s.set_color((0.3,0.6,0.9,0.5))
    ax.text(0.05,0.85,LABELS[pi],fontsize=11,color='white',fontweight='bold',transform=ax.transAxes)
    ax.text(0.05,0.65,f"Step {pi+1}/8",fontsize=9,color=(0.6,0.8,1),transform=ax.transAxes)
    ax.text(0.05,0.45,f"{t:.1f}s / {SEC}s",fontsize=9,color=(0.6,0.8,1),transform=ax.transAxes)
    ax.text(0.05,0.25,f"L: {'GRIP' if gL else 'OPEN'}",fontsize=8,color='#00ff88' if gL else '#ffaa00',transform=ax.transAxes)
    ax.text(0.55,0.25,f"R: {'GRIP' if gR else 'OPEN'}",fontsize=8,color='#00ff88' if gR else '#ffaa00',transform=ax.transAxes)
    
    ax_f = fig.add_axes([0.02,0.08,0.25,0.15])
    ax_f.set_xlim(0,1); ax_f.set_ylim(0,1); ax_f.set_facecolor((0.05,0.08,0.14,0.92))
    ax_f.set_xticks([]); ax_f.set_yticks([])
    for s in ax_f.spines.values(): s.set_color((0.3,0.6,0.9,0.5))
    ax_f.text(0.05,0.85,"Force",fontsize=7,color=(0.6,0.8,1),transform=ax_f.transAxes)
    ax_f.barh(0.55,min(fL/20,0.9),height=0.3,left=0.05,color=(0.2,0.7,1.0),edgecolor='none')
    ax_f.text(0.05,0.2,f"L:{fL:.0f}N",fontsize=6,color='white',transform=ax_f.transAxes)
    ax_f.barh(0.55,min(fR/20,0.9),height=0.3,left=0.55,color=(1.0,0.4,0.2),edgecolor='none')
    ax_f.text(0.55,0.2,f"R:{fR:.0f}N",fontsize=6,color='white',transform=ax_f.transAxes)
    
    ax_t = fig.add_axes([0.35,0.02,0.3,0.08])
    ax_t.set_xlim(0,1); ax_t.set_ylim(0,1); ax_t.set_facecolor((0.05,0.08,0.14,0.92))
    ax_t.set_xticks([]); ax_t.set_yticks([])
    for s in ax_t.spines.values(): s.set_color((0.3,0.6,0.9,0.5))
    ax_t.text(0.5,0.5,"SPACE MODULE ASSEMBLY",fontsize=9,color=(0.5,0.8,1),ha='center',va='center',transform=ax_t.transAxes,fontweight='bold')
    
    ax_p = fig.add_axes([0.7,0.02,0.28,0.08])
    ax_p.set_xlim(0,1); ax_p.set_ylim(0,1); ax_p.set_facecolor((0.05,0.08,0.14,0.92))
    ax_p.set_xticks([]); ax_p.set_yticks([])
    for s in ax_p.spines.values(): s.set_color((0.3,0.6,0.9,0.5))
    p = t/SEC
    ax_p.barh(0.5,0.95,height=0.3,left=0.025,color=(0.15,0.2,0.3),edgecolor='none')
    ax_p.barh(0.5,p*0.95,height=0.3,left=0.025,color=(0.2,0.7,1.0),edgecolor='none')
    ax_p.text(0.5,0.5,f"{p*100:.0f}%",fontsize=7,color='white',ha='center',va='center',transform=ax_p.transAxes)
    
    buf = io.BytesIO(); fig.savefig(buf,format='png',dpi=100,facecolor='none',edgecolor='none'); plt.close(fig); buf.seek(0)
    return np.array(Image.open(buf))

print("\nRendering...")
mujoco.mj_resetData(model, data); mujoco.mj_forward(model, data)
frames = []
cL, cR, cfL, cfR = qL_home[:], qR_home[:], 0.04, 0.04

for si, (name, tL, tR, fl, fr, dur, fw) in enumerate(STEPS):
    nf = int(dur * FPS); sL, sR, sfL, sfR = cL[:], cR[:], cfL, cfR
    az, el = CAMS[si]
    
    for f in range(nf):
        gf = sum(int(s[5]*FPS) for s in STEPS[:si]) + f; t = gf/FPS
        s = 0.5 - 0.5*np.cos(np.pi*f/max(nf-1,1))
        
        for i in range(7):
            data.qpos[L_Q[i]] = sL[i]+(tL[i]-sL[i])*s; data.qpos[R_Q[i]] = sR[i]+(tR[i]-sR[i])*s
            data.ctrl[L_CI[i]] = data.qpos[L_Q[i]]; data.ctrl[R_CI[i]] = data.qpos[R_Q[i]]
        
        vL = sfL+(fl-sfL)*s; vR = sfR+(fr-sfR)*s
        for i in range(2): data.qpos[L_FQ[i]]=vL; data.qpos[R_FQ[i]]=vR
        data.ctrl[L_FI[0]]=vL; data.ctrl[R_FI[0]]=vR
        
        mujoco.mj_forward(model, data)
        
        if fw in ("L","B"): p=data.xpos[hL].copy(); p[2]-=0.05; data.qpos[mA_q:mA_q+3]=p
        if fw in ("R","B"): p=data.xpos[hR].copy(); p[2]-=0.05; data.qpos[mB_q:mB_q+3]=p
        if si>=6: p=data.xpos[hR].copy(); p[2]-=0.04; data.qpos[mC_q:mC_q+3]=p
        
        mujoco.mj_forward(model, data)
        
        fL=max(0,(0.04-vL)/0.04*15) if fw in ("L","B") else 0
        fR=max(0,(0.04-vR)/0.04*15) if fw in ("R","B") else 0
        
        cam = make_cam(az, el)
        rend.update_scene(data, camera=cam)
        frame = rend.render().copy()
        
        ov = overlay(si, t, fL, fR, vL<0.01, vR<0.01)
        ov = np.array(Image.fromarray(ov).resize((W,H), Image.LANCZOS))
        if ov.shape[2]==4: a=ov[:,:,3:4].astype(float)/255; bf=frame*(1-a)+ov[:,:,:3]*a
        else: bf=ov[:,:,:3]
        frames.append(bf.astype(np.uint8))
    
    cL, cR, cfL, cfR = tL[:], tR[:], fl, fr

while len(frames)<NFRAMES: frames.append(frames[-1].copy())
print(f"Frames: {len(frames)}")
import imageio; imageio.mimsave(OUT, frames[:NFRAMES], fps=FPS, quality=8)
print(f"Done: {OUT}")
