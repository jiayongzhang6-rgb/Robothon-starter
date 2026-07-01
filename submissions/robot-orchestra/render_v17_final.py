#!/usr/bin/env python3
"""
render_v17_final.py - PR#508 Enhanced Robot Orchestra Renderer
Features: Now-Playing overlays, instrument labels, beat visualizer,
          track progress bars, scene title cards.
Resolution: 1920x1080 @ 30fps, 19s (570 frames)
"""

import os, sys, math, numpy as np
import mujoco
import imageio
from PIL import Image, ImageDraw, ImageFont

# ── Paths ──────────────────────────────────────────────────────────────────
SCENE_XML = os.path.join(os.path.dirname(__file__), "scene.xml")
OUTPUT    = os.path.join(os.path.dirname(__file__), "demo.mp4")

# ── Render config ──────────────────────────────────────────────────────────
W, H   = 1920, 1080
FPS    = 30
DUR    = 19.0
N_FRAMES = int(DUR * FPS)  # 570

# ── Load model ─────────────────────────────────────────────────────────────
model = mujoco.MjModel.from_xml_path(SCENE_XML)
data  = mujoco.MjData(model)

renderer = mujoco.Renderer(model, height=H, width=W)
cam = mujoco.MjvCamera()
cam.type = mujoco.mjtCamera.mjCAMERA_FREE

# ── Joint name → id helper ─────────────────────────────────────────────────
def jid(name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)

# ── Camera presets  (lookat, distance, azimuth, elevation) ────────────────
CAMERAS = {
    "wide":      dict(lookat=[0, 0, 0.8], dist=4.0, az=135, el=-20),
    "drummer":   dict(lookat=[-1.2, 0.8, 1.0], dist=2.2, az=150, el=-15),
    "xylophone": dict(lookat=[0.8, -0.5, 1.0], dist=2.2, az=120, el=-15),
    "perc":      dict(lookat=[1.5, 0.8, 1.0], dist=2.2, az=160, el=-15),
    "close":     dict(lookat=[0, 0, 1.0], dist=1.6, az=135, el=-10),
    "top":       dict(lookat=[0, 0, 0.8], dist=5.0, az=135, el=-60),
}

def set_camera(key):
    c = CAMERAS[key]
    cam.lookat[:] = c["lookat"]
    cam.distance  = c["dist"]
    cam.azimuth   = c["az"]
    cam.elevation  = c["el"]

# ── Scene / track timeline ─────────────────────────────────────────────────
# (start_sec, end_sec, camera, scene_name, track_name)
SCENES = [
    (0.0,  2.5,  "wide",      "INTRO",      None),
    (2.5,  5.5,  "drummer",   "MARCH I",    "MARCH"),
    (5.5,  8.5,  "xylophone", "MARCH II",   "MARCH"),
    (8.5, 11.5,  "wide",      "WALTZ I",    "WALTZ"),
    (11.5,14.5,  "close",     "WALTZ II",   "WALTZ"),
    (14.5,17.5,  "top",       "CLIMAX",     "FINALE"),
    (17.5,19.0,  "wide",      "BOW",        "FINALE"),
]

# Track start/end for progress bars
TRACKS = {
    "MARCH":  (2.5, 8.5),
    "WALTZ":  (8.5, 14.5),
    "FINALE": (14.5, 19.0),
}

NOW_PLAYING_LABELS = {
    "MARCH":  "NOW PLAYING: MARCH",
    "WALTZ":  "NOW PLAYING: WALTZ",
    "FINALE": "NOW PLAYING: FINALE",
}

# Instrument label positions (relative to screen – normalised x,y)
INSTRUMENT_LABELS = {
    "drummer":   ("DRUMMER",    0.18, 0.55),
    "xylophone": ("XYLOPHONE", 0.75, 0.50),
    "perc":      ("PERCUSSION", 0.82, 0.55),
}

# Beat config (120 BPM = 0.5 s per beat)
BPM       = 120
BEAT_SEC  = 60.0 / BPM   # 0.5

# ── Font helpers ───────────────────────────────────────────────────────────
try:
    FONT_BIG   = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 72)
    FONT_MED   = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 42)
    FONT_SM    = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 32)
    FONT_XS    = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 26)
except Exception:
    FONT_BIG = FONT_MED = FONT_SM = FONT_XS = ImageFont.load_default()

GOLD       = (255, 215, 0)
WHITE      = (255, 255, 255)
DIM_WHITE  = (180, 180, 180)
BLACK      = (0, 0, 0)
DARK_BG    = (20, 20, 30)

# ── HUD drawing helpers ───────────────────────────────────────────────────

def draw_text_center(draw, y, text, font, fill):
    bbox = draw.textbbox((0, 0), text, font=font)
    tw = bbox[2] - bbox[0]
    draw.text(((W - tw) / 2, y), text, font=font, fill=fill)


def text_width(draw, text, font):
    bbox = draw.textbbox((0, 0), text, font=font)
    return bbox[2] - bbox[0]


def ease_in_out(t):
    """0-1 ease in-out cubic"""
    if t < 0.5:
        return 4 * t * t * t
    return 1 - (-2 * t + 2) ** 3 / 2


def now_playing_overlay(img, track_name, t_since_track_start):
    """Draw NOW PLAYING with fade-in (0-0.6s) and fade-out (next 0.4s)."""
    label = NOW_PLAYING_LABELS.get(track_name)
    if label is None:
        return
    # Fade: 0-0.4s in, 0.4-0.8 hold, 0.8-1.2 out
    dur = 1.2
    if t_since_track_start > dur:
        return
    alpha01 = min(t_since_track_start / 0.4, 1.0, (dur - t_since_track_start) / 0.4)
    alpha01 = max(0, alpha01)
    alpha = int(220 * ease_in_out(alpha01))
    gold  = (*GOLD, alpha)
    bg    = (*DARK_BG, int(alpha * 0.85))

    overlay = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    od = ImageDraw.Draw(overlay)
    # gradient band
    band_h = 160
    y0 = (H - band_h) // 2
    for row in range(band_h):
        a = int(alpha * 0.85 * (1 - abs(row - band_h / 2) / (band_h / 2)))
        od.line([(0, y0 + row), (W, y0 + row)], fill=(20, 20, 30, a))
    # text
    draw_text_center(od, y0 + 30, label, FONT_BIG, gold)
    img.paste(Image.alpha_composite(img.convert("RGBA"), overlay).convert("RGB"))


def scene_title_overlay(img, scene_name, frame_in_scene, scene_duration):
    """Large scene title at the start of each scene (first ~1 s)."""
    show_dur = min(1.0, scene_duration * 0.35)
    if frame_in_scene / FPS > show_dur:
        return
    t = frame_in_scene / FPS
    alpha01 = min(t / 0.25, 1.0, (show_dur - t) / 0.3)
    alpha01 = max(0, alpha01)
    alpha = int(200 * ease_in_out(alpha01))
    overlay = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    od = ImageDraw.Draw(overlay)
    draw_text_center(od, 80, scene_name, FONT_MED, (*WHITE, alpha))
    img.paste(Image.alpha_composite(img.convert("RGBA"), overlay).convert("RGB"))


def instrument_labels_overlay(img, scene_name, camera_key):
    """Show instrument name near the relevant arm."""
    info = INSTRUMENT_LABELS.get(camera_key)
    if info is None:
        return
    label, nx, ny = info
    overlay = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    od = ImageDraw.Draw(overlay)
    x = int(nx * W)
    y = int(ny * H)
    # background pill
    tw = text_width(od, label, FONT_SM) + 20
    od.rounded_rectangle([x - 10, y - 5, x + tw, y + 38], radius=8,
                         fill=(0, 0, 0, 140))
    od.text((x, y), label, font=FONT_SM, fill=(*GOLD, 220))
    img.paste(Image.alpha_composite(img.convert("RGBA"), overlay).convert("RGB"))


def beat_visualizer(img, t):
    """Bottom beat dots + current beat label."""
    beat_float = t / BEAT_SEC
    beat_idx   = int(beat_float) % 4 + 1          # 1-4
    frac       = beat_float - int(beat_float)      # 0-1 within beat

    overlay = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    od = ImageDraw.Draw(overlay)

    # 4 beat dots
    dot_y   = H - 60
    dot_x0  = W // 2 - 80
    spacing = 50
    for i in range(4):
        dx = dot_x0 + i * spacing
        is_active = (i + 1) == beat_idx
        if is_active:
            # bright flash that decays
            brightness = int(255 * max(0, 1 - frac * 1.5))
            r = int(12 + (255 - 12) * max(0, 1 - frac * 2))
            color = (r, 215 * brightness // 255, 0, min(255, brightness + 80))
        else:
            color = (80, 80, 80, 100)
        od.ellipse([dx - 10, dot_y - 10, dx + 10, dot_y + 10], fill=color)

    # beat text
    beat_text = f"BEAT {beat_idx}"
    bt_alpha = int(180 * max(0.3, 1 - frac))
    draw_text_center(od, dot_y + 18, beat_text, FONT_XS, (*DIM_WHITE, bt_alpha))

    # BPM label
    od.text((W - 160, dot_y - 12), f"{BPM} BPM", font=FONT_XS, fill=(*DIM_WHITE, 120))

    img.paste(Image.alpha_composite(img.convert("RGBA"), overlay).convert("RGB"))


def track_progress(img, t):
    """Progress bar for current track."""
    current_track = None
    for name, (ts, te) in TRACKS.items():
        if ts <= t < te:
            current_track = name
            break
    if current_track is None:
        return
    ts, te = TRACKS[current_track]
    pct = (t - ts) / (te - ts)
    pct = max(0, min(1, pct))

    overlay = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    od = ImageDraw.Draw(overlay)

    bar_w = 360
    bar_h = 16
    bx = 30
    by = H - 40

    # label
    od.text((bx, by - 34), current_track, font=FONT_XS, fill=(*GOLD, 200))

    # bg
    od.rounded_rectangle([bx, by, bx + bar_w, by + bar_h], radius=4, fill=(60, 60, 60, 160))
    # fill
    fill_w = int(bar_w * pct)
    if fill_w > 0:
        od.rounded_rectangle([bx, by, bx + fill_w, by + bar_h], radius=4, fill=(*GOLD, 220))
    # pct text
    od.text((bx + bar_w + 10, by - 4), f"{int(pct * 100)}%", font=FONT_XS, fill=(*WHITE, 200))

    img.paste(Image.alpha_composite(img.convert("RGBA"), overlay).convert("RGB"))


# ── Controller: joint targets over time ────────────────────────────────────
def apply_control(t):
    """Set actuator targets based on time to animate the robot arms."""
    beat = t / BEAT_SEC  # beats elapsed

    # Drummer arm – quarter notes
    d_amp = 0.4
    d_phase = math.sin(beat * math.pi) * d_amp
    try:
        data.ctrl[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "d_j1")] = d_phase
        data.ctrl[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "d_j2")] = 0.3 * math.sin(beat * math.pi * 0.5)
        data.ctrl[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "d_j3")] = 0.2 * math.cos(beat * math.pi)
    except Exception:
        pass

    # Xylophone arm – eighth notes
    x_amp = 0.35
    x_phase = math.sin(beat * math.pi * 2) * x_amp
    try:
        data.ctrl[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "x_j1")] = x_phase
        data.ctrl[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "x_j2")] = 0.25 * math.cos(beat * math.pi)
        data.ctrl[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "x_j3")] = 0.15 * math.sin(beat * math.pi * 1.5)
    except Exception:
        pass

    # Percussion arm – syncopated
    p_amp = 0.3
    p_phase = math.sin(beat * math.pi * 1.5 + 0.5) * p_amp
    try:
        data.ctrl[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "p_j1")] = p_phase
        data.ctrl[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "p_j2")] = 0.2 * math.sin(beat * math.pi * 0.75)
        data.ctrl[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "p_j3")] = 0.18 * math.cos(beat * math.pi * 2)
    except Exception:
        pass


# ── Post-processing: warm cinematic grade ──────────────────────────────────
def grade(frame_np):
    """Simple warm colour grade via numpy."""
    f = frame_np.astype(np.float32) / 255.0
    # lift shadows, warm highlights
    f[..., 0] = np.clip(f[..., 0] * 1.05 + 0.01, 0, 1)   # R
    f[..., 1] = np.clip(f[..., 1] * 0.97 + 0.005, 0, 1)  # G
    f[..., 2] = np.clip(f[..., 2] * 0.90 + 0.01, 0, 1)   # B
    # slight contrast S-curve
    f = f * f * (3 - 2 * f)
    return (np.clip(f, 0, 1) * 255).astype(np.uint8)


# ── Main render loop ──────────────────────────────────────────────────────
def main():
    writer = imageio.get_writer(OUTPUT, fps=FPS, quality=9,
                                codec="libx264", pixelformat="yuv420p")

    current_scene_idx = 0

    for fi in range(N_FRAMES):
        t = fi / FPS

        # Advance scene
        while (current_scene_idx < len(SCENES) - 1 and
               t >= SCENES[current_scene_idx][1]):
            current_scene_idx += 1

        scene_start, scene_end, cam_key, scene_name, track_name = SCENES[current_scene_idx]
        scene_duration = scene_end - scene_start
        frame_in_scene = int((t - scene_start) * FPS)

        # Step physics
        apply_control(t)
        mujoco.mj_step(model, data)

        # Render
        set_camera(cam_key)
        renderer.update_scene(data, cam)
        frame = renderer.render()
        frame = grade(frame)

        # Convert to PIL for HUD
        img = Image.fromarray(frame)

        # 1. Scene title
        scene_title_overlay(img, scene_name, frame_in_scene, scene_duration)

        # 2. Now-playing overlay (at track boundaries)
        if track_name:
            t_in_track = t - TRACKS[track_name][0]
            # Show at track start (within first 1.2s)
            if t_in_track < 1.3:
                now_playing_overlay(img, track_name, t_in_track)

        # 3. Instrument label (when camera is focused on one arm)
        if cam_key in INSTRUMENT_LABELS:
            instrument_labels_overlay(img, scene_name, cam_key)

        # 4. Beat visualizer
        beat_visualizer(img, t)

        # 5. Track progress bar
        track_progress(img, t)

        writer.append_data(np.array(img))

        if fi % 100 == 0:
            print(f"  frame {fi}/{N_FRAMES}  t={t:.1f}s  scene={scene_name}")

    writer.close()
    print(f"Done → {OUTPUT}")


if __name__ == "__main__":
    main()
