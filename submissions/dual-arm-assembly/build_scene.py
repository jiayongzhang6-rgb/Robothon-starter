#!/usr/bin/env python3
"""用真实Panda mesh构建双臂场景 - 完整版"""
import re, os, mujoco

BASE = os.path.dirname(os.path.abspath(__file__))
PANDA = os.path.join(BASE, "../vendor/mujoco_menagerie/franka_emika_panda/panda.xml")
MESHDIR = os.path.join(BASE, "../vendor/mujoco_menagerie/franka_emika_panda/assets")

with open(PANDA) as f:
    panda = f.read()

def extract_balanced(xml, tag):
    pattern = f'<{tag}[^>]*>'
    m = re.search(pattern, xml)
    if not m: return ''
    start = m.end()
    depth = 1; pos = start
    while depth > 0 and pos < len(xml):
        no = xml.find(f'<{tag}', pos)
        nc = xml.find(f'</{tag}>', pos)
        if nc == -1: break
        if no != -1 and no < nc:
            depth += 1; pos = no + len(f'<{tag}')
        else:
            depth -= 1
            if depth == 0: return xml[start:nc]
            pos = nc + len(f'</{tag}>')
    return ''

defaults = extract_balanced(panda, 'default')
assets = extract_balanced(panda, 'asset')
wbody = extract_balanced(panda, 'worldbody')
actuators = extract_balanced(panda, 'actuator')
tendons = extract_balanced(panda, 'tendon')
equality = extract_balanced(panda, 'equality')

def ren(text, sfx):
    text = re.sub(r'\bname="([^"]+)"', lambda m: f'name="{m.group(1)}_{sfx}"', text)
    text = re.sub(r'\bjoint="([^"]+)"', lambda m: f'joint="{m.group(1)}_{sfx}"', text)
    text = re.sub(r'\btendon="([^"]+)"', lambda m: f'tendon="{m.group(1)}_{sfx}"', text)
    text = re.sub(r'\bjoint1="([^"]+)"', lambda m: f'joint1="{m.group(1)}_{sfx}"', text)
    text = re.sub(r'\bjoint2="([^"]+)"', lambda m: f'joint2="{m.group(1)}_{sfx}"', text)
    return text

xml = f'''<mujoco model="dual arm assembly">
  <compiler angle="radian" meshdir="{MESHDIR}" autolimits="true"/>
  <option timestep="0.002" integrator="implicitfast">
    <flag eulerdamp="disable"/>
  </option>
  <visual>
    <global offwidth="1280" offheight="720"/>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3"/>
  </visual>
  <default>
    {defaults}
  </default>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.2 0.35 0.5" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="grid" builtin="checker" rgb1=".8 .8 .8" rgb2=".5 .5 .5" width="512" height="512"/>
    <material name="grid_mat" texture="grid" texrepeat="6 6" reflectance="0.15"/>
    {assets}
  </asset>
  <worldbody>
    <geom name="floor" size="2 2 0.1" type="plane" material="grid_mat"/>
    <light pos="0 0 2.5" dir="0 0 -1" diffuse=".8 .8 .8"/>
    <light pos="1 1 3" dir="-.5 -.5 -1" diffuse=".5 .5 .5"/>
    <body name="base_L" pos="-0.4 0 0">
      {ren(wbody, 'L')}
    </body>
    <body name="base_R" pos="0.4 0 0" euler="0 0 3.14159">
      {ren(wbody, 'R')}
    </body>
    <body name="module_a" pos="-0.15 0 0.08">
      <freejoint/>
      <geom type="cylinder" size=".04 .04" rgba=".2 .6 .9 1" contype="2" conaffinity="1"/>
    </body>
    <body name="module_b" pos="0.15 0 0.04">
      <freejoint/>
      <geom type="cylinder" size=".035 .02" rgba=".9 .3 .1 1" contype="2" conaffinity="1"/>
    </body>
    <body name="module_c" pos="0 -.2 0.03">
      <freejoint/>
      <geom type="box" size=".025 .015 .015" rgba=".3 .9 .2 1" contype="2" conaffinity="1"/>
    </body>
  </worldbody>
  <tendon>
    {ren(tendons, 'L')}
    {ren(tendons, 'R')}
  </tendon>
  <equality>
    {ren(equality, 'L')}
    {ren(equality, 'R')}
  </equality>
  <actuator>
    {ren(actuators, 'L')}
    {ren(actuators, 'R')}
  </actuator>
</mujoco>'''

out = os.path.join(BASE, "scene_dual.xml")
with open(out, 'w') as f:
    f.write(xml)

m = mujoco.MjModel.from_xml_path(out)
print(f"✅ nq={m.nq} nu={m.nu}")

d = mujoco.MjData(m)
mujoco.mj_forward(m, d)
hL = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "hand_L")
hR = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "hand_R")
mA = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "module_a")
print(f"左手: {d.xpos[hL]}")
print(f"右手: {d.xpos[hR]}")
print(f"模块A: {d.xpos[mA]}")
