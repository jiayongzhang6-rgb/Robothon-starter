"""MuJoCo 3D renderer for AEGIS v6."""

import mujoco
import mujoco.renderer
import numpy as np


class MuJoCoRenderer:
    """3D scene rendering with MuJoCo."""
    
    def __init__(self, xml_string, width=1280, height=720):
        self.model = mujoco.MjModel.from_xml_string(xml_string)
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.renderer.Renderer(self.model, height=height, width=width)
        self.width = width
        self.height = height
    
    def set_robot_pose(self, x, y, heading):
        """Set robot position and orientation."""
        self.data.qpos[0] = x
        self.data.qpos[1] = y
        self.data.qpos[2] = 0.6
        self.data.qpos[3] = 1.0
        self.data.qpos[4] = 0.0
        self.data.qpos[5] = 0.0
        self.data.qpos[6] = np.sin(heading / 2)
    
    def render(self, camera="overhead"):
        """Render frame from camera."""
        mujoco.mj_forward(self.model, self.data)
        self.renderer.update_scene(self.data, camera=camera)
        frame = self.renderer.render()
        return frame
    
    def get_geom_id(self, name):
        """Get geom ID by name."""
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name)
    
    def set_geom_alpha(self, name, alpha):
        """Set geom transparency."""
        geom_id = self.get_geom_id(name)
        self.model.geom_rgba[geom_id, 3] = alpha
