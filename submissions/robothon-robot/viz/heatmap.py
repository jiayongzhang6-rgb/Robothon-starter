"""Risk heatmap visualization for AEGIS v6."""

import cv2
import numpy as np


class Heatmap:
    """Risk field visualization as colored overlay."""
    
    def __init__(self, grid_res, world_size):
        self.grid_res = grid_res
        self.world_size = world_size
    
    def draw(self, frame, risk_field, offset=(140, 60)):
        """Draw risk heatmap on frame."""
        if risk_field.max() < 0.05:
            return frame
        
        h, w = frame.shape[:2]
        ox, oy = offset
        cell_w = (w - 2*ox) / self.grid_res
        cell_h = (h - 2*oy) / self.grid_res
        
        overlay = frame.copy()
        for i in range(self.grid_res):
            for j in range(self.grid_res):
                if risk_field[i, j] > 0.05:
                    x1 = int(ox + i * cell_w)
                    y1 = int(oy + (self.grid_res - 1 - j) * cell_h)
                    x2 = int(ox + (i + 1) * cell_w)
                    y2 = int(oy + (self.grid_res - j) * cell_h)
                    
                    intensity = risk_field[i, j]
                    color = (0, 0, int(200 * intensity))
                    cv2.rectangle(overlay, (x1, y1), (x2, y2), color, -1)
        
        cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)
        return frame
