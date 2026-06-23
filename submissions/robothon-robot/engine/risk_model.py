"""Risk assessment for AEGIS v6."""

import numpy as np


class RiskModel:
    """Spatial risk field with temporal decay."""
    
    def __init__(self, size):
        self.grid = np.zeros(size)
    
    def increase_risk(self, center):
        """Increase risk at center point."""
        x, y = center
        self.grid[max(0, x-2):x+3, max(0, y-2):y+3] += 0.6
    
    def decay(self):
        """Apply temporal decay to risk field."""
        self.grid *= 0.98
    
    def get_risk(self, pos):
        """Get risk level at position."""
        x, y = int(pos[0]), int(pos[1])
        if 0 <= x < self.grid.shape[0] and 0 <= y < self.grid.shape[1]:
            return self.grid[x, y]
        return 0.0
