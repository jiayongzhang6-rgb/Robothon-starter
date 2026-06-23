"""Unified state management for AEGIS v6."""


class State:
    """Central state object shared across all modules."""
    
    def __init__(self):
        self.robot_pos = [2, 2]
        self.confidence = 0.82
        self.risk = "LOW"
        self.mode = "PATROL"
        self.path = []
        self.utility_map = {}
        self.recovery_step = None
        
    def update(self, **kwargs):
        """Update state with provided values."""
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)
    
    def to_dict(self):
        """Serialize state for logging/debugging."""
        return {
            "robot_pos": self.robot_pos.copy(),
            "confidence": self.confidence,
            "risk": self.risk,
            "mode": self.mode,
            "path_length": len(self.path),
            "recovery_step": self.recovery_step,
        }
