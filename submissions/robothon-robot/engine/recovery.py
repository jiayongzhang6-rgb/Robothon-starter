"""Recovery pipeline for AEGIS v6."""


class RecoverySystem:
    """3-phase recovery: observe → localize → replan."""
    
    def __init__(self, state):
        self.state = state
        self.steps = ["RE_OBSERVE", "RE_LOCALIZE", "RE_PLAN"]
        self.current = 0
        self.active = False
    
    def trigger(self):
        """Start recovery sequence."""
        self.state.mode = "RECOVERY"
        self.current = 0
        self.active = True
    
    def step(self):
        """Execute next recovery step."""
        if not self.active:
            return None
        
        if self.current < len(self.steps):
            action = self.steps[self.current]
            self.current += 1
            return action
        
        # Recovery complete
        self.active = False
        self.state.mode = "PATROL"
        return "DONE"
    
    def reset(self):
        """Reset recovery state."""
        self.current = 0
        self.active = False
