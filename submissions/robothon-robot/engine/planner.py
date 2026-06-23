"""Path planning for AEGIS v6."""


class Planner:
    """Utility-based path planner with risk avoidance."""
    
    def select_node(self, utility_map):
        """Select highest utility node."""
        if not utility_map:
            return None
        return max(utility_map.items(), key=lambda x: x[1])[0]
    
    def replan(self, robot_pos, risk_map):
        """Replan path avoiding high-risk areas."""
        # Simple safe direction bias
        safe_x = robot_pos[0] + 1
        safe_y = robot_pos[1]
        
        # Check if path is blocked
        if risk_map and risk_map.get((safe_x, safe_y), 0) > 0.5:
            safe_y = robot_pos[1] - 1
        
        return [safe_x, safe_y]
