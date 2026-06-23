"""HUD overlay rendering for AEGIS v6."""

import cv2


class Overlay:
    """Minimal HUD overlay with decision bubbles."""
    
    def draw(self, frame, state, recovery=None):
        """Draw overlay elements on frame."""
        h, w = frame.shape[:2]
        
        # Confidence bar (top-right)
        bar_x, bar_y, bar_w, bar_h = w - 230, 20, 180, 16
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), 
                      (50, 50, 50), -1)
        fill_w = int(bar_w * state.confidence)
        color = (50, 200, 50) if state.confidence > 0.6 else (0, 180, 255) if state.confidence > 0.4 else (0, 0, 255)
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x + fill_w, bar_y + bar_h), 
                      color, -1)
        cv2.putText(frame, f"CONF {state.confidence:.0%}", (bar_x, bar_y + bar_h + 18), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
        
        # Mode (bottom-center)
        cv2.putText(frame, state.mode, (w // 2 - 60, h - 25), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
        
        return frame
    
    def draw_bubble(self, frame, text, pos, color=(200, 200, 200)):
        """Draw decision explanation bubble."""
        x, y = pos
        lines = text.split('\n')
        
        # Calculate bubble size
        text_size = cv2.getTextSize(lines[0], cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)[0]
        bubble_w = text_size[0] + 20
        bubble_h = len(lines) * 18 + 12
        
        # Draw bubble
        cv2.rectangle(frame, (x - bubble_w//2, y - bubble_h), 
                      (x + bubble_w//2, y), (30, 30, 40), -1)
        cv2.rectangle(frame, (x - bubble_w//2, y - bubble_h), 
                      (x + bubble_w//2, y), color, 1)
        
        # Draw text
        for i, line in enumerate(lines):
            text_y = y - bubble_h + 15 + i * 18
            cv2.putText(frame, line, (x - text_size[0]//2, text_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (240, 240, 240), 1)
        
        return frame
