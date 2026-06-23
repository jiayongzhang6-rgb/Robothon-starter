"""Event-driven architecture for AEGIS v6."""


class EventBus:
    """Simple pub/sub event system."""
    
    def __init__(self):
        self.subscribers = {}
    
    def subscribe(self, event_type, fn):
        """Register callback for event type."""
        self.subscribers.setdefault(event_type, []).append(fn)
    
    def emit(self, event_type, payload=None):
        """Emit event to all subscribers."""
        if event_type in self.subscribers:
            for fn in self.subscribers[event_type]:
                fn(payload)
    
    def clear(self):
        """Remove all subscribers."""
        self.subscribers.clear()
