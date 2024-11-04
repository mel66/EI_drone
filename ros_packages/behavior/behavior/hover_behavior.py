from basic_behavior import BaseBehavior

class HoverBehavior(BaseBehavior):
    def __init__(self):
        super().__init__('Hover')

    def on_status(self):
        if self.active:
            # Logique pour activer Hover
            self.get_logger().info("Hovering...")
        else:
            # Logique pour désactiver Hover
            self.get_logger().info("Stopped hovering.")
