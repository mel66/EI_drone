from basic_behavior import BaseBehavior

class ForwardBehavior(BaseBehavior):
    def __init__(self):
        super().__init__('Forward')

    def on_status(self):
        if self.active:
            # Logique pour avancer
            self.get_logger().info("Moving forward...")
        else:
            # Logique pour arrêter l'avance
            self.get_logger().info("Stopped moving forward.")
