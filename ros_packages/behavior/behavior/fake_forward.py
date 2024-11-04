import rclpy
from .basic_behavior import BaseBehavior  # Assurez-vous que l'importation est correcte

class FakeForwardBehavior(BaseBehavior):
    def __init__(self):
        super().__init__('FakeForward')

    def on_status(self):
        if self.active:
            self.get_logger().info("FakeForward is now active: Moving forward...")
        else:
            self.get_logger().info("FakeForward is now inactive: Stopped moving forward.")

def main(args=None):
    
    rclpy.init(args=args)
    forward_behavior = FakeForwardBehavior()
    rclpy.spin(forward_behavior)
    forward_behavior.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()