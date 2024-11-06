import rclpy
from .basic_behavior import BaseBehavior  # Assurez-vous que l'importation est correcte

class FakeHoverBehavior(BaseBehavior):
    def __init__(self):
        super().__init__('FakeHover')

    def on_status(self):
        if self.active:
            self.get_logger().info("FakeHover is now active: Hovering...")
        else:
            self.get_logger().info("FakeHover is now inactive: Stopped hovering.")

def main(args=None):
    rclpy.init(args=args)
    hover_behavior = FakeHoverBehavior()
    rclpy.spin(hover_behavior)
    hover_behavior.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()