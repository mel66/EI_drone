# hover.py
import rclpy
from std_msgs.msg import Empty
from .basic_behavior import BaseBehavior

class HoverBehavior(BaseBehavior):
    def __init__(self):
        super().__init__('Hover')
        self.hover_publisher = self.create_publisher(Empty, 'hover', 10)

    def on_status(self):
        if self.active:
            self.get_logger().info("Hover is active.")
            self.hover_publisher.publish(Empty())
        else:
            self.get_logger().info("Hover is now inactive.")


def main(args=None):
    
    rclpy.init(args=args)
    Hover = HoverBehavior()
    rclpy.spin(Hover)
    Hover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()