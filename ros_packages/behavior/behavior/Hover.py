# hover.py
import rclpy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

from .basic_behavior import BaseBehavior

class HoverBehavior(BaseBehavior):
    def __init__(self):
        super().__init__('Hover')
        self.hover_publisher = self.create_publisher(Bool, 'hover', 10)
        self.pub_zero = self.create_publisher(Twist, '/bebop/cmd_vel', 10)

    def on_status(self):
        if self.active:
            self.get_logger().info("Hover is active.")
            self.hover_publisher.publish(Bool(data=True))
        else:
            self.pub_zero.publish(Twist())
            self.get_logger().info("Hover is now inactive.")


def main(args=None):
    
    rclpy.init(args=args)
    Hover = HoverBehavior()
    rclpy.spin(Hover)
    Hover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()