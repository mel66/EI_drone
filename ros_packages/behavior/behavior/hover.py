# hover.py
import rclpy
from std_msgs.msg import Empty
from .auto_off import AutoOffBehavior

class HoverBehavior(AutoOffBehavior):
    def __init__(self):
        super().__init__('Hover', off_duration=2.0)
        self.hover_publisher = self.create_publisher(Empty, 'hover', 10)

    def on_status(self):
        if self.active:
            self.get_logger().info("Hover is active.")
            self.hover_publisher.publish(Empty())
            self.request_off()
        else:
            self.get_logger().info("Hover is now inactive.")
