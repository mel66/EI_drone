# land.py
import rclpy
from std_msgs.msg import Empty
from .auto_off import AutoOffBehavior

class LandBehavior(AutoOffBehavior):
    def __init__(self):
        super().__init__('Land', off_duration=2.0)
        self.land_publisher = self.create_publisher(Empty, 'land', 10)

    def on_status(self):
        if self.active:
            self.get_logger().info("Land is active: Initiating landing.")
            self.land_publisher.publish(Empty())
            self.request_off()
        else:
            self.get_logger().info("Land is now inactive.")
