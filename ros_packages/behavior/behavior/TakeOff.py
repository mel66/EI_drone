# takeoff.py
import rclpy
from std_msgs.msg import Empty
from .auto_off import AutoOffBehavior

class TakeOffBehavior(AutoOffBehavior):
    def __init__(self):
        super().__init__('TakeOff', off_duration=2.0)
        self.takeoff_publisher = self.create_publisher(Empty, 'take_off', 10)

    def on_status(self):
        if self.active:
            self.get_logger().info("TakeOff is active: Initiating takeoff.")
            self.takeoff_publisher.publish(Empty())
            self.request_off()
        else:
            self.get_logger().info("TakeOff is now inactive.")
