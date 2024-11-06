import rclpy
from rclpy.node import Node
from behavior_interface.msg import BehaviorStatus
from .basic_behavior import BaseBehavior

class AutoOffBehavior(BaseBehavior):
    def __init__(self, name, off_duration=2.0):
        super().__init__(name)
        self.off_duration = off_duration  # Time after which behavior deactivates
        self.off_timer = None
        self.off_timer = self.create_timer(self.off_duration, self.turn_off)
        self.off_timer.cancel()  # Cancel any previous timer



    def on_status(self):
        if self.active:
            self.off_timer.reset()
            self.get_logger().info(f"{self.name} is now active.")
        else:
            self.get_logger().info(f"{self.name} is now inactive.")


    def turn_off(self):
        """Deactivate the behavior and stop the timer."""
        self.active = False
        self.get_logger().info(f"{self.name} deactivated automatically after {self.off_duration} seconds.")
        self.on_status()
        self.off_timer.cancel()
