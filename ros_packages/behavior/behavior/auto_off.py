import rclpy
from rclpy.node import Node
from behavior_interface.msg import BehaviorStatus
from .base_behavior import BaseBehavior

class AutoOffBehavior(BaseBehavior):
    def __init__(self, name, off_duration=2.0):
        super().__init__(name)
        self.off_duration = off_duration  # Time after which behavior deactivates
        self.off_timer = None

    def on_status(self):
        if self.active:
            self.get_logger().info(f"{self.name} is now active.")
            self.request_off()  # Start countdown to turn off
        else:
            self.get_logger().info(f"{self.name} is now inactive.")

    def request_off(self):
        """Start a timer that will turn the behavior off after `off_duration`."""
        if self.off_timer:
            self.off_timer.cancel()  # Cancel any previous timer
        self.off_timer = self.create_timer(self.off_duration, self.turn_off)

    def turn_off(self):
        """Deactivate the behavior and stop the timer."""
        self.active = False
        self.get_logger().info(f"{self.name} deactivated automatically after {self.off_duration} seconds.")
        self.off_timer.cancel()
        self.off_timer = None
        self.send_status()  # Update the behavior status
