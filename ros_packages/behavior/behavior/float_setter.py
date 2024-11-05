# float_setter.py
import rclpy
from std_msgs.msg import Float32
from .auto_off import AutoOffBehavior

class FloatSetter(AutoOffBehavior):
    def __init__(self, name, topic, value, duration=2.0):
        super().__init__(name, off_duration=duration)
        self.publisher = self.create_publisher(Float32, topic, 10)
        self.value = value

    def on_status(self):
        if self.active:
            self.off_timer.reset()

            self.get_logger().info(f"{self.name} active: Setting {self.value} on {self.publisher.topic_name}.")
            self.publisher.publish(Float32(data=self.value))
        else:
            self.get_logger().info(f"{self.name} inactive.")
