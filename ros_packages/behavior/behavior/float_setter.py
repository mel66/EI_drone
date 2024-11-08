# float_setter.py
import rclpy
from std_msgs.msg import Float32
from .basic_behavior import BaseBehavior

from geometry_msgs.msg import Twist


class FloatSetter(BaseBehavior):
    def __init__(self, name, topic, value):
        super().__init__(name)
        self.publisher = self.create_publisher(Float32, topic, 10)

        self.value = value
       
        

    def on_status(self):
        if self.active:

            self.get_logger().info(f"{self.name} active: Setting {self.value} on {self.publisher.topic_name}.")
            self.publisher.publish(Float32(data=self.value))

        else:
            pass
            # self.publisher.publish(Float32(data=0.0))
            self.get_logger().info(f"{self.name} inactive.")
