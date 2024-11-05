# float_setter.py
import rclpy
from std_msgs.msg import Float32
from .auto_off import AutoOffBehavior
from geometry_msgs.msg import Twist


class FloatSetter(AutoOffBehavior):
    def __init__(self, name, topic, value, duration=2.0):
        super().__init__(name, off_duration=duration)
        #self.publisher = self.create_publisher(Float32, topic, 10)
        self.publisher = self.create_publisher(Twist, topic, 10)

        self.value = value
        self.twist= Twist()

        if name == "MoveForward" or "MoveBackward":
            self.twist.linear.x = self.value  # Avancer
        elif name == "MoveRight" or "MoveLeft":
            self.twist.linear.y = self.value  # Aller à droite
    
        elif name == "MoveUp" or "MoveDown":
            self.twist.linear.z = self.value  # Monter
        
        elif name == "TurnLeft" or "TurnRight":
            self.twist.angular.z = self.value  # Tourner à gauche
        

    def on_status(self):
        if self.active:
            self.off_timer.reset()

            self.get_logger().info(f"{self.name} active: Setting {self.value} on {self.publisher.topic_name}.")
            # self.publisher.publish(Float32(data=self.value))
            self.publisher.publish(self.twist)

        else:
            self.get_logger().info(f"{self.name} inactive.")
