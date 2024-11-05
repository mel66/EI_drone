#!/usr/bin/env python

import rclpy
from rclpy.node import Node

# import the message types you need
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist

# For image processing
import numpy as np
import cv2
from cv_bridge import CvBridge

class Speed_Controller(Node):
    def __init__(self):  
        super().__init__("speed_controller")

        # Subscribers
        self.hover_sub = self.create_subscription(Bool, "hover", self.hover_callback, 1)
        self.linear_x_sub = self.create_subscription(Float32, "linear_x", self.linear_x_callback, 1)
        self.linear_y_sub = self.create_subscription(Float32, "linear_y", self.linear_y_callback, 1)
        self.linear_z_sub = self.create_subscription(Float32, "linear_z", self.linear_z_callback, 1)
        self.angular_z_sub = self.create_subscription(Float32, "angular_z", self.angular_z_callback, 1)

        # Publishers
        self.hover_pub = self.create_publisher(Bool, "hover", 1)
        self.target_vel = self.create_publisher(Twist, "target_vel", 1)

        # Store the received values for velocity
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0
        self.angular_z = 0.0
        self.hover = True
        self.create_timer(0.1, callback=self.transform)
        self.get_logger().info("Speed_Controller node has been started")

    def hover_callback(self, msg):
        # Callback for hover subscription
        self.hover = msg.data
        self.get_logger().info(f"Received hover message: {msg.data}")

    def linear_x_callback(self, msg):
        self.linear_x = msg.data

    def linear_y_callback(self, msg):
        self.linear_y = msg.data

    def linear_z_callback(self, msg):
        self.linear_z = msg.data
        self.get_logger().info(f"Received hover message: {msg.data}")

    def angular_z_callback(self, msg):
        self.angular_z = msg.data
        
        
    def hover_publish(self):
        self.get_logger().info("Publishing hover message: False")
        msg = Bool()
        msg.data = False
        self.hover_pub.publish(msg)

    def transform(self): 
        if not(self.hover):
            self.get_logger().info("Publishing velocity command")
            twist = Twist()
            twist.linear.x = self.linear_x
            print(self.linear_z)
            twist.linear.y = self.linear_y
            twist.linear.z = self.linear_z 
            twist.angular.z = self.angular_z
            self.get_logger().info(f"Received target vel message : {twist}")
            self.target_vel.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    my_node = Speed_Controller()
    rclpy.spin(my_node)  # This is a blocking function

    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
