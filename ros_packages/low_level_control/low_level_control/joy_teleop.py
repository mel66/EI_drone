#!/usr/bin/env python
#
# Software License Agreement (BSD)
# ... (license text truncated for brevity)
#

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

# Button and axis mappings
BUTTON_LB = 4
AXIS_LEFT_HORIZONTAL = 0
AXIS_LEFT_VERTICAL = 1

BUTTON_DEADMAN = BUTTON_LB
AXIS_LINEAR = AXIS_LEFT_VERTICAL
AXIS_ANGULAR = AXIS_LEFT_HORIZONTAL

class JoyTeleop(Node):
    def __init__(self):
        super().__init__("joy_teleop")

        # Declare parameters for scaling
        self.declare_parameter("linear_factor", 1)
        self.declare_parameter("angular_factor", 1)

        # Initialize subscribers and publishers
        self.sub_joy = self.create_subscription(Joy, "joy", self.on_joy, 1)
        self.linear_z_pub = self.create_publisher(Float32, "linear_z", 1)
        self.angular_z_pub = self.create_publisher(Float32, "angular_z", 1)

        # Tolerance for joystick deadzone
        self.axis_tolerance = 0.1

    def on_joy(self, msg):
        # Retrieve parameters
        linear_factor = self.get_parameter("linear_factor").value
        angular_factor = self.get_parameter("angular_factor").value

        # Check if the deadman button is pressed
        if msg.buttons[BUTTON_DEADMAN] == 1:
            linear_input = msg.axes[AXIS_LINEAR]
            angular_input = msg.axes[AXIS_ANGULAR]

            # Check if inputs are outside the deadzone
            if abs(linear_input) > self.axis_tolerance or abs(angular_input) > self.axis_tolerance:
                self.get_logger().info("Publishing linear and angular velocities")

                # Create and publish linear_z message
                linear_msg = Float32()
                linear_msg.data = linear_input * linear_factor
                self.linear_z_pub.publish(linear_msg)

                # Create and publish angular_z message
                angular_msg = Float32()
                angular_msg.data = angular_input * angular_factor
                self.angular_z_pub.publish(angular_msg)
            else:
                self.get_logger().info("In deadzone - no movement")
        else:
            # Stop command when deadman button is released
            self.get_logger().info("Publishing stop signal")

            # Publish zero velocity
            stop_msg = Float32()
            stop_msg.data = 0.0
            self.linear_z_pub.publish(stop_msg)
            self.angular_z_pub.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)

    joy_teleop = JoyTeleop()
    rclpy.spin(joy_teleop)

    joy_teleop.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
