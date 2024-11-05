#!/usr/bin/env python
#
# Software License Agreement (BSD)
#
# \file      joy_teleop.py
# \authors   Jeremy Fix <jeremy.fix@centralesupelec.fr>
# \copyright Copyright (c) 2022, CentraleSupélec, All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation and/or
#    other materials provided with the distribution.
#  * Neither the name of Autonomy Lab nor the names of its contributors may be
#    used to endorse or promote products derived from this software without specific
#    prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WAR- RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, IN- DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
# ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist,Vector3
BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 2
BUTTON_Y = 3
BUTTON_LB = 4
BUTTON_RB = 5
BUTTON_BACK = 6
BUTTON_SELECT = 7
BUTTON_LOGITECH = 8
BUTTON_CLICK_LEFT_PAD = 9
BUTTON_CLICK_RIGHT_PAD = 10

AXIS_LEFT_HORIZONTAL = 0
AXIS_LEFT_VERTICAL = 1
AXIS_LT = 2
AXIS_RIGHT_HORIZONTAL = 3
AXIS_RIGHT_VERTICAL = 4
AXIS_RT = 5
AXIS_CROSS_HORIZONTAL = 6
AXIS_CROSS_VERTICAL = 7

BUTTON_DEADMAN = BUTTON_LB
AXIS_LINEAR = AXIS_LEFT_VERTICAL
AXIS_ANGULAR = AXIS_LEFT_HORIZONTAL


class JoyTeleop(Node):
    def __init__(self):
        super().__init__("joy_teleop")

        self.declare_parameter("linear_factor", 0.2)
        self.declare_parameter("angular_factor", 0.2)

        self.sub_joy = self.create_subscription(Joy, "joy", self.on_joy, 1)
        self.cmd_pub = self.create_publisher(Twist, "target_vel", 1)

        self.axis_tolerance = 0.1
       

    def on_joy(self,msg):

        linear_factor = self.get_parameter("linear_factor").value
        angular_factor = self.get_parameter("angular_factor").value
        if msg.buttons[BUTTON_DEADMAN]==1:
            linear_input = msg.axes[AXIS_LINEAR]
            angular_input = msg.axes[AXIS_ANGULAR]
            if abs(linear_input) > self.axis_tolerance or abs(angular_input)>self.axis_tolerance: 
                self.get_logger().info("Publishing : Linear")
                twist = Twist()
                twist.linear.x = linear_input*linear_factor 
                twist.angular.z = angular_factor*angular_input
                print(twist)
                self.cmd_pub.publish(twist)
                
            else:
                self.get_logger().info("Deadzone")
        else:
                self.get_logger().info("Publishing stop")
                twist = Twist()
                self.cmd_pub.publish(twist)
                
def main(args=None):
    rclpy.init(args=args)

    joy_teleop = JoyTeleop()
    rclpy.spin(joy_teleop)

    joy_teleop.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()