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

import datetime
import numpy as np

#odometry
from nav_msgs.msg import Odometry

class PID:

    def __init__(self, gains):
        '''
        Builds a PID

        Arguments:
            gains (dict): with keys Kp, Kd, Ki
        '''
        self.gains = gains.copy()
        self.errors = {'error': 0.,
                       'd_error': 0.,
                       'i_error': 0.,
                       'time': None
                      }
        if not ('Kp' in gains and 'Kd' in gains and 'Ki' in gains):
            raise RuntimeError('''Some keys are missing in the gains, '''
                               '''we expect Kp, Kd and Ki''')


    def reset(self):
        '''
        Reset the errors of the PID
        '''
        self.errors = {'error': 0.,
                       'd_error': 0.,
                       'i_error': 0.,
                       'time': None
                      }

    def update(self,
               time: float,
               error: float):
        '''
        Update the error, its derivative and integral
        '''
        prev_error = self.errors['error']
        prev_time = self.errors['time']
        if prev_time is None:
            self.errors['error'], self.errors['time'] = error, time
            return

        self.errors['error'] = error
        self.errors['time'] = time
        dt = (time - prev_time)
        self.errors['i_error'] += dt * (prev_error + error)/2.0
        self.errors['d_error'] = (error - prev_error)/dt

    @property
    def command(self):
        return self.gains['Kp'] * self.errors['error'] + \
                self.gains['Kd'] * self.errors['d_error'] + \
                self.gains['Ki'] * self.errors['i_error']



class Speed_Controller(Node):
    def __init__(self):  
        super().__init__("speed_controller")

        # Subscribers
        self.hover_sub = self.create_subscription(Bool, "hover", self.hover_callback, 10)
        self.linear_x_sub = self.create_subscription(Float32, "linear_x", self.linear_x_callback, 10)
        self.linear_y_sub = self.create_subscription(Float32, "linear_y", self.linear_y_callback, 10)
        self.linear_z_sub = self.create_subscription(Float32, "linear_z", self.linear_z_callback, 10)
        self.angular_z_sub = self.create_subscription(Float32, "angular_z", self.angular_z_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, "/bebop/odom", self.on_odom, qos_profile=1)

        # Publishers

        self.target_vel = self.create_publisher(Twist, "/bebop/cmd_vel", 10)

        # Store the received values for velocity
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0
        self.angular_z = 0.0
        self.hover = True
        self.odom = None
        self.create_timer(0.1, callback=self.transform)
        self.get_logger().info("Speed_Controller node has been started")

        #PIDs
        #create PID for twistx and twisty
        
        Ku1 = 1
        Tu1 = 4.0

        Kp1 = 0.9
        Kd1 = 0#.125*Kp1*Tu1
        Ki1 =  0#2*Kp1/Tu1

        Ku2 = 0.0
        Tu2 = 0.0

        Kp2 = 0.9 #0.6*Ku2
        Kd2 = 0 #0.125*Kp2*Tu2
        Ki2 = 0 #2*Kp2/Tu2
        self.gains_x = {'Kp': Kp1, 'Kd': Kd1 , 'Ki': Ki1}
        self.gains_y = {'Kp': Kp2, 'Kd': Kd2, 'Ki': Ki2 }
        self.pid_x = PID(self.gains_x)
        self.pid_y = PID(self.gains_y)
        self.t0 = self.get_clock().now()
        self.t1 = self.get_clock().now()
        self.t = (self.t1 - self.t0).nanoseconds*1e-9 
    
    def on_odom(self, msg):
        self.odom = msg.twist.twist
        # self.get_logger().info(f"translated odom: {self.odom}")

    def hover_callback(self, msg):
        # Callback for hover subscription
        self.hover = msg.data
        # self.get_logger().info(f"Received hover message: {msg.data}")

    def linear_x_callback(self, msg):
        self.hover = False
        self.linear_x = msg.data

    def linear_y_callback(self, msg):
        self.hover = False
        self.linear_y = msg.data

    def linear_z_callback(self, msg):
        self.hover = False
        self.linear_z = msg.data
        # self.get_logger().info(f"Received hover message: {msg.data}")

    def angular_z_callback(self, msg):
        self.hover = False
        self.angular_z = msg.data
        
        
    def hover_publish(self):
        # self.get_logger().info("Publishing hover message: False")
        msg = Bool()
        msg.data = False
        self.hover_pub.publish(msg)

    def transform(self):
        if not(self.hover) and self.odom!=None:

            twist = Twist()
            twist.linear.x = self.linear_x
            twist.linear.y = self.linear_y
            twist.linear.z = self.linear_z 
            twist.angular.z = self.angular_z
            # self.get_logger().info(f"Received target vel message : {twist}")
            

            #using pid
            self.t1 = self.get_clock().now()
            self.t = (self.t1 - self.t0).nanoseconds*1e-9 
            self.pid_x.update(self.t, twist.linear.x - self.odom.linear.x)
            self.pid_y.update(self.t, twist.linear.y - self.odom.linear.y)
            Fx = self.pid_x.command
            Fy = self.pid_y.command
            twist.linear.x += Fx
            twist.linear.y += Fy

            #publishing corrected command
            # self.get_logger().info(f"Publishing PIDcorrected velocity command {twist}")
            self.target_vel.publish(twist)


        else:
            self.pid_x.reset()
            self.pid_y.reset()
            self.t0 = self.get_clock().now()
            self.t1 = self.get_clock().now()
            self.t = (self.t1 - self.t0).nanoseconds*1e-9 
            twist = Twist()
            self.target_vel.publish(twist)


def main(args=None):


    rclpy.init(args=args)

    my_node = Speed_Controller()
    rclpy.spin(my_node)  # This is a blocking function

    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
