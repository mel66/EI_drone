import rclpy
from rclpy.node import Node
from geometry_msgs.msg import  Twist
from std_msgs.msg import Float32


class AlignCorridor(Node):
    def __init__(self):
        super().__init__('align_corridor')
        self.horizontal_offset_sub = self.create_subscription(Float32, 'vp_offset',self.align_callback, 10)
        self.angle_ratio_sub = self.create_subscription(Float32, 'vp_angle',self.align_callback, 10)
        self.publisher = self.create_publisher(Float32, 'angular_z', 10)

    def align_callback(self, msg):
        twist_msg = Twist()
        image_center_x = 320  # Assuming a 640x480 image
        x_offset = msg.x - image_center_x

        # Adjust angular_z to align drone
        twist_msg.angular.z = -0.005 * x_offset  # Scale appropriately
        self.publisher.publish(twist_msg)

def AlignCorridormain(args=None):
    rclpy.init(args=args)
    align_corridor = AlignCorridor()
    rclpy.spin(align_corridor)
    align_corridor.destroy_node()
    rclpy.shutdown()



