import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist

class CenterCorridor(Node):
    def __init__(self):
        super().__init__('center_corridor')
        self.subscriber = self.create_subscription(Point, 'vanishing_point', self.center_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def center_callback(self, msg):
        twist_msg = Twist()
        image_center_y = 240  # Assuming a 640x480 image
        y_offset = msg.y - image_center_y

        # Adjust linear_y to center drone
        twist_msg.linear.y = -0.005 * y_offset  # Scale appropriately
        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    center_corridor = CenterCorridor()
    rclpy.spin(center_corridor)
    center_corridor.destroy_node()
    rclpy.shutdown()
