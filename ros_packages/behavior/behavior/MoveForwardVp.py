import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist

class MoveForwardVp(Node):
    def __init__(self):
        super().__init__('move_forward_vp')
        self.subscriber = self.create_subscription(Point, 'vanishing_point', self.forward_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def forward_callback(self, msg):
        twist_msg = Twist()
        
        if msg:
            twist_msg.linear.x = 0.2  # Move forward at a constant speed
        else:
            twist_msg.linear.x = 0  # Stop if no vanishing point is detected
            
        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    move_forward_vp = MoveForwardVp()
    rclpy.spin(move_forward_vp)
    move_forward_vp.destroy_node()
    rclpy.shutdown()
