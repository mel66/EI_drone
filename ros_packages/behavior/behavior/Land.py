# land.py
import rclpy
from std_msgs.msg import Empty
from .auto_off import AutoOffBehavior

class LandBehavior(AutoOffBehavior):
    def __init__(self):
        super().__init__('Land', off_duration=2.0)
        self.land_publisher = self.create_publisher(Empty, '/bebop/land', 10)
        self.hover_pub = self.create_publisher(Empty, '/bebop/hover', 10)

    def on_status(self):
        if self.active:
            self.off_timer.reset()
            self.get_logger().info("Land is active: Initiating landing.")
            self.hover_pub.publish(Empty())
            self.land_publisher.publish(Empty())
            
          
        else:
            self.get_logger().info("Land is now inactive.")

def main(args=None):
    
    rclpy.init(args=args)
    Land = LandBehavior()
    rclpy.spin(Land)
    Land.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()