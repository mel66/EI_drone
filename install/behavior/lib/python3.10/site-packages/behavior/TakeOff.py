# takeoff.py
import rclpy
from std_msgs.msg import Empty
from .auto_off import AutoOffBehavior

class TakeOffBehavior(AutoOffBehavior):
    def __init__(self):
        super().__init__('TakeOff', off_duration=2.0)
        self.takeoff_publisher = self.create_publisher(Empty, '/bebop/takeoff', 10)

    def on_status(self):
        if self.active:
            self.off_timer.reset()
            self.get_logger().info("TakeOff is active: Initiating takeoff.")
            self.takeoff_publisher.publish(Empty())
        
        else:
            self.get_logger().info("TakeOff is now inactive.")

def main(args=None):
    
    rclpy.init(args=args)
    TakeOff = TakeOffBehavior()
    rclpy.spin(TakeOff)
    TakeOff.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()