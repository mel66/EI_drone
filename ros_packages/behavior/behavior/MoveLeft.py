# move_left.py
import rclpy
from .float_setter import FloatSetter

class MoveLeftBehavior(FloatSetter):
    def __init__(self):
        # super().__init__('MoveLeft', 'linear_y', -1.0)
        super().__init__('MoveLeft', '/bebop/cmd_vel', -1.0)


def main(args=None):
    
    rclpy.init(args=args)
    MoveLeft = MoveLeftBehavior()
    rclpy.spin(MoveLeft)
    MoveLeft.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()