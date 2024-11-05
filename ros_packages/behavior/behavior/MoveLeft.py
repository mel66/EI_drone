# move_left.py
import rclpy
from .float_setter import FloatSetter
from .command import SLOW_SPEED


class MoveLeftBehavior(FloatSetter):
    def __init__(self):
        # super().__init__('MoveLeft', 'linear_y', -SLOW_SPEED)
        super().__init__('MoveLeft', '/bebop/cmd_vel', -SLOW_SPEED)


def main(args=None):
    
    rclpy.init(args=args)
    MoveLeft = MoveLeftBehavior()
    rclpy.spin(MoveLeft)
    MoveLeft.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()