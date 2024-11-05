# move_forward.py
import rclpy
from .float_setter import FloatSetter
from .command import SLOW_SPEED


class MoveRightBehavior(FloatSetter):
    def __init__(self):
        #super().__init__('MoveRight', 'linear_y', SLOW_SPEED)
        super().__init__('MoveRight', '/bebop/cmd_vel', SLOW_SPEED)


def main(args=None):
    
    rclpy.init(args=args)
    MoveRight = MoveRightBehavior()
    rclpy.spin(MoveRight)
    MoveRight.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()