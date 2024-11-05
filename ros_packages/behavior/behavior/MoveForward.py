# move_forward.py
import rclpy
from .float_setter import FloatSetter
from .command import SLOW_SPEED


class MoveForwardBehavior(FloatSetter):
    def __init__(self):
        #super().__init__('MoveForward', 'linear_x', SLOW_SPEED)
        super().__init__('MoveForward', '/bebop/cmd_vel', SLOW_SPEED)


def main(args=None):
    
    rclpy.init(args=args)
    MoveForward = MoveForwardBehavior()
    rclpy.spin(MoveForward)
    MoveForward.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()