# move_forward.py
import rclpy
from .float_setter import FloatSetter
from .command import SLOW_SPEED


class MoveUpBehavior(FloatSetter):
    def __init__(self):
        #super().__init__('MoveUp', 'linear_z', SLOW_SPEED)
        super().__init__('MoveUp', '/bebop/cmd_vel', SLOW_SPEED)


def main(args=None):
    
    rclpy.init(args=args)
    MoveUp = MoveUpBehavior()
    rclpy.spin(MoveUp)
    MoveUp.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()