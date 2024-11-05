# move_forward.py
import rclpy
from .float_setter import FloatSetter
from .command import SLOW_SPEED


class TurnRightBehavior(FloatSetter):
    def __init__(self):
        #super().__init__('TurnRight', 'angular_z', SLOW_SPEED)
        super().__init__('TurnRight', '/bebop/cmd_vel', SLOW_SPEED)


def main(args=None):
    
    rclpy.init(args=args)
    TurnRight = TurnRightBehavior()
    rclpy.spin(TurnRight)
    TurnRight.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()