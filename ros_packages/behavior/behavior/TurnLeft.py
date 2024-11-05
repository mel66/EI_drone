# move_forward.py
import rclpy
from .float_setter import FloatSetter
from .command import SLOW_SPEED


class TurnLeftBehavior(FloatSetter):
    def __init__(self):
        #super().__init__('TurnLeft', 'angular', -SLOW_SPEED)
        super().__init__('TurnLeft', '/bebop/cmd_vel', -SLOW_SPEED)


def main(args=None):
    
    rclpy.init(args=args)
    TurnLeft = TurnLeftBehavior()
    rclpy.spin(TurnLeft)
    TurnLeft.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()