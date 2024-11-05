# move_forward.py
import rclpy
from .float_setter import FloatSetter
from .command import SLOW_SPEED


class MoveDownBehavior(FloatSetter):
    def __init__(self):
        #super().__init__('MoveDown', 'linear_z', -SLOW_SPEED)
        super().__init__('MoveDown', '/bebop/cmd_vel', -SLOW_SPEED)


def main(args=None):
    
    rclpy.init(args=args)
    MoveDown = MoveDownBehavior()
    rclpy.spin(MoveDown)
    MoveDown.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()