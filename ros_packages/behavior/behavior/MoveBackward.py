# move_forward.py
import rclpy
from .float_setter import FloatSetter
from .command import SLOW_SPEED


class MoveBackwordBehavior(FloatSetter):
    def __init__(self):
        #super().__init__('MoveBackword', 'linear_x', -SLOW_SPEED)
        super().__init__('MoveBackword', '/bebop/cmd_vel', -SLOW_SPEED)


def main(args=None):
    
    rclpy.init(args=args)
    MoveBackword = MoveBackwordBehavior()
    rclpy.spin(MoveBackword)
    MoveBackword.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()