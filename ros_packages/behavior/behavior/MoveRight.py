# move_forward.py
import rclpy
from .float_setter import FloatSetter

class MoveRightBehavior(FloatSetter):
    def __init__(self):
        #super().__init__('MoveRight', 'linear_y', 0.2)
        super().__init__('MoveRight', '/bebop/cmd_vel', 0.2)


def main(args=None):
    
    rclpy.init(args=args)
    MoveRight = MoveRightBehavior()
    rclpy.spin(MoveRight)
    MoveRight.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()