# move_forward.py
import rclpy
from .float_setter import FloatSetter

class MoveBackwordBehavior(FloatSetter):
    def __init__(self):
        #super().__init__('MoveBackword', 'linear_x', -0.2)
        super().__init__('MoveBackword', '/bebop/cmd_vel', -0.2)


def main(args=None):
    
    rclpy.init(args=args)
    MoveBackword = MoveBackwordBehavior()
    rclpy.spin(MoveBackword)
    MoveBackword.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()