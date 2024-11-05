# move_forward.py
import rclpy
from .float_setter import FloatSetter

class MoveUpBehavior(FloatSetter):
    def __init__(self):
        #super().__init__('MoveUp', 'linear_z', 0.2)
        super().__init__('MoveUp', '/bebop/cmd_vel', 0.2)


def main(args=None):
    
    rclpy.init(args=args)
    MoveUp = MoveUpBehavior()
    rclpy.spin(MoveUp)
    MoveUp.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()