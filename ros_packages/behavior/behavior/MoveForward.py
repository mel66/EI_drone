# move_forward.py
import rclpy
from .float_setter import FloatSetter

class MoveForwardBehavior(FloatSetter):
    def __init__(self):
        #super().__init__('MoveForward', 'linear_x', 1.0)
        super().__init__('MoveForward', '/bebop/cmd_vel', 1.0)


def main(args=None):
    
    rclpy.init(args=args)
    MoveForward = MoveForwardBehavior()
    rclpy.spin(MoveForward)
    MoveForward.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()