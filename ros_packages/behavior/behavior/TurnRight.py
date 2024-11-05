# move_forward.py
import rclpy
from .float_setter import FloatSetter

class TurnRightBehavior(FloatSetter):
    def __init__(self):
        #super().__init__('TurnRight', 'angular_z', 0.2)
        super().__init__('TurnRight', '/bebop/cmd_vel', 0.2)


def main(args=None):
    
    rclpy.init(args=args)
    TurnRight = TurnRightBehavior()
    rclpy.spin(TurnRight)
    TurnRight.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()