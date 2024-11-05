# move_forward.py
import rclpy
from .float_setter import FloatSetter

class TurnLeftBehavior(FloatSetter):
    def __init__(self):
        #super().__init__('TurnLeft', 'angular', -0.2)
        super().__init__('TurnLeft', '/bebop/cmd_vel', -0.2)


def main(args=None):
    
    rclpy.init(args=args)
    TurnLeft = TurnLeftBehavior()
    rclpy.spin(TurnLeft)
    TurnLeft.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()