# move_forward.py
import rclpy
from .float_setter import FloatSetter

class MoveDownBehavior(FloatSetter):
    def __init__(self):
        #super().__init__('MoveDown', 'linear_z', -0.2)
        super().__init__('MoveDown', '/bebop/cmd_vel', -0.2)


def main(args=None):
    
    rclpy.init(args=args)
    MoveDown = MoveDownBehavior()
    rclpy.spin(MoveDown)
    MoveDown.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()