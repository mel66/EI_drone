# move_left.py
import rclpy
from .float_setter import FloatSetter

class MoveLeft(FloatSetter):
    def __init__(self):
        super().__init__('MoveLeft', 'linear/y', -1.0)


def main(args=None):
    
    rclpy.init(args=args)
    MoveLeft = MoveLeft()
    rclpy.spin(MoveLeft)
    MoveLeft.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()