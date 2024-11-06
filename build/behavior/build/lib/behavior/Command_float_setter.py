# move_forward.py
import rclpy
from .float_setter import FloatSetter
from .command import SLOW_SPEED


class MoveBackwardBehavior(FloatSetter):
    def __init__(self):
        super().__init__('MoveBackward', 'linear_x', -SLOW_SPEED)
        # super().__init__('MoveBackward', '/bebop/cmd_vel', -SLOW_SPEED)


def MoveBackwardmain(args=None):
    
    rclpy.init(args=args)
    MoveBackward = MoveBackwardBehavior()
    rclpy.spin(MoveBackward)
    MoveBackward.destroy_node()
    rclpy.shutdown()



class MoveForwardBehavior(FloatSetter):
    def __init__(self):
        super().__init__('MoveForward', 'linear_x', SLOW_SPEED)
        # super().__init__('MoveForward', '/bebop/cmd_vel', SLOW_SPEED)


def MoveForwardmain(args=None):
    
    rclpy.init(args=args)
    MoveForward = MoveForwardBehavior()
    rclpy.spin(MoveForward)
    MoveForward.destroy_node()
    rclpy.shutdown()



class MoveDownBehavior(FloatSetter):
    def __init__(self):
        super().__init__('MoveDown', 'linear_z', -SLOW_SPEED/2)
        # super().__init__('MoveDown', '/bebop/cmd_vel', -SLOW_SPEED)


def MoveDownmain(args=None):
    
    rclpy.init(args=args)
    MoveDown = MoveDownBehavior()
    rclpy.spin(MoveDown)
    MoveDown.destroy_node()
    rclpy.shutdown()


class MoveUpBehavior(FloatSetter):
    def __init__(self):
        super().__init__('MoveUp', 'linear_z', SLOW_SPEED/2)
        # super().__init__('MoveUp', '/bebop/cmd_vel', SLOW_SPEED)


def MoveUpmain(args=None):
    
    rclpy.init(args=args)
    MoveUp = MoveUpBehavior()
    rclpy.spin(MoveUp)
    MoveUp.destroy_node()
    rclpy.shutdown()


class MoveLeftBehavior(FloatSetter):
    def __init__(self):
        super().__init__('MoveLeft', 'linear_y', -SLOW_SPEED)
        # super().__init__('MoveLeft', '/bebop/cmd_vel', -SLOW_SPEED)


def MoveLeftmain(args=None):
    
    rclpy.init(args=args)
    MoveLeft = MoveLeftBehavior()
    rclpy.spin(MoveLeft)
    MoveLeft.destroy_node()
    rclpy.shutdown()



class MoveRightBehavior(FloatSetter):
    def __init__(self):
        super().__init__('MoveRight', 'linear_y', SLOW_SPEED)
        # super().__init__('MoveRight', '/bebop/cmd_vel', SLOW_SPEED)


def MoveRightmain(args=None):
    
    rclpy.init(args=args)
    MoveRight = MoveRightBehavior()
    rclpy.spin(MoveRight)
    MoveRight.destroy_node()
    rclpy.shutdown()



class TurnRightBehavior(FloatSetter):
    def __init__(self):
        super().__init__('TurnRight', 'angular_z', SLOW_SPEED/2)
        # super().__init__('TurnRight', '/bebop/cmd_vel', SLOW_SPEED)


def TurnRightmain(args=None):
    
    rclpy.init(args=args)
    TurnRight = TurnRightBehavior()
    rclpy.spin(TurnRight)
    TurnRight.destroy_node()
    rclpy.shutdown()




class TurnLeftBehavior(FloatSetter):
    def __init__(self):
        super().__init__('TurnLeft', 'angular_z', -SLOW_SPEED/2)
        # super().__init__('TurnLeft', '/bebop/cmd_vel', SLOW_SPEED)


def TurnLeftmain(args=None):
    
    rclpy.init(args=args)
    TurnLeft = TurnLeftBehavior()
    rclpy.spin(TurnLeft)
    TurnLeft.destroy_node()
    rclpy.shutdown()
