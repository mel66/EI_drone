# move_forward.py
from .float_setter import FloatSetter

class MoveForward(FloatSetter):
    def __init__(self):
        super().__init__('MoveForward', 'linear_x', 1.0)

