# move_left.py
from .float_setter import FloatSetter

class MoveLeft(FloatSetter):
    def __init__(self):
        super().__init__('MoveLeft', 'linear_y', -1.0)
