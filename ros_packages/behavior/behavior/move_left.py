# move_left.py
from .float_setter import FloatSetter

class MoveLeft(FloatSetter):
    def __init__(self):
        super().__init__('MoveLeft', 'linear_y', -1.0)
    # <node pkg="behavior" exec="land" name="Land"/>
    # <node pkg="behavior" exec="hover" name="Hover"/>
    # <node pkg="behavior" exec="move_forward" name="MoveForward"/>
    # <node pkg="behavior" exec="move_left" name="MoveLeft"/>