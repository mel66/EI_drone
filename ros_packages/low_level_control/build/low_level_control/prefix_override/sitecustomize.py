import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pierres/EI_drone/ros_packages/low_level_control/install/low_level_control'
