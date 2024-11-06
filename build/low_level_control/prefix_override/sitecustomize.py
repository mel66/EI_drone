import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/st5dronelab/Desktop/EI_drone/install/low_level_control'
