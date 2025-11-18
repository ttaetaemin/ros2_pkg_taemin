import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/addinedu/pinky/src/install/pinky_simple_navigator'
