import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/fernando/Documents/miniAGV/agv_ws/install/car_drivers'
