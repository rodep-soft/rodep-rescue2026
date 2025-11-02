import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/root/microros_ws/firmware/dev_ws/install/ament_pycodestyle'
