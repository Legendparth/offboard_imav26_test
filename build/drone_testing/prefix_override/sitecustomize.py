import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ark-jetson-orin/imav26_ws/src/drone_testing/install/drone_testing'
