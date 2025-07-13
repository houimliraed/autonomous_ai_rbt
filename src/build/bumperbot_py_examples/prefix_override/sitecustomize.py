import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/install/bumperbot_py_examples'
