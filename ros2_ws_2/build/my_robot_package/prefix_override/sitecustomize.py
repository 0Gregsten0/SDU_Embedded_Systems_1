import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/gregsten/Documents/Embedded_ROS/ros2_ws_2/install/my_robot_package'
