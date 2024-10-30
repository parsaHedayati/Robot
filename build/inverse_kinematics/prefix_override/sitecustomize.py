import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/parsa/ros2_urdf_ws/install/inverse_kinematics'
