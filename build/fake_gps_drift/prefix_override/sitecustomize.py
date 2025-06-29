import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/boko/airship_simulation_ros_jazzy/install/fake_gps_drift'
