import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/Gery/Marble_Balancing_Robotic_Arm/40_Simulation/MBR_ws/install/Marble_control'
