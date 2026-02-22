import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/lincoln/Marble_Balancing_Robotic_Arm/40_Simulation/cs558/install/OldOne_description'
