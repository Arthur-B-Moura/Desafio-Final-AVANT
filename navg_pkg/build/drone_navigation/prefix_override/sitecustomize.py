import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/gabriel/Desafio-Final-AVANT/navg_pkg/install/drone_navigation'
