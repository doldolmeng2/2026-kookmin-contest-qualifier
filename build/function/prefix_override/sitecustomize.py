import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/xytron/2026-kookmin-contest-qualifier/install/function'
