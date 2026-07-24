#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2026 Westwood Robotics Corporation"
__date__      = "July 29, 2025"
__project__   = "PyBEAR"
__version__   = "0.1.3"
__status__    = "Production"

'''
Test bulk_read speed
'''

import time
from pybear import Manager


BEAR_LIST = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

port = 'COM11'
baud = 8000000
pbm = Manager.BEAR(port=port, baudrate=baud, bulk_timeout=0.002)

start_time = time.time()
for i in range(1000):
    pbm.bulk_read(BEAR_LIST, ['present_position', 'present_velocity', 'present_iq'])
end_time = time.time()
freq = 1000 / (end_time - start_time)

print("bulk_read frequency: %2.4f Hz" % freq)