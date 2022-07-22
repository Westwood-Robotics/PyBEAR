#!usr/bin/env python
__author__ = "Xiaoguang Zhang"
__email__ = "xzhang@westwoodrobotics.io"
__copyright__ = "Copyright 2022 Westwood Robotics"
__date__ = "July 18, 2022"
__version__ = "0.0.1"
__status__ = "Prototype"

# Test bulk_comm speed

from pybear import Manager
import time

BEAR_LIST = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

port = 'COM11'
baud = 8000000
pbm = Manager.BEAR(port=port, baudrate=baud, bulk_timeout=0.002)


start_time = time.time()
for i in range(1000):
    pbm.bulk_read(BEAR_LIST, ['present_position', 'present_velocity', 'present_iq'])
end_time = time.time()
freq = 1000/(end_time-start_time)

print("Bulk_comm frequency: %2.4f" % freq)


