#!usr/bin/env python
__author__ = "Xiaoguang Zhang"
__email__ = "xzhang@westwoodrobotics.io"
__copyright__ = "Copyright 2020 Westwood Robotics"
__date__ = "Jan 8, 2021"
__version__ = "0.1.3"
__status__ = "Production"

# Ping and search for available BEARs
from pybear import Manager

# Define port and baud rate
bear_port = "COM7"
bear_baudrate = 8000000
# Define ID search range
id_range = range(0, 9)

bear = Manager.BEAR(port=bear_port, baudrate=bear_baudrate)
bear_list = []
found = False
for i in id_range:
    m_id = i
    print("Pinging BEAR with ID %d" % m_id)
    data = bear.ping(m_id)[0]
    if data:
        print("Found BEAR with ID %d." % m_id)
        found = True
        bear_list.append(m_id)
if found:
    print("Search done. Total of %d BEARs found. And their IDs are:\n" % len(bear_list))
    print(bear_list)
else:
    print("Search done. No BEAR found.")
