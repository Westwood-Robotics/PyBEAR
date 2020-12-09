#!usr/bin/env python
__author__ = "Xiaoguang Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics"
__date__ = "Jan 8, 2020"
__version__ = "0.0.1"
__status__ = "Prototype"

# -----------------------------
# Simple code to set ID for BEAR

from pybear import Manager

bear = Manager.BEAR(port="COM18", baudrate=8000000)
m_id = int(input("Enter the present ID and press enter.\n"))
print("Present ID entered is: %d" % m_id)
if bear.ping(m_id):
    print("BEAR connected.")
    m_id_new = int(input("Enter the new ID and press enter.\n"))
    bear.set_id((m_id, m_id_new))
    bear.save_config(m_id_new)
    if bear.ping(m_id_new):
        print("BEAR ID has been changed from %d to %d" % (m_id, m_id_new))
    else:
        print("BEAR ID change unsuccessful. Please try again.")
else:
    print("Seems like that BEAR is offline, please double check your entry and connections.")
