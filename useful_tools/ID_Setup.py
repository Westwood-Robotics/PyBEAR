#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2026 Westwood Robotics Corporation"
__date__      = "July 29, 2025"
__project__   = "PyBEAR"
__version__   = "0.1.3"
__status__    = "Production"

'''
Simple script to set ID for BEAR
'''

from pybear import Manager

bear = Manager.BEAR(port="COM7", baudrate=8000000)
m_id = int(input("Enter the present ID and press enter: "))
print("Present ID entered is %02d." % m_id)
if bear.ping(m_id)[0][1] is not None:
    print("BEAR connected.")
    m_id_new = int(input("Enter the new ID and press enter: "))
    if m_id_new == m_id:
        print("Please enter a different ID.")
    else:
        bear.set_id((m_id, m_id_new))
        bear.save_config(m_id_new)
        if bear.ping(m_id_new)[0][1] is not None:
            print("BEAR ID has been changed from %02d to %02d." % (m_id, m_id_new))
        else:
            print("BEAR ID change is unsuccessful. Please try again.")
else:
    print("Seems like that BEAR is offline. Please double check your entry and connection.")