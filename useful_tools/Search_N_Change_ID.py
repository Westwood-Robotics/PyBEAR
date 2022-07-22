#!usr/bin/env python
__author__ = "Xiaoguang Zhang"
__email__ = "xzhang@westwoodrobotics.io"
__copyright__ = "Copyright 2022 Westwood Robotics"
__date__ = "July 18, 2022"
__version__ = "0.0.1"
__status__ = "Prototype"

# -----------------------------
# Search and list all motors in the chain
# Prompt to change ID for each motor

# Ping and search for available BEARs
from pybear import Manager

# Define port and baud rate
bear_port = "COM11"
bear_baudrate = 8000000
# Define ID search range
id_range = range(0, 253)

bear = Manager.BEAR(port=bear_port, baudrate=bear_baudrate)
bear_list = []
found = False
for i in id_range:
    m_id = i
    data = bear.ping(m_id)[0]
    if data:
        found = True
        bear_list.append(m_id)
if found:
    print("Search done. Total of %d BEARs found. And their IDs are:" % len(bear_list))
    print(bear_list)
else:
    print("Search done. No BEAR found.")
    exit()

# Change ID if found BEAR(s).
if found:
    print("Changing ID...")
    for idx, m_id in enumerate(bear_list):
        print("Current object: %d of %d" % (idx+1, len(bear_list)))
        change_in_progress = True
        while change_in_progress:
            usr = input("Present ID: %d, enter new id or 'N/n' to skip:" % m_id)
            if usr in ['n', 'N']:
                change_in_progress = False
            else:
                try:
                    usr = int(usr)
                    if -1 < usr < 254:
                        if usr not in bear_list:
                            bear.set_id((m_id, usr))
                            bear.save_config(usr)
                            if bear.ping(usr):
                                print("Present BEAR's ID has been changed from %d to %d" % (m_id, usr))
                                change_in_progress = False
                            else:
                                print("BEAR ID change unsuccessful. Please debug.")
                                exit()
                        else:
                            print("Entered ID already exists in chain. Please try again.")
                    else:
                        print("Invalid entrance. Only integers from 0 to 253 accepted.")
                except:
                    print("Invalid entrance. Only integers from 0 to 253 accepted.")
    print("All done.")
