#!usr/bin/env python
__author__ = "Xiaoguang Zhang"
__email__ = "xzhang@westwoodrobotics.io"
__copyright__ = "Copyright 2020 Westwood Robotics"
__date__ = "Aug 20, 2020"
__version__ = "0.1.0"
__status__ = "Production"

# -----------------------------
# Move BEAR from the current angle to a specified angle
# For more details, please check the SDK manual (https://www.westwoodrobotics.io/support/)

from pybear import Manager
import time

error = False
bear = Manager.BEAR(port="/dev/ttyUSB0", baudrate=8000000)  # need to identify the port name on your PC

m_id = 1  # BEAR ID (default is 1)

p_gain = 5.0  # Set P gain as spring stiffness
d_gain = 0.2  # Set D gain as damper strength
i_gain = 0.0  # I gain is usually not needed
iq_max = 1.5  # Max iq

BEAR_connected = bear.ping(m_id)[0]
if not BEAR_connected:
    # BEAR is offline
    print("BEAR is offline. Check power and communication.")
    error = True
    exit()

if not error:
    # BEAR is online
    # Set PID, mode, and limit
    print("Welcome aboard, Captain!")
    # PID id/iq
    bear.set_p_gain_iq((m_id, 0.02))
    bear.set_i_gain_iq((m_id, 0.02))
    bear.set_d_gain_iq((m_id, 0))
    bear.set_p_gain_id((m_id, 0.02))
    bear.set_i_gain_id((m_id, 0.02))
    bear.set_d_gain_id((m_id, 0))

    # PID position mode
    bear.set_p_gain_position((m_id, p_gain))
    bear.set_i_gain_position((m_id, i_gain))
    bear.set_d_gain_position((m_id, d_gain))

    # Put into position mode
    bear.set_mode((m_id, 2))

    # Set iq limit
    bear.set_limit_iq_max((m_id, iq_max))

    # Start demo
    input('Press Enter to start demo. ')

    # Get home position
    home = bear.get_present_position(m_id)[0][0][0]
    print(home)

    # Set goal position before enabling BEAR
    bear.set_goal_position((m_id, home))

    # Enable BEAR
    bear.set_torque_enable((m_id, 1))

    # Demo spring-damping system
    print('You can play with BEAR now! It is simulating a spring-damping system.')
    time.sleep(2)

    # Get command position
    angle = float(input('Input the angle in radians you want to move BEAR to (e.g., -0.2): '))

    # Let's move to the target angle smoothly
    num = 100                  # split it into 100 pieces
    delta_angle = angle / num  # angle for each time
    for i in range(num):
        goal_pos = home + delta_angle * (i + 1)
        bear.set_goal_position((m_id, goal_pos))
        time.sleep(0.01)

    print('BEAR arrived target angle!')
    time.sleep(2)

    # Turn off BEAR
    input('Press Enter to turn off BEAR.')

    # Disable BEAR
    bear.set_torque_enable((m_id, 0))
    print("Thanks for using BEAR!")
