# PyBEAR
This is the Python SDK for the Westwood Robotics actuator module BEAR (Back-drivable Electromechanical Actuator for Robotics).

Current version: 0.0.5

### Contact
Email: info@westwoodrobotics.net

### Notes
It is advised to use the Boosted USB2BEAR/USB2RoMeLa device for maximum speed.

### Disclaimer
Use at own risk when using other generic RS485 dangles.

###

## Udev Rules
When using the Boosted USB2BEAR/USB2RoMeLa device, you can use udev rules to allow local machine to asign a unique SYMLINK according to the serial number of the USB dangle.
1. Move 00-WestwoodRobotics.rules file into /etc/udev/rules.d/ with 'sudo cp'
2. Reload the rules
```bash
sudo udevadm control --reload
```

Everytime a  Boosted USB2BEAR/USB2RoMeLa device is plugged in, a symlink that is: '/dev/serial#' will be created.
For example, a device with serial number UB021 will have this SYMLINK: /dev/UB021

USB2RoMeLa from RoMeLa has 3 digit hexadecimal serial numbers.

USB2BEAR from Westwood Robotics has 4 digit hexadecimal serial numbers.


## Installation Procedure
1. Modify the permissions of your computer such that PyBEAR can access the serial port.
```bash
sudo chown -R your_username /usr/local
sudo usermod -a -G dialout your_username
```

2. Install dependencies.
```bash
pip install numpy pyserial
```

3. Run installation on your local Python installation or your virtual environment.
```bash
python setup.py install
```
4. Enjoy!

## Example
```python
# Import Manager from PyBEAR
from pybear import Manager

# Create a serial port object
bear = Manager.BEAR(port='/dev/ttyUSB0', baudrate=8000000)

# Enable torque
bear.set_torque_enable((1,1))

# Enable multiple torques
bear.set_torque_enable((1,1), (2,1))

# Enable Direct Force mode
bear.set_mode(1,3)

# Apply Direct Force D gain
bear.set_d_gain_direct_force((1,0.1))
```

