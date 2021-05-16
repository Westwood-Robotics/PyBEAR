# PyBEAR
This is the Python SDK for the Westwood Robotics actuator module BEAR (Back-drivable Electromechanical Actuator for Robotics).

Current version: 0.1.1

In this version, the return format is always [[[data1, data2 ...], error], [[data1, data2 ...], error] ...] no matter if returning only one register or more per BEAR.

### Contact
Website: www.westwoodrobotics.io

Email: info@westwoodrobotics.io

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

2. CD into PyBEAR/ directory and use pip/pip3 to install the package.

Python2
```bash
pip install .
```

Python3
```bash
pip3 install .
```
3.Make sure pyserial and numpy are installed. You can check this by running 
```bash
pip show pyserial numpy
```
or 
```bash
pip3 show pyserial numpy
```

Othewise, install the missing package(s).

4. Enjoy!

## SDK Manual
Download the latest SDK Manual from www.westwoodrobotics.io for detailed instructions, helpful tips and various examples.

For detailed examples please refer to the SDK: https://westwoodrobotics.io/support/

