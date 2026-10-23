# PyBEAR
**Note: We have officially stopped supporting Python2. While it is still possible to use PyBEAR with Python 2, please be aware that certain functions may no longer be functional.**

This is the Python SDK for the Westwood Robotics actuator module BEAR (Back-drivable Electromechanical Actuator for Robotics).

Current version: 1.0.0

In this version, the return format is always [([data1, data2, ...], error), ([data1, data2, ...], error), ...] no matter if returning only one register or more per BEAR. 

### New function: set_posi()
A new function set_posi() is added to clear absolute position error on BEAR, e.g., set_posi((id, position, tolerance)).

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

2. CD into PyBEAR/ directory and use pip3 to install the package(Python3).

```bash
pip3 install .
```

3. Make sure pyserial, numpy, termcolor are installed. Othewise, install the missing package(s). You can check this by running: 

```bash
pip3 show pyserial numpy termcolor
```

4. Enjoy!

## SDK Manual
Download the latest SDK Manual from www.westwoodrobotics.io for detailed instructions, helpful tips, and various examples.

For detailed examples please refer to the SDK from https://westwoodrobotics.io/support/.

### New function in 1.0.0: Return Delay, Torque Comp, AFC_EN, and Friction Comp
**Firmware update required.**

These settings are configuration registers on recent BEAR Firmware versions (dated 10/23/2026 or newer). You need to update your BEAR actuators with the new corresponding firmwares to use these new features.


```python
# Set a 250 us delay between receiving a request and sending its RS485 reply.
bear.set_return_delay((1, 250))  # valid range: 0..500 us; 0 disables the delay
delay = bear.get_return_delay(1)  # [([250], error_code)]

# Request torque compensation and check whether calibration allows it.
bear.set_torque_comp((1, 1))     # write 0 to turn it off, 1 to request it on
state = bear.get_torque_comp(1)  # [([0, 1, or 2], error_code)]

# Control adaptive feedforward cancellation when included in the firmware.
bear.set_afc_en((1, 1))          # write 0 to turn it off, 1 to request it on
afc = bear.get_afc_en(1)         # [([0 or 1], error_code)]

# Friction compensation has its own calibration and enable state.
bear.set_friction_comp((1, 1))   # write 0 to turn it off, 1 to request it on
friction = bear.get_friction_comp(1)  # [([0, 1, or 2], error_code)]

# Persist the settings across power cycles if desired.
bear.save_config(1)
```

`get_torque_comp()` returns `0` when disabled, `1` when enabled, or `2` when enabled was requested but the required calibration is invalid. Only `0` and `1` are valid write values. A nonzero Return Delay adds to reply latency; increase `single_timeout` when constructing `BEAR` if your connection needs more than the default 2 ms.

`get_afc_en()` returns `0` when AFC is off or was not included in this firmware build, and `1` when enabled. A firmware build without AFC rejects an attempt to write `1`. `get_friction_comp()` returns `0` when disabled, `1` when enabled, or `2` when enable was requested but its own friction calibration is invalid. AFC_EN and Friction Comp accept only `0` and `1` for writes; Friction Comp is independent of Torque Comp calibration.

They can also be accessed by name through `get_register()` and `set_register()`.

### New function in 0.1.3: set_posi()
A new function set_posi() is added to clear absolute position error on BEAR, e.g., set_posi((id, position, tolerance)). If tolerance is non-zero, BEAR tries to find the multi-turn value to match the expected position within the tolerance. If tolerance is zero, BEAR modifies the homing offset to match the expected position. A save_config() action will be performed automatically once set_posi() is successfully excuted. 

When BEAR is free from Absolute Position error, Hardware Fault or Initialization error, set_posi() can be used to find homing offset autonomously. For example:
```bash
# Find and set homing offset for BEAR with ID 1 so that the present position is 2.0
id = 1
position = 2.0
tolerance = 0.0
bear.set_posi((id, position, tolerance))
```
If BEAR is in Absolute Position error, set_posi() can be used to clear it. 

**Note: set to non-zero tolerance when using set_posi() to clear Absolute Position error, then disable to clear the error.**

For example:
```bash
# Clear Absolute Position error on BEAR with ID 5 and find offset to make present position 0.0
id = 5
position = 0.0
tolerance = 0.2
bear.set_posi((id, position, tolerance))
bear.set_torque_enable((id,0))
```


