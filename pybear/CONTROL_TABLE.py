#!usr/bin/env python


################################################################################
# Copyright 2020 Westwood Robotics Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

__author__ = "Westwood Robotics Corporation"
__email__ = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2020 Westwood Robotics"
__date__ = "Jan. 01, 2020"

__version__ = "0.1.2"
__status__ = "Production"

"""
Control Table
"""


class INSTRUCTION:
    """
    Instruction Constants
    ---------------------
    This is the field that defines the purpose of the Packet.
    """
    PING = 0x01
    READ_STAT = 0x02
    WRITE_STAT = 0x03
    READ_CFG = 0x04
    WRITE_CFG = 0x05
    SAVE_CFG = 0x06
    BULK_COMM = 0x12


class CFG_REG:
    """Configuration Registers"""
    ID = 0x00
    MODE = 0x01
    BAUDRATE = 0x02
    HOMING_OFFSET = 0x03
    # Gains for Id current loop
    P_GAIN_ID = 0x04
    I_GAIN_ID = 0x05
    D_GAIN_ID = 0x06
    # Gains for Iq current loop
    P_GAIN_IQ = 0x07
    I_GAIN_IQ = 0x08
    D_GAIN_IQ = 0x09
    # Gains for velocity loop
    P_GAIN_VEL = 0x0A
    I_GAIN_VEL = 0x0B
    D_GAIN_VEL = 0x0C
    # Gains for position loop
    P_GAIN_POS = 0x0D
    I_GAIN_POS = 0x0E
    D_GAIN_POS = 0x0F
    # Gains for direct force loop
    P_GAIN_FORCE = 0x10
    I_GAIN_FORCE = 0x11
    D_GAIN_FORCE = 0x12
    # Limits
    LIMIT_ACC_MAX = 0x13
    LIMIT_I_MAX = 0x14
    LIMIT_VEL_MAX = 0x15
    LIMIT_POS_MIN = 0x16
    LIMIT_POS_MAX = 0x17
    MIN_VOLTAGE = 0x18
    MAX_VOLTAGE = 0x19
    # LOW_VOLTAGE_WARNING = 0x1A
    WATCHDOG_TIMEOUT = 0x1A
    TEMP_LIMIT_LOW = 0x1B  # Motor will start to limit power
    TEMP_LIMIT_HIGH = 0x1C  # Motor will shutdown

    UINT_REG = [ID, MODE, BAUDRATE, WATCHDOG_TIMEOUT]


CFG_REG_DIC = {'id': CFG_REG.ID,
               'mode': CFG_REG.MODE,
               'baudrate': CFG_REG.BAUDRATE,
               'homing_offset': CFG_REG.HOMING_OFFSET,
               'p_gain_id': CFG_REG.P_GAIN_ID,
               'i_gain_id': CFG_REG.I_GAIN_ID,
               'd_gain_id': CFG_REG.D_GAIN_ID,
               'p_gain_iq': CFG_REG.P_GAIN_IQ,
               'i_gain_iq': CFG_REG.I_GAIN_IQ,
               'd_gain_iq': CFG_REG.D_GAIN_IQ,
               'p_gain_velocity': CFG_REG.P_GAIN_VEL,
               'i_gain_velocity': CFG_REG.I_GAIN_VEL,
               'd_gain_velocity': CFG_REG.D_GAIN_VEL,
               'p_gain_position': CFG_REG.P_GAIN_POS,
               'i_gain_position': CFG_REG.I_GAIN_POS,
               'd_gain_position': CFG_REG.D_GAIN_POS,
               'p_gain_direct_force': CFG_REG.P_GAIN_FORCE,
               'i_gain_direct_force': CFG_REG.I_GAIN_FORCE,
               'd_gain_direct_force': CFG_REG.D_GAIN_FORCE,
               'limit_acc_max': CFG_REG.LIMIT_ACC_MAX,
               'limit_i_max': CFG_REG.LIMIT_I_MAX,
               'limit_iq_max': CFG_REG.LIMIT_I_MAX,
               'limit_velocity_max': CFG_REG.LIMIT_VEL_MAX,
               'limit_position_min': CFG_REG.LIMIT_POS_MIN,
               'limit_position_max': CFG_REG.LIMIT_POS_MAX,
               'min_voltage': CFG_REG.MIN_VOLTAGE,
               'max_voltage': CFG_REG.MAX_VOLTAGE,
               # 'low_voltage_warning': CFG_REG.LOW_VOLTAGE_WARNING,
               'watchdog_timeout': CFG_REG.WATCHDOG_TIMEOUT,
               'temp_limit_low': CFG_REG.TEMP_LIMIT_LOW,
               'temp_limit_high': CFG_REG.TEMP_LIMIT_HIGH}


class STAT_REG:
    """Status Registers"""
    TORQUE_ENABLE = 0x00  # Enable output
    HOMING_COMPLETE = 0x01
    GOAL_ID = 0x02
    GOAL_IQ = 0x03
    GOAL_VEL = 0x04
    GOAL_POS = 0x05
    PRESENT_ID = 0x06
    PRESENT_IQ = 0x07
    PRESENT_VEL = 0x08
    PRESENT_POS = 0x09
    INPUT_VOLTAGE = 0x0A
    WINDING_TEMP = 0x0B
    POWERSTAGE_TEMP = 0x0C
    IC_TEMP = 0x0D
    ERROR_STATUS = 0x0E
    WARNING_STATUS = 0x0F


STAT_REG_DIC = {'torque_enable': STAT_REG.TORQUE_ENABLE,
                'homing_complete': STAT_REG.HOMING_COMPLETE,
                'goal_id': STAT_REG.GOAL_ID,
                'goal_iq': STAT_REG.GOAL_IQ,
                'goal_velocity': STAT_REG.GOAL_VEL,
                'goal_position': STAT_REG.GOAL_POS,
                'present_id': STAT_REG.PRESENT_ID,
                'present_iq': STAT_REG.PRESENT_IQ,
                'present_velocity': STAT_REG.PRESENT_VEL,
                'present_position': STAT_REG.PRESENT_POS,
                'input_voltage': STAT_REG.INPUT_VOLTAGE,
                'winding_temperature': STAT_REG.WINDING_TEMP,
                'powerstage_temperature': STAT_REG.POWERSTAGE_TEMP,
                'ic_temperature': STAT_REG.IC_TEMP,
                'error_status': STAT_REG.ERROR_STATUS,
                'warning_status': STAT_REG.WARNING_STATUS}
