#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2026 Westwood Robotics Corporation"
__date__      = "July 24, 2026"
__project__   = "PyBEAR"
__version__   = "0.1.3"
__status__    = "Production"

'''
BEAR Manager
'''

from pybear import Packet
from termcolor import cprint
from pybear.CONTROL_TABLE import *


class BEAR(Packet.PKT):
    def __init__(self, 
                 port='/dev/ttyUSB0', 
                 baudrate=8000000,
                 single_timeout=None, 
                 single_try_num=None, 
                 bulk_timeout=None, 
                 bulk_try_num=None):
        # serial communication
        self.port           = port
        self.baudrate       = baudrate
        self.single_timeout = single_timeout
        self.single_try_num = single_try_num
        self.bulk_timeout   = bulk_timeout
        self.bulk_try_num   = bulk_try_num
        super(BEAR, self).__init__(self.port, 
                                   self.baudrate, 
                                   single_timeout=self.single_timeout, 
                                   single_try_num=self.single_try_num, 
                                   bulk_timeout=self.bulk_timeout, 
                                   bulk_try_num=self.bulk_try_num)

        # BEAR info
        self.enable_status  = {'disable': 0, 'enable': 1, 'error': 2, 'damping': 3}
        self.operating_mode = {'torque': 0, 'velocity': 1, 'position': 2, 'force': 3}
        self.error_status   = {'Communication': 0, 'Overheat': 1, 'Absolute Position': 2, 'E-STOP': 3, 'Joint Limit': 4, 'Hardware Fault': 5, 'Initialization': 6}

        # startup
        self.print_welcome_msg()

    # =================================================================================================================
    # ===== Configuration Registers
    def get_id(self, *argv):
        return [self.single_read(data, ['id']) for data in argv]
    
    def set_id(self, *argv):
        for data in argv:
            self.single_write(data[0], ['id'], [data[1]])

    def get_mode(self, *argv):
        return [self.single_read(data, ['mode']) for data in argv]
    
    def set_mode(self, *argv):
        for data in argv:
            self.single_write(data[0], ['mode'], [data[1]])

    def get_baudrate(self, *argv):
        return [self.single_read(data, ['baudrate']) for data in argv]
    
    def set_baudrate(self, *argv):
        for data in argv:
            self.single_write(data[0], ['baudrate'], [data[1]])

    def get_homing_offset(self, *argv):
        return [self.single_read(data, ['homing_offset']) for data in argv]
    
    def set_homing_offset(self, *argv):
        for data in argv:
            self.single_write(data[0], ['homing_offset'], [data[1]])

    def get_p_gain_id(self, *argv):
        return [self.single_read(data, ['p_gain_id']) for data in argv]
    
    def set_p_gain_id(self, *argv):
        for data in argv:
            self.single_write(data[0], ['p_gain_id'], [data[1]])
    
    def get_i_gain_id(self, *argv):
        return [self.single_read(data, ['i_gain_id']) for data in argv]
    
    def set_i_gain_id(self, *argv):
        for data in argv:
            self.single_write(data[0], ['i_gain_id'], [data[1]])

    def get_d_gain_id(self, *argv):
        return [self.single_read(data, ['d_gain_id']) for data in argv]
    
    def set_d_gain_id(self, *argv):
        for data in argv:
            self.single_write(data[0], ['d_gain_id'], [data[1]])

    def get_p_gain_iq(self, *argv):
        return [self.single_read(data, ['p_gain_iq']) for data in argv]
    
    def set_p_gain_iq(self, *argv):
        for data in argv:
            self.single_write(data[0], ['p_gain_iq'], [data[1]])
    
    def get_i_gain_iq(self, *argv):
        return [self.single_read(data, ['i_gain_iq']) for data in argv]
    
    def set_i_gain_iq(self, *argv):
        for data in argv:
            self.single_write(data[0], ['i_gain_iq'], [data[1]])

    def get_d_gain_iq(self, *argv):
        return [self.single_read(data, ['d_gain_iq']) for data in argv]
    
    def set_d_gain_iq(self, *argv):
        for data in argv:
            self.single_write(data[0], ['d_gain_iq'], [data[1]])

    def get_p_gain_velocity(self, *argv):
        return [self.single_read(data, ['p_gain_velocity']) for data in argv]
    
    def set_p_gain_velocity(self, *argv):
        for data in argv:
            self.single_write(data[0], ['p_gain_velocity'], [data[1]])
    
    def get_i_gain_velocity(self, *argv):
        return [self.single_read(data, ['i_gain_velocity']) for data in argv]
    
    def set_i_gain_velocity(self, *argv):
        for data in argv:
            self.single_write(data[0], ['i_gain_velocity'], [data[1]])

    def get_d_gain_velocity(self, *argv):
        return [self.single_read(data, ['d_gain_velocity']) for data in argv]
    
    def set_d_gain_velocity(self, *argv):
        for data in argv:
            self.single_write(data[0], ['d_gain_velocity'], [data[1]])

    def get_p_gain_position(self, *argv):
        return [self.single_read(data, ['p_gain_position']) for data in argv]
    
    def set_p_gain_position(self, *argv):
        for data in argv:
            self.single_write(data[0], ['p_gain_position'], [data[1]])
    
    def get_i_gain_position(self, *argv):
        return [self.single_read(data, ['i_gain_position']) for data in argv]
    
    def set_i_gain_position(self, *argv):
        for data in argv:
            self.single_write(data[0], ['i_gain_position'], [data[1]])

    def get_d_gain_position(self, *argv):
        return [self.single_read(data, ['d_gain_position']) for data in argv]
    
    def set_d_gain_position(self, *argv):
        for data in argv:
            self.single_write(data[0], ['d_gain_position'], [data[1]])

    def get_p_gain_force(self, *argv):
        return [self.single_read(data, ['p_gain_force']) for data in argv]
    
    def set_p_gain_force(self, *argv):
        for data in argv:
            self.single_write(data[0], ['p_gain_force'], [data[1]])
    
    def get_i_gain_force(self, *argv):
        return [self.single_read(data, ['i_gain_force']) for data in argv]
    
    def set_i_gain_force(self, *argv):
        for data in argv:
            self.single_write(data[0], ['i_gain_force'], [data[1]])

    def get_d_gain_force(self, *argv):
        return [self.single_read(data, ['d_gain_force']) for data in argv]
    
    def set_d_gain_force(self, *argv):
        for data in argv:
            self.single_write(data[0], ['d_gain_force'], [data[1]])

    def get_limit_acc_max(self, *argv):
        return [self.single_read(data, ['limit_acc_max']) for data in argv]
    
    def set_limit_acc_max(self, *argv):
        for data in argv:
            self.single_write(data[0], ['limit_acc_max'], [data[1]])

    def get_limit_i_max(self, *argv):
        return [self.single_read(data, ['limit_i_max']) for data in argv]
    
    def set_limit_i_max(self, *argv):
        for data in argv:
            self.single_write(data[0], ['limit_i_max'], [data[1]])
    
    def get_limit_velocity_max(self, *argv):
        return [self.single_read(data, ['limit_velocity_max']) for data in argv]
    
    def set_limit_velocity_max(self, *argv):
        for data in argv:
            self.single_write(data[0], ['limit_velocity_max'], [data[1]])

    def get_limit_position_min(self, *argv):
        return [self.single_read(data, ['limit_position_min']) for data in argv]
    
    def set_limit_position_min(self, *argv):
        for data in argv:
            self.single_write(data[0], ['limit_position_min'], [data[1]])

    def get_limit_position_max(self, *argv):
        return [self.single_read(data, ['limit_position_max']) for data in argv]
    
    def set_limit_position_max(self, *argv):
        for data in argv:
            self.single_write(data[0], ['limit_position_max'], [data[1]])

    def get_min_voltage(self, *argv):
        return [self.single_read(data, ['min_voltage']) for data in argv]
    
    def set_min_voltage(self, *argv):
        for data in argv:
            self.single_write(data[0], ['min_voltage'], [data[1]])

    def get_max_voltage(self, *argv):
        return [self.single_read(data, ['max_voltage']) for data in argv]
    
    def set_max_voltage(self, *argv):
        for data in argv:
            self.single_write(data[0], ['max_voltage'], [data[1]])

    def get_watchdog_timeout(self, *argv):
        return [self.single_read(data, ['watchdog_timeout']) for data in argv]
    
    def set_watchdog_timeout(self, *argv):
        for data in argv:
            self.single_write(data[0], ['watchdog_timeout'], [data[1]])
    
    def get_temp_limit_low(self, *argv):
        return [self.single_read(data, ['temp_limit_low']) for data in argv]
    
    def set_temp_limit_low(self, *argv):
        for data in argv:
            self.single_write(data[0], ['temp_limit_low'], [data[1]])
    
    def get_temp_limit_high(self, *argv):
        return [self.single_read(data, ['temp_limit_high']) for data in argv]
    
    def set_temp_limit_high(self, *argv):
        for data in argv:
            self.single_write(data[0], ['temp_limit_high'], [data[1]])
        
    # =================================================================================================================
    # ===== Status Registers
    def get_torque_enable(self, *argv):
        return [self.single_read(data, ['torque_enable']) for data in argv]
    
    def set_torque_enable(self, *argv):
        for data in argv:
            self.single_write(data[0], ['torque_enable'], [data[1]])
    
    def get_goal_id(self, *argv):
        return [self.single_read(data, ['goal_id']) for data in argv]
    
    def set_goal_id(self, *argv):
        for data in argv:
            self.single_write(data[0], ['goal_id'], [data[1]])

    def get_goal_iq(self, *argv):
        return [self.single_read(data, ['goal_iq']) for data in argv]
    
    def set_goal_iq(self, *argv):
        for data in argv:
            self.single_write(data[0], ['goal_iq'], [data[1]])

    def get_goal_velocity(self, *argv):
        return [self.single_read(data, ['goal_velocity']) for data in argv]
    
    def set_goal_velocity(self, *argv):
        for data in argv:
            self.single_write(data[0], ['goal_velocity'], [data[1]])

    def get_goal_position(self, *argv):
        return [self.single_read(data, ['goal_position']) for data in argv]
    
    def set_goal_position(self, *argv):
        for data in argv:
            self.single_write(data[0], ['goal_position'], [data[1]])
    
    def get_present_id(self, *argv):
        return [self.single_read(data, ['present_id']) for data in argv]

    def get_present_iq(self, *argv):
        return [self.single_read(data, ['present_iq']) for data in argv]

    def get_present_velocity(self, *argv):
        return [self.single_read(data, ['present_velocity']) for data in argv]

    def get_present_position(self, *argv):
        return [self.single_read(data, ['present_position']) for data in argv]

    def get_input_voltage(self, *argv):
        return [self.single_read(data, ['input_voltage']) for data in argv]

    def get_winding_temperature(self, *argv):
        return [self.single_read(data, ['winding_temperature']) for data in argv]

    def get_powerstage_temperature(self, *argv):
        return [self.single_read(data, ['powerstage_temperature']) for data in argv]

    def get_ic_temperature(self, *argv):
        return [self.single_read(data, ['ic_temperature']) for data in argv]
    
    # =================================================================================================================
    # ===== Others
    def print_welcome_msg(self):
        print("==============================================================")
        print("   __        __          _                               _    ")
        print("   \ \      / /___  ___ | |_ __      __ ___    ___    __| |   ")
        print("    \ \ /\ / // _ \/ __|| __|\ \ /\ / // _ \  / _ \  / _` |   ")
        print("     \ V  V /|  __/\__ \| |_  \ V  V /| (_) || (_) || (_| |   ")
        print("      \_/\_/  \___||___/ \__|  \_/\_/  \___/  \___/  \__,_|   ")
        print("              ____         _             _    _               ")
        print("             |  _ \  ___  | |__    ___  | |_ (_)  ___  ___    ")
        print("             | |_) |/ _ \ | '_ \  / _ \ | __|| | / __|/ __|   ")
        print("             |  _ <| (_) || |_) || (_) || |_ | || (__ \__ \   ")
        print("             |_| \_\\___/ |_.__/  \___/  \__||_| \___||___/   ")
        print("                                                              ")
        print("==============================================================")
        print("=== PyBEAR by Westwood Robotics -- Last Updated 2025.07.29 ===")
        print("==============================================================")
        print()
    
    def ping(self, *argv: int):
        """
        Function used to detect BEAR(s). Will return firmware and hardware version info, e.g., ping(id1, id2)
        Return None if pinging unsuccessful.
        """
        return [self._ping(data) for data in argv]
    
    def save_config(self, *argv: int):
        """
        Save BEAR configuration registers, e.g., save_config(id1, id2)
        """
        for data in argv:
            self._save_config(data)

    def set_posi(self, *argv):
        """
        Set BEAR absolute position, e.g., set_posi((id1, pos1, tol1), (id2, pos2, tol2))
        """
        for data in argv:
            self._set_posi(data[0], data[1], data[2])

    def single_read(self, bear, reg_name_list):
        """
        Read multiple registers of a single BEAR, e.g., single_read(id, [name1, name2])
        return: list of register data, e.g., [data1, data2], error
        """
        cfg_name_list = []
        stat_name_list = []
        for idx, reg_name in enumerate(reg_name_list):
            if REGISTER['REG_TYPE'][reg_name] == 'cfg':
                cfg_name_list.append(reg_name)
            elif REGISTER['REG_TYPE'][reg_name] == 'stat':
                stat_name_list.append(reg_name)

        if cfg_name_list:
            cfg_data, error = self._single_read(bear, cfg_name_list, reg_type='cfg')

        if stat_name_list:
            stat_data, error = self._single_read(bear, stat_name_list, reg_type='stat')

        reg_data = []
        for reg_name in reg_name_list:
            if REGISTER['REG_TYPE'][reg_name] == 'cfg':
                reg_data.append(cfg_data[0])
                cfg_data.pop(0)
            elif REGISTER['REG_TYPE'][reg_name] == 'stat':
                reg_data.append(stat_data[0])
                stat_data.pop(0)
        return reg_data, error

    def single_write(self, bear, reg_name_list, data_list):
        """
        Write multiple registers of a single BEAR, e.g., single_write(id, [name1, name2], [data1, data2])
        """
        cfg_name_list = []
        cfg_data_list = []
        stat_name_list = []
        stat_data_list = []
        for idx, reg_name in enumerate(reg_name_list):
            data = data_list[idx]
            if REGISTER['REG_TYPE'][reg_name] == 'cfg':
                cfg_name_list.append(reg_name)
                cfg_data_list.append(data)
            elif REGISTER['REG_TYPE'][reg_name] == 'stat':
                stat_name_list.append(reg_name)
                stat_data_list.append(data)

        if cfg_name_list:
            self._single_write(bear, cfg_name_list, cfg_data_list, reg_type='cfg')

        if stat_data_list:
            self._single_write(bear, stat_name_list, stat_data_list, reg_type='stat')

    def bulk_read(self, bear_list, read_stat_name_list):
        """
        Read multiple *STATUS* registers of multiple BEARs, e.g., bulk_read([id1, id2], [read_name1, read_name2])
        return: list of status data, e.g., [([read_data11, read_data12], error1), ([read_data21, read_data22], error2)]
        """
        read_stat_data_list, error_list = self._bulk_read_write_stat(bear_list, read_stat_name_list, [], [])
        return [(read_stat_data_list[idx], error_list[idx]) for idx in range(len(bear_list))]

    def bulk_write(self, bear_list, write_stat_name_list, data_list):
        """
        Write multiple *STATUS* registers of multiple BEARs, e.g., bulk_write([id1, id2], [write_name1, write_name2], [[write_data11, write_data12], [write_data21, write_data22]])
        """
        self._bulk_read_write_stat(bear_list, [], write_stat_name_list, data_list)

    def bulk_read_write(self, bear_list, read_stat_name_list, write_stat_name_list, write_stat_data_list):
        """
        Read and write multiple *STATUS* registers of multiple BEARs, e.g., bulk_read_write([id1, id2], [read_name1, read_name2], [write_name1, write_name2], [[write_data11, write_data12], [write_data21, write_data22]])
        return: list of status data, e.g., [([read_data11, read_data12], error1), ([read_data21, read_data22], error2)]
        """
        read_stat_data_list, error_list = self._bulk_read_write_stat(bear_list, read_stat_name_list, write_stat_name_list, write_stat_data_list)
        return [(read_stat_data_list[idx], error_list[idx]) for idx in range(len(bear_list))]
    
    def get_register(self, *argv):
        """
        Read multiple registers from multiple BEARs in one packet but goes through them one-by-one, e.g. get_reg((id1, reg11, reg12, ...), (id2, reg21, reg22, ...), ...)
        """
        return [self.single_read(data[0], [data[1:]]) for data in argv]
    
    def set_register(self, *argv):
        """
        Write to multiple registers on multiple BEARs in one packet but goes through them one-by-one, e.g., set_reg((id1, reg11, data11, reg12, data12, ...), (id2, reg21, data21, reg22, data22, ...), ...)
        """
        for data in argv:
            self.single_write(data[0], [data[1::2]], [data[2::2]])

    def decode_error(self, error_code):
        """
        Decode BEAR error code, e.g., decode_error(error_code)
        return: error names
        """
        msg = ''
        if error_code >> 7 != 1:
            cprint("Invalid Error Code!!!", 'red')
            return msg
        else:
            error_num = 0
            for idx in range(7):
                if error_code >> idx & 1:
                    error_num += 1
                    if error_num > 1:
                        msg += ' & '
                    msg += list(self.error_status.keys())[list(self.error_status.values()).index(idx)]
            if error_num:
                return msg
            else:
                return "No Error!"