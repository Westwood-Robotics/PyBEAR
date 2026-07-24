#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2026 Westwood Robotics Corporation"
__date__      = "July 29, 2025"
__project__   = "PyBEAR"
__version__   = "0.1.3"
__status__    = "Production"

'''
BEAR Control Table
'''

from collections import defaultdict


INSTRUCTION = defaultdict()
INSTRUCTION['PING']       = 0x01
INSTRUCTION['READ_STAT']  = 0x02
INSTRUCTION['WRITE_STAT'] = 0x03
INSTRUCTION['READ_CFG']   = 0x04
INSTRUCTION['WRITE_CFG']  = 0x05
INSTRUCTION['SAVE_CFG']   = 0x06
INSTRUCTION['SET_POSI']   = 0x08
INSTRUCTION['BULK_STAT']  = 0x12


REGISTER = defaultdict(lambda: defaultdict())

# Configuration
REGISTER['REG_TYPE']['id']                 = 'cfg'
REGISTER['REG_TYPE']['mode']               = 'cfg'
REGISTER['REG_TYPE']['baudrate']           = 'cfg'
REGISTER['REG_TYPE']['homing_offset']      = 'cfg'
REGISTER['REG_TYPE']['p_gain_id']          = 'cfg'
REGISTER['REG_TYPE']['i_gain_id']          = 'cfg'
REGISTER['REG_TYPE']['d_gain_id']          = 'cfg'
REGISTER['REG_TYPE']['p_gain_iq']          = 'cfg'
REGISTER['REG_TYPE']['i_gain_iq']          = 'cfg'
REGISTER['REG_TYPE']['d_gain_iq']          = 'cfg'
REGISTER['REG_TYPE']['p_gain_velocity']    = 'cfg'
REGISTER['REG_TYPE']['i_gain_velocity']    = 'cfg'
REGISTER['REG_TYPE']['d_gain_velocity']    = 'cfg'
REGISTER['REG_TYPE']['p_gain_position']    = 'cfg'
REGISTER['REG_TYPE']['i_gain_position']    = 'cfg'
REGISTER['REG_TYPE']['d_gain_position']    = 'cfg'
REGISTER['REG_TYPE']['p_gain_force']       = 'cfg'
REGISTER['REG_TYPE']['i_gain_force']       = 'cfg'
REGISTER['REG_TYPE']['d_gain_force']       = 'cfg'
REGISTER['REG_TYPE']['limit_acc_max']      = 'cfg'
REGISTER['REG_TYPE']['limit_i_max']        = 'cfg'
REGISTER['REG_TYPE']['limit_velocity_max'] = 'cfg'
REGISTER['REG_TYPE']['limit_position_min'] = 'cfg'
REGISTER['REG_TYPE']['limit_position_max'] = 'cfg'
REGISTER['REG_TYPE']['min_voltage']        = 'cfg'
REGISTER['REG_TYPE']['max_voltage']        = 'cfg'
REGISTER['REG_TYPE']['watchdog_timeout']   = 'cfg'
REGISTER['REG_TYPE']['temp_limit_low']     = 'cfg'
REGISTER['REG_TYPE']['temp_limit_high']    = 'cfg'

REGISTER['ADDRESS']['id']                 = 0x00
REGISTER['ADDRESS']['mode']               = 0x01
REGISTER['ADDRESS']['baudrate']           = 0x02
REGISTER['ADDRESS']['homing_offset']      = 0x03
REGISTER['ADDRESS']['p_gain_id']          = 0x04
REGISTER['ADDRESS']['i_gain_id']          = 0x05
REGISTER['ADDRESS']['d_gain_id']          = 0x06
REGISTER['ADDRESS']['p_gain_iq']          = 0x07
REGISTER['ADDRESS']['i_gain_iq']          = 0x08
REGISTER['ADDRESS']['d_gain_iq']          = 0x09
REGISTER['ADDRESS']['p_gain_velocity']    = 0x0A
REGISTER['ADDRESS']['i_gain_velocity']    = 0x0B
REGISTER['ADDRESS']['d_gain_velocity']    = 0x0C
REGISTER['ADDRESS']['p_gain_position']    = 0x0D
REGISTER['ADDRESS']['i_gain_position']    = 0x0E
REGISTER['ADDRESS']['d_gain_position']    = 0x0F
REGISTER['ADDRESS']['p_gain_force']       = 0x10
REGISTER['ADDRESS']['i_gain_force']       = 0x11
REGISTER['ADDRESS']['d_gain_force']       = 0x12
REGISTER['ADDRESS']['limit_acc_max']      = 0x13
REGISTER['ADDRESS']['limit_i_max']        = 0x14
REGISTER['ADDRESS']['limit_velocity_max'] = 0x15
REGISTER['ADDRESS']['limit_position_min'] = 0x16
REGISTER['ADDRESS']['limit_position_max'] = 0x17
REGISTER['ADDRESS']['min_voltage']        = 0x18
REGISTER['ADDRESS']['max_voltage']        = 0x19
REGISTER['ADDRESS']['watchdog_timeout']   = 0x1A
REGISTER['ADDRESS']['temp_limit_low']     = 0x1B
REGISTER['ADDRESS']['temp_limit_high']    = 0x1C

REGISTER['DATA_TYPE']['id']                 = 'u32'
REGISTER['DATA_TYPE']['mode']               = 'u32'
REGISTER['DATA_TYPE']['baudrate']           = 'u32'
REGISTER['DATA_TYPE']['homing_offset']      = 'f32'
REGISTER['DATA_TYPE']['p_gain_id']          = 'f32'
REGISTER['DATA_TYPE']['i_gain_id']          = 'f32'
REGISTER['DATA_TYPE']['d_gain_id']          = 'f32'
REGISTER['DATA_TYPE']['p_gain_iq']          = 'f32'
REGISTER['DATA_TYPE']['i_gain_iq']          = 'f32'
REGISTER['DATA_TYPE']['d_gain_iq']          = 'f32'
REGISTER['DATA_TYPE']['p_gain_velocity']    = 'f32'
REGISTER['DATA_TYPE']['i_gain_velocity']    = 'f32'
REGISTER['DATA_TYPE']['d_gain_velocity']    = 'f32'
REGISTER['DATA_TYPE']['p_gain_position']    = 'f32'
REGISTER['DATA_TYPE']['i_gain_position']    = 'f32'
REGISTER['DATA_TYPE']['d_gain_position']    = 'f32'
REGISTER['DATA_TYPE']['p_gain_force']       = 'f32'
REGISTER['DATA_TYPE']['i_gain_force']       = 'f32'
REGISTER['DATA_TYPE']['d_gain_force']       = 'f32'
REGISTER['DATA_TYPE']['limit_acc_max']      = 'f32'
REGISTER['DATA_TYPE']['limit_i_max']        = 'f32'
REGISTER['DATA_TYPE']['limit_velocity_max'] = 'f32'
REGISTER['DATA_TYPE']['limit_position_min'] = 'f32'
REGISTER['DATA_TYPE']['limit_position_max'] = 'f32'
REGISTER['DATA_TYPE']['min_voltage']        = 'f32'
REGISTER['DATA_TYPE']['max_voltage']        = 'f32'
REGISTER['DATA_TYPE']['watchdog_timeout']   = 'u32'
REGISTER['DATA_TYPE']['temp_limit_low']     = 'f32'
REGISTER['DATA_TYPE']['temp_limit_high']    = 'f32'

# Status
REGISTER['REG_TYPE']['torque_enable']          = 'stat'
REGISTER['REG_TYPE']['homing_complete']        = 'stat'
REGISTER['REG_TYPE']['goal_id']                = 'stat'
REGISTER['REG_TYPE']['goal_iq']                = 'stat'
REGISTER['REG_TYPE']['goal_velocity']          = 'stat'
REGISTER['REG_TYPE']['goal_position']          = 'stat'
REGISTER['REG_TYPE']['present_id']             = 'stat'
REGISTER['REG_TYPE']['present_iq']             = 'stat'
REGISTER['REG_TYPE']['present_velocity']       = 'stat'
REGISTER['REG_TYPE']['present_position']       = 'stat'
REGISTER['REG_TYPE']['input_voltage']          = 'stat'
REGISTER['REG_TYPE']['winding_temperature']    = 'stat'
REGISTER['REG_TYPE']['powerstage_temperature'] = 'stat'
REGISTER['REG_TYPE']['ic_temperature']         = 'stat'
REGISTER['REG_TYPE']['error_status']           = 'stat'
REGISTER['REG_TYPE']['warning_status']         = 'stat'

REGISTER['ADDRESS']['torque_enable']          = 0x00
REGISTER['ADDRESS']['homing_complete']        = 0x01
REGISTER['ADDRESS']['goal_id']                = 0x02
REGISTER['ADDRESS']['goal_iq']                = 0x03
REGISTER['ADDRESS']['goal_velocity']          = 0x04
REGISTER['ADDRESS']['goal_position']          = 0x05
REGISTER['ADDRESS']['present_id']             = 0x06
REGISTER['ADDRESS']['present_iq']             = 0x07
REGISTER['ADDRESS']['present_velocity']       = 0x08
REGISTER['ADDRESS']['present_position']       = 0x09
REGISTER['ADDRESS']['input_voltage']          = 0x0A
REGISTER['ADDRESS']['winding_temperature']    = 0x0B
REGISTER['ADDRESS']['powerstage_temperature'] = 0x0C
REGISTER['ADDRESS']['ic_temperature']         = 0x0D
REGISTER['ADDRESS']['error_status']           = 0x0E
REGISTER['ADDRESS']['warning_status']         = 0x0F

REGISTER['DATA_TYPE']['torque_enable']          = 'u32'
REGISTER['DATA_TYPE']['homing_complete']        = 'u32'
REGISTER['DATA_TYPE']['goal_id']                = 'f32'
REGISTER['DATA_TYPE']['goal_iq']                = 'f32'
REGISTER['DATA_TYPE']['goal_velocity']          = 'f32'
REGISTER['DATA_TYPE']['goal_position']          = 'f32'
REGISTER['DATA_TYPE']['present_id']             = 'f32'
REGISTER['DATA_TYPE']['present_iq']             = 'f32'
REGISTER['DATA_TYPE']['present_velocity']       = 'f32'
REGISTER['DATA_TYPE']['present_position']       = 'f32'
REGISTER['DATA_TYPE']['input_voltage']          = 'f32'
REGISTER['DATA_TYPE']['winding_temperature']    = 'f32'
REGISTER['DATA_TYPE']['powerstage_temperature'] = 'f32'
REGISTER['DATA_TYPE']['ic_temperature']         = 'f32'
REGISTER['DATA_TYPE']['error_status']           = 'u32'
REGISTER['DATA_TYPE']['warning_status']         = 'u32'