#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2026 Westwood Robotics Corporation"
__date__      = "July 29, 2025"
__project__   = "PyBEAR"
__version__   = "0.1.3"
__status__    = "Production"

'''
BEAR Serial Communication
'''

import time
import serial
import struct
from termcolor import cprint
from pybear.CONTROL_TABLE import *


class PKT(object):
    def __init__(self, port, baudrate, single_timeout=None, single_try_num=None, bulk_timeout=None, bulk_try_num=None):
        # serial communication
        self.port = port
        self.baudrate = baudrate
        self.open_port()

        # timeout/number of retries for single/multiple BEARs communication 
        self.single_timeout = 0.002 if single_timeout is None else single_timeout
        self.single_try_num = 2     if single_try_num is None else single_try_num
        self.bulk_timeout   = 0.002 if bulk_timeout   is None else bulk_timeout
        self.bulk_try_num   = 2     if bulk_try_num   is None else bulk_try_num

    def open_port(self):
        """
        Open the serial port
        """
        self.ser = serial.Serial(self.port,
                                 self.baudrate,
                                 bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 timeout=0)

    def close_port(self):
        """
        Close the serial port
        """
        if self.ser:
            self.ser.close()

    def send_packet(self, packet):
        """
        Send packet to the serial port
        """
        self.ser.reset_input_buffer()
        self.ser.write(bytearray(packet))

    def single_read_packet(self, instruction_packet):
        """
        Single read packet from the serial port (only one BEAR each time)
        Instruction: [0xFF, 0xFF, ID, Length, Instruction, P_1, P_2, ..., P_N, Checksum]
                     Length = # of rest bytes (i.e., from Instruction to Checksum)
                        P_i = Read_Address_i or [Write_Address_i, Write_Data_i (4-byte)]
             Return: [0xFF, 0xFF, ID, Length, Error, P_1, P_2, ..., P_N, Checksum]
                     Length = # of rest bytes (i.e., from Error to Checksum)
                        P_i = Read_Data_i (4-byte)
        """
        # send the instruction packet first
        self.send_packet(instruction_packet)

        # now waiting for response
        t0 = time.time()
        c0 = 1  # try count
        while self.ser.in_waiting < 4:                  # so at least we got the packet length info
            if time.time() - t0 > self.single_timeout:  # if timeout
                if c0 >= self.single_try_num:           # if max try number
                    self.print_warning(warning='max_try_num')
                    return None, None
                self.print_warning(warning='single_timeout')
                self.send_packet(instruction_packet)    # resend the instruction packet
                t0 = time.time()
                c0 += 1

        # ok we have something in the buffer
        return_packet = []
        return_packet.extend(self.ser.read(4))
        rest_length = return_packet[3]
        t0 = time.time()
        while self.ser.in_waiting < rest_length:          # make sure the packet is complete
            if time.time() - t0 > self.single_timeout:    # if timeout here, packet is incomplete
                self.print_warning(warning='bad_data')
                return None, None
        return_packet.extend(self.ser.read(rest_length))  # get the whole packet

        # check return packet
        if self.checksum(return_packet) == return_packet[-1]:
            error_code  = return_packet[4]
            data_packet = return_packet[5:-1]
            return data_packet, error_code
        else:
            self.print_warning(warning='bad_data')
            return None, None

    def bulk_read_packet(self, instruction_packet, bear_num):
        """
        Bulk read packet from the serial port (multiple BEARs each time)
        Instruction: [0xFF, 0xFF, 0xFE, Length, 0x08, IDs, R/Ws, R_1, R_2, ..., R_L, W_1, W_2, ..., W_N, P_1, P_2, ..., P_M, Checksum]
                     Length = # of rest bytes (i.e., from 0x08 to Checksum)
                        IDs = # of BEARs, i.e., M
                       R/Ws = # of Read/Write Addresses, i.e., 0xLN
                        R_i = Read_Address_i, i = 1, ..., L
                        W_i = Write_Address_i, i = 1, ..., N
                        P_i = ID_i or [ID_i, Write_Data_1, Write_Data_2, ..., Write_Data_N (4-byte)], i = 1, ..., M
             Return: [0xFF, 0xFF, ID 1, Length, Error 1, P_11, P_12, ..., P_1L, Checksum 1,
                      0xFF, 0xFF, ID 2, Length, Error 2, P_21, P_22, ..., P_2L, Checksum 2,
                      ...
                      0xFF, 0xFF, ID M, Length, Error M, P_M1, P_M2, ..., P_ML, Checksum M]
                     Length = # of rest bytes for each component packet (i.e., from Error to Checksum)
                       P_ij = Read_Data_ij (4-byte), i = 1, ..., M, j = 1, ..., L
        """
        # send the instruction packet first
        self.send_packet(instruction_packet)

        # now waiting for response
        t0 = time.time()
        c0 = 1  # try count
        while self.ser.in_waiting < 4:                # so at least we got the component packet length info
            if time.time() - t0 > self.bulk_timeout:  # if timeout
                if c0 == self.single_try_num:         # if max try number
                    self.print_warning(warning='max_try_num')
                    return None, None
                self.print_warning(warning='bulk_timeout')
                self.send_packet(instruction_packet)  # resend the instruction packet
                t0 = time.time()
                c0 += 1

        # ok we have something in the buffer
        return_packet = []
        return_packet.extend(self.ser.read(4))
        component_packet_length = return_packet[3] + 4
        rest_length = component_packet_length * bear_num - 4
        t0 = time.time()
        while self.ser.in_waiting < rest_length:          # make sure the packet is complete
            if time.time() - t0 > self.bulk_timeout:      # if timeout here, packet is incomplete
                self.print_warning(warning='bad_data')
                return None, None
        return_packet.extend(self.ser.read(rest_length))  # get the whole packet

        data_packet = []
        error_code = []
        for idx in range(bear_num):
            id0 = idx * component_packet_length
            id1 = id0 + 4
            id2 = id1 + 1
            id4 = id0 + component_packet_length
            id3 = id4 - 1
            if self.checksum(return_packet[id0:id4]) == return_packet[id3]:
                error_code.append(return_packet[id1])
                data_packet.append(return_packet[id2:id3])
            else:
                error_code.append(None)
                data_packet.append(None)
        return data_packet, error_code

    def _ping(self, m_id):
        """
        Ping a single BEAR
        """
        instruction_packet      = [0xFF, 0xFF, m_id, 2, INSTRUCTION['PING'], 0]
        instruction_packet[-1]  = self.checksum(instruction_packet)
        data_packet, error_code = self.single_read_packet(instruction_packet)
        data = self.hex_to_int32(data_packet) if data_packet is not None else None
        return [data], error_code

    def _save_config(self, m_id):
        """
        Save configuration registers of a single BEAR
        """
        instruction_packet     = [0xFF, 0xFF, m_id, 2, INSTRUCTION['SAVE_CFG'], 0]
        instruction_packet[-1] = self.checksum(instruction_packet)
        self.send_packet(instruction_packet)

    def _set_posi(self, m_id, position, tolerance):
        """
        Set absolute position of a single BEAR
        """
        instruction_packet     = [0xFF, 0xFF, m_id, 10, INSTRUCTION['SET_POSI']] + self.float32_to_hex(position) + self.float32_to_hex(tolerance) + [0]
        instruction_packet[-1] = self.checksum(instruction_packet)
        self.send_packet(instruction_packet)

    def _single_read(self, m_id, reg_name_list, reg_type=None):
        """
        Read multiple registers (of same register type) of a single BEAR
        reg_name_list: list of register names, e.g., [name1, name2]
        """
        if reg_type == 'cfg':
            instruction = INSTRUCTION['READ_CFG']
        elif reg_type == 'stat':
            instruction = INSTRUCTION['READ_STAT']

        reg_num = len(reg_name_list)
        reg_add_list = []
        for reg_name in reg_name_list:
            reg_add_list.append(REGISTER['ADDRESS'][reg_name])
        instruction_packet     = [0xFF, 0xFF, m_id, reg_num + 2, instruction] + reg_add_list + [0]
        instruction_packet[-1] = self.checksum(instruction_packet)

        data_packet, error_code = self.single_read_packet(instruction_packet)
        if data_packet is not None:
            data = []
            for idx, name in enumerate(reg_name_list):
                id0 = idx * 4
                id1 = id0 + 4
                if REGISTER['DATA_TYPE'][name] == 'f32':
                    data.append(self.hex_to_float32(data_packet[id0:id1]))
                elif REGISTER['DATA_TYPE'][name] == 'u32':
                    data.append(self.hex_to_int32(data_packet[id0:id1]))
            return data, error_code
        else:
            return [None] * reg_num, None

    def _single_write(self, m_id, reg_name_list, data_list, reg_type=None):
        """
        Write multiple registers (of same register type) of a single BEAR
        e.g., _single_write(id, [name1, name2], [data1, data2], reg_type)
        """
        if reg_type == 'cfg':
            instruction = INSTRUCTION['WRITE_CFG']
        elif reg_type == 'stat':
            instruction = INSTRUCTION['WRITE_STAT']

        reg_num = len(reg_name_list)
        reg_add_and_data_list = []
        for idx, reg_name in enumerate(reg_name_list):
            reg_add_and_data_list.append(REGISTER['ADDRESS'][reg_name])
            if REGISTER['DATA_TYPE'][reg_name] == 'f32':
                reg_add_and_data_list += self.float32_to_hex(data_list[idx])
            elif REGISTER['DATA_TYPE'][reg_name] == 'u32':
                reg_add_and_data_list += self.int32_to_hex(data_list[idx])
        instruction_packet     = [0xFF, 0xFF, m_id, reg_num * 5 + 2, instruction] + reg_add_and_data_list + [0]
        instruction_packet[-1] = self.checksum(instruction_packet)
        self.send_packet(instruction_packet)
        
    def _bulk_read_write_stat(self, m_ids, read_stat_name_list, write_stat_name_list, data_list):
        """
        Read and write multiple status registers of multiple BEARs
        e.g., _bulk_read_write([id1, id2], [read_name1, read_name2], [write_name1, write_name2], [[data11, data12], [data21, data22]])
        """
        instruction = INSTRUCTION['BULK_STAT']

        m_num = len(m_ids)
        read_stat_num = len(read_stat_name_list)
        write_stat_num = len(write_stat_name_list)
        total_stat_num = write_stat_num | read_stat_num << 4

        read_stat_add_list = []
        if read_stat_num:
            for read_stat_name in read_stat_name_list:
                read_stat_add_list.append(REGISTER['ADDRESS'][read_stat_name])

        write_stat_add_list = []
        if write_stat_num:
            for write_stat_name in write_stat_name_list:
                write_stat_add_list.append(REGISTER['ADDRESS'][write_stat_name])

            m_id_and_data_list = []
            for idx in range(m_num):
                m_id_and_data_list.append(m_ids[idx])
                for jdx, write_stat_name in enumerate(write_stat_name_list):
                    if REGISTER['DATA_TYPE'][write_stat_name] == 'f32':
                        m_id_and_data_list += self.float32_to_hex(data_list[idx][jdx])
                    elif REGISTER['DATA_TYPE'][write_stat_name] == 'u32':
                        m_id_and_data_list += self.int32_to_hex(data_list[idx][jdx])
        else:
            m_id_and_data_list = m_ids

        length = m_num + read_stat_num + write_stat_num + write_stat_num * m_num * 4 + 4
        instruction_packet = [0xFF, 0xFF, 0xFE, length, instruction, m_num, total_stat_num] + read_stat_add_list + write_stat_add_list + m_id_and_data_list + [0]
        instruction_packet[-1] = self.checksum(instruction_packet)

        if read_stat_num:
            data_packet, error_code = self.bulk_read_packet(instruction_packet, m_num)
            if data_packet is not None:
                data = []
                for idx in range(m_num):
                    data_i = []
                    if data_packet[idx] is not None:
                        for jdx, read_stat_name in enumerate(read_stat_name_list):
                            jd0 = jdx * 4
                            jd1 = jd0 + 4
                            if REGISTER['DATA_TYPE'][read_stat_name] == 'f32':
                                data_i.append(self.hex_to_float32(data_packet[idx][jd0:jd1]))
                            elif REGISTER['DATA_TYPE'][read_stat_name] == 'u32':
                                data_i.append(self.hex_to_int32(data_packet[idx][jd0:jd1]))
                        data.append(data_i)
                    else:
                        data.append([None] * read_stat_num)
                return data, error_code
            else:
                return [[None] * read_stat_num] * m_num, [None] * m_num
        else:
            self.send_packet(instruction_packet)

    @staticmethod
    def checksum(packet):
        return 255 - sum(packet[2:-1]) % 256

    @staticmethod
    def hex_to_float32(byte):
        return struct.unpack('f', bytearray(byte))[0]

    @staticmethod
    def float32_to_hex(data):
        byte = struct.pack('f', data)
        return [byte[0], byte[1], byte[2], byte[3]]

    @staticmethod
    def hex_to_int32(byte):
        return byte[0] | byte[1] << 8 | byte[2] << 16 | byte[3] << 24

    @staticmethod
    def int32_to_hex(data):
        byte = struct.pack('i', data)
        return [byte[0], byte[1], byte[2], byte[3]]

    @staticmethod
    def print_warning(warning):
        msg = "[PyBEAR | WARNING] :: "
        if warning == 'single_timeout':
            msg += "Single read timed out. Now try again ..."
            cprint(msg, 'yellow')
        elif warning == 'bulk_timeout':
            msg += "Bulk read timed out. Now try again ..."
            cprint(msg, 'yellow')
        elif warning == 'max_try_num':
            msg += "Maximum try number reached. Just move on."
            cprint(msg, 'red')
        elif warning == 'bad_data':
            msg += "Data corrupted. Just move on."
            cprint(msg, 'red')