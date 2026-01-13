from ._ble_backend import APP3Backend
import time
from dataclasses import dataclass, field
import collections
import numpy as np
import struct
import pandas as pd
# from _app3 import APP3

WRITE_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
READ_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
N_LINES_UP = '\033[{}A'
LINE_CLEAR = '\x1b[2K'

BAN_ANSWER_TYPE_CENTRAL_RAW_DATA = 125
BAN_ANSWER_TYPE_CENTRAL_NONREP_SUB_SCORE = 126
BAN_ANSWER_TYPE_CENTRAL_QUATERNION_DATA = 131
BAN_ANSWER_TYPE_CENTRAL_BATCHED_QUATERNION_DATA = 140
# t is coming in ms
RAW_DATA_LINE_PRINT = 'T_{} [s]: {}, ax: {:5.2f}, ay: {:5.2f}, az: {:5.2f}, ' + \
                                    'gx: {:5.2f}, gy: {:5.2f}, gz: {:5.2f}'
QUATERNION_DATA_LINE_PRINT = 'T_{} [s]: {}, qx: {:9.5f}, qy: {:9.5f}, qz: {:9.5f}, qw: {:9.5f}, q_accuracy [rad]: {:1.5f}'

# move the cursor up for printing data
def lines_up_in_print(n):
    print(N_LINES_UP.format(n), end='')


class SCS:
    """Implementation of the SCS hardware interface for data access
    """
    # Consider parsing the settins dictionary instead half of those parameters a time
    def __init__(self, address: str, print_received_data: bool = False, default_cb = print, is_ban = False,
                 mounting_points = None, activity_type = 'grv', settings_scs: dict = {}):
        self.address = address
        self.flashed = False
        self.print_received_data = print_received_data
        self.data_buff = []
        self.last_ts_table = {}
        self.sensor_callback_table = {}
        self.active_sensors = []
        self.sensor_description_table = {}
        self.backend = APP3Backend(self.address, self.__callback)
        self.default_cb = default_cb
        self.is_ban = is_ban
        self.mounting_points = settings_scs['mounting_points']
        self.enabled_mounting_points = settings_scs['enabled_mounting_points']
        self.n_mounting_points = settings_scs['num_devices']
        self.sensor_id = settings_scs['sensor_id']
        self.enabled_mac_addresses = settings_scs['enabled_mac_addresses']
        self.dataset_groups = settings_scs['dataset_groups']
        self.devices_mounting = settings_scs['devices_mounting']
        self.data_record = False

    def get_sensor_odr(self, sensor_id):
        if sensor_id in self.last_ts_table:
            return 1 / np.mean(np.diff(np.array(self.last_ts_table[sensor_id])))
        else:
            return None

    @property
    def avaliable_sensors(self):
        """Method for getting available sensors in the device

        Returns:
            List (Sensor): List of available sensors as Sensor class instances
        """
        return self.get_avaliable_sensors()

    @property
    def connected(self):
        """Check methods for device status

        Returns:
            Bool: True if connected, False otherwise
        """
        status = self.backend.status
        if status == 'Disconnected':
            return False
        else:
            return True

    @property
    def status(self):
        return self.backend.status

    def append_to_buffer(self, data: bytes):
        self.data_buff.append(data)

    def send_data(self, data: bytes):
        self.backend.send_data(data)

    def send_command(self, command: str, delimiter='\r\n'):
        data = bytes(command, 'utf-8') + bytes(delimiter, 'utf-8')
        self.backend.send_data(data)

    def print_single_line_quaternion_data(self, decoded_data, m):
        line_print = QUATERNION_DATA_LINE_PRINT.format( m,
                                                        decoded_data.time/1000,
                                                        decoded_data.sensor_data['qx_{}'.format(m)],
                                                        decoded_data.sensor_data['qy_{}'.format(m)],
                                                        decoded_data.sensor_data['qz_{}'.format(m)],
                                                        decoded_data.sensor_data['qw_{}'.format(m)],
                                                        decoded_data.sensor_data['q_accuracy_{}'.format(m)])
        self.default_cb(line_print)

    def print_quaternion_data(self, decoded_data):
        # keep it in the same lines
        lines_up_in_print(self.n_mounting_points)

        # print for each device
        for m in self.enabled_mounting_points:
            self.print_single_line_quaternion_data(decoded_data, m)

    def print_and_store_quaternion_data(self, decoded_data):
        # keep it in the same lines
        lines_up_in_print(self.n_mounting_points)

        # data_pkt = [group timestamp followed by quaternions components and accuracy]
        data_pkt = [decoded_data.time/1000.]
        for m in self.enabled_mounting_points:
            # print for each device
            self.print_single_line_quaternion_data(decoded_data, m)
            # pack quaternion data
            data_pkt += [   decoded_data.sensor_data['qx_{}'.format(m)],
                            decoded_data.sensor_data['qy_{}'.format(m)],
                            decoded_data.sensor_data['qz_{}'.format(m)],
                            decoded_data.sensor_data['qw_{}'.format(m)],
                            decoded_data.sensor_data['q_accuracy_{}'.format(m)] ]
        # update buffer
        self.data_buff.append(data_pkt)


    def print_single_line_central_raw_data(self, decoded_data, m):
        line_print = RAW_DATA_LINE_PRINT.format(m,
                                                decoded_data.time/1000,
                                                decoded_data.sensor_data['ax_{}'.format(m)],
                                                decoded_data.sensor_data['ay_{}'.format(m)],
                                                decoded_data.sensor_data['az_{}'.format(m)],
                                                decoded_data.sensor_data['gx_{}'.format(m)],
                                                decoded_data.sensor_data['gy_{}'.format(m)],
                                                decoded_data.sensor_data['gz_{}'.format(m)], )
        self.default_cb(line_print)


    def print_central_raw_data(self, decoded_data):
        # keep it in the same lines
        lines_up_in_print(self.n_mounting_points)

        for m in self.enabled_mounting_points:
            self.print_single_line_central_raw_data(decoded_data, m)


    def print_and_store_central_raw_data(self, decoded_data):
        # keep it in the same lines
        lines_up_in_print(self.n_mounting_points)

        # data_pkt = [group timestamp followed by acc and gyro raw data for each direction]
        data_pkt = [decoded_data.time/1000.]
        for m in self.enabled_mounting_points:
            self.print_single_line_central_raw_data(decoded_data, m)
            # pack raw data for each device
            data_pkt += [   decoded_data.sensor_data['ax_{}'.format(m)],
                            decoded_data.sensor_data['ay_{}'.format(m)],
                            decoded_data.sensor_data['az_{}'.format(m)],
                            decoded_data.sensor_data['gx_{}'.format(m)],
                            decoded_data.sensor_data['gy_{}'.format(m)],
                            decoded_data.sensor_data['gz_{}'.format(m)], ]
        self.data_buff.append(data_pkt)


    def __callback(self, _: int, data: bytes):
        """Method for parsing a byte format data

        Args:
            _ (int): _description_
            data (bytes): data as a byte string from all devices

            Data structure description is here: https://inside-docupedia.bosch.com/confluence/pages/viewpage.action?pageId=2910539115
        """
        decoded_data_list = self.sensor_data_parser(data)
        if not isinstance(decoded_data_list, list):
            decoded_data_list = [decoded_data_list]

        for decoded_data in decoded_data_list:
            self._process_decoded_data(decoded_data, data)

    def _process_decoded_data(self, decoded_data, raw_data):
        if self.is_ban:
            if self.print_received_data and (decoded_data.name != 'status_message'):
                if decoded_data.sensor_id == BAN_ANSWER_TYPE_CENTRAL_QUATERNION_DATA:
                    # t is coming in ms
                    for m in self.mounting_points:
                        self.default_cb('T_{} [s]: {}, qx: {:5.2f}, qy: {:5.2f}, qz: {:5.2f}, qw: {:5.2f}, q_accuracy [rad]: {:1.5f}'.format(m,
                                                                                                                                                decoded_data.timestamp["t"],
                                                                                                                                                decoded_data.sensor_data['qx_{}'.format(m)],
                                                                                                                                                decoded_data.sensor_data['qy_{}'.format(m)],
                                                                                                                                                decoded_data.sensor_data['qz_{}'.format(m)],
                                                                                                                                                decoded_data.sensor_data['qw_{}'.format(m)],
                                                                                                                                                decoded_data.sensor_data['q_accuracy_{}'.format(m)]))

                    # for _ in range(len(self.mounting_points)):
                    #     print(LINE_CLEAR, end=LINE_UP)
                    lines_to_move = len(self.mounting_points) + 1
                    cursor_up_sequence = f'\033[{lines_to_move}A'
                    print(cursor_up_sequence, end='')

                elif decoded_data.sensor_id == BAN_ANSWER_TYPE_CENTRAL_RAW_DATA:
                    for m in self.mounting_points:
                        self.default_cb('T_{}: {:5.2f}, ax: {:5.2f}, ay: {:5.2f}, az: {:5.2f}, gx: {:5.2f}, gy: {:5.2f}, gz: {:5.2f}'.format(m,
                                                                                                                                             decoded_data.time/1000,
                                                                                                                                             decoded_data.sensor_data['ax_{}'.format(m)],
                                                                                                                                             decoded_data.sensor_data['ay_{}'.format(m)],
                                                                                                                                             decoded_data.sensor_data['az_{}'.format(m)],
                                                                                                                                             decoded_data.sensor_data['gx_{}'.format(m)],
                                                                                                                                             decoded_data.sensor_data['gy_{}'.format(m)],
                                                                                                                                             decoded_data.sensor_data['gz_{}'.format(m)]))
                    lines_to_move = len(self.mounting_points)
                    cursor_up_sequence = f'\033[{lines_to_move}A'
                    print(cursor_up_sequence, end='')


                elif decoded_data.sensor_id == BAN_ANSWER_TYPE_CENTRAL_NONREP_SUB_SCORE:
                    self.default_cb('Scores: {}, nbrPeaksWithin: {}, deviceId: {}, patternId: {}, gestureId: {}'.format(decoded_data.sensor_data['scores'],
                                                                                                                        decoded_data.sensor_data['nbrPeaksWithin'],
                                                                                                                        decoded_data.sensor_data['deviceId' ],
                                                                                                                        decoded_data.sensor_data['patternId'],
                                                                                                                        decoded_data.sensor_data['gestureId'],))

                # BAN_ANSWER_TYPE_CENTRAL_RAW_DATA
                elif decoded_data.sensor_id == 125 and not self.data_record:
                    self.print_central_raw_data(decoded_data)

                elif decoded_data.sensor_id == 125 and self.data_record:
                    self.print_and_store_central_raw_data(decoded_data)

                else:
                    self.sensor_callback_table[0](decoded_data)

        if decoded_data.sensor_id is not None:
            if decoded_data.sensor_id not in self.active_sensors:
                self.active_sensors.append(decoded_data.sensor_id)
            if decoded_data.sensor_id not in self.last_ts_table:
                self.last_ts_table[decoded_data.sensor_id] = collections.deque(maxlen=10)

            # If sensor is an IMU, decode using the IMU parser
            if decoded_data is not None:
                self.last_ts_table[decoded_data.sensor_id].append(decoded_data.timestamp)
                # if sensor has callback, send decoded data to callback
                if decoded_data.sensor_id in self.sensor_callback_table:
                    # Ii sends data to a callback function in the dictionary
                    self.sensor_callback_table[decoded_data.sensor_id](decoded_data)
        else:
            self.default_cb(raw_data) # Required for ble_test_cliscs.py
            self.append_to_buffer(raw_data)

    def connect(self):
        self.backend.connect()
        time.sleep(3)

        if not self.is_ban:
            self.deactivate_all_sensors()
            if self.check_if_flashed():
                self.get_avaliable_sensors()

    def deactivate_all_sensors(self):
        self.deactivate_sensors(self.active_sensors)

    def clear_buffer(self):
        self.data_buff = []

    def check_if_flashed(self):
        self.clear_buffer()
        self.send_data(b'info\r\n')
        time.sleep(1)
        for data in self.data_buff:
            if data == b'[E][Sensor error] Bootloader reports: Firmware Upload Failed: FW Header Missing\r\n':
                self.flashed = False
                return False
        self.flashed = True
        return True

    def get_avaliable_firmwares(self):
        self.clear_buffer()
        self.send_data(b'ls\r\n')
        time.sleep(3)
        avaliable_firmwares = []
        for data in self.data_buff:
            if b'.fw' in data:
                avaliable_firmwares.append(data.decode("utf-8").split(' | ')[0].strip())
        return avaliable_firmwares

    def flash_firmware(self, firmware: str):
        self.clear_buffer()
        self.send_data(b'ramb ' + bytes(firmware, 'utf-8') + b'\r\n')
        time.sleep(5)
        for data in self.data_buff:
            if b'Booting from RAM successful\r\n' in data:
                self.flashed = True
                return True
        self.flashed = False
        return False

    def disconnect(self):
        self.backend.disconnect()

    def set_sensor_callback(self, sensor, callback):
        if type(sensor) is SensorDescription:
            self.sensor_callback_table[sensor.id] = callback
        else:
            self.sensor_callback_table[sensor] = callback

    def deactivate_sensors(self, sensors):
        if not hasattr(sensors, '__iter__'):
            sensors = [sensors]
        for sensor in sensors:
            self.activate_sensors(sensor, 0)
            time.sleep(1)
        for sensor in sensors:
            if sensor in self.active_sensors:
                self.active_sensors.remove(sensor)

    def activate_sensor(self, activity_type, sampling_frequency, mock_flag, leaf_flag,  mac_addresses, mac_addresses_string, control_commands, callback_functions):
        """Method to activate sensors in central and/or leaf nodes

        Args:
            sensors (int): id of a sensor to activate
            sampling_frequencies (int): _description_
            callback_functions (_type_, optional): _description_. Defaults to None.
        """
        frequency_byte = struct.pack('b', sampling_frequency)
        if isinstance(activity_type, list):
            activity_type = activity_type[0]

        match activity_type:
            case 'grv':
                sensor_id = BAN_ANSWER_TYPE_CENTRAL_QUATERNION_DATA
                exercise_id = b'\xF0' # 240
            case 'raw_data':
                sensor_id = 125
                exercise_id = b'\x00' # 0
            case 'lwt':
                exercise_id = b'\x64' # 100

        if mock_flag:
            mock_byte = b'\x01'
            print('enable mock data')
        else:
            mock_byte = b'\x00'
            print('enable real data')

        if leaf_flag:
            print('enable leaf')
            self.send_data(b'\x32\x04\x01\x00\x00\x00')
            time.sleep(1)

            self.send_data(b'\x33\x08\x00\x00\x00\x00\x00' + frequency_byte + exercise_id + mock_byte)
        else:
            print('enable central')
            cmd = b'\x00\x38'
            # BAN_COMMAND_TYPE_CENTRAL_CONFIG
            # for mac_address, control_command in zip(mac_addresses, control_commands):
            #     cmd += mac_address+control_command
            #     print(control_commands)
            # set the mac and command, old method #
            mac_addresses_conv = [address[::-1] for address in mac_addresses]
            for i in range(len(mac_addresses_string)):
                cmd += mac_addresses_conv[i] + struct.pack('b', control_commands[i][0])
            for i in range(8 - len(mac_addresses_string)):
                cmd += b"\x00\x00\x00\x00\x00\x00\x00"

            self.send_data(cmd)
            time.sleep(3)
            # BAN_COMMAND_TYPE_CENTRAL_CONTROL
            # self.send_data(b'\x19\x0C\x00\x00\x00\x00\x00\x00\x00\x00' + frequency_byte + exercise_id + mock_byte + b'\x00')
            cmd = b'\x19\x0C'
            log_time = b'\x00\x00\x00\x00'
            log_algo_enable = b'\x00'
            log_sens_enable = b'\x00'
            feedback_coach_enable = b'\x00'
            adaptive_pattern_enable = b'\x00'
            cmd += log_time + log_algo_enable + log_sens_enable + feedback_coach_enable + adaptive_pattern_enable
            cmd += frequency_byte + exercise_id + mock_byte + b'\x00'
            self.send_data(cmd)

        if callback_functions is not None:
            print('set callback functions')
            self.sensor_callback_table[sensor_id] = callback_functions

    def sensor_data_parser(self, data: bytes):
        '''Method for extraction of the sensor_id, timestamp, and data from sensor data stream
        We use 8 devices here

        Returns:
            SensorData: instance of SensorData class

        '''
        answer_type = data[0]

        if answer_type == BAN_ANSWER_TYPE_CENTRAL_QUATERNION_DATA:
            data_name = 'BAN_ANSWER_TYPE_CENTRAL_QUATERNION_DATA'
            byte_length = 15
            timestamp = int.from_bytes(data[3:5], byteorder='little')
            # Data type is grv if 108 and the corresponding length should be 84 for timestamp and data
            # Quaternion inclides x, y, z, w, accuracy fields
            ban_idx = data[2]
            out_data = SensorData(answer_type, data_name)

            if ban_idx < len(self.enabled_mounting_points):
                m = self.enabled_mounting_points[ban_idx]
                out_data.timestamp["t"] = timestamp
                out_data.sensor_data['qx_{}'.format(m)] = int.from_bytes(data[5:7], byteorder='little', signed=True) / (16384 * 1)
                out_data.sensor_data['qy_{}'.format(m)] = int.from_bytes(data[7:9], byteorder='little', signed=True) / (16384 * 1)
                out_data.sensor_data['qz_{}'.format(m)] = int.from_bytes(data[9:11], byteorder='little', signed=True) / (16384 * 1)
                out_data.sensor_data['qw_{}'.format(m)] = int.from_bytes(data[11:13], byteorder='little', signed=True) / (16384 * 1)
                out_data.sensor_data['q_accuracy_{}'.format(m)] = int.from_bytes(data[13:15], byteorder='little', signed=False) / (16384 * 1)
                out_data.meta_data["mac"] = self.enabled_mac_addresses[ban_idx]
                out_data.meta_data['mounting_{}'.format(m)] = m
                out_data.meta_data['idx'] = ban_idx
            time_status = timestamp

            return out_data

        elif answer_type == BAN_ANSWER_TYPE_CENTRAL_RAW_DATA:
            data_name = 'BAN_ANSWER_TYPE_CENTRAL_RAW_DATA'
            byte_length = 12
            # Non-rep sensor data includes accelerometer and gyroscope
            timestamp = int.from_bytes(data[2:6], byteorder='little')

            # The dynamic ranges are defined in the FW setup
            dynamic_range_acc = 16
            dynamic_range_gyro = 4096

            out_data = SensorData(answer_type, timestamp, data_name)
            for ii, m in enumerate(self.enabled_mounting_points):
                out_data.sensor_data['ax_{}'.format(m)] = int.from_bytes(data[byte_length*ii+6:byte_length*ii+8], byteorder='little', signed=True) / (16384 * 2 / dynamic_range_acc)
                out_data.sensor_data['ay_{}'.format(m)] = int.from_bytes(data[byte_length*ii+8:byte_length*ii+10], byteorder='little', signed=True) / (16384 * 2 / dynamic_range_acc)
                out_data.sensor_data['az_{}'.format(m)] = int.from_bytes(data[byte_length*ii+10:byte_length*ii+12], byteorder='little', signed=True) / (16384 * 2 / dynamic_range_acc)

                out_data.sensor_data['gx_{}'.format(m)] = int.from_bytes(data[byte_length*ii+12:byte_length*ii+14], byteorder='little', signed=True) / (16384 * 2 / dynamic_range_gyro)
                out_data.sensor_data['gy_{}'.format(m)] = int.from_bytes(data[byte_length*ii+14:byte_length*ii+16], byteorder='little', signed=True) / (16384 * 2 / dynamic_range_gyro)
                out_data.sensor_data['gz_{}'.format(m)] = int.from_bytes(data[byte_length*ii+16:byte_length*ii+18], byteorder='little', signed=True) / (16384 * 2 / dynamic_range_gyro)
            return out_data

        elif answer_type == BAN_ANSWER_TYPE_CENTRAL_BATCHED_QUATERNION_DATA:
            # Batching Protocol Spec (Hypothetical):
            # [Type(1)][Count(1)][BanIdx(1)][BaseTS(2)][Reserved(3?)]...[Data(10) x Count]
            # Data(10): qx(2), qy(2), qz(2), qw(2), acc(2)
            data_name = 'BAN_ANSWER_TYPE_CENTRAL_BATCHED_QUATERNION_DATA'
            count = data[1]
            ban_idx = data[2]
            base_timestamp = int.from_bytes(data[3:5], byteorder='little')

            out_list = []
            if ban_idx < len(self.enabled_mounting_points):
                m = self.enabled_mounting_points[ban_idx]

                # Assume 50Hz (20ms) interval for interpolation if needed,
                # or just use BaseTS for all (simplified)
                # Header size = 8 bytes (Type, Count, Idx, TS, +3 padding/reserved to align?)
                # Let's assume compact: Header = 5 bytes.
                # Start of data = 5.
                offset = 5
                block_size = 10

                for i in range(count):
                    if offset + block_size > len(data): break

                    sample_ts = base_timestamp + (i * 20) # Mock 20ms delta
                    out_data = SensorData(answer_type, data_name)
                    out_data.timestamp["t"] = sample_ts

                    out_data.sensor_data['qx_{}'.format(m)] = int.from_bytes(data[offset:offset+2], byteorder='little', signed=True) / (16384 * 1)
                    out_data.sensor_data['qy_{}'.format(m)] = int.from_bytes(data[offset+2:offset+4], byteorder='little', signed=True) / (16384 * 1)
                    out_data.sensor_data['qz_{}'.format(m)] = int.from_bytes(data[offset+4:offset+6], byteorder='little', signed=True) / (16384 * 1)
                    out_data.sensor_data['qw_{}'.format(m)] = int.from_bytes(data[offset+6:offset+8], byteorder='little', signed=True) / (16384 * 1)
                    out_data.sensor_data['q_accuracy_{}'.format(m)] = int.from_bytes(data[offset+8:offset+10], byteorder='little', signed=False) / (16384 * 1)

                    out_data.meta_data["mac"] = self.enabled_mac_addresses[ban_idx]
                    out_data.meta_data['mounting_{}'.format(m)] = m
                    out_data.meta_data['idx'] = ban_idx

                    # For batched data, we override sensor_id to treat it like normal quaternion data for downstream logic
                    out_data.sensor_id = BAN_ANSWER_TYPE_CENTRAL_QUATERNION_DATA

                    out_list.append(out_data)
                    offset += block_size

            return out_list

        elif answer_type == BAN_ANSWER_TYPE_CENTRAL_NONREP_SUB_SCORE:
            data_name = 'BAN_ANSWER_TYPE_LEAF_NONREP_SUB_SCORE'
            byte_length = 8

            out_data = SensorData(answer_type, 0, data_name)

            out_data.sensor_data['scores'] = struct.unpack('f', data[2:6])
            out_data.sensor_data['nbrPeaksWithin'] = int.from_bytes(data[6:7], byteorder='little', signed=True)
            out_data.sensor_data['deviceId'] = int.from_bytes(data[7:8], byteorder='little', signed=False)
            out_data.sensor_data['patternId'] = int.from_bytes(data[8:9], byteorder='little', signed=False)
            out_data.sensor_data['gestureId'] = int.from_bytes(data[9:10], byteorder='little', signed=False)
            return out_data

        else:
            sensor_name = 'status_message'
            out_data = SensorData(None, sensor_name)
            return out_data


@dataclass
class SensorDescription:
    activity_type: str
    frequency: int
    name: str

    def __post_init__(self):
        self.activity_type = self.activity_type
        self.name = self.name
        self.frequency = int(self.frequency)

# @dataclass
# class SensorDescription:
#     activity_type: int
#     name: str
#     id: int
#     version: int
#     min_rate: float
#     max_rate: float

#     def __post_init__(self):
#         self.sensor_id = int(self.sensor_id)
#         self.id = int(self.id)
#         self.version = int(self.version)
#         self.min_rate = float(self.min_rate)
#         self.max_rate = float(self.max_rate)


@dataclass
class SensorData:
    sensor_id: int
    name: str
    timestamp: dict = field(default_factory=dict, init=False)
    sensor_data: dict = field(default_factory=dict, init=False)
    meta_data: dict = field(default_factory=dict, init=False)
