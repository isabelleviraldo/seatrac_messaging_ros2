#!/usr/bin/env python3

import numpy
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import binascii
import time

import serial
import struct

import std_msgs.msg
import seatrac_driver.msg

import seatrac_constants


class SeaTracDriver(Node):
    # Define ROS parameter defaults for this node
    _DEFAULT_CONTROL_RATE = 20.0 # [Hz]
    _DEFAULT_DEVICE = '/dev/ttyUSB0'
    _DEFAULT_BAUDRATE = 115200
    _DEFAULT_BUFFER_READ_SIZE = 1024

    def __init__(self):
        
        super().__init__('seatrac_driver')
        self.get_logger().info('SeaTrac Driver Node Initilized')

        #reentrant callback group
        self._callback_group = ReentrantCallbackGroup()

        if not rclpy.ok():
            self.get_logger().info(f'ROS master not running!')
            return

        # Handle parameters
        self.declare_parameter('rate', self._DEFAULT_CONTROL_RATE)
        self.rate_hz = self.get_parameter('rate').value
        self.declare_parameter('device', self._DEFAULT_DEVICE)
        self.device = self.get_parameter('device').value
        self.declare_parameter('baudrate', self._DEFAULT_BAUDRATE)
        self.baudrate = self.get_parameter('baudrate').value

        self.declare_parameter('log_io', False)
        self._log_io = self.get_parameter('log_io').value

        # Initial definition of the connection to the SeaTrac device
        self.connection = None

        # Define a clean buffer for incoming data from the SeaTrac device
        self.buffer = bytearray()

        # Stores the driver status variables. Note, the first incoming data packet processed by
        # this driver may be incomplete and therefore a single corrupt packet is expected at
        # driver startup
        self.sent_command_count = 0
        self.corrupt_packet_count = 0
        self.parsed_packet_count = 0
        self.unhandled_packet_count = 0
        self.ping_sent = 0
        self.ping_received = 0
        self.ping_error = 0
        self.data_sent = 0
        self.data_received = 0

        self._driver_status_publisher = self.create_publisher(seatrac_driver.msg.DriverStatus,
                                                              '/seatrac/driver_status', 
                                                              10,
                                                              callback_group=self._callback_group)

        self._system_alive_command_subscriber = self.create_subscription(std_msgs.msg.Empty, 
                                                                '/seatrac/system_alive_command',
                                                                self._on_system_alive_command,
                                                                10,
                                                                callback_group=self._callback_group)
        self._system_alive_response_publisher = self.create_publisher(seatrac_driver.msg.SystemAliveResponse, 
                                                                      '/seatrac/system_alive_response', 
                                                                      10,
                                                                      callback_group=self._callback_group)

        self._system_info_command_subscriber = self.create_subscription(std_msgs.msg.Empty, 
                                                                '/seatrac/system_info_command',
                                                                self._on_system_info_command,
                                                                10,
                                                                callback_group=self._callback_group)
        self._system_info_response_publisher = self.create_publisher(seatrac_driver.msg.SystemInfoResponse, 
                                                                     '/seatrac/system_info_response',
                                                                     10,
                                                                     callback_group=self._callback_group)

        self._system_reboot_command_subscriber = self.create_subscription(std_msgs.msg.Empty, 
                                                                '/seatrac/system_reboot_command',
                                                                self._on_system_reboot_command,
                                                                10,
                                                                callback_group=self._callback_group)
        self._system_reboot_response_publisher = self.create_publisher(seatrac_driver.msg.SystemRebootResponse, 
                                                                       '/seatrac/system_reboot_response', 
                                                                       10,
                                                                       callback_group=self._callback_group)

        self._status_command_subscriber = self.create_subscription(seatrac_driver.msg.StatusCommand, 
                                                                '/seatrac/status_command',
                                                                self._on_status_command,
                                                                10,
                                                                callback_group=self._callback_group)
        self._status_response_publisher = self.create_publisher(seatrac_driver.msg.StatusResponse, 
                                                                '/seatrac/status_response',
                                                                10,
                                                                callback_group=self._callback_group)

        self._status_config_get_command_subscriber = self.create_subscription(std_msgs.msg.Empty, 
                                                                '/seatrac/status_config_get_command',
                                                                self._on_status_config_get_command,
                                                                10,
                                                                callback_group=self._callback_group)
        self._status_config_get_response_publisher = self.create_publisher(seatrac_driver.msg.StatusConfigGetResponse, 
                                                                           '/seatrac/status_config_get_response', 
                                                                           10,
                                                                           callback_group=self._callback_group)

        self._status_config_set_command_subscriber = self.create_subscription(seatrac_driver.msg.StatusConfigSetCommand, 
                                                                '/seatrac/status_config_set_command',
                                                                self._on_status_config_set_command,
                                                                10,
                                                                callback_group=self._callback_group)
        self._status_config_set_response_publisher = self.create_publisher(seatrac_driver.msg.StatusConfigSetResponse, 
                                                                           '/seatrac/status_config_set_response', 
                                                                           10,
                                                                           callback_group=self._callback_group)

        self._settings_get_command_subscriber = self.create_subscription(std_msgs.msg.Empty,
                                                                        '/seatrac/settings_get_command',
                                                                        self._on_settings_get_command,
                                                                        10,
                                                                        callback_group=self._callback_group)

        self._settings_get_response_publisher = self.create_publisher(seatrac_driver.msg.SettingsGetResponse,
                                                                      '/seatrac/settings_get_response',
                                                                      10,
                                                                      callback_group=self._callback_group)

        self._settings_set_command_subscriber = self.create_subscription(seatrac_driver.msg.SettingsSetCommand, 
                                                                '/seatrac/settings_set_command',
                                                                self._on_settings_set_command,
                                                                10,
                                                                callback_group=self._callback_group)
        self._settings_set_response_publisher = self.create_publisher(seatrac_driver.msg.SettingsSetResponse,
                                                                '/seatrac/settings_set_response',
                                                                10,
                                                                callback_group=self._callback_group)

        self._settings_load_command_subscriber = self.create_subscription(std_msgs.msg.Empty, 
                                                                '/seatrac/settings_load_command',
                                                                self._on_settings_load_command,
                                                                10,
                                                                callback_group=self._callback_group)
        self._settings_load_response_publisher = self.create_publisher(seatrac_driver.msg.SettingsLoadResponse, 
                                                                       '/seatrac/settings_load_response', 
                                                                       10,
                                                                       callback_group=self._callback_group)

        self._settings_save_command_subscriber = self.create_subscription(std_msgs.msg.Empty, 
                                                                '/seatrac/settings_save_command',
                                                                self._on_settings_save_command,
                                                                10,
                                                                callback_group=self._callback_group)
        self._settings_save_response_publisher = self.create_publisher(seatrac_driver.msg.SettingsSaveResponse, 
                                                                    '/seatrac/settings_save_response', 
                                                                    10,
                                                                    callback_group=self._callback_group)

        self._settings_reset_command_subscriber = self.create_subscription(std_msgs.msg.Empty, 
                                                                '/seatrac/settings_reset_command',
                                                                self._on_settings_reset_command,
                                                                10,
                                                                callback_group=self._callback_group)
        self._settings_reset_response_publisher = self.create_publisher(seatrac_driver.msg.SettingsResetResponse, 
                                                                        '/seatrac/settings_reset_response', 
                                                                        10,
                                                                        callback_group=self._callback_group)

        self._calibration_action_command_subscriber = self.create_subscription(seatrac_driver.msg.CalibrationActionCommand, 
                                                                '/seatrac/calibration_action_command',
                                                                self._on_cal_action_command,
                                                                10,
                                                                callback_group=self._callback_group)
        self._calibration_action_response_publisher = self.create_publisher(seatrac_driver.msg.CalibrationActionResponse, 
                                                                            '/seatrac/calibration_action_response',
                                                                            10,
                                                                            callback_group=self._callback_group)

        self._calibration_get_command_subscriber = self.create_subscription(std_msgs.msg.Empty, 
                                                                '/seatrac/calibration_get_command',
                                                                self._on_ahrs_cal_get_command,
                                                                10,
                                                                callback_group=self._callback_group)
        self._calibration_get_response_publisher = self.create_publisher(seatrac_driver.msg.CalibrationGetResponse, 
                                                                         '/seatrac/calibration_get_response',
                                                                         10,
                                                                         callback_group=self._callback_group)

        self._calibration_set_command_subscriber = self.create_subscription(seatrac_driver.msg.CalibrationSetCommand, 
                                                                '/seatrac/calibration_set_command',
                                                                self._on_ahrs_cal_set_command,
                                                                10,
                                                                callback_group=self._callback_group)
        self._calibration_set_response_publisher = self.create_publisher(seatrac_driver.msg.CalibrationSetResponse, 
                                                                         '/seatrac/calibration_set_response',
                                                                         10,
                                                                         callback_group=self._callback_group)

        self._transceiver_analyze_command_subscriber = self.create_subscription(std_msgs.msg.Empty, 
                                                                '/seatrac/transceiver_analyze_command',
                                                                self._on_transceiver_analyze_command,
                                                                10,
                                                                callback_group=self._callback_group)
        self._transceiver_analyze_response_publisher = self.create_publisher(seatrac_driver.msg.TransceiverAnalyzeResponse, 
                                                                             '/seatrac/transceiver_analyze_response',
                                                                             10,
                                                                             callback_group=self._callback_group)

        self._tranceiver_fix_status_publisher = self.create_publisher(seatrac_driver.msg.TransceiverFixStatus, 
                                                                      '/seatrac/transceiver_fix_status', 
                                                                      10,
                                                                      callback_group=self._callback_group)

        self._transceiver_status_command_subscriber = self.create_subscription(std_msgs.msg.Empty, 
                                                                '/seatrac/transceiver_status_command',
                                                                self._on_transceiver_status_command,
                                                                10,
                                                                callback_group=self._callback_group)
        self._transceiver_status_response_publisher = self.create_publisher(seatrac_driver.msg.TransceiverStatusResponse, 
                                                                            '/seatrac/transceiver_status_response', 
                                                                            10,
                                                                            callback_group=self._callback_group)

        self._ping_send_command_subscriber = self.create_subscription(seatrac_driver.msg.PingSendCommand, 
                                                                '/seatrac/ping_send_command',
                                                                self._on_ping_send_command,
                                                                10,
                                                                callback_group=self._callback_group)
        self._ping_send_response_publisher = self.create_publisher(seatrac_driver.msg.PingSendResponse, 
                                                                   '/seatrac/ping_send_response', 
                                                                   10,
                                                                   callback_group=self._callback_group)

        self._ping_request_status_publisher = self.create_publisher(seatrac_driver.msg.PingRequestStatus, 
                                                                    '/seatrac/ping_request_status', 
                                                                    10,
                                                                    callback_group=self._callback_group)

        self._ping_response_status_publisher = self.create_publisher(seatrac_driver.msg.PingResponseStatus, 
                                                                     '/seatrac/ping_response_status', 
                                                                     10,
                                                                     callback_group=self._callback_group)

        self._ping_error_status_publisher = self.create_publisher(seatrac_driver.msg.PingErrorStatus, 
                                                                  '/seatrac/ping_error_status', 
                                                                  10,
                                                                  callback_group=self._callback_group)

        self._navigation_query_send_command_subscriber = self.create_subscription(seatrac_driver.msg.NavigationQuerySendCommand, 
                                                                '/seatrac/navigation_query_send_command',
                                                                self._on_nav_query_send_command,
                                                                10,
                                                                callback_group=self._callback_group)
        self._navigation_query_send_response_publisher = self.create_publisher(seatrac_driver.msg.NavigationQuerySendResponse,
                                                                               '/seatrac/navigation_query_send_response',
                                                                               10,
                                                                               callback_group=self._callback_group)

        self._navigation_query_request_status_publisher = self.create_publisher(seatrac_driver.msg.NavigationQueryRequestStatus, 
                                                                                '/seatrac/navigation_query_request_status',
                                                                                10,
                                                                                callback_group=self._callback_group)

        self._navigation_query_response_status_publisher = self.create_publisher(seatrac_driver.msg.NavigationQueryResponseStatus,
                                                                                 '/seatrac/navigation_query_response_status',
                                                                                 10,
                                                                                 callback_group=self._callback_group)

        self._navigation_error_status_publisher = self.create_publisher(seatrac_driver.msg.NavigationErrorStatus, 
                                                                        '/seatrac/navigation_error_status',
                                                                        10,
                                                                        callback_group=self._callback_group)

        self._navigation_status_send_commmand_subscriber = self.create_subscription(seatrac_driver.msg.NavigationStatusSendCommand, 
                                                                '/seatrac/navigation_status_send_command',
                                                                self._on_nav_status_send_command,
                                                                10,
                                                                callback_group=self._callback_group)
        self._navigation_status_send_response_publisher = self.create_publisher(seatrac_driver.msg.NavigationStatusSendResponse, 
                                                                                '/seatrac/navigation_status_send_response',
                                                                                10,
                                                                                callback_group=self._callback_group)

        self._navigation_status_receive_status_publisher = self.create_publisher(seatrac_driver.msg.NavigationStatusReceiveStatus, 
                                                                                 '/seatrac/navigation_status_receive_status',
                                                                                 10,
                                                                                 callback_group=self._callback_group)

        self._data_send_command_subscriber = self.create_subscription(seatrac_driver.msg.DataSendCommand, 
                                                                '/seatrac/data_send_command',
                                                                self._on_dat_send_command,
                                                                10,
                                                                callback_group=self._callback_group)
        self._data_send_response_publisher = self.create_publisher(seatrac_driver.msg.DataSendResponse, 
                                                                   '/seatrac/data_send_response',
                                                                   10,
                                                                   callback_group=self._callback_group)

        self._data_receive_status_publisher = self.create_publisher(seatrac_driver.msg.DataReceiveStatus, 
                                                                    '/seatrac/data_receive_status',
                                                                    10,
                                                                    callback_group=self._callback_group)

        self._data_error_status_publisher = self.create_publisher(seatrac_driver.msg.DataErrorStatus, 
                                                                  '/seatrac/data_error_status',
                                                                  10,
                                                                  callback_group=self._callback_group)

    def run_once(self):
        if (self.connection is None) or (not self.connection.is_open):
            try:
                self.connection = serial.Serial(
                    port=self.device,
                    baudrate=self.baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_TWO,
                    timeout=0,
                    xonxoff=False)
                self.connection.flush()
                time.sleep(2)
                self.connection.reset_input_buffer()
                time.sleep(2)
                if self.connection.is_open:
                    self.get_logger().info(f'Connected to {self.device}!')
            except serial.SerialException as e:
                self.get_logger().warning('Connection not established')
                return

        try:
            self._read_device()
        except Exception as e:
            self.get_logger().error(f'Unexpected error in _read_device: {e}')


    def _read_device(self):
        try:
            # Attempt to read as many bytes as possible (non-blocking) from this connection. If
            # an error is encounted, close the connection and retry connectivity on the next cycle
            self.buffer += self.connection.read(self._DEFAULT_BUFFER_READ_SIZE)
        except serial.SerialException:
            self.get_logger().warning(f'Unable to read from device {self.device}. Attempting to re-establish')
            self.buffer = bytearray()
            self.connection.close()
            return

        all_full_packets_processed = False
        

        # There is no data, all packets have been processed
        if not self.buffer:
            all_full_packets_processed = True

        # IMPORTANT:
        #
        # The SeaTrac data format is "unconventional". Blueprint Subsea has decided to provide its
        # data as a string of hexidecimal representative characters (i.e. '0'-'9', 'A'-'F'). The
        # only non-hex representative characters that will be sent in the datastream will be '$'
        # (denoting incoming resonse/status), '#' (denoting outgoing commands to the SeaTrac), and
        # the '\r\n' character sequence representing end of message.
        #
        # This is definitely not the most space-conscious decision since sending a hex
        # representation of data in bytes doubles the size, but I'm just implementing this circus.

        while not all_full_packets_processed:
            # Find the first sync byte
            start = self.buffer.find(b'$')

            if start == -1:
                # No sync byte was found in the buffer. Purge the buffer and exit
                self.corrupt_packet_count += 1
                self.buffer = bytearray()
                break

            # Move past the sync character and find the message end '\cr\lf'
            start += 1
            end = self.buffer.find(b'\x0D\x0A', start)

            if end == -1:
                # No message end was found. Wait for the next cycle to process this packet
                all_full_packets_processed = True
                continue

            if len(self.buffer[start:end]) % 2 != 0:
                self.get_logger().warning('Error while parsing packet. Hex string has odd length. Cannot convert to binary.')
                # self.get_logger().warn(f'  {}'.format(self.buffer))
                self.corrupt_packet_count += 1
                self.buffer = self.buffer[end+2:]
                continue

            # Convert message from hexidecimal character representation to binary representation
            try:
                packet_binary = binascii.unhexlify(self.buffer[start:end])
            except (binascii.Error, ValueError) as e:
                self.get_logger().warning('Non-hex characters in packet, Skipping corrupt packet.')
                self.corrupt_packet_count += 1
                self.buffer = self.buffer[end+2:]
                return

            # Calculate the checksum of the packet (minus the checksum value)
            calculated_checksum = self._checksum(packet_binary[:-2])

            try:
                packet_checksum = struct.unpack('<H', packet_binary[-2:])[0]
            except struct.error:
                self.get_logger().warning('Error while unpacking checksum value. Skipping corrupt packet.')
                self.corrupt_packet_count += 1
                self.buffer = self.buffer[end+2:]
                continue

            if calculated_checksum != packet_checksum:
                self.get_logger().warning('Incoming packet checksum does not match calculated. Skipping corrupt packet')
                self.corrupt_packet_count += 1
                self.buffer = self.buffer[end+2:]
                continue

            try:
                command_id = packet_binary[0]
            except struct.error:
                self.get_logger().warning('Error while unpacking command ID value. Skipping corrupt packet.')
                self.corrupt_packet_count += 1
                self.buffer = self.buffer[end+2:]
                continue

            # The payload is everything except the command ID and checksum
            payload = packet_binary[1:-2]

            if self._log_io:
                payload_hex = ''
                if len(payload) > 0:
                    payload_hex = ['{:02x}'.format(struct.unpack('<B', byte)[0]) for byte in payload]
                    payload_hex = ' '.join(payload_hex)
                #self.get_logger().info(f'READ_DEVICE ({} bytes): CMD_ID "{:02X}" "{}"'.format(len(payload), command_id, payload_hex))

            successful = self._parse_packet(command_id, payload)

            if successful:
                self.parsed_packet_count += 1
            else:
                self.unhandled_packet_count += 1
            
            self.buffer = self.buffer[end+2:]

            if not self.buffer:
                all_full_packets_processed = True

        # Publish driver statistics for diagnosis
        driver_status = seatrac_driver.msg.DriverStatus()
        driver_status.device_connected = (self.connection is not None) and self.connection.is_open
        driver_status.sent_command_count = self.sent_command_count
        driver_status.parsed_packet_count = self.parsed_packet_count
        driver_status.unhandled_packet_count = self.unhandled_packet_count
        driver_status.corrupt_packet_count = self.corrupt_packet_count
        driver_status.ping_sent = self.ping_sent
        driver_status.ping_received = self.ping_received
        driver_status.ping_error = self.ping_error
        driver_status.data_sent = self.data_sent
        driver_status.data_received = self.data_received
        self._driver_status_publisher.publish(driver_status)

    def _parse_packet(self, command_id, payload):
        # Parse the incoming device packet based on command ID (CID)
        if command_id == seatrac_constants.CID_SYS_ALIVE['cid']:
            self._on_system_alive_response(payload)
        elif command_id == seatrac_constants.CID_SYS_INFO['cid']:
            self._on_system_info_response(payload)
        elif command_id == seatrac_constants.CID_SYS_REBOOT['cid']:
            self._on_system_reboot_response(payload)
        elif command_id == seatrac_constants.CID_STATUS['cid']:
            self._on_status_response(payload)
        elif command_id == seatrac_constants.CID_STATUS_CFG_GET['cid']:
            self._on_status_config_get_response(payload)
        elif command_id == seatrac_constants.CID_STATUS_CFG_SET['cid']:
            self._on_status_config_set_response(payload)
        elif command_id == seatrac_constants.CID_SETTINGS_GET['cid']:
            self._on_settings_get_response(payload)
        elif command_id == seatrac_constants.CID_SETTING_SET['cid']:
            self._on_settings_set_response(payload)
        elif command_id == seatrac_constants.CID_SETTINGS_LOAD['cid']:
            self._on_settings_load_response(payload)
        elif command_id == seatrac_constants.CID_SETTINGS_SAVE['cid']:
            self._on_settings_save_response(payload)
        elif command_id == seatrac_constants.CID_SETTINGS_RESET['cid']:
            self._on_settings_reset_response(payload)
        elif command_id == seatrac_constants.CID_CAL_ACTION['cid']:
            self._on_cal_action_response(payload)
        elif command_id == seatrac_constants.CID_AHRS_CAL_GET['cid']:
            self._on_ahrs_cal_get_response(payload)
        elif command_id == seatrac_constants.CID_AHRS_CAL_SET['cid']:
            self._on_ahrs_cal_set_response(payload)
        elif command_id == seatrac_constants.CID_XCVR_ANALYSE['cid']:
            self._on_transceiver_analyze_response(payload)
        elif command_id == seatrac_constants.CID_XCVR_FIX['cid']:
            self._on_transceiver_fix_status(payload)
        elif command_id == seatrac_constants.CID_XCVR_STATUS['cid']:
            self._on_transceiver_status_response(payload)
        elif command_id == seatrac_constants.CID_PING_SEND['cid']:
            self._on_ping_send_response(payload)
        elif command_id == seatrac_constants.CID_PING_REQ['cid']:
            self._on_ping_req_status(payload)
        elif command_id == seatrac_constants.CID_PING_RESP['cid']:
            self._on_ping_resp_status(payload)
        elif command_id == seatrac_constants.CID_PING_ERROR['cid']:
            self._on_ping_error_status(payload)
        elif command_id == seatrac_constants.CID_NAV_QUERY_SEND['cid']:
            self._on_nav_query_send_response(payload)
        elif command_id == seatrac_constants.CID_NAV_QUERY_REQ['cid']:
            self._on_nav_query_req_status(payload)
        elif command_id == seatrac_constants.CID_NAV_QUERY_RESP['cid']:
            self._on_nav_query_resp_status(payload)
        elif command_id == seatrac_constants.CID_NAV_ERROR['cid']:
            self._on_nav_error_status(payload)
        elif command_id == seatrac_constants.CID_NAV_STATUS_SEND['cid']:
            self._on_nav_status_send_response(payload)
        elif command_id == seatrac_constants.CID_NAV_STATUS_RECEIVE['cid']:
            self._on_nav_status_receive_status(payload)
        elif command_id == seatrac_constants.CID_DAT_SEND['cid']:
            self._on_dat_send_response(payload)
        elif command_id == seatrac_constants.CID_DAT_RECEIVE['cid']:
            self._on_dat_receive_status(payload)
        elif command_id == seatrac_constants.CID_DAT_ERROR['cid']:
            self._on_dat_error_status(payload)
        else:
            self.get_logger().warning('Parsing of packet is not supported')

            return False

        return True

    def _on_system_alive_command(self, msg):
        cid = seatrac_constants.CID_SYS_ALIVE['cid']

        structure = seatrac_constants.CID_SYS_ALIVE['command']
        data = {}
        payload = structure.build(data)

        self._packetize(cid, payload)

    def _on_system_alive_response(self, payload):
        # Parse CID_SYS_ALIVE response
        parsed_payload = seatrac_constants.CID_SYS_ALIVE['response'].parse(payload)

        system_alive_response_msg = seatrac_driver.msg.SystemAliveResponse()
        system_alive_response_msg.seconds = parsed_payload.SECONDS
        self._system_alive_response_publisher.publish(system_alive_response_msg)

    def _on_system_info_command(self, msg):
        cid = seatrac_constants.CID_SYS_INFO['cid']

        structure = seatrac_constants.CID_SYS_INFO['command']
        data = {}
        payload = structure.build(data)

        self._packetize(cid, payload)

    def _on_system_info_response(self, payload):
        # Parse CID_SYS_INFO response
        parsed_payload = seatrac_constants.CID_SYS_INFO['response'].parse(payload)

        system_info_response_msg = seatrac_driver.msg.SystemInfoResponse()
        system_info_response_msg.seconds = parsed_payload.SECONDS
        system_info_response_msg.section = parsed_payload.SECTION
        system_info_response_msg.hardware.part_number = parsed_payload.HARDWARE.PART_NUMBER
        system_info_response_msg.hardware.part_revision = parsed_payload.HARDWARE.PART_REV
        system_info_response_msg.hardware.serial_number = parsed_payload.HARDWARE.SERIAL_NUMBER
        system_info_response_msg.hardware.system_flags = parsed_payload.HARDWARE.FLAGS_SYS
        system_info_response_msg.hardware.user_flags = parsed_payload.HARDWARE.FLAGS_USR
        system_info_response_msg.boot_firmware.valid = parsed_payload.BOOT_FIRMWARE.VALID
        system_info_response_msg.boot_firmware.part_number = parsed_payload.BOOT_FIRMWARE.PART_NUMBER
        system_info_response_msg.boot_firmware.version_major = parsed_payload.BOOT_FIRMWARE.VERSION_MAJ
        system_info_response_msg.boot_firmware.version_minor = parsed_payload.BOOT_FIRMWARE.VERSION_MIN
        system_info_response_msg.boot_firmware.version_build = parsed_payload.BOOT_FIRMWARE.VERSION_BUILD
        system_info_response_msg.boot_firmware.checksum = parsed_payload.BOOT_FIRMWARE.CHECKSUM
        system_info_response_msg.main_firmware.valid = parsed_payload.MAIN_FIRMWARE.VALID
        system_info_response_msg.main_firmware.part_number = parsed_payload.MAIN_FIRMWARE.PART_NUMBER
        system_info_response_msg.main_firmware.version_major = parsed_payload.MAIN_FIRMWARE.VERSION_MAJ
        system_info_response_msg.main_firmware.version_minor = parsed_payload.MAIN_FIRMWARE.VERSION_MIN
        system_info_response_msg.main_firmware.version_build = parsed_payload.MAIN_FIRMWARE.VERSION_BUILD
        system_info_response_msg.main_firmware.checksum = parsed_payload.MAIN_FIRMWARE.CHECKSUM
        system_info_response_msg.board_revision = parsed_payload.BOARD_REV
        self._system_info_response_publisher.publish(system_info_response_msg)

    def _on_system_reboot_command(self, msg):
        cid = seatrac_constants.CID_SYS_REBOOT['cid']

        structure = seatrac_constants.CID_SYS_REBOOT['command']
        data = {}
        payload = structure.build(data)

        self._packetize(cid, payload)

    def _on_system_reboot_response(self, payload):
        # Parse CID_SYS_REBOOT
        parsed_payload = seatrac_constants.CID_SYS_REBOOT['response'].parse(payload)

        system_reboot_response_msg = seatrac_driver.msg.SystemRebootResponse()
        system_reboot_response_msg.status = parsed_payload.STATUS
        self._system_reboot_response_publisher.publish(system_reboot_response_msg)

    def _on_status_command(self, msg):
        cid = seatrac_constants.CID_STATUS['cid']

        structure = seatrac_constants.CID_STATUS['command']
        data = {
            'STATUS_OUTPUT': {
                'ENVIRONMENT': msg.output_environment,
                'ATTITUDE': msg.output_attitude,
                'MAG_CAL': msg.output_mag_cal,
                'ACC_CAL': msg.output_acc_cal,
                'AHRS_RAW_DATA': msg.output_ahrs_raw_data,
                'AHRS_COMP_DATA': msg.output_ahrs_comp_data,
            },
        }
        payload = structure.build(data)

        self._packetize(cid, payload)

    def _on_status_response(self, payload):
        # Parse CID_STATUS response
        parsed_payload = seatrac_constants.CID_STATUS['response'].parse(payload)

        status_response_msg = seatrac_driver.msg.StatusResponse()
        status_response_msg.environment_outputted = parsed_payload.STATUS_OUTPUT.ENVIRONMENT
        status_response_msg.attitude_outputted = parsed_payload.STATUS_OUTPUT.ATTITUDE
        status_response_msg.mag_cal_outputted = parsed_payload.STATUS_OUTPUT.MAG_CAL
        status_response_msg.acc_cal_outputted = parsed_payload.STATUS_OUTPUT.ACC_CAL
        status_response_msg.ahrs_raw_data_outputted = parsed_payload.STATUS_OUTPUT.AHRS_RAW_DATA
        status_response_msg.ahrs_comp_data_outputted = parsed_payload.STATUS_OUTPUT.AHRS_COMP_DATA

        if status_response_msg.environment_outputted:
            status_response_msg.env_supply = parsed_payload.ENVIRONMENT.ENV_SUPPLY / 1000.0
            status_response_msg.env_temperature = parsed_payload.ENVIRONMENT.ENV_TEMP / 10.0
            status_response_msg.env_pressure = parsed_payload.ENVIRONMENT.ENV_PRESSURE / 1000.0
            status_response_msg.env_depth = parsed_payload.ENVIRONMENT.ENV_DEPTH / 10.0
            status_response_msg.env_vos = parsed_payload.ENVIRONMENT.ENV_VOS / 10.0

        if status_response_msg.attitude_outputted:
            status_response_msg.attitude_yaw = parsed_payload.ATTITUDE.ATT_YAW / 10.0
            status_response_msg.attitude_pitch = parsed_payload.ATTITUDE.ATT_PITCH / 10.0
            status_response_msg.attitude_roll = parsed_payload.ATTITUDE.ATT_ROLL / 10.0

        if status_response_msg.mag_cal_outputted:
            status_response_msg.mag_cal_buf = parsed_payload.MAG_CAL.MAG_CAL_BUF
            status_response_msg.mag_cal_valid = parsed_payload.MAG_CAL.MAG_CAL_VALID
            status_response_msg.mag_cal_age = parsed_payload.MAG_CAL.MAG_CAL_AGE
            status_response_msg.mag_cal_fit = parsed_payload.MAG_CAL.MAG_CAL_FIT

        if status_response_msg.acc_cal_outputted:
            status_response_msg.acceleration_limit_min[0] = parsed_payload.ACC_CAL.ACC_LIM_MIN_X
            status_response_msg.acceleration_limit_min[1] = parsed_payload.ACC_CAL.ACC_LIM_MIN_Y
            status_response_msg.acceleration_limit_min[2] = parsed_payload.ACC_CAL.ACC_LIM_MIN_Z
            status_response_msg.acceleration_limit_max[0] = parsed_payload.ACC_CAL.ACC_LIM_MAX_X
            status_response_msg.acceleration_limit_max[1] = parsed_payload.ACC_CAL.ACC_LIM_MAX_Y
            status_response_msg.acceleration_limit_max[2] = parsed_payload.ACC_CAL.ACC_LIM_MAX_Z

        if status_response_msg.ahrs_raw_data_outputted:
            status_response_msg.ahrs_raw_acc[0] = parsed_payload.AHRS_RAW_DATA.AHRS_RAW_ACC_X
            status_response_msg.ahrs_raw_acc[1] = parsed_payload.AHRS_RAW_DATA.AHRS_RAW_ACC_Y
            status_response_msg.ahrs_raw_acc[2] = parsed_payload.AHRS_RAW_DATA.AHRS_RAW_ACC_Z
            status_response_msg.ahrs_raw_mag[0] = parsed_payload.AHRS_RAW_DATA.AHRS_RAW_MAG_X
            status_response_msg.ahrs_raw_mag[1] = parsed_payload.AHRS_RAW_DATA.AHRS_RAW_MAG_Y
            status_response_msg.ahrs_raw_mag[2] = parsed_payload.AHRS_RAW_DATA.AHRS_RAW_MAG_Z
            status_response_msg.ahrs_raw_gyro[0] = parsed_payload.AHRS_RAW_DATA.AHRS_RAW_GYRO_X
            status_response_msg.ahrs_raw_gyro[1] = parsed_payload.AHRS_RAW_DATA.AHRS_RAW_GYRO_Y
            status_response_msg.ahrs_raw_gyro[2] = parsed_payload.AHRS_RAW_DATA.AHRS_RAW_GYRO_Z

        if status_response_msg.ahrs_comp_data_outputted:
            status_response_msg.ahrs_comp_acc.x = parsed_payload.AHRS_COMP_DATA.AHRS_COMP_ACC_X
            status_response_msg.ahrs_comp_acc.y = parsed_payload.AHRS_COMP_DATA.AHRS_COMP_ACC_Y
            status_response_msg.ahrs_comp_acc.z = parsed_payload.AHRS_COMP_DATA.AHRS_COMP_ACC_Z
            status_response_msg.ahrs_comp_mag.x = parsed_payload.AHRS_COMP_DATA.AHRS_COMP_MAG_X
            status_response_msg.ahrs_comp_mag.y = parsed_payload.AHRS_COMP_DATA.AHRS_COMP_MAG_Y
            status_response_msg.ahrs_comp_mag.z = parsed_payload.AHRS_COMP_DATA.AHRS_COMP_MAG_Z
            status_response_msg.ahrs_comp_gyro.x = parsed_payload.AHRS_COMP_DATA.AHRS_COMP_GYRO_X
            status_response_msg.ahrs_comp_gyro.y = parsed_payload.AHRS_COMP_DATA.AHRS_COMP_GYRO_Y
            status_response_msg.ahrs_comp_gyro.z = parsed_payload.AHRS_COMP_DATA.AHRS_COMP_GYRO_Z

        self._status_response_publisher.publish(status_response_msg)
        
    def _on_status_config_get_command(self, msg):
        cid = seatrac_constants.CID_STATUS_CFG_GET['cid']

        structure = seatrac_constants.CID_STATUS_CFG_GET['command']
        data = {}
        payload = structure.build(data)

        self._packetize(cid, payload)

    def _on_status_config_get_response(self, payload):
        # Parse CID_STATUS_CFG_GET response
        parsed_payload = seatrac_constants.CID_STATUS_CFG_GET['response'].parse(payload)

        status_config_get_response_msg = seatrac_driver.msg.StatusConfigGetResponse()
        status_config_get_response_msg.environment_outputted = parsed_payload.STATUS_OUTPUT.ENVIRONMENT
        status_config_get_response_msg.attitude_outputted = parsed_payload.STATUS_OUTPUT.ATTITUDE
        status_config_get_response_msg.mag_cal_outputted = parsed_payload.STATUS_OUTPUT.MAG_CAL
        status_config_get_response_msg.acc_cal_outputted = parsed_payload.STATUS_OUTPUT.ACC_CAL
        status_config_get_response_msg.ahrs_raw_data_outputted = parsed_payload.STATUS_OUTPUT.AHRS_RAW_DATA
        status_config_get_response_msg.ahrs_comp_data_outputted = parsed_payload.STATUS_OUTPUT.AHRS_COMP_DATA
        status_config_get_response_msg.status_mode = parsed_payload.STATUS_MODE
        self._status_config_get_response_publisher.publish(status_config_get_response_msg)

    def _on_status_config_set_command(self, msg):
        cid = seatrac_constants.CID_STATUS_CFG_SET['cid']

        structure = seatrac_constants.CID_STATUS_CFG_SET['command']
        data = {
            'STATUS_OUTPUT': {
                'ENVIRONMENT': msg.output_environment,
                'ATTITUDE': msg.output_attitude,
                'MAG_CAL': msg.output_mag_cal,
                'ACC_CAL': msg.output_acc_cal,
                'AHRS_RAW_DATA': msg.output_ahrs_raw_data,
                'AHRS_COMP_DATA': msg.output_ahrs_comp_data,
            },
            'STATUS_MODE': msg.status_mode,
        }
        payload = structure.build(data)

        self._packetize(cid, payload)

    def _on_status_config_set_response(self, payload):
        # Parse CID_STATUS_CFG_SET response
        parsed_payload = seatrac_constants.CID_STATUS_CFG_SET['response'].parse(payload)

        status_config_set_response_msg = seatrac_driver.msg.StatusConfigSetResponse()
        status_config_set_response_msg.status = parsed_payload.STATUS
        self._status_config_set_response_publisher.publish(status_config_set_response_msg)

    def _on_settings_get_command(self, msg):
        cid = seatrac_constants.CID_SETTINGS_GET['cid']

        structure = seatrac_constants.CID_SETTINGS_GET['command']
        data = {}
        payload = structure.build(data)

        self._packetize(cid, payload)

    def _on_settings_get_response(self, payload):

        parsed_payload = seatrac_constants.CID_SETTINGS_GET['response'].parse(payload)

        settings_get_response_msg = seatrac_driver.msg.SettingsGetResponse()
        settings_get_response_msg.status_mode = int(parsed_payload.SETTINGS.STATUS_FLAGS.STATUS_MODE)
        settings_get_response_msg.status_mode_str = parsed_payload.SETTINGS.STATUS_FLAGS.STATUS_MODE
        settings_get_response_msg.status_output_environment = parsed_payload.SETTINGS.STATUS_OUTPUT.ENVIRONMENT
        settings_get_response_msg.status_output_attitude = parsed_payload.SETTINGS.STATUS_OUTPUT.ATTITUDE
        settings_get_response_msg.status_output_mag_cal = parsed_payload.SETTINGS.STATUS_OUTPUT.MAG_CAL
        settings_get_response_msg.status_output_acc_cal = parsed_payload.SETTINGS.STATUS_OUTPUT.ACC_CAL
        settings_get_response_msg.status_output_ahrs_raw_data = parsed_payload.SETTINGS.STATUS_OUTPUT.AHRS_RAW_DATA
        settings_get_response_msg.status_output_ahrs_comp_data = parsed_payload.SETTINGS.STATUS_OUTPUT.AHRS_COMP_DATA
        settings_get_response_msg.uart_main_baud = int(parsed_payload.SETTINGS.UART_MAIN_BAUD)
        settings_get_response_msg.uart_main_baud_str = parsed_payload.SETTINGS.UART_MAIN_BAUD
        # All other settings reserved for future use are intentionally ignored
        settings_get_response_msg.enable_automatic_pressure_offset_cal = parsed_payload.SETTINGS.ENV_FLAGS.AUTO_PRESSURE_OFS
        settings_get_response_msg.enable_automatic_vos = parsed_payload.SETTINGS.ENV_FLAGS.AUTO_VOS
        settings_get_response_msg.manual_pressure_offset_bar = parsed_payload.SETTINGS.ENV_PRESSURE_OFS / 1000.0
        settings_get_response_msg.salinity_ppt = parsed_payload.SETTINGS.ENV_SALINITY / 10.0
        settings_get_response_msg.manual_vos = parsed_payload.SETTINGS.ENV_VOS / 10.0
        settings_get_response_msg.enable_automatic_magnetometer_cal = parsed_payload.SETTINGS.AHRS_FLAGS.AUTO_CAL_MAG
        settings_get_response_msg.ahrs_cal_acc_min_x = parsed_payload.SETTINGS.AHRS_CAL.ACC_MIN_X
        settings_get_response_msg.ahrs_cal_acc_min_y = parsed_payload.SETTINGS.AHRS_CAL.ACC_MIN_Y
        settings_get_response_msg.ahrs_cal_acc_min_z = parsed_payload.SETTINGS.AHRS_CAL.ACC_MIN_Z
        settings_get_response_msg.ahrs_cal_acc_max_x = parsed_payload.SETTINGS.AHRS_CAL.ACC_MAX_X
        settings_get_response_msg.ahrs_cal_acc_max_y = parsed_payload.SETTINGS.AHRS_CAL.ACC_MAX_Y
        settings_get_response_msg.ahrs_cal_acc_max_z = parsed_payload.SETTINGS.AHRS_CAL.ACC_MAX_Z
        settings_get_response_msg.ahrs_cal_mag_valid = parsed_payload.SETTINGS.AHRS_CAL.MAG_VALID
        settings_get_response_msg.ahrs_cal_mag_hard_x = parsed_payload.SETTINGS.AHRS_CAL.MAG_HARD_X
        settings_get_response_msg.ahrs_cal_mag_hard_y = parsed_payload.SETTINGS.AHRS_CAL.MAG_HARD_Y
        settings_get_response_msg.ahrs_cal_mag_hard_z = parsed_payload.SETTINGS.AHRS_CAL.MAG_HARD_Z
        settings_get_response_msg.ahrs_cal_mag_soft_x = parsed_payload.SETTINGS.AHRS_CAL.MAG_SOFT_X
        settings_get_response_msg.ahrs_cal_mag_soft_y = parsed_payload.SETTINGS.AHRS_CAL.MAG_SOFT_Y
        settings_get_response_msg.ahrs_cal_mag_soft_z = parsed_payload.SETTINGS.AHRS_CAL.MAG_SOFT_Z
        settings_get_response_msg.ahrs_cal_mag_field = parsed_payload.SETTINGS.AHRS_CAL.MAG_FIELD
        settings_get_response_msg.ahrs_cal_mag_error = parsed_payload.SETTINGS.AHRS_CAL.MAG_ERROR
        settings_get_response_msg.ahrs_cal_gyro_offset_x = parsed_payload.SETTINGS.AHRS_CAL.GYRO_OFFSET_X
        settings_get_response_msg.ahrs_cal_gyro_offset_y = parsed_payload.SETTINGS.AHRS_CAL.GYRO_OFFSET_Y
        settings_get_response_msg.ahrs_cal_gyro_offset_z = parsed_payload.SETTINGS.AHRS_CAL.GYRO_OFFSET_Z
        settings_get_response_msg.ahrs_yaw_offset_deg = parsed_payload.SETTINGS.AHRS_YAW_OFS / 10.0
        settings_get_response_msg.ahrs_pitch_offset_deg = parsed_payload.SETTINGS.AHRS_PITCH_OFS / 10.0
        settings_get_response_msg.ahrs_roll_offset_deg = parsed_payload.SETTINGS.AHRS_ROLL_OFS / 10.0
        settings_get_response_msg.enable_transceiver_diagnostic_messages = parsed_payload.SETTINGS.XCVR_FLAGS.XCVR_DIAG_MSGS
        settings_get_response_msg.enable_transceiver_fix_messages = parsed_payload.SETTINGS.XCVR_FLAGS.XCVR_FIX_MSGS
        settings_get_response_msg.enable_transceiver_usbl_messages = parsed_payload.SETTINGS.XCVR_FLAGS.XCVR_USBL_MSGS
        settings_get_response_msg.enable_transceiver_position_filter = parsed_payload.SETTINGS.XCVR_FLAGS.XCVR_POSFLT_ENABLE
        settings_get_response_msg.enable_transceiver_automatic_ahrs = parsed_payload.SETTINGS.XCVR_FLAGS.USBL_USE_AHRS
        settings_get_response_msg.transceiver_beacon_id = int(parsed_payload.SETTINGS.XCVR_BEACON_ID)
        settings_get_response_msg.transceiver_timeout_range_m = parsed_payload.SETTINGS.XCVR_RANGE_TMO
        settings_get_response_msg.transceiver_response_time_ms = parsed_payload.SETTINGS.XCVR_RESP_TIME
        settings_get_response_msg.transceiver_manual_ahrs_yaw_deg = parsed_payload.SETTINGS.XCVR_YAW / 10.0
        settings_get_response_msg.transceiver_manual_ahrs_pitch_deg = parsed_payload.SETTINGS.XCVR_PITCH / 10.0
        settings_get_response_msg.transceiver_manual_ahrs_roll_deg = parsed_payload.SETTINGS.XCVR_ROLL / 10.0
        settings_get_response_msg.transceiver_position_filter_max_velocity = parsed_payload.SETTINGS.XCVR_POSFLT_VEL
        settings_get_response_msg.transceiver_position_filter_max_angle = parsed_payload.SETTINGS.XCVR_POSFLT_ANG
        settings_get_response_msg.transceiver_position_filter_reset_timeout = parsed_payload.SETTINGS.XCVR_POSFLT_TMO
        self._settings_get_response_publisher.publish(settings_get_response_msg)

    def _on_settings_set_command(self, msg):
        cid = seatrac_constants.CID_SETTING_SET['cid']

        structure = seatrac_constants.CID_SETTING_SET['command']
        data = {
            'SETTINGS': {
                'STATUS_FLAGS': {
                    'STATUS_MODE': msg.status_mode,
                },
                'STATUS_OUTPUT': {
                    'ENVIRONMENT': msg.output_environment,
                    'ATTITUDE': msg.output_attitude,
                    'MAG_CAL': msg.output_mag_cal,
                    'ACC_CAL': msg.output_acc_cal,
                    'AHRS_RAW_DATA': msg.output_ahrs_raw_data,
                    'AHRS_COMP_DATA': msg.output_ahrs_comp_data,
                },
                'UART_MAIN_BAUD': msg.uart_main_baud,
                # All 'Reserved' future values with defaults set are intentially skipped
                'ENV_FLAGS': {
                    'AUTO_PRESSURE_OFS': msg.enable_automatic_pressure_offset_cal,
                    'AUTO_VOS': msg.enable_automatic_vos,
                },
                'ENV_PRESSURE_OFS': int(msg.manual_pressure_offset_bar * 1000.0),
                'ENV_SALINITY': int(msg.salinity_ppt * 10.0),
                'ENV_VOS': int(msg.manual_vos * 10.0),
                'AHRS_FLAGS': {
                    'AUTO_CAL_MAG': msg.enable_automatic_magnetometer_cal,
                },
                'AHRS_CAL': {
                    'ACC_MIN_X': msg.ahrs_cal_acc_min_x,
                    'ACC_MIN_Y': msg.ahrs_cal_acc_min_y,
                    'ACC_MIN_Z': msg.ahrs_cal_acc_min_z,
                    'ACC_MAX_X': msg.ahrs_cal_acc_max_x,
                    'ACC_MAX_Y': msg.ahrs_cal_acc_max_y,
                    'ACC_MAX_Z': msg.ahrs_cal_acc_max_z,
                    'MAG_VALID': msg.ahrs_cal_mag_valid,
                    'MAG_HARD_X': msg.ahrs_cal_mag_hard_x,
                    'MAG_HARD_Y': msg.ahrs_cal_mag_hard_y,
                    'MAG_HARD_Z': msg.ahrs_cal_mag_hard_z,
                    'MAG_SOFT_X': msg.ahrs_cal_mag_soft_x,
                    'MAG_SOFT_Y': msg.ahrs_cal_mag_soft_y,
                    'MAG_SOFT_Z': msg.ahrs_cal_mag_soft_z,
                    'MAG_FIELD': msg.ahrs_cal_mag_field,
                    'MAG_ERROR': msg.ahrs_cal_mag_error,
                    'GYRO_OFFSET_X': msg.ahrs_cal_gyro_offset_x,
                    'GYRO_OFFSET_Y': msg.ahrs_cal_gyro_offset_y,
                    'GYRO_OFFSET_Z': msg.ahrs_cal_gyro_offset_z,
                },
                'AHRS_YAW_OFS': int(msg.ahrs_yaw_offset_deg * 10.0),
                'AHRS_PITCH_OFS': int(msg.ahrs_pitch_offset_deg * 10.0),
                'AHRS_ROLL_OFS': int(msg.ahrs_roll_offset_deg * 10.0),
                'XCVR_FLAGS': {
                    'XCVR_DIAG_MSGS': msg.enable_transceiver_diagnostic_messages,
                    'XCVR_FIX_MSGS': msg.enable_transceiver_fix_messages,
                    'XCVR_USBL_MSGS': msg.enable_transceiver_usbl_messages,
                    'XCVR_POSFLT_ENABLE': msg.enable_transceiver_position_filter,
                    'USBL_USE_AHRS': msg.enable_transceiver_automatic_ahrs,
                },
                'XCVR_BEACON_ID': msg.transceiver_beacon_id,
                'XCVR_RANGE_TMO': msg.transceiver_timeout_range_m,
                'XCVR_RESP_TIME': msg.transceiver_response_time_ms,
                'XCVR_YAW': int(msg.transceiver_manual_ahrs_yaw_deg * 10.0),
                'XCVR_PITCH': int(msg.transceiver_manual_ahrs_pitch_deg * 10.0),
                'XCVR_ROLL': int(msg.transceiver_manual_ahrs_roll_deg * 10.0),
                'XCVR_POSFLT_VEL': msg.transceiver_position_filter_max_velocity,
                'XCVR_POSFLT_ANG': msg.transceiver_position_filter_max_angle,
                'XCVR_POSFLT_TMO': msg.transceiver_position_filter_reset_timeout,
            },
        }
        payload = structure.build(data)

        self._packetize(cid, payload)

    def _on_settings_set_response(self, payload):
        parsed_payload = seatrac_constants.CID_SETTING_SET['response'].parse(payload)

        settings_set_response_msg = seatrac_driver.msg.SettingsSetResponse()
        settings_set_response_msg.status = parsed_payload.STATUS
        self._settings_set_response_publisher.publish(settings_set_response_msg)

    def _on_settings_load_command(self, msg):
        cid = seatrac_constants.CID_SETTINGS_LOAD['cid']

        structure = seatrac_constants.CID_SETTINGS_LOAD['command']
        data = {}
        payload = structure.build(data)

        self._packetize(cid, payload)

    def _on_settings_load_response(self, payload):
        parsed_payload = seatrac_constants.CID_SETTINGS_LOAD['response'].parse(payload)

        settings_load_response_msg = seatrac_driver.msg.SettingsLoadResponse()
        settings_load_response_msg.status = parsed_payload.STATUS
        self._settings_load_response_publisher.publish(settings_load_response_msg)

    def _on_settings_save_command(self, msg):
        cid = seatrac_constants.CID_SETTINGS_SAVE['cid']

        structure = seatrac_constants.CID_SETTINGS_SAVE['command']
        data = {}
        payload = structure.build(data)

        self._packetize(cid, payload)

    def _on_settings_save_response(self, payload):
        parsed_payload = seatrac_constants.CID_SETTINGS_SAVE['response'].parse(payload)

        settings_save_response_msg = seatrac_driver.msg.SettingsSaveResponse()
        settings_save_response_msg.status = parsed_payload.STATUS
        self._settings_save_response_publisher.publish(settings_save_response_msg)

    def _on_settings_reset_command(self, msg):
        cid = seatrac_constants.CID_SETTINGS_RESET['cid']

        structure = seatrac_constants.CID_SETTINGS_RESET['command']
        data = {}
        payload = structure.build(data)

        self._packetize(cid, payload)

    def _on_settings_reset_response(self, payload):
        parsed_payload = seatrac_constants.CID_SETTINGS_RESET['response'].parse(payload)

        settings_reset_response_msg = seatrac_driver.msg.SettingsResetResponse()
        settings_reset_response_msg.status = parsed_payload.STATUS
        self._settings_reset_response_publisher.publish(settings_reset_response_msg)

    def _on_cal_action_command(self, msg):
        cid = seatrac_constants.CID_CAL_ACTION['cid']

        structure = seatrac_constants.CID_CAL_ACTION['command']
        data = {
            'ACTION': msg.action,
        }
        payload = structure.build(data)

        self._packetize(cid, payload)

    def _on_cal_action_response(self, payload):
        parsed_payload = seatrac_constants.CID_CAL_ACTION['response'].parse(payload)

        calibration_action_response_msg = seatrac_driver.msg.CalibrationActionResponse()
        calibration_action_response_msg.status = parsed_payload.STATUS
        self._calibration_action_response_publisher.publish(calibration_action_response_msg)

    def _on_ahrs_cal_get_command(self, msg):
        cid = seatrac_constants.CID_AHRS_CAL_GET['cid']

        structure = seatrac_constants.CID_AHRS_CAL_GET['command']
        data = {}
        payload = structure.build(data)

        self._packetize(cid, payload)

    def _on_ahrs_cal_get_response(self, payload):
        parsed_payload = seatrac_constants.CID_AHRS_CAL_GET['response'].parse(payload)

        calibration_get_response_msg = seatrac_driver.msg.CalibrationGetResponse()
        calibration_get_response_msg.acc_min_x = parsed_payload.AHRS_CAL.ACC_MIN_X
        calibration_get_response_msg.acc_min_y = parsed_payload.AHRS_CAL.ACC_MIN_Y
        calibration_get_response_msg.acc_min_z = parsed_payload.AHRS_CAL.ACC_MIN_Z
        calibration_get_response_msg.acc_max_x = parsed_payload.AHRS_CAL.ACC_MAX_X
        calibration_get_response_msg.acc_max_y = parsed_payload.AHRS_CAL.ACC_MAX_Y
        calibration_get_response_msg.acc_max_z = parsed_payload.AHRS_CAL.ACC_MAX_Z
        calibration_get_response_msg.mag_valid = parsed_payload.AHRS_CAL.MAG_VALID
        calibration_get_response_msg.mag_hard_x = parsed_payload.AHRS_CAL.MAG_HARD_X
        calibration_get_response_msg.mag_hard_y = parsed_payload.AHRS_CAL.MAG_HARD_Y
        calibration_get_response_msg.mag_hard_z = parsed_payload.AHRS_CAL.MAG_HARD_Z
        calibration_get_response_msg.mag_soft_x = parsed_payload.AHRS_CAL.MAG_SOFT_X
        calibration_get_response_msg.mag_soft_y = parsed_payload.AHRS_CAL.MAG_SOFT_Y
        calibration_get_response_msg.mag_soft_z = parsed_payload.AHRS_CAL.MAG_SOFT_Z
        calibration_get_response_msg.mag_field = parsed_payload.AHRS_CAL.MAG_FIELD
        calibration_get_response_msg.mag_error = parsed_payload.AHRS_CAL.MAG_ERROR
        calibration_get_response_msg.gyro_offset_x = parsed_payload.AHRS_CAL.GYRO_OFFSET_X
        calibration_get_response_msg.gyro_offset_y = parsed_payload.AHRS_CAL.GYRO_OFFSET_Y
        calibration_get_response_msg.gyro_offset_z = parsed_payload.AHRS_CAL.GYRO_OFFSET_Z
        self._calibration_get_response_publisher.publish(calibration_get_response_msg)

    def _on_ahrs_cal_set_command(self, msg):
        cid = seatrac_constants.CID_AHRS_CAL_SET['cid']

        structure = seatrac_constants.CID_AHRS_CAL_SET['command']
        data = {
            'AHRS_CAL': {
                'ACC_MIN_X': msg.acc_min_x,
                'ACC_MIN_Y': msg.acc_min_y,
                'ACC_MIN_Z': msg.acc_min_z,
                'ACC_MAX_X': msg.acc_max_x,
                'ACC_MAX_Y': msg.acc_max_y,
                'ACC_MAX_Z': msg.acc_max_z,
                'MAG_HARD_X': msg.mag_hard_x,
                'MAG_HARD_Y': msg.mag_hard_y,
                'MAG_HARD_Z': msg.mag_hard_z,
                'MAG_SOFT_X': msg.mag_soft_x,
                'MAG_SOFT_Y': msg.mag_soft_y,
                'MAG_SOFT_Z': msg.mag_soft_z,
                'MAG_FIELD': msg.mag_field,
                'GYRO_OFFSET_X': msg.gyro_offset_x,
                'GYRO_OFFSET_Y': msg.gyro_offset_y,
                'GYRO_OFFSET_Z': msg.gyro_offset_z,
            }
        }
        payload = structure.build(data)

        self._packetize(cid, payload)

    def _on_ahrs_cal_set_response(self, payload):
        parsed_payload = seatrac_constants.CID_AHRS_CAL_SET['response'].parse(payload)

        calibration_set_response_msg = seatrac_driver.msg.CalibrationActionResponse()
        calibration_set_response_msg.status = parsed_payload.STATUS
        self._calibration_set_response_publisher.publish(calibration_set_response_msg)

    def _on_transceiver_analyze_command(self, msg):
        cid = seatrac_constants.CID_XCVR_ANALYSE['cid']

        structure = seatrac_constants.CID_XCVR_ANALYSE['command']
        data = {}
        payload = structure.build(data)

        self._packetize(cid, payload)

    def _on_transceiver_analyze_response(self, payload):
        parsed_payload = seatrac_constants.CID_XCVR_ANALYSE['response'].parse(payload)

        transceiver_analyze_response_msg = seatrac_driver.msg.TransceiverAnalyzeResponse()
        transceiver_analyze_response_msg.status = parsed_payload.STATUS
        transceiver_analyze_response_msg.adc_mean = parsed_payload.ADC_MEAN
        transceiver_analyze_response_msg.adc_peak_to_peak = parsed_payload.ADC_PKPK
        transceiver_analyze_response_msg.adc_rms = parsed_payload.ADC_RMS
        transceiver_analyze_response_msg.rx_level_peak_to_peak = parsed_payload.RX_LEVEL_PKPK / 10.0
        transceiver_analyze_response_msg.rx_level_rms = parsed_payload.RX_LEVEL_RMS / 10.0
        self._transceiver_analyze_response_publisher.publish(transceiver_analyze_response_msg)

    def _on_transceiver_fix_status(self, payload):
        parsed_payload = seatrac_constants.CID_XCVR_FIX['status'].parse(payload)

        transceiver_fix_status_msg = seatrac_driver.msg.TransceiverFixStatus()
        self._populate_acofix_structure(transceiver_fix_status_msg.fix, parsed_payload.ACO_FIX)
        self._tranceiver_fix_status_publisher.publish(transceiver_fix_status_msg)

    def _on_transceiver_status_command(self, msg):
        cid = seatrac_constants.CID_XCVR_STATUS['cid']

        structure = seatrac_constants.CID_XCVR_STATUS['command']
        data = {}
        payload = structure.build(data)

        self._packetize(cid, payload)

    def _on_transceiver_status_response(self, payload):
        parsed_payload = seatrac_constants.CID_XCVR_STATUS['response'].parse(payload)

        transceiver_status_response_msg = seatrac_driver.msg.TransceiverStatusResponse()
        transceiver_status_response_msg.status = parsed_payload.STATUS
        self._transceiver_status_response_publisher.publish(transceiver_status_response_msg)
    
    def _on_ping_send_command(self, msg):
        cid = seatrac_constants.CID_PING_SEND['cid']

        structure = seatrac_constants.CID_PING_SEND['command']
        data = {
            'DEST_ID': msg.destination_id,
            'MSG_TYPE': msg.msg_type,
        }
        payload = structure.build(data)

        #self.get_logger().info(f'SENDING PING to beacon #{}.'.format(msg.destination_id))

        self._packetize(cid, payload)

        self.ping_sent += 1


    def _on_ping_send_response(self, payload):
        parsed_payload = seatrac_constants.CID_PING_SEND['response'].parse(payload)

        ping_send_response_msg = seatrac_driver.msg.PingSendResponse()
        ping_send_response_msg.status = parsed_payload.STATUS
        ping_send_response_msg.beacon_id = int(parsed_payload.BEACON_ID)
        self._ping_send_response_publisher.publish(ping_send_response_msg)

    def _on_ping_req_status(self, payload):
        parsed_payload = seatrac_constants.CID_PING_REQ['status'].parse(payload)

        ping_request_status_msg = seatrac_driver.msg.PingRequestStatus()
        self._populate_acofix_structure(ping_request_status_msg.fix, parsed_payload.ACO_FIX)
        self._ping_request_status_publisher.publish(ping_request_status_msg)

    def _on_ping_resp_status(self, payload):
        parsed_payload = seatrac_constants.CID_PING_RESP['status'].parse(payload)

        self.ping_received += 1

        ping_response_status_msg = seatrac_driver.msg.PingResponseStatus()
        self._populate_acofix_structure(ping_response_status_msg.fix, parsed_payload.ACO_FIX)
        self._ping_response_status_publisher.publish(ping_response_status_msg)

    def _on_ping_error_status(self, payload):
        parsed_payload = seatrac_constants.CID_PING_ERROR['status'].parse(payload)

        self.ping_error += 1

        ping_error_status_msg = seatrac_driver.msg.PingErrorStatus()
        ping_error_status_msg.status = parsed_payload.STATUS
        ping_error_status_msg.beacon_id = int(parsed_payload.BEACON_ID)
        self._ping_error_status_publisher.publish(ping_error_status_msg)

    def _on_nav_query_send_command(self, msg):
        cid = seatrac_constants.CID_NAV_QUERY_SEND['cid']

        structure = seatrac_constants.CID_NAV_QUERY_SEND['command']
        data = {
            'DEST_ID': msg.destination_id,
            'QUERY_FLAGS': {
                'QRY_DEPTH': msg.query_depth,
                'QRY_SUPPLY': msg.query_supply,
                'QRY_TEMP': msg.query_temp,
                'QRY_ATTITUDE': msg.query_attitude,
                'QRY_DATA': msg.query_data,
            },
            'PACKET_LEN': len(msg.packet_data),
            'PACKET_DATA': list(bytearray(msg.packet_data)),
        }
        payload = structure.build(data)

        self._packetize(cid, payload)

    def _on_nav_query_send_response(self, payload):
        parsed_payload = seatrac_constants.CID_NAV_QUERY_SEND['response'].parse(payload)

        navigation_query_send_response_msg = seatrac_driver.msg.NavigationQuerySendResponse()
        navigation_query_send_response_msg.status = parsed_payload.STATUS
        navigation_query_send_response_msg.destination_id = int(parsed_payload.DEST_ID)
        self._navigation_query_send_response_publisher.publish(navigation_query_send_response_msg)

    def _on_nav_query_req_status(self, payload):
        parsed_payload = seatrac_constants.CID_NAV_QUERY_REQ['status'].parse(payload)

        navigation_query_request_status_msg = seatrac_driver.msg.NavigationQueryRequestStatus()
        self._populate_acofix_structure(navigation_query_request_status_msg.fix, parsed_payload.ACO_FIX)
        navigation_query_request_status_msg.query_depth = parsed_payload.QUERY_FLAGS.QRY_DEPTH
        navigation_query_request_status_msg.query_supply = parsed_payload.QUERY_FLAGS.QRY_SUPPLY
        navigation_query_request_status_msg.query_temp = parsed_payload.QUERY_FLAGS.QRY_TEMP
        navigation_query_request_status_msg.query_attitude = parsed_payload.QUERY_FLAGS.QRY_ATTITUDE
        navigation_query_request_status_msg.query_data = parsed_payload.QUERY_FLAGS.QRY_DATA
        navigation_query_request_status_msg.packet_length = parsed_payload.PACKET_LEN
        navigation_query_request_status_msg.packet_data = parsed_payload.PACKET_DATA
        navigation_query_request_status_msg.local_flag = parsed_payload.LOCAL_FLAG
        self._navigation_query_request_status_publisher.publish(navigation_query_request_status_msg)

    def _on_nav_query_resp_status(self, payload):
        parsed_payload = seatrac_constants.CID_NAV_QUERY_RESP['status'].parse(payload)

        navigation_query_response_status_msg = seatrac_driver.msg.NavigationQueryResponseStatus()
        self._populate_acofix_structure(navigation_query_response_status_msg.fix, parsed_payload.ACO_FIX)
        navigation_query_response_status_msg.query_depth = parsed_payload.QUERY_FLAGS.QRY_DEPTH
        navigation_query_response_status_msg.query_supply = parsed_payload.QUERY_FLAGS.QRY_SUPPLY
        navigation_query_response_status_msg.query_temp = parsed_payload.QUERY_FLAGS.QRY_TEMP
        navigation_query_response_status_msg.query_attitude = parsed_payload.QUERY_FLAGS.QRY_ATTITUDE
        navigation_query_response_status_msg.query_data = parsed_payload.QUERY_FLAGS.QRY_DATA

        if navigation_query_response_status_msg.query_depth:
            navigation_query_response_status_msg.remote_depth = parsed_payload.REMOTE_DEPTH / 10.0

        if navigation_query_response_status_msg.query_supply:
            navigation_query_response_status_msg.remote_supply = parsed_payload.REMOTE_SUPPLY / 1000.0
    
        if navigation_query_response_status_msg.query_temp:
            navigation_query_response_status_msg.remote_temp = parsed_payload.REMOTE_TEMP / 10.0

        if navigation_query_response_status_msg.query_attitude:
            navigation_query_response_status_msg.remote_yaw = parsed_payload.REMOTE_YAW / 10.0
            navigation_query_response_status_msg.remote_pitch = parsed_payload.REMOTE_PITCH / 10.0
            navigation_query_response_status_msg.remote_roll = parsed_payload.REMOTE_ROLL / 10.0

        if navigation_query_response_status_msg.query_data:
            navigation_query_response_status_msg.packet_length = parsed_payload.PACKET_LEN
            navigation_query_response_status_msg.packet_data = parsed_payload.PACKET_DATA

        navigation_query_response_status_msg.local_flag = parsed_payload.LOCAL_FLAG
        self._navigation_query_response_status_publisher.publish(navigation_query_response_status_msg)

    def _on_nav_error_status(self, payload):
        parsed_payload = seatrac_constants.CID_NAV_ERROR['status'].parse(payload)

        navigation_error_status_msg = seatrac_driver.msg.NavigationErrorStatus()
        navigation_error_status_msg.status = parsed_payload.STATUS
        navigation_error_status_msg.beacon_id = int(parsed_payload.BEACON_ID)
        self._navigation_error_status_publisher.publish(navigation_error_status_msg)

    def _on_nav_status_send_command(self, msg):
        cid = seatrac_constants.CID_NAV_STATUS_SEND['cid']

        structure = seatrac_constants.CID_NAV_STATUS_SEND['command']
        data = {
            'BEACON_ID': msg.beacon_id,
            'PACKET_LEN': len(msg.packet_data),
            'PACKET_DATA': list(bytearray(msg.packet_data)),
        }
        payload = structure.build(data)

        self._packetize(cid, payload)

    def _on_nav_status_send_response(self, payload):
        parsed_payload = seatrac_constants.CID_NAV_STATUS_SEND['response'].parse(payload)

        navigation_status_send_response_msg = seatrac_driver.msg.NavigationStatusSendResponse()
        navigation_status_send_response_msg.status = parsed_payload.STATUS
        navigation_status_send_response_msg.beacon_id = int(parsed_payload.BEACON_ID)
        self._navigation_status_send_response_publisher.publish(navigation_status_send_response_msg)

    def _on_nav_status_receive_status(self, payload):
        parsed_payload = seatrac_constants.CID_NAV_STATUS_RECEIVE['status'].parse(payload)

        navigation_status_receive_status_msg = seatrac_driver.msg.NavigationStatusReceiveStatus()
        self._populate_acofix_structure(navigation_status_receive_status_msg.fix, parsed_payload.ACO_FIX)
        navigation_status_receive_status_msg.beacon_id = int(parsed_payload.BEACON_ID)
        navigation_status_receive_status_msg.packet_length = parsed_payload.PACKET_LEN
        navigation_status_receive_status_msg.packet_data = parsed_payload.PACKET_DATA
        navigation_status_receive_status_msg.local_flag = parsed_payload.LOCAL_FLAG
        self._navigation_status_receive_status_publisher.publish(navigation_status_receive_status_msg)

    def _on_dat_send_command(self, msg):
        cid = seatrac_constants.CID_DAT_SEND['cid']

        structure = seatrac_constants.CID_DAT_SEND['command']
        data = {
            'DEST_ID': msg.destination_id,
            'MSG_TYPE': msg.msg_type,
            'PACKET_LEN': len(msg.packet_data),
            'PACKET_DATA': list(bytearray(msg.packet_data)),
        }
        payload = structure.build(data)

        self._packetize(cid, payload)

        self.data_sent += 1

    def _on_dat_send_response(self, payload):
        parsed_payload = seatrac_constants.CID_DAT_SEND['response'].parse(payload)

        data_send_response_msg = seatrac_driver.msg.DataSendResponse()
        data_send_response_msg.status = parsed_payload.STATUS
        data_send_response_msg.beacon_id = int(parsed_payload.BEACON_ID)
        self._data_send_response_publisher.publish(data_send_response_msg)

    def _on_dat_receive_status(self, payload):
        parsed_payload = seatrac_constants.CID_DAT_RECEIVE['status'].parse(payload)

        data_receive_status_msg = seatrac_driver.msg.DataReceiveStatus()
        self._populate_acofix_structure(data_receive_status_msg.fix, parsed_payload.ACO_FIX)
        data_receive_status_msg.ack_flag = parsed_payload.ACK_FLAG
        data_receive_status_msg.packet_length = parsed_payload.PACKET_LEN
        data_receive_status_msg.packet_data = list(parsed_payload.PACKET_DATA)
        data_receive_status_msg.local_flag = parsed_payload.LOCAL_FLAG

        self.data_received += 1

        self._data_receive_status_publisher.publish(data_receive_status_msg)

    def _on_dat_error_status(self, payload):
        parsed_payload = seatrac_constants.CID_DAT_ERROR['status'].parse(payload)

        data_error_status_msg = seatrac_driver.msg.DataErrorStatus()
        data_error_status_msg.status = parsed_payload.STATUS
        data_error_status_msg.beacon_id = int(parsed_payload.BEACON_ID)
        self._data_error_status_publisher.publish(data_error_status_msg)

    def _populate_acofix_structure(self, ros_component, construct_component):
        ros_component.destination_id = int(construct_component.DEST_ID)
        ros_component.source_id = int(construct_component.SRC_ID)
        ros_component.range_valid = construct_component.FLAGS.RANGE_VALID
        ros_component.usbl_valid = construct_component.FLAGS.USBL_VALID
        ros_component.position_valid = construct_component.FLAGS.POSITION_VALID
        ros_component.position_enhanced = construct_component.FLAGS.POSITION_ENHANCED
        ros_component.position_filter_error = construct_component.FLAGS.POSITION_FLT_ERROR
        ros_component.msg_type = int(construct_component.MSG_TYPE)
        ros_component.attitude_yaw = construct_component.ATTITUDE_YAW / 10.0
        ros_component.attitude_pitch = construct_component.ATTITUDE_PITCH / 10.0
        ros_component.attitude_roll = construct_component.ATTITUDE_ROLL / 10.0
        ros_component.depth_local = construct_component.DEPTH_LOCAL / 10.0
        ros_component.vos = construct_component.VOS / 10.0
        ros_component.rssi = construct_component.RSSI / 10.0
        if construct_component.FLAGS.RANGE_VALID:
            ros_component.range_count = construct_component.RANGE.RANGE_COUNT
            ros_component.range_time = construct_component.RANGE.RANGE_TIME / 1.0e7
            ros_component.range_dist = construct_component.RANGE.RANGE_DIST / 10.0
        # USBL channels and per-channel RSSI intentionally skipped (unused)
        if construct_component.FLAGS.USBL_VALID:
            ros_component.usbl_azimuth = construct_component.USBL.USBL_AZIMUTH / 10.0
            ros_component.usbl_elevation = construct_component.USBL.USBL_ELEVATION / 10.0
            ros_component.usbl_fit_error = construct_component.USBL.USBL_FIT_ERROR / 100.0
        if construct_component.FLAGS.POSITION_VALID:
            ros_component.position_easting = construct_component.POSITION.POSITION_EASTING / 10.0
            ros_component.position_northing = construct_component.POSITION.POSITION_NORTHING / 10.0
            ros_component.position_depth = construct_component.POSITION.POSITION_DEPTH / 10.0

    def _packetize(self, command_id, payload=None):
        # Construct the start of the command message
        data = struct.pack('<B', command_id)

        if payload is not None:
            data += payload

        # Calculate the checksum of the command ID and payload data
        calculated_checksum = self._checksum(data)

        # Add checksum and ending characters to the command message
        data += struct.pack('<H', calculated_checksum)
        data = b'#' + binascii.hexlify(data) + b'\x0D\x0A'

        if self._log_io:
            data_hex = ''
            if len(data) > 0:
                data_hex = ['{:02x}'.format(struct.unpack('<B', byte)[0]) for byte in data]
                data_hex = ' '.join(data_hex)
            #self.get_logger().info(f'PACKETIZE ({} bytes): CMD_ID "{:02X}" "{}"'.format(len(data), command_id, data_hex))

        try:
            self.connection.write(data)
            self.sent_command_count += 1
        except serial.SerialException:
            self.get_logger().warning(f'Unable to write to device {self.device}. Attempting to re-establish')
            self.connection.close()

    def _checksum(self, data):
        # Computes the CRC16 checksum of the data provided. For SeaTrac data, the checksum should
        # compute only the command ID plus the payload data. The checksum bytes and end sync bytes
        # should not be included
        poly = numpy.uint16(0xA001)
        crc = numpy.uint16(0)

        for b in bytearray(data):
            for _ in range(8):
                if (b & 0x01) ^ (crc & 0x01):
                    crc = (crc >> 1) ^ poly
                else:
                    crc >>= 1

                b >>= 1

        return numpy.uint16(crc)


if __name__ == '__main__':
    
    rclpy.init()
    node = SeaTracDriver()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Run your logic periodically using a timer instead of a blocking loop
    def tick():
        node.run_once()

    node.create_timer(1.0 / node.rate_hz, tick)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


