#!/usr/bin/env python3

import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import binascii
import math
from tf_transformations import euler_from_quaternion

import serial
import struct
import numpy as np
import math
import traceback
import time
import threading
from threading import Event

import nav_msgs.msg
import std_msgs.msg
import seatrac_driver.msg

import seatrac_constants


class SeaTracDriverStub(Node):
    # Define ROS parameter defaults for this node
    _DEFAULT_CONTROL_RATE = 20.0 # [Hz]
    _DEFAULT_DEVICE = '/dev/ttyUSB0'
    _DEFAULT_BAUDRATE = 115200
    _DEFAULT_BUFFER_READ_SIZE = 1024
    _DEFAULT_SIM_COMMS_DROP = True
    _DEFAULT_SIM_TX_RATE = True
    _DEFAULT_SIM_WATER_BAUD = 100
    _DEFAULT_SIM_MODEM_TYPE = 'x010'
    _DEFAULT_WATER_VOS = 1518.0

    _VALID_MODEM_TYPES = ['x010','x100','x150']

    # According to the datasheet, the SeaTrac has an acoustic range of 1km
    _SEATRAC_RANGE_M = 1000.0

    # Response delay for local commands/responses (no inter-modem traffic)
    _SIM_LOCAL_RESPONSE_DELAY_SEC = 0.5

    # Simulated serial response delay
    _SIM_RESPONSE_DELAY_SEC = 0.01

    def __init__(self):

        super().__init__('seatrac_driver')
        self.get_logger().info('SeaTrac Driver [SIMULATED]. Namespace = seatrac_driver')

        # Handle parameters
        self.declare_parameter('rate', self._DEFAULT_CONTROL_RATE)
        self.rate_hz = self.get_parameter('rate').value
        self.declare_parameter('device', self._DEFAULT_DEVICE)
        self.device = self.get_parameter('device').value
        self.declare_parameter('baudrate', self._DEFAULT_BAUDRATE)
        self.baudrate = self.get_parameter('baudrate').value
        self.declare_parameter('asset_id')
        self.asset_id = self.get_parameter('asset_id').value
        self._namespace = self.get_namespace()

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

        # Simulation parameters.
        #self.get_logger().info(f'SeaTrac Driver [Simulated]: Loading simulation settings.')
        self.declare_parameter('sim_comms_drop', self._DEFAULT_SIM_COMMS_DROP)
        self._simulate_comms_drop = self.get_parameter('sim_comms_drop').value
        #self.get_logger().info(f' > Simulate Comms Drop: {}'.format(self._simulate_comms_drop))
        self.declare_parameter('sim_tx_rate', self._DEFAULT_SIM_TX_RATE)
        self._simulate_tx_rate = self.get_parameter('sim_tx_rate').value
        #self.get_logger().info(f' > TX rate: {}'.format(self._simulate_tx_rate))
        self.declare_parameter('sim_water_baud', self._DEFAULT_SIM_WATER_BAUD)
        self._simulated_water_baud = self.get_parameter('sim_water_baud').value
        #self.get_logger().info(f' > Water BAUD: {}'.format(self._simulated_water_baud))
        self.declare_parameter('sim_modem_type', self._DEFAULT_SIM_MODEM_TYPE)
        self._simulated_modem_type = self.get_parameter('sim_modem_type').value
        #self.get_logger().info(f' > Modem Type: {}'.format(self._simulated_modem_type))
        self.declare_parameter('sim_water_vos', self._DEFAULT_WATER_VOS)
        self._simulated_water_vos = self.get_parameter('sim_water_vos').value
        #self.get_logger().info(f' > Water VOS: {}'.format(self._simulated_water_vos))

        self.declare_parameter('beacon_pose_ned', [])
        beacon_pose_ned = self.get_parameter('beacon_pose_ned').value
        if len(beacon_pose_ned) != 4:
            self.get_logger().error('SEATRAC: beacon_pose_ned must be 4-vector')
            return

        self.declare_parameter('beacon_pose_pitch', 0.0)
        self.declare_parameter('beacon_pose_roll', 0.0)

        self._pose_gt = {
            'x' : beacon_pose_ned[0],
            'y' : beacon_pose_ned[1],
            'z' : beacon_pose_ned[2],
            'yaw' : beacon_pose_ned[3],
            'pitch' : self.get_parameter('beacon_pose_pitch').value,         
            'roll' : self.get_parameter('beacon_pose_roll').value
            }
        #self.get_logger().info(f ' > Beacon Pose: [{}, {}, {}, {}, {}, {}]'.format(
        #    self._pose_gt['x'], self._pose_gt['y'], self._pose_gt['z'],
        #    self._pose_gt['yaw'], self._pose_gt['pitch'], self._pose_gt['roll']))

        self.declare_parameter('sim_pose_src','pose_gt')
        self._pose_src = self.get_parameter('sim_pose_src').value
        if self._pose_src not in ['args', 'pose_gt']:
            self.get_logger().warn('  > WARN: Invalid "sim_pose_src". Defaulting to "pose_gt".')
            self._pose_src = 'pose_gt'
        #self.get_logger().info(f ' > Beacon Pose Source: "{}"'.format(self._pose_src))


        # Sanitize the modem type. If the modem type is not recognized, default to X010
        self._simulated_modem_type = self._simulated_modem_type.lower()
        if self._simulated_modem_type not in self._VALID_MODEM_TYPES:
            self.get_logger().warn('Found invalid simulated modem type ')
            self._simulated_modem_type = self._DEFAULT_SIM_MODEM_TYPE

        # Default beacon settings per the SeaTrac manual.
        self.current_settings_msg = seatrac_driver.msg.SettingsGetResponse()
        self.current_settings_msg.status_mode = 3
        self.current_settings_msg.status_mode_str = 'STATUS_MODE_5HZ'
        self.current_settings_msg.status_output_environment = False
        self.current_settings_msg.status_output_attitude = False
        self.current_settings_msg.status_output_mag_cal = False
        self.current_settings_msg.status_output_acc_cal = False
        self.current_settings_msg.status_output_ahrs_raw_data = False
        self.current_settings_msg.status_output_ahrs_comp_data = False
        self.current_settings_msg.uart_main_baud = 13 #BAUD_115200
        self.current_settings_msg.uart_main_baud_str = '115200'
        # All other settings reserved for future use are intentionally ignored
        self.current_settings_msg.enable_automatic_pressure_offset_cal = True
        self.current_settings_msg.enable_automatic_vos = True
        self.current_settings_msg.manual_pressure_offset_bar = 0 / 1000.0
        self.current_settings_msg.salinity_ppt = 100 / 10.0
        self.current_settings_msg.manual_vos = 1000 / 10.0
        self.current_settings_msg.enable_automatic_magnetometer_cal = True
        self.current_settings_msg.ahrs_cal_acc_min_x = -270
        self.current_settings_msg.ahrs_cal_acc_min_y = -270
        self.current_settings_msg.ahrs_cal_acc_min_z = -270
        self.current_settings_msg.ahrs_cal_acc_max_x = 270
        self.current_settings_msg.ahrs_cal_acc_max_y = 270
        self.current_settings_msg.ahrs_cal_acc_max_z = 270
        self.current_settings_msg.ahrs_cal_mag_valid = True
        self.current_settings_msg.ahrs_cal_mag_hard_x = 0
        self.current_settings_msg.ahrs_cal_mag_hard_y = 0
        self.current_settings_msg.ahrs_cal_mag_hard_z = 0
        self.current_settings_msg.ahrs_cal_mag_soft_x = 1
        self.current_settings_msg.ahrs_cal_mag_soft_y = 1
        self.current_settings_msg.ahrs_cal_mag_soft_z = 1
        self.current_settings_msg.ahrs_cal_mag_field = 0
        self.current_settings_msg.ahrs_cal_mag_error = 100
        self.current_settings_msg.ahrs_cal_gyro_offset_x = 0
        self.current_settings_msg.ahrs_cal_gyro_offset_y = 0
        self.current_settings_msg.ahrs_cal_gyro_offset_z = 0
        self.current_settings_msg.ahrs_yaw_offset_deg = 0 / 10.0
        self.current_settings_msg.ahrs_pitch_offset_deg = 0 / 10.0
        self.current_settings_msg.ahrs_roll_offset_deg = 0 / 10.0
        self.current_settings_msg.enable_transceiver_diagnostic_messages = True
        self.current_settings_msg.enable_transceiver_fix_messages = True
        self.current_settings_msg.enable_transceiver_usbl_messages = True
        self.current_settings_msg.enable_transceiver_position_filter = True
        self.current_settings_msg.enable_transceiver_automatic_ahrs = False
        self.current_settings_msg.transceiver_beacon_id = 1
        self.current_settings_msg.transceiver_timeout_range_m = 100
        self.current_settings_msg.transceiver_response_time_ms = 100
        self.current_settings_msg.transceiver_manual_ahrs_yaw_deg = 0 / 10.0
        self.current_settings_msg.transceiver_manual_ahrs_pitch_deg = 0 / 10.0
        self.current_settings_msg.transceiver_manual_ahrs_roll_deg = 0 / 10.0
        self.current_settings_msg.transceiver_position_filter_max_velocity = 0.2
        self.current_settings_msg.transceiver_position_filter_max_angle = 10
        self.current_settings_msg.transceiver_position_filter_reset_timeout = 60

        # Simulated ACO_FIX attributes
        self._simulated_aco_fix = {
            'DEST_ID' : 0,
            'SRC_ID' : 0,
            'FLAGS' : {
                'POSITION_FLT_ERROR' : False,
                'POSITION_ENHANCED' : False,
                'POSITION_VALID' : True,
                'USBL_VALID' : True,
                'RANGE_VALID' : True
            },
            'MSG_TYPE' : 0,
            'ATTITUDE_YAW' : 0,
            'ATTITUDE_PITCH' : 0,
            'ATTITUDE_ROLL' : 0,
            'DEPTH_LOCAL' : 0,
            'VOS' : 0,
            'RSSI' : 0,
            'RANGE' : {
                'RANGE_COUNT' : 0,
                'RANGE_TIME' : 0,
                'RANGE_DIST' : 0
            },
            'USBL' : {
                'USBL_CHANNELS' : 1,
                'USBL_RSSI' : [0],
                'USBL_AZIMUTH' : 0,
                'USBL_ELEVATION' : 0,
                'USBL_FIT_ERROR' : 0
            },
            'POSITION' : {
                'POSITION_EASTING' : 0,
                'POSITION_NORTHING' : 0,
                'POSITION_DEPTH' : 0
            }
        }


        # The SeaTrac Driver stub emulates serial over ROS topic. All SeaTrac messages are serialized
        # into the messages to the same ROS topic. Ever node publishes and subscribes to the same topic
        # to share data.
        self._internal_serial_data_publisher = self.create_publisher(seatrac_driver.msg.SerialInternal, 
                                                                    '/seatrac/seatrac_serial_internal',
                                                                    10)
        self._internal_serial_data_subscriber = self.create_subscription(seatrac_driver.msg.SerialInternal, 
                                                                        '/seatrac/seatrac_serial_internal',
                                                                        self._on_serial_internal,
                                                                        10)

        self._driver_status_publisher = self.create_publisher(seatrac_driver.msg.DriverStatus, 
                                                                    '/seatrac/driver_status',
                                                                    10)

        self._system_alive_command_subscriber = self.create_subscription(std_msgs.msg.Empty, 
                                                                        '/seatrac/system_alive_command',
                                                                        self._on_system_alive_command,
                                                                        10)
        self._system_alive_response_publisher = self.create_publisher(seatrac_driver.msg.SystemAliveResponse, 
                                                                    '/seatrac/system_alive_response',
                                                                    10)

        self._system_info_command_subscriber = self.create_subscription(std_msgs.msg.Empty, 
                                                                        '/seatrac/system_info_command',
                                                                        self._on_system_info_command,
                                                                        10)
        self._system_info_response_publisher = self.create_publisher(seatrac_driver.msg.SystemInfoResponse, 
                                                                    '/seatrac/system_info_response',
                                                                    10)

        self._system_reboot_command_subscriber = self.create_subscription(std_msgs.msg.Empty, 
                                                                        '/seatrac/system_reboot_command',
                                                                        self._on_system_reboot_command,
                                                                        10)
        self._system_reboot_response_publisher = self.create_publisher(seatrac_driver.msg.SystemRebootResponse, 
                                                                    '/seatrac/system_reboot_response',
                                                                    10)

        self._status_command_subscriber = self.create_subscription(seatrac_driver.msg.StatusCommand, 
                                                                        '/seatrac/status_command',
                                                                        self._on_status_command,
                                                                        10)
        self._status_response_publisher = self.create_publisher(seatrac_driver.msg.StatusResponse, 
                                                                    '/seatrac/status_response',
                                                                    10)

        self._status_config_get_command_subscriber = self.create_subscription(std_msgs.msg.Empty, 
                                                                        '/seatrac/status_config_get_command',
                                                                        self._on_status_config_get_command,
                                                                        10)
        self._status_config_get_response_publisher = self.create_publisher(seatrac_driver.msg.StatusConfigGetResponse, 
                                                                    '/seatrac/status_config_get_response',
                                                                    10)

        self._status_config_set_command_subscriber = self.create_subscription(seatrac_driver.msg.StatusConfigSetCommand, 
                                                                        '/seatrac/status_config_set_command',
                                                                        self._on_status_config_set_command,
                                                                        10)
        self._status_config_set_response_publisher = self.create_publisher(seatrac_driver.msg.StatusConfigSetResponse, 
                                                                    '/seatrac/status_config_set_response',
                                                                    10)

        self._settings_get_command_subscriber = self.create_subscription(std_msgs.msg.Empty, 
                                                                        '/seatrac/settings_get_command',
                                                                        self._on_settings_get_command,
                                                                        10)
        self._settings_get_response_publisher = self.create_publisher(seatrac_driver.msg.SettingsGetResponse, 
                                                                    '/seatrac/settings_get_response',
                                                                    10)

        self._settings_set_command_subscriber = self.create_subscription(seatrac_driver.msg.SettingsSetCommand, 
                                                                        '/seatrac/settings_set_command',
                                                                        self._on_settings_set_command,
                                                                        10)
        self._settings_set_response_publisher = self.create_publisher(seatrac_driver.msg.SettingsSetResponse, 
                                                                    '/seatrac/settings_set_response',
                                                                    10)

        self._settings_load_command_subscriber = self.create_subscription(std_msgs.msg.Empty, 
                                                                        '/seatrac/settings_load_command',
                                                                        self._on_settings_load_command,
                                                                        10)
        self._settings_load_response_publisher = self.create_publisher(seatrac_driver.msg.SettingsLoadResponse, 
                                                                    '/seatrac/settings_load_response',
                                                                    10)

        self._settings_save_command_subscriber = self.create_subscription(std_msgs.msg.Empty, 
                                                                        '/seatrac/settings_save_command',
                                                                        self._on_settings_save_command,
                                                                        10)
        self._settings_save_response_publisher = self.create_publisher(seatrac_driver.msg.SettingsSaveResponse, 
                                                                    '/seatrac/settings_save_response',
                                                                    10)

        self._settings_reset_command_subscriber = self.create_subscription(std_msgs.msg.Empty, 
                                                                        '/seatrac/settings_reset_command',
                                                                        self._on_settings_reset_command,
                                                                        10)
        self._settings_reset_response_publisher = self.create_publisher(seatrac_driver.msg.SettingsResetResponse, 
                                                                    '/seatrac/settings_reset_response',
                                                                    10)

        self._calibration_action_command_subscriber = self.create_subscription(seatrac_driver.msg.CalibrationActionCommand, 
                                                                        '/seatrac/calibration_action_command',
                                                                        self._on_cal_action_command,
                                                                        10)
        self._calibration_action_response_publisher = self.create_publisher(seatrac_driver.msg.CalibrationActionResponse, 
                                                                    '/seatrac/calibration_action_response',
                                                                    10)

        self._calibration_get_command_subscriber = self.create_subscription(std_msgs.msg.Empty, 
                                                                        '/seatrac/calibration_get_command',
                                                                        self._on_ahrs_cal_get_command,
                                                                        10)
        self._calibration_get_response_publisher = self.create_publisher(seatrac_driver.msg.CalibrationGetResponse, 
                                                                    '/seatrac/calibration_get_response',
                                                                    10)

        self._calibration_set_command_subscriber = self.create_subscription(seatrac_driver.msg.CalibrationSetCommand, 
                                                                        '/seatrac/calibration_set_command',
                                                                        self._on_ahrs_cal_set_command,
                                                                        10)
        self._calibration_set_response_publisher = self.create_publisher(seatrac_driver.msg.CalibrationSetResponse, 
                                                                    '/seatrac/calibration_set_response',
                                                                    10)

        self._transceiver_analyze_command_subscriber = self.create_subscription(std_msgs.msg.Empty, 
                                                                        '/seatrac/transceiver_analyze_command',
                                                                        self._on_transceiver_analyze_command,
                                                                        10)
        self._transceiver_analyze_response_publisher = self.create_publisher(seatrac_driver.msg.TransceiverAnalyzeResponse, 
                                                                    '/seatrac/transceiver_analyze_response',
                                                                    10)

        self._tranceiver_fix_status_publisher = self.create_publisher(seatrac_driver.msg.TransceiverFixStatus, 
                                                                    '/seatrac/transceiver_fix_status',
                                                                    10)

        self._transceiver_status_command_subscriber = self.create_subscription(std_msgs.msg.Empty, 
                                                                        '/seatrac/transceiver_status_command',
                                                                        self._on_transceiver_status_command,
                                                                        10)
        self._transceiver_status_response_publisher = self.create_publisher(seatrac_driver.msg.TransceiverStatusResponse, 
                                                                    '/seatrac/transceiver_status_response',
                                                                    10)

        self._ping_send_command_subscriber = self.create_subscription(seatrac_driver.msg.PingSendCommand, 
                                                                        '/seatrac/ping_send_command',
                                                                        self._on_ping_send_command,
                                                                        10)
        self._ping_send_response_publisher = self.create_publisher(seatrac_driver.msg.PingSendResponse, 
                                                                    '/seatrac/ping_send_response',
                                                                    10)

        self._ping_request_status_publisher = self.create_publisher(seatrac_driver.msg.PingRequestStatus, 
                                                                    '/seatrac/ping_request_status',
                                                                    10)

        self._ping_response_status_publisher = self.create_publisher(seatrac_driver.msg.PingResponseStatus, 
                                                                    '/seatrac/ping_response_status',
                                                                    10)

        self._ping_error_status_publisher = self.create_publisher(seatrac_driver.msg.PingErrorStatus, 
                                                                    '/seatrac/ping_error_status',
                                                                    10)

        self._navigation_query_send_command_subscriber = self.create_subscription(seatrac_driver.msg.NavigationQuerySendCommand, 
                                                                        '/seatrac/navigation_query_send_command',
                                                                        self._on_nav_query_send_command,
                                                                        10)
        self._navigation_query_send_response_publisher = self.create_publisher(seatrac_driver.msg.NavigationQuerySendResponse, 
                                                                    '/seatrac/navigation_query_send_response',
                                                                    10)

        self._navigation_query_request_status_publisher = self.create_publisher(seatrac_driver.msg.NavigationQueryRequestStatus, 
                                                                    '/seatrac/navigation_query_request_status',
                                                                    10)

        self._navigation_query_response_status_publisher = self.create_publisher(seatrac_driver.msg.NavigationQueryResponseStatus, 
                                                                    '/seatrac/navigation_query_response_status',
                                                                    10)

        self._navigation_error_status_publisher = self.create_publisher(seatrac_driver.msg.NavigationErrorStatus, 
                                                                    '/seatrac/navigation_error_status',
                                                                    10)

        self._navigation_status_send_commmand_subscriber = self.create_subscription(seatrac_driver.msg.NavigationStatusSendCommand, 
                                                                        '/seatrac/navigation_status_send_command',
                                                                        self._on_nav_status_send_command,
                                                                        10)
        self._navigation_status_send_response_publisher = self.create_publisher(seatrac_driver.msg.NavigationStatusSendResponse, 
                                                                    '/seatrac/navigation_status_send_response',
                                                                    10)

        self._navigation_status_receive_status_publisher = self.create_publisher(seatrac_driver.msg.NavigationStatusReceiveStatus, 
                                                                    '/seatrac/navigation_status_receive_status',
                                                                    10)

        self._data_send_command_subscriber = self.create_subscription(seatrac_driver.msg.DataSendCommand, 
                                                                        '/seatrac/data_send_command',
                                                                        self._on_dat_send_command,
                                                                        10)
        self._data_send_response_publisher = self.create_publisher(seatrac_driver.msg.DataSendResponse, 
                                                                    '/seatrac/data_send_response',
                                                                    10)

        self._data_receive_status_publisher = self.create_publisher(seatrac_driver.msg.DataReceiveStatus, 
                                                                    '/seatrac/data_receive_status',
                                                                    10)

        self._data_error_status_publisher = self.create_publisher(seatrac_driver.msg.DataErrorStatus, 
                                                                    '/seatrac/data_error_status',
                                                                    10)

        self.init_time = self.get_clock().now()


    def run(self):
        self.get_logger().info(f'Connecting to SeaTrac. Device = {self.device}')

        # Define the process execution rate
        self.get_logger().info(f'Running at {self.rate_hz} [Hz]')
        r = self.create_rate(self.rate_hz)


        last_time = self.get_clock().now()
        while rclpy.ok():

            if self._status_mode_to_period():
                if (self.get_clock().now() - last_time).to_sec() > self._status_mode_to_period():
                    self._simulate_status_response()
                    last_time = self.get_clock().now()

            r.sleep()


    def _status_mode_to_period(self):
        if self.current_settings_msg.status_mode == seatrac_constants.STATUSMODE_E.STATUS_MODE_1HZ.intvalue:
            return 1.0
        elif self.current_settings_msg.status_mode == seatrac_constants.STATUSMODE_E.STATUS_MODE_2HZ5.intvalue:
            return 0.4
        elif self.current_settings_msg.status_mode == seatrac_constants.STATUSMODE_E.STATUS_MODE_5HZ.intvalue:
            return 0.2
        elif self.current_settings_msg.status_mode == seatrac_constants.STATUSMODE_E.STATUS_MODE_10HZ.intvalue:
            return 0.1
        elif self.current_settings_msg.status_mode == seatrac_constants.STATUSMODE_E.STATUS_MODE_25HZ.intvalue:
            return 0.04
        else:
            return None

    def wait_for_message(self, topic, msg_type, timeout_sec=None):
        msg_evt = Event()
        msg_container = {}

        def _cb(msg):
            msg_container['msg'] = msg
            msg_evt.set()

        sub = self.create_subscription(msg_type, topic, _cb, 10)
        got = msg_evt.wait(timeout=timeout_sec)
        self.destroy_subscription(sub)

        if not got:
            raise TimeoutError(f'No message on {topic} after {timeout_sec}s')
        return msg_container['msg']

    def _on_serial_internal(self,msg):

        # Filter out messages that originated from this self.
        if msg.source_namespace == self.get_namespace():
            return
        
        packet_binary = msg.data

        successful = True

        # In simulation, the serial data is not a stream. It's a datagram containing the serial command.
        try:
            command_id = packet_binary[0]
            # r ospy.logdebug('SeaTrac Driver: _on_serial_internal - command_id="{}" "{}"'.format(command_id,':'.join('{:02x}'.format(ord(c)) for c in packet_binary)))
        except struct.error:
            traceback.print_exc()
            self.get_logger().warn('Error while unpacking command ID value. Skipping corrupt packet.')
            self.corrupt_packet_count += 1
            successful = False


        # Model signal attenuation and dropout based on the distance between the source and destination.
        # /bluerov2_#/pose_gt
        if successful and self._simulate_comms_drop:

            # Get the position of the sender and compute the distance between the sending beacon and
            # this beacon.
            try:
                source_pose_gt = self.wait_for_message(os.path.join(msg.source_namespace,'pose_gt'),
                    nav_msgs.msg.Odometry, timeout=1.0)
            except TimeoutError as e:
                self.get_logger().warn(str(e))
                source_pose_gt = None

            try:
                self_pose_gt = self.wait_for_message(os.path.join(self._namespace,'pose_gt'),
                    nav_msgs.msg.Odometry, timeout=1.0)
            except TimeoutError as e:
                self.get_logger().warn(str(e))
                self_pose_gt = None

            if source_pose_gt and self_pose_gt:

                source_pos = source_pose_gt.pose.pose.position
                self_pos = self_pose_gt.pose.pose.position

                radius_m = math.sqrt((source_pos.x - self_pos.x)**2 +
                                     (source_pos.y - self_pos.y)**2 +
                                     (source_pos.z - self_pos.z)**2)

                # Use a very simple linear attenuation model. 
                # y = m * x + b = (-1.0 / SEATRAC_RANGE) * x + 1.0
                prob_arrival = (-1.0 * radius_m) / self._SEATRAC_RANGE_M + 1.0

                # Model comms drop out.
                if np.random.uniform() > prob_arrival:
                    #r ospy.logdebug('SeaTrac Driver [SIMULATION]: Simulating packet drop from UUV ({}) to UUV ({}) radius ({} m). Probability of arrival is {}.'.format(
                    #    msg.source_asset_id, self.asset_id, radius_m, prob_arrival))
                    self.corrupt_packet_count += 1
                    successful = False


        # The SeaTrac has a very limited communication bandwidth. Add a delay proportional to the size of the packet.
        if successful and self._simulate_tx_rate:

            # Compute how long it would take to send the packet at the maximum data rate.
            payload_size_bits = len(packet_binary) * 8
            tx_time_sec = payload_size_bits / self._simulated_water_baud

            elapsed_sec = self.get_clock().now().to_msg() - msg.send_time.to_sec()
            if tx_time_sec > elapsed_sec:
                delay_period_sec = tx_time_sec - elapsed_sec
                #r ospy.logdebug('SeaTrac Driver [SIMULATION]: Delaying delivery of message from UUV ({}) to UUV ({}) by {} seconds.'.format(
                #    msg.source_asset_id, self.asset_id, delay_period_sec))
                time.sleep(delay_period_sec)

        if successful:
            payload = packet_binary[1:]
            successful = self._parse_packet(msg, command_id, payload)

            if successful:
                self.parsed_packet_count += 1
            else:
                self.unhandled_packet_count += 1


        # Publish driver statistics for diagnosis
        driver_status = seatrac_driver.msg.DriverStatus()
        driver_status.device_connected = True
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


    def _get_pose(self):

        pose = {}
        if self._pose_src == 'pose_gt':

            try:
                self_pose_gt = self.wait_for_message(os.path.join(self._namespace,'pose_gt'),
                    nav_msgs.msg.Odometry, timeout=1.0)
            except TimeoutError as e:
                self.get_logger().warn('{e}')
                self_pose_gt = None

            if self_pose_gt:

                self_orientation_quat = (
                    self_pose_gt.pose.pose.orientation.x,
                    self_pose_gt.pose.pose.orientation.y,
                    self_pose_gt.pose.pose.orientation.z,
                    self_pose_gt.pose.pose.orientation.w)
                self_orientation_euler = euler_from_quaternion(self_orientation_quat)

                self_roll_world = self_orientation_euler[0]
                self_pitch_world = self_orientation_euler[1]
                self_yaw_world = self_orientation_euler[2]

                pose.update({
                    'x' : self_pose_gt.pose.pose.position.x,
                    'y' : self_pose_gt.pose.pose.position.y,
                    'z' : self_pose_gt.pose.pose.position.z,
                    'yaw' : self_yaw_world,
                    'pitch' : self_pitch_world,
                    'roll' : self_roll_world,
                })

        else:
            pose = self._pose_gt

        return pose

    def _parse_packet(self, msg, command_id, payload):
        # Parse the incoming device packet based on command ID (CID)
        if command_id == seatrac_constants.CID_SYS_INFO['cid']:
            self._on_system_info_response(payload)
        elif command_id == seatrac_constants.CID_SYS_REBOOT['cid']:
            self._on_system_reboot_response(payload)
        # elif command_id == seatrac_constants.CID_STATUS['cid']:
        #     self._on_status_response(payload)
        elif command_id == seatrac_constants.CID_STATUS_CFG_GET['cid']:
            self._on_status_config_get_response(payload)
        elif command_id == seatrac_constants.CID_STATUS_CFG_SET['cid']:
            self._on_status_config_set_response(payload)
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

            # If we received a ping addressed to us, respond with a simulation ping response.
            ping_cmd = seatrac_constants.CID_PING_SEND['command'].parse(payload)

            # If we are the addressed beacon, respond to the ping.
            if int(ping_cmd.DEST_ID) == self.current_settings_msg.transceiver_beacon_id:
                data = {
                    'ACO_FIX' : {
                        'DEST_ID' : msg.source_beacon_id,
                        'SRC_ID' : ping_cmd.DEST_ID,
                        'FLAGS' : {
                            'POSITION_FLT_ERROR' : False,
                            'POSITION_ENHANCED' : False,
                            'POSITION_VALID' : False,
                            'USBL_VALID' : False,
                            'RANGE_VALID' : True
                        },
                        'MSG_TYPE' : ping_cmd.MSG_TYPE,
                        'ATTITUDE_YAW' : 0,
                        'ATTITUDE_PITCH' : 0,
                        'ATTITUDE_ROLL' : 0,
                        'DEPTH_LOCAL' : 0,
                        'VOS' : 0,
                        'RSSI' : 0,
                        'RANGE' : {
                            'RANGE_COUNT' : 0,
                            'RANGE_TIME' : 0,
                            'RANGE_DIST' : 0
                        },
                        'USBL' : {
                            'USBL_CHANNELS' : 1,
                            'USBL_RSSI' : [0],
                            'USBL_AZIMUTH' : 0,
                            'USBL_ELEVATION' : 0,
                            'USBL_FIT_ERROR' : 0
                        },
                        'POSITION' : {
                            'POSITION_EASTING' : 0,
                            'POSITION_NORTHING' : 0,
                            'POSITION_DEPTH' : 0
                        }
                    }
                }
                resp_payload = seatrac_constants.CID_PING_RESP['status'].build(data)
                self._packetize(seatrac_constants.CID_PING_RESP['cid'], resp_payload)

            # Send a local ping request status.
            ping_request_status = seatrac_driver.msg.PingRequestStatus()
            ping_request_status.fix.destination_id = int(ping_cmd.DEST_ID)
            ping_request_status.fix.msg_type = ping_cmd.MSG_TYPE
            ping_request_status.fix.range_valid = False
            ping_request_status.fix.position_valid = False
            ping_request_status.fix.usbl_valid = False
            ping_request_status.fix.rssi = 0.0
            ping_request_status.fix.vos = 0
            ping_request_status.fix.attitude_pitch = 0.0
            ping_request_status.fix.attitude_roll = 0.0
            ping_request_status.fix.attitude_yaw = 0.0
            ping_request_status.fix.source_id = msg.source_beacon_id
            self._ping_request_status_publisher.publish(ping_request_status)

        elif command_id == seatrac_constants.CID_PING_RESP['cid']:
            self._on_ping_resp_status(payload, msg)
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

            # Convert the command into a response and sent a simulated response.
            command = seatrac_constants.CID_DAT_SEND['command'].parse(payload)
            self._simulated_aco_fix['DEST_ID'] = command.DEST_ID
            self._simulated_aco_fix['SRC_ID'] = msg.source_beacon_id
            payload = {
                'ACO_FIX' : self._simulated_aco_fix,
                'ACK_FLAG' : 1,
                'PACKET_LEN' : command.PACKET_LEN,
                'PACKET_DATA' : command.PACKET_DATA,
                'LOCAL_FLAG' : 0
            }
            structure_payload = seatrac_constants.CID_DAT_RECEIVE['status'].build(payload)
            self._on_dat_receive_status(structure_payload)

        elif command_id == seatrac_constants.CID_DAT_ERROR['cid']:
            self._on_dat_error_status(payload)
        else:
            self.get_logger().warn(f'Parsing of packet with command code {command_id} is not supported')

            return False

        return True

    def _on_system_alive_command(self, msg):

        # Simulated response delay.
        time.sleep(self._SIM_RESPONSE_DELAY_SEC)
        current_time = self.get_clock().now()

        system_alive_response_msg = seatrac_driver.msg.SystemAliveResponse()
        system_alive_response_msg.seconds = current_time.to_sec() - self.init_time.to_sec()
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
        # cid = seatrac_constants.CID_STATUS['cid']

        # structure = seatrac_constants.CID_STATUS['command']
        # data = {
        #     'STATUS_OUTPUT': {
        #         'ENVIRONMENT': msg.output_environment,
        #         'ATTITUDE': msg.output_attitude,
        #         'MAG_CAL': msg.output_mag_cal,
        #         'ACC_CAL': msg.output_acc_cal,
        #         'AHRS_RAW_DATA': msg.output_ahrs_raw_data,
        #         'AHRS_COMP_DATA': msg.output_ahrs_comp_data,
        #     },
        # }
        # payload = structure.build(data)

        # self._packetize(cid, payload)
        time.sleep(self._SIM_RESPONSE_DELAY_SEC)
        self._simulate_status_response()


    def _simulate_status_response(self):

        # Get our latest truth pose.
        pose = self._get_pose()

        # try:
        #     self_pose_gt = r ospy.wait_for_message(os.path.join(self._namespace,'pose_gt'),
        #         nav_msgs.msg.Odometry, timeout=1.0)
        # except TimeoutError as e:
        #     self.get_logger().warn('{e}')
        #     self_pose_gt = None

        # if self_pose_gt:

        #     self_orientation_quat = (
        #         self_pose_gt.pose.pose.orientation.x,
        #         self_pose_gt.pose.pose.orientation.y,
        #         self_pose_gt.pose.pose.orientation.z,
        #         self_pose_gt.pose.pose.orientation.w)
        #     self_orientation_euler = tf.transformations.euler_from_quaternion(self_orientation_quat)

        #     self_roll_world = self_orientation_euler[0]
        #     self_pitch_world = self_orientation_euler[1]
        #     self_yaw_world = self_orientation_euler[2]

        status_response_msg = seatrac_driver.msg.StatusResponse()
        status_response_msg.environment_outputted = False
        status_response_msg.attitude_outputted = True if pose else False
        status_response_msg.mag_cal_outputted = False
        status_response_msg.acc_cal_outputted = False
        status_response_msg.ahrs_raw_data_outputted = False
        status_response_msg.ahrs_comp_data_outputted = False

        if status_response_msg.environment_outputted:
            # status_response_msg.env_supply = parsed_payload.ENVIRONMENT.ENV_SUPPLY / 1000.0
            # status_response_msg.env_temperature = parsed_payload.ENVIRONMENT.ENV_TEMP / 10.0
            # status_response_msg.env_pressure = parsed_payload.ENVIRONMENT.ENV_PRESSURE / 1000.0
            # status_response_msg.env_depth = parsed_payload.ENVIRONMENT.ENV_DEPTH / 10.0
            # status_response_msg.env_vos = parsed_payload.ENVIRONMENT.ENV_VOS / 10.0
            pass

        if status_response_msg.attitude_outputted:
            status_response_msg.attitude_yaw = math.degrees(pose['yaw'])
            status_response_msg.attitude_pitch = math.degrees(pose['pitch'])
            status_response_msg.attitude_roll = math.degrees(pose['roll'])

        if status_response_msg.mag_cal_outputted:
            # status_response_msg.mag_cal_buf = parsed_payload.MAG_CAL.MAG_CAL_BUF
            # status_response_msg.mag_cal_valid = parsed_payload.MAG_CAL.MAG_CAL_VALID
            # status_response_msg.mag_cal_age = parsed_payload.MAG_CAL.MAG_CAL_AGE
            # status_response_msg.mag_cal_fit = parsed_payload.MAG_CAL.MAG_CAL_FIT
            pass

        if status_response_msg.acc_cal_outputted:
            # status_response_msg.acceleration_limit_min[0] = parsed_payload.ACC_CAL.ACC_LIM_MIN_X
            # status_response_msg.acceleration_limit_min[1] = parsed_payload.ACC_CAL.ACC_LIM_MIN_Y
            # status_response_msg.acceleration_limit_min[2] = parsed_payload.ACC_CAL.ACC_LIM_MIN_Z
            # status_response_msg.acceleration_limit_max[0] = parsed_payload.ACC_CAL.ACC_LIM_MAX_X
            # status_response_msg.acceleration_limit_max[1] = parsed_payload.ACC_CAL.ACC_LIM_MAX_Y
            # status_response_msg.acceleration_limit_max[2] = parsed_payload.ACC_CAL.ACC_LIM_MAX_Z
            pass

        if status_response_msg.ahrs_raw_data_outputted:
            # status_response_msg.ahrs_raw_acc[0] = parsed_payload.AHRS_RAW_DATA.AHRS_RAW_ACC_X
            # status_response_msg.ahrs_raw_acc[1] = parsed_payload.AHRS_RAW_DATA.AHRS_RAW_ACC_Y
            # status_response_msg.ahrs_raw_acc[2] = parsed_payload.AHRS_RAW_DATA.AHRS_RAW_ACC_Z
            # status_response_msg.ahrs_raw_mag[0] = parsed_payload.AHRS_RAW_DATA.AHRS_RAW_MAG_X
            # status_response_msg.ahrs_raw_mag[1] = parsed_payload.AHRS_RAW_DATA.AHRS_RAW_MAG_Y
            # status_response_msg.ahrs_raw_mag[2] = parsed_payload.AHRS_RAW_DATA.AHRS_RAW_MAG_Z
            # status_response_msg.ahrs_raw_gyro[0] = parsed_payload.AHRS_RAW_DATA.AHRS_RAW_GYRO_X
            # status_response_msg.ahrs_raw_gyro[1] = parsed_payload.AHRS_RAW_DATA.AHRS_RAW_GYRO_Y
            # status_response_msg.ahrs_raw_gyro[2] = parsed_payload.AHRS_RAW_DATA.AHRS_RAW_GYRO_Z
            pass

        if status_response_msg.ahrs_comp_data_outputted:
            # status_response_msg.ahrs_comp_acc.x = parsed_payload.AHRS_COMP_DATA.AHRS_COMP_ACC_X
            # status_response_msg.ahrs_comp_acc.y = parsed_payload.AHRS_COMP_DATA.AHRS_COMP_ACC_Y
            # status_response_msg.ahrs_comp_acc.z = parsed_payload.AHRS_COMP_DATA.AHRS_COMP_ACC_Z
            # status_response_msg.ahrs_comp_mag.x = parsed_payload.AHRS_COMP_DATA.AHRS_COMP_MAG_X
            # status_response_msg.ahrs_comp_mag.y = parsed_payload.AHRS_COMP_DATA.AHRS_COMP_MAG_Y
            # status_response_msg.ahrs_comp_mag.z = parsed_payload.AHRS_COMP_DATA.AHRS_COMP_MAG_Z
            # status_response_msg.ahrs_comp_gyro.x = parsed_payload.AHRS_COMP_DATA.AHRS_COMP_GYRO_X
            # status_response_msg.ahrs_comp_gyro.y = parsed_payload.AHRS_COMP_DATA.AHRS_COMP_GYRO_Y
            # status_response_msg.ahrs_comp_gyro.z = parsed_payload.AHRS_COMP_DATA.AHRS_COMP_GYRO_Z
            pass

        self.parsed_packet_count += 1

        # Publish driver statistics for diagnosis
        driver_status = seatrac_driver.msg.DriverStatus()
        driver_status.device_connected = True
        driver_status.sent_command_count = self.sent_command_count
        driver_status.parsed_packet_count = self.parsed_packet_count
        driver_status.unhandled_packet_count = self.unhandled_packet_count
        driver_status.corrupt_packet_count = self.corrupt_packet_count
        self._driver_status_publisher.publish(driver_status)

        self._status_response_publisher.publish(status_response_msg)



    # def _on_status_response(self, payload):
    #     # Parse CID_STATUS response
    #     parsed_payload = seatrac_constants.CID_STATUS['response'].parse(payload)

    #     status_response_msg = seatrac_driver.msg.StatusResponse()
    #     status_response_msg.environment_outputted = parsed_payload.STATUS_OUTPUT.ENVIRONMENT
    #     status_response_msg.attitude_outputted = parsed_payload.STATUS_OUTPUT.ATTITUDE
    #     status_response_msg.mag_cal_outputted = parsed_payload.STATUS_OUTPUT.MAG_CAL
    #     status_response_msg.acc_cal_outputted = parsed_payload.STATUS_OUTPUT.ACC_CAL
    #     status_response_msg.ahrs_raw_data_outputted = parsed_payload.STATUS_OUTPUT.AHRS_RAW_DATA
    #     status_response_msg.ahrs_comp_data_outputted = parsed_payload.STATUS_OUTPUT.AHRS_COMP_DATA

    #     if status_response_msg.environment_outputted:
    #         status_response_msg.env_supply = parsed_payload.ENVIRONMENT.ENV_SUPPLY / 1000.0
    #         status_response_msg.env_temperature = parsed_payload.ENVIRONMENT.ENV_TEMP / 10.0
    #         status_response_msg.env_pressure = parsed_payload.ENVIRONMENT.ENV_PRESSURE / 1000.0
    #         status_response_msg.env_depth = parsed_payload.ENVIRONMENT.ENV_DEPTH / 10.0
    #         status_response_msg.env_vos = parsed_payload.ENVIRONMENT.ENV_VOS / 10.0

    #     if status_response_msg.attitude_outputted:
    #         status_response_msg.attitude_yaw = parsed_payload.ATTITUDE.ATT_YAW / 10.0
    #         status_response_msg.attitude_pitch = parsed_payload.ATTITUDE.ATT_PITCH / 10.0
    #         status_response_msg.attitude_roll = parsed_payload.ATTITUDE.ATT_ROLL / 10.0

    #     if status_response_msg.mag_cal_outputted:
    #         status_response_msg.mag_cal_buf = parsed_payload.MAG_CAL.MAG_CAL_BUF
    #         status_response_msg.mag_cal_valid = parsed_payload.MAG_CAL.MAG_CAL_VALID
    #         status_response_msg.mag_cal_age = parsed_payload.MAG_CAL.MAG_CAL_AGE
    #         status_response_msg.mag_cal_fit = parsed_payload.MAG_CAL.MAG_CAL_FIT

    #     if status_response_msg.acc_cal_outputted:
    #         status_response_msg.acceleration_limit_min[0] = parsed_payload.ACC_CAL.ACC_LIM_MIN_X
    #         status_response_msg.acceleration_limit_min[1] = parsed_payload.ACC_CAL.ACC_LIM_MIN_Y
    #         status_response_msg.acceleration_limit_min[2] = parsed_payload.ACC_CAL.ACC_LIM_MIN_Z
    #         status_response_msg.acceleration_limit_max[0] = parsed_payload.ACC_CAL.ACC_LIM_MAX_X
    #         status_response_msg.acceleration_limit_max[1] = parsed_payload.ACC_CAL.ACC_LIM_MAX_Y
    #         status_response_msg.acceleration_limit_max[2] = parsed_payload.ACC_CAL.ACC_LIM_MAX_Z

    #     if status_response_msg.ahrs_raw_data_outputted:
    #         status_response_msg.ahrs_raw_acc[0] = parsed_payload.AHRS_RAW_DATA.AHRS_RAW_ACC_X
    #         status_response_msg.ahrs_raw_acc[1] = parsed_payload.AHRS_RAW_DATA.AHRS_RAW_ACC_Y
    #         status_response_msg.ahrs_raw_acc[2] = parsed_payload.AHRS_RAW_DATA.AHRS_RAW_ACC_Z
    #         status_response_msg.ahrs_raw_mag[0] = parsed_payload.AHRS_RAW_DATA.AHRS_RAW_MAG_X
    #         status_response_msg.ahrs_raw_mag[1] = parsed_payload.AHRS_RAW_DATA.AHRS_RAW_MAG_Y
    #         status_response_msg.ahrs_raw_mag[2] = parsed_payload.AHRS_RAW_DATA.AHRS_RAW_MAG_Z
    #         status_response_msg.ahrs_raw_gyro[0] = parsed_payload.AHRS_RAW_DATA.AHRS_RAW_GYRO_X
    #         status_response_msg.ahrs_raw_gyro[1] = parsed_payload.AHRS_RAW_DATA.AHRS_RAW_GYRO_Y
    #         status_response_msg.ahrs_raw_gyro[2] = parsed_payload.AHRS_RAW_DATA.AHRS_RAW_GYRO_Z

    #     if status_response_msg.ahrs_comp_data_outputted:
    #         status_response_msg.ahrs_comp_acc.x = parsed_payload.AHRS_COMP_DATA.AHRS_COMP_ACC_X
    #         status_response_msg.ahrs_comp_acc.y = parsed_payload.AHRS_COMP_DATA.AHRS_COMP_ACC_Y
    #         status_response_msg.ahrs_comp_acc.z = parsed_payload.AHRS_COMP_DATA.AHRS_COMP_ACC_Z
    #         status_response_msg.ahrs_comp_mag.x = parsed_payload.AHRS_COMP_DATA.AHRS_COMP_MAG_X
    #         status_response_msg.ahrs_comp_mag.y = parsed_payload.AHRS_COMP_DATA.AHRS_COMP_MAG_Y
    #         status_response_msg.ahrs_comp_mag.z = parsed_payload.AHRS_COMP_DATA.AHRS_COMP_MAG_Z
    #         status_response_msg.ahrs_comp_gyro.x = parsed_payload.AHRS_COMP_DATA.AHRS_COMP_GYRO_X
    #         status_response_msg.ahrs_comp_gyro.y = parsed_payload.AHRS_COMP_DATA.AHRS_COMP_GYRO_Y
    #         status_response_msg.ahrs_comp_gyro.z = parsed_payload.AHRS_COMP_DATA.AHRS_COMP_GYRO_Z

    #     self._status_response_publisher.publish(status_response_msg)
        
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

        # Simulated delay
        time.sleep(self._SIM_LOCAL_RESPONSE_DELAY_SEC)

        self._settings_get_response_publisher.publish(self.current_settings_msg)

    def _on_settings_set_command(self, msg):

        # self._packetize(cid, payload)
        self.current_settings_msg.status_mode = msg.status_mode
        self.current_settings_msg.status_output_environment = msg.output_environment
        self.current_settings_msg.status_output_attitude = msg.output_attitude
        self.current_settings_msg.status_output_mag_cal = msg.output_mag_cal
        self.current_settings_msg.status_output_acc_cal = msg.output_acc_cal
        self.current_settings_msg.status_output_ahrs_raw_data = msg.output_ahrs_raw_data
        self.current_settings_msg.status_output_ahrs_comp_data = msg.output_ahrs_comp_data
        self.current_settings_msg.uart_main_baud = msg.uart_main_baud
        self.current_settings_msg.enable_automatic_pressure_offset_cal = msg.enable_automatic_pressure_offset_cal
        self.current_settings_msg.enable_automatic_vos = msg.enable_automatic_vos
        self.current_settings_msg.manual_pressure_offset_bar = msg.manual_pressure_offset_bar
        self.current_settings_msg.salinity_ppt = msg.salinity_ppt
        self.current_settings_msg.manual_vos = msg.manual_vos
        self.current_settings_msg.enable_automatic_magnetometer_cal = msg.enable_automatic_magnetometer_cal
        self.current_settings_msg.ahrs_cal_acc_min_x = msg.ahrs_cal_acc_min_x
        self.current_settings_msg.ahrs_cal_acc_min_y = msg.ahrs_cal_acc_min_y
        self.current_settings_msg.ahrs_cal_acc_min_z = msg.ahrs_cal_acc_min_z
        self.current_settings_msg.ahrs_cal_acc_max_x = msg.ahrs_cal_acc_max_x
        self.current_settings_msg.ahrs_cal_acc_max_y = msg.ahrs_cal_acc_max_y
        self.current_settings_msg.ahrs_cal_acc_max_z = msg.ahrs_cal_acc_max_z
        self.current_settings_msg.ahrs_cal_mag_valid = msg.ahrs_cal_mag_valid
        self.current_settings_msg.ahrs_yaw_offset_deg = msg.ahrs_yaw_offset_deg
        self.current_settings_msg.ahrs_pitch_offset_deg = msg.ahrs_pitch_offset_deg
        self.current_settings_msg.ahrs_roll_offset_deg = msg.ahrs_roll_offset_deg
        self.current_settings_msg.enable_transceiver_diagnostic_messages = msg.enable_transceiver_diagnostic_messages
        self.current_settings_msg.enable_transceiver_fix_messages = msg.enable_transceiver_fix_messages
        self.current_settings_msg.enable_transceiver_usbl_messages = msg.enable_transceiver_usbl_messages
        self.current_settings_msg.enable_transceiver_position_filter = msg.enable_transceiver_position_filter
        self.current_settings_msg.enable_transceiver_automatic_ahrs = msg.enable_transceiver_automatic_ahrs
        self.current_settings_msg.transceiver_beacon_id = msg.transceiver_beacon_id
        self.current_settings_msg.transceiver_timeout_range_m = msg.transceiver_timeout_range_m
        self.current_settings_msg.transceiver_response_time_ms = msg.transceiver_response_time_ms
        self.current_settings_msg.transceiver_manual_ahrs_yaw_deg = msg.transceiver_manual_ahrs_yaw_deg
        self.current_settings_msg.transceiver_manual_ahrs_pitch_deg = msg.transceiver_manual_ahrs_pitch_deg
        self.current_settings_msg.transceiver_manual_ahrs_roll_deg = msg.transceiver_manual_ahrs_roll_deg
        self.current_settings_msg.transceiver_position_filter_max_velocity = msg.transceiver_position_filter_max_velocity
        self.current_settings_msg.transceiver_position_filter_max_angle = msg.transceiver_position_filter_max_angle
        self.current_settings_msg.transceiver_position_filter_reset_timeout = msg.transceiver_position_filter_reset_timeout

        # Simulated delay
        time.sleep(self._SIM_LOCAL_RESPONSE_DELAY_SEC)

        settings_set_response_msg = seatrac_driver.msg.SettingsSetResponse()
        settings_set_response_msg.status = 'CST_OK'
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

        calibration_get_response_msg = seatrac_driver.msg.CalibrationActionResponse()
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

        time.sleep(self._SIM_RESPONSE_DELAY_SEC)

        self._packetize(cid, payload)

        self.ping_sent += 1

        ping_send_response_msg = seatrac_driver.msg.PingSendResponse()
        ping_send_response_msg.status = 'CST_OK'
        ping_send_response_msg.beacon_id = msg.destination_id
        self._ping_send_response_publisher.publish(ping_send_response_msg)

    def _on_ping_resp_status(self, payload, msg):
        parsed_payload = seatrac_constants.CID_PING_RESP['status'].parse(payload)

        self.ping_received += 1

        ping_response_status_msg = seatrac_driver.msg.PingResponseStatus()
        self._compute_aco_fix(ping_response_status_msg.fix, parsed_payload.ACO_FIX, msg.source_namespace)
        self._ping_response_status_publisher.publish(ping_response_status_msg)

    
    def _compute_aco_fix(self, aco_fix_out, aco_fix_in, source_namespace):
        aco_fix_out.destination_id = aco_fix_in.DEST_ID
        aco_fix_out.source_id = aco_fix_in.SRC_ID
        aco_fix_out.msg_type = aco_fix_in.MSG_TYPE

        aco_fix_out.vos = self._simulated_water_vos
        # TBD: How do we simulate these?
        aco_fix_out.rssi = aco_fix_in.RSSI / 10.0

        # Get our latest truth pose.
        pose = self._get_pose()
        # try:
        #     self_pose_gt = r ospy.wait_for_message(os.path.join(self._namespace,'pose_gt'),
        #         nav_msgs.msg.Odometry, timeout=1.0)
        # except TimeoutError as e:
        #     self.get_logger().warn('{e}')
        #     self_pose_gt = None


        # If this is a ping response to this beacon node, the range will be valid.
        if int(aco_fix_out.destination_id) == self.current_settings_msg.transceiver_beacon_id:
            aco_fix_out.range_valid = True
        
            if self._simulated_modem_type == 'x150':
                aco_fix_out.usbl_valid = True
                aco_fix_out.position_valid = True
                aco_fix_out.position_enhanced = True
            
            # Get our latest truth pose for the beacon that echoed the ping.
            try:
                target_pose_gt = self.wait_for_message(os.path.join(source_namespace,'pose_gt'),
                    nav_msgs.msg.Odometry, timeout=1.0)
            except TimeoutError as e:
                self.get_logger().warn('{e}')
                target_pose_gt = None


            if pose:

                # Convert to degrees and divide by ten
                aco_fix_out.attitude_roll = math.degrees(pose['roll'])
                aco_fix_out.attitude_pitch = math.degrees(pose['pitch'])
                aco_fix_out.attitude_yaw = math.degrees(pose['yaw'])
                aco_fix_out.depth_local = pose['z']

                # self_heading_world = np.array([
                #     math.cos(pose['yaw']) * math.cos(pose['pitch']),
                #     math.sin(pose['yaw']) * math.cos(pose['pitch']),
                #     math.sin(pose['pitch'])
                # ])


                if target_pose_gt:
                    range_vector = [
                        target_pose_gt.pose.pose.position.x - pose['x'],
                        target_pose_gt.pose.pose.position.y - pose['y'],
                        target_pose_gt.pose.pose.position.z - pose['z']
                    ]
                    range_vector = np.array(range_vector)
                    range_to_target = np.linalg.norm(range_vector)
                    # unit_range_vector = range_vector / range_to_target

                    # Compute the yaw angle of target with respect to our position.
                    target_yaw_world = math.atan2(range_vector[1], range_vector[0])

                    # Compute the target's inertial yaw by subtracting from our yaw.
                    # target_yaw_inertial = pose['yaw'] - target_yaw_world

                    if aco_fix_out.range_valid:
                        aco_fix_out.range_count = 0
                        aco_fix_out.range_time = 0
                        aco_fix_out.range_dist = range_to_target
                    else:
                        aco_fix_out.range_count = 0.0
                        aco_fix_out.range_time = 0.0
                        aco_fix_out.range_dist = 0.0

                    if aco_fix_out.position_valid:
                        aco_fix_out.position_easting = target_pose_gt.pose.pose.position.x
                        aco_fix_out.position_northing = target_pose_gt.pose.pose.position.y
                        aco_fix_out.position_depth = target_pose_gt.pose.pose.position.z
                    else:
                        aco_fix_out.position_easting = 0.0
                        aco_fix_out.position_northing = 0.0
                        aco_fix_out.position_depth = 0.0

                    # USBL channels and per-channel RSSI intentionally skipped (unused)
                    if aco_fix_out.usbl_valid:
                        aco_fix_out.usbl_azimuth = math.degrees(target_yaw_world)
                        aco_fix_out.usbl_elevation = 0.0
                        aco_fix_out.usbl_fit_error = 1.0
                    else:
                        aco_fix_out.usbl_azimuth = 0.0
                        aco_fix_out.usbl_elevation = 0.0
                        aco_fix_out.usbl_fit_error = 0.0


    def _on_ping_error_status(self, payload):
        parsed_payload = seatrac_constants.CID_PING_ERROR['status'].parse(payload)

        ping_error_status_msg = seatrac_driver.msg.PingErrorStatus()
        ping_error_status_msg.status = parsed_payload.STATUS
        ping_error_status_msg.beacon_id = parsed_payload.BEACON_ID
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
        navigation_query_send_response_msg.destination_id = parsed_payload.DEST_ID
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
        navigation_query_response_status_msg.remote_depth = parsed_payload.REMOTE_DEPTH / 10.0
        navigation_query_response_status_msg.remote_supply = parsed_payload.REMOTE_SUPPLY / 1000.0
        navigation_query_response_status_msg.remote_temp = parsed_payload.REMOTE_TEMP / 10.0
        navigation_query_response_status_msg.remote_yaw = parsed_payload.REMOTE_YAW / 10.0
        navigation_query_response_status_msg.remote_pitch = parsed_payload.REMOTE_PITCH / 10.0
        navigation_query_response_status_msg.remote_roll = parsed_payload.REMOTE_ROLL / 10.0
        navigation_query_response_status_msg.packet_length = parsed_payload.PACKET_LEN
        navigation_query_response_status_msg.packet_data = parsed_payload.PACKET_DATA
        navigation_query_response_status_msg.local_flag = parsed_payload.LOCAL_FLAG
        self._navigation_query_response_status_publisher.publish(navigation_query_response_status_msg)

    def _on_nav_error_status(self, payload):
        parsed_payload = seatrac_constants.CID_NAV_ERROR['status'].parse(payload)

        navigation_error_status_msg = seatrac_driver.msg.NavigationErrorStatus()
        navigation_error_status_msg.status = parsed_payload.STATUS
        navigation_error_status_msg.beacon_id = parsed_payload.BEACON_ID
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
        navigation_status_send_response_msg.beacon_id = parsed_payload.BEACON_ID
        self._navigation_status_send_response_publisher.publish(navigation_status_send_response_msg)

    def _on_nav_status_receive_status(self, payload):
        parsed_payload = seatrac_constants.CID_NAV_STATUS_RECEIVE['status'].parse(payload)

        navigation_status_receive_status_msg = seatrac_driver.msg.NavigationStatusReceiveStatus()
        self._populate_acofix_structure(navigation_status_receive_status_msg.fix, parsed_payload.ACO_FIX)
        navigation_status_receive_status_msg.beacon_id = parsed_payload.BEACON_ID
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

        # Simulated delay
        time.sleep(self._SIM_RESPONSE_DELAY_SEC)

        self._packetize(cid, payload)

        self.data_sent += 1

        # Put the message into the stream as a response.
        data_send_response_msg = seatrac_driver.msg.DataSendResponse()
        data_send_response_msg.status = 'CST_OK'
        data_send_response_msg.beacon_id =self.get_parameter('asset_id').value 
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
        data_error_status_msg.beacon_id = parsed_payload.BEACON_ID
        self._data_error_status_publisher.publish(data_error_status_msg)

    def _populate_acofix_structure(self, ros_component, construct_component):
        ros_component.destination_id = construct_component.DEST_ID
        ros_component.source_id = construct_component.SRC_ID
        ros_component.range_valid = construct_component.FLAGS.RANGE_VALID
        ros_component.usbl_valid = construct_component.FLAGS.USBL_VALID
        ros_component.position_valid = construct_component.FLAGS.POSITION_VALID
        ros_component.position_enhanced = construct_component.FLAGS.POSITION_ENHANCED
        ros_component.position_filter_error = construct_component.FLAGS.POSITION_FLT_ERROR
        ros_component.msg_type = construct_component.MSG_TYPE
        ros_component.attitude_yaw = construct_component.ATTITUDE_YAW / 10.0
        ros_component.attitude_pitch = construct_component.ATTITUDE_PITCH / 10.0
        ros_component.attitude_roll = construct_component.ATTITUDE_ROLL / 10.0
        ros_component.depth_local = construct_component.DEPTH_LOCAL / 10.0
        ros_component.vos = construct_component.VOS / 10.0
        ros_component.rssi = construct_component.RSSI / 10.0
        ros_component.range_count = construct_component.RANGE.RANGE_COUNT
        ros_component.range_time = construct_component.RANGE.RANGE_TIME / 1.0e7
        ros_component.range_dist = construct_component.RANGE.RANGE_DIST / 10.0
        # USBL channels and per-channel RSSI intentionally skipped (unused)
        ros_component.usbl_azimuth = construct_component.USBL.USBL_AZIMUTH / 10.0
        ros_component.usbl_elevation = construct_component.USBL.USBL_ELEVATION / 10.0
        ros_component.usbl_fit_error = construct_component.USBL.USBL_FIT_ERROR / 100.0
        ros_component.position_easting = construct_component.POSITION.POSITION_EASTING / 10.0
        ros_component.position_northing = construct_component.POSITION.POSITION_NORTHING / 10.0
        ros_component.position_depth = construct_component.POSITION.POSITION_DEPTH / 10.0

    def _packetize(self, command_id, payload=None):
        # Construct the start of the command message
        data = struct.pack('<B', command_id)

        if payload is not None:
            data += payload

        serial_internal_msg = seatrac_driver.msg.SerialInternal()
        serial_internal_msg.source_namespace = self.get_namespace()
        serial_internal_msg.source_asset_id = self.asset_id
        serial_internal_msg.source_beacon_id = self.current_settings_msg.transceiver_beacon_id
        serial_internal_msg.send_time = self.get_clock().now()
        serial_internal_msg.data = data
        self._internal_serial_data_publisher.publish(serial_internal_msg)


    def _checksum(self, data):
        # Computes the CRC16 checksum of the data provided. For SeaTrac data, the checksum should
        # compute only the command ID plus the payload data. The checksum bytes and end sync bytes
        # should not be included
        poly = np.uint16(0xA001)
        crc = np.uint16(0)

        for b in bytearray(data):
            for _ in range(8):
                if (b & 0x01) ^ (crc & 0x01):
                    crc = (crc >> 1) ^ poly
                else:
                    crc >>= 1

                b >>= 1

        return np.uint16(crc)

def main():
    # --- ROS2 style init / spin / shutdown ---
    rclpy.init(args=sys.argv)
    node = SeaTracDriverStub()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down SeaTrac Driver')
        node.destroy_node()
        rclpy.shutdown()   

if __name__ == '__main__':
    main()
