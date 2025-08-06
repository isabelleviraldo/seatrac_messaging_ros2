#!/usr/bin/env python3
import numpy
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import transforms3d.euler
import transforms3d.quaternions
import sys
import time
import struct
import socket

import os
import threading
from threading import Event

#ros messages
import std_msgs.msg
from std_msgs.msg import String
import geometry_msgs.msg
import sensor_msgs.msg
import seatrac_driver.msg

# my files located within this filespace
from vigenere_cipher import v_encrypt, v_decrypt

_DEFAULT_CONTROL_RATE = 20.0  # [Hz]
_ENCRYPTION_KEY = 'OEWPNDYTRLNSDFHBOW' # Change for your code

class SeaTracMessagingHandler(Node):

    def __init__(self):
        
        super().__init__('SeaTrac_Messaging_Handler')

        # initilize parameters
        self._init_parameters()

        # initilize ros topic subs and pubs
        self._init_ros_communication()

    def _init_parameters(self):
        # Handle parameters

        #reentrant callback group
        self._callback_group = ReentrantCallbackGroup()

        #managing the temporary callback groups within set_settings
        self._temporary_cb_group = ReentrantCallbackGroup()
        self._temporary_msg_buffers = {}
        self._temporary_subs = {}

        # make sure only one instance of run is running at a time
        self._run_sender_lock = threading.Lock()

        # make sure that it sends to the correct id
        self.declare_parameter('self_beacon_id', 0)
        self._asset_id = self.get_parameter('self_beacon_id').get_parameter_value().integer_value

        self.declare_parameter('beacon_destination_id', 0)
        self._beacon_destination_id = self.get_parameter('beacon_destination_id').get_parameter_value().integer_value

        self.rate_hz = _DEFAULT_CONTROL_RATE

        # Handle state
        self._beacon_connected = False
        self._beacon_packets_recvd = 0

        #DATA command
        self._data_in_progress = False

        # GPS and positioning 
        self._remote_beacon_northing = 0 # Destination beacon's northing heading relative to the topside node
        self._remote_beacon_easting = 0 # Destination beacon's easting heading relative to the topside node
        self._node_trigger_range = 10 # Range in meters from threat to node that will cause the node to activate
        self._compass_heading = 0 # Topside node heading. 0 is due north


    def _init_ros_communication(self):
        # The following are ROS message structures either sent or received by the SeaTrac data
        # interface. These messages are translated from their expanded ROS formats to their
        # packed SeaTrac formats for communication of specific information
        
        # Send Message Publisher
        self._beacon_data_send_publisher = self.create_publisher(seatrac_driver.msg.DataSendCommand, 
                                                                 '/seatrac/data_send_command',
                                                                 10,
                                                                 callback_group=self._callback_group)

        # Handle SeaTrac messages
        self._driver_status = self.create_subscription(seatrac_driver.msg.DriverStatus, 
                                                    '/seatrac/driver_status',
                                                    self._on_driver_status,
                                                    10,
                                                    callback_group=self._callback_group)
        self._beacon_status = self.create_subscription(seatrac_driver.msg.StatusResponse, 
                                                    '/seatrac/status_response',
                                                    self._on_beacon_status,
                                                    10,
                                                    callback_group=self._callback_group)

        # Get and Set Seatrac Settings
        self._beacon_settings_get_command_publisher = self.create_publisher(std_msgs.msg.Empty, 
                                                                            '/seatrac/settings_get_command',
                                                                            10,
                                                                            callback_group=self._callback_group)
        self._beacon_settings_set_command_publisher = self.create_publisher(seatrac_driver.msg.SettingsSetCommand,
                                                                            '/seatrac/settings_set_command',
                                                                            10,
                                                                            callback_group=self._callback_group)

        # The following are ROS message structures either sent or received by the SeaTrac data
        # interface. These messages are translated from their expanded ROS formats to their
        # packed SeaTrac formats for fleet communication of specific information

        # message to send out subscriber (this can be published to by any code that wants to send a message) (also named so it makes more sense for the user)
        self._data_to_send_subscriber = self.create_subscription(std_msgs.msg.String,
                                                    '/seatrac/messaging/send_data', 
                                                    self.run_sender,
                                                    10,
                                                    callback_group=self._callback_group)

        # publish received data to rostopic (this is named so subscribing to it makes more sense)
        self._data_recieved_publisher = self.create_publisher(std_msgs.msg.String,
                                                              '/seatrac/messaging/incoming_data',
                                                              10,
                                                              callback_group=self._callback_group)
        # DATA Command
        self._data_receive_status_subscriber = self.create_subscription(seatrac_driver.msg.DataReceiveStatus,
                                                    '/seatrac/data_receive_status',
                                                    self._on_data_receive_status,
                                                    10,
                                                    callback_group=self._callback_group)

    def _set_beacon_settings(self, salinity_ppt=None, yaw_offset=None, response_time_ms=None):
        settings_received = False
        settings_max_attempts = 5
        settings_current_attempts = 0

        while not settings_received:
            # Request the beacon settings and collect the settings response
            self._beacon_settings_get_command_publisher.publish(std_msgs.msg.Empty())
            settings_current_attempts += 1

            try:
                old = self.wait_for_message('/seatrac/settings_get_response',
                                             seatrac_driver.msg.SettingsGetResponse, timeout_sec=1.0)
                if old is None:
                    if settings_current_attempts > settings_max_attempts:
                        self.get_logger().error('Max attempts exceeded. Aborting.')
                        return False
                    continue
            except TimeoutError as e:
                self.get_logger().warning(f'{e}')

                if settings_current_attempts > settings_max_attempts:
                    self.get_logger().error('Maximum num of attempts to get beacon settings was exceeded')

                    return False

                continue

            settings_received = True

        self.get_logger().info(f'Received beacon settings after {settings_current_attempts} attempts')

        # Begin to define the new settings parameters
        new = seatrac_driver.msg.SettingsSetCommand()
        new.status_mode = seatrac_driver.msg.SettingsSetCommand.STATUS_MODE_25HZ
        new.output_environment = True
        new.output_attitude = True
        new.output_mag_cal = True
        new.output_acc_cal = True
        new.output_ahrs_raw_data = False
        new.output_ahrs_comp_data = False
        new.uart_main_baud = old.uart_main_baud
        new.enable_automatic_pressure_offset_cal = old.enable_automatic_pressure_offset_cal
        new.enable_automatic_vos = old.enable_automatic_vos
        new.manual_pressure_offset_bar = old.manual_pressure_offset_bar

        # If a salinity environmental value is given then use it
        if salinity_ppt is None:
            new.salinity_ppt = old.salinity_ppt
        else:
            new.salinity_ppt = salinity_ppt

        new.manual_vos = old.manual_vos
        new.enable_automatic_magnetometer_cal = old.enable_automatic_magnetometer_cal
        new.ahrs_cal_acc_min_x = old.ahrs_cal_acc_min_x
        new.ahrs_cal_acc_min_y = old.ahrs_cal_acc_min_y
        new.ahrs_cal_acc_min_z = old.ahrs_cal_acc_min_z
        new.ahrs_cal_acc_max_x = old.ahrs_cal_acc_max_x
        new.ahrs_cal_acc_max_y = old.ahrs_cal_acc_max_y
        new.ahrs_cal_acc_max_z = old.ahrs_cal_acc_max_z
        new.ahrs_cal_mag_valid = old.ahrs_cal_mag_valid
        new.ahrs_cal_mag_hard_x = old.ahrs_cal_mag_hard_x
        new.ahrs_cal_mag_hard_y = old.ahrs_cal_mag_hard_y
        new.ahrs_cal_mag_hard_z = old.ahrs_cal_mag_hard_z
        new.ahrs_cal_mag_soft_x = old.ahrs_cal_mag_soft_x
        new.ahrs_cal_mag_soft_y = old.ahrs_cal_mag_soft_y
        new.ahrs_cal_mag_soft_z = old.ahrs_cal_mag_soft_z
        new.ahrs_cal_mag_field = old.ahrs_cal_mag_field
        new.ahrs_cal_mag_error = old.ahrs_cal_mag_error
        new.ahrs_cal_gyro_offset_x = old.ahrs_cal_gyro_offset_x
        new.ahrs_cal_gyro_offset_y = old.ahrs_cal_gyro_offset_y
        new.ahrs_cal_gyro_offset_z = old.ahrs_cal_gyro_offset_z

        if yaw_offset is None:
            new.ahrs_yaw_offset_deg = old.ahrs_yaw_offset_deg
        else:
            new.ahrs_yaw_offset_deg = yaw_offset

        new.ahrs_pitch_offset_deg = old.ahrs_pitch_offset_deg
        new.ahrs_roll_offset_deg = old.ahrs_roll_offset_deg
        new.enable_transceiver_diagnostic_messages = old.enable_transceiver_diagnostic_messages
        new.enable_transceiver_fix_messages = old.enable_transceiver_fix_messages
        new.enable_transceiver_usbl_messages = old.enable_transceiver_usbl_messages
        new.enable_transceiver_position_filter = old.enable_transceiver_position_filter
        new.enable_transceiver_automatic_ahrs = old.enable_transceiver_automatic_ahrs
        new.transceiver_beacon_id = self._asset_id
        new.transceiver_timeout_range_m = old.transceiver_timeout_range_m

        # If a response time value is given then use it
        if response_time_ms is None:
            new.transceiver_response_time_ms = old.transceiver_response_time_ms
        else:
            new.transceiver_response_time_ms = response_time_ms

        new.transceiver_manual_ahrs_yaw_deg = old.transceiver_manual_ahrs_yaw_deg
        new.transceiver_manual_ahrs_pitch_deg = old.transceiver_manual_ahrs_pitch_deg
        new.transceiver_manual_ahrs_roll_deg = old.transceiver_manual_ahrs_roll_deg
        new.transceiver_position_filter_max_velocity = old.transceiver_position_filter_max_velocity
        new.transceiver_position_filter_max_angle = old.transceiver_position_filter_max_angle
        new.transceiver_position_filter_reset_timeout = old.transceiver_position_filter_reset_timeout

        # Set new settings into SeaTrac device RAM
        settings_set = False
        settings_max_attempts = 5
        settings_current_attempts = 0

        while not settings_set:
            # Request the beacon settings and collect the settings response
            self._beacon_settings_set_command_publisher.publish(new)
            settings_current_attempts += 1

            try:
                settings_set_response = self.wait_for_message('/seatrac/settings_set_response',
                                                               seatrac_driver.msg.SettingsSetResponse, timeout_sec=1.0)
                
                if settings_set_response is None:
                    self.get_logger().warning("No settings_set_response received; possibly shutting down.")
                    if settings_current_attempts > settings_max_attempts:
                        self.get_logger().error('Max attempts exceeded. Aborting.')
                        return False
                    continue
            except TimeoutError as e:
                self.get_logger().warning(f'{e}')
                if settings_current_attempts > settings_max_attempts:
                    self.get_logger().error('Maximum num of attempts to get beacon settings was exceeded')
                    return False
                continue

            settings_set = True
        self.get_logger().info(f'Set new beacon settings after {settings_current_attempts} attempts')
        success = (settings_set_response.status == 'CST_OK')

        if not success:
            self.get_logger().warning(f'Settings response: {settings_set_response.status}')

        return success

    def run_sender(self, data):
        if not self._run_sender_lock.acquire(blocking=False):
            self._skipped_sends += 1
            self.get_logger().warning(f"run_sender() already running, skipping send. Total skipped: {self._skipped_sends}")
            return

        self.get_logger().info("Starting run_sender")

        try:
            r = self.create_rate(self.rate_hz)

            while (not self._beacon_connected) or (self._beacon_packets_recvd == 0):
                if not rclpy.ok():
                    return
                r.sleep()

            success = self._set_beacon_settings()
            if not success:
                self.get_logger().error('Unable to set beacon settings. Aborting send...')
                return

            # Track time for watchdog
            self._data_send_start_time = time.time()

            self.execute(data.data)

        finally:
            self._run_sender_lock.release()

    # Execute is plan that can be loaded
    def execute(self, data):
        # For more info on the differend data types go to page 136 on:
        # https://www.blueprintsubsea.com/downloads/seatrac/UM-140-D00221-13.pdf
        msg_type = "OWAY" #one way message, no range/position info
        # msg_type = "OWAYU" #one way message, angle info available, no range/position info
        # msg_type = "REQ" #message with response, range info available
        # msg_type = "REQU" #message with response, position and range info available
        # msg_type = "REQX" #message with response (enhanced?), position and range info available
        
        # manage data (time, type, etc)
        data_format = self._data_handler(data_type=msg_type, beacon=self._beacon_destination_id,data=data) # data type, one way comm, beacon is needed or breaks, the for loop is for the sequence of commands
        
        # encrypt data
        e_data = self.encrypt_data(data, data_format)

        # send data
        self.send_data(e_data)
        
        time.sleep(1) # after sending data, wait 6 seconds. # will break if not here

    ####how it sends to sends to send if i want to see the path
    def _data_handler(self, data_type, beacon, data, delay=None):

        timeout = time.time() + 10
        if delay is True: #this is so that the test runs after a second to keep timeout as a minimum impact.
            timeout = time.time() + 1
        while self._data_in_progress == True:
            if time.time() > timeout:
                self._data_in_progress = False
                break
            time.sleep(1)

        if not self._data_in_progress:

            data_send_command_msg = seatrac_driver.msg.DataSendCommand()
            data_send_command_msg.destination_id = beacon

            # Set the data message type according to SeaTrac's documentation
            # For more info on the differend data types go to page 136 on:
            # https://www.blueprintsubsea.com/downloads/seatrac/UM-140-D00221-13.pdf
            if data_type == "OWAY":
                data_send_command_msg.msg_type = data_send_command_msg.MSG_TYPE_OWAY
            elif data_type == "OWAYU":
                data_send_command_msg.msg_type = data_send_command_msg.MSG_TYPE_OWAYU
            elif data_type == "REQ":
                data_send_command_msg.msg_type = data_send_command_msg.MSG_TYPE_REQ
            elif data_type == "REQU":
                data_send_command_msg.msg_type = data_send_command_msg.MSG_TYPE_REQU
            elif data_type == "REQX":
                data_send_command_msg.msg_type = data_send_command_msg.MSG_TYPE_REQX
            
            # data_send_command_msg.packet_data = list(numpy.array(data, dtype=numpy.uint8))
            #data_send_command_msg.packet_data = list(bytearray(data, encoding='utf-8'))
            
            return data_send_command_msg

    def encrypt_data(self, data, data_format):
        data_send_command_msg = data_format
    
        # encrypt the data
        encrypted_data = v_encrypt(data, _ENCRYPTION_KEY)
    
        # co nvert encrypted string to list of byte values
        cipher_int = [ord(c) for c in encrypted_data]
        data_send_command_msg.packet_data = cipher_int
    
        return data_send_command_msg
    
    def send_data(self, e_data):
        self._beacon_data_send_publisher.publish(e_data)
        self.get_logger().info('Data Sent')
        self._data_in_progress = True

    ### SUBSCRIBER CALLBACKS

    # _driver_status subscriber
    def _on_driver_status(self, data):
        self._beacon_connected = data.device_connected
        self._beacon_packets_recvd = data.parsed_packet_count

    # _beacon_status subscriber
    def _on_beacon_status(self, data):
        # The beacon status is received at a rate defined on `fleet_communication` node
        # initialization. Useful data includes accurate depth, temperature, supply voltage, and
        # AHRS roll/pitch/yaw converted into body orientation w.r.t. ENU coordinate frame.

        # Get the orientation from world NED to body FRD
        roll = numpy.deg2rad(data.attitude_roll)
        pitch = numpy.deg2rad(data.attitude_pitch)
        yaw = numpy.deg2rad(data.attitude_yaw)

        # Note: returns [w, x, y, z]
        q_wxyz = transforms3d.euler.euler2quat(roll, pitch, yaw, axes='sxyz')

        # Reorder to [x, y, z, w] as ROS expects
        q_rotation_ned_to_frd = [q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]]

        # Define the rotation from world NED to world ESD
        q_rotation_esd_to_ned = [0.0, 0.0, -0.707, 0.707]  # [x, y, z, w]

        # Reorder to [w, x, y, z] to multiply
        q1 = [q_rotation_esd_to_ned[3], q_rotation_esd_to_ned[0],
              q_rotation_esd_to_ned[1], q_rotation_esd_to_ned[2]]
        q2 = [q_rotation_ned_to_frd[3], q_rotation_ned_to_frd[0],
              q_rotation_ned_to_frd[1], q_rotation_ned_to_frd[2]]

        q_result_wxyz = transforms3d.quaternions.qmult(q1, q2)

        # Reorder back to [x, y, z, w] for ROS
        q_rotation_esd_to_frd = [q_result_wxyz[1], q_result_wxyz[2],
                                 q_result_wxyz[3], q_result_wxyz[0]]


        # Determine the rotation from world ENU to body FLU (ROS standard)
        q_rotation_enu_to_flu = [
            q_rotation_esd_to_frd[0],
            -q_rotation_esd_to_frd[1],
            -q_rotation_esd_to_frd[2],
            q_rotation_esd_to_frd[3],
        ]

        # Generate and publish the beacon orientation
        beacon_imu_msg = sensor_msgs.msg.Imu()
        beacon_imu_msg.header.stamp = self.get_clock().now().to_msg()
        beacon_imu_msg.header.frame_id = 'base_link'
        beacon_imu_msg.orientation.x = q_rotation_enu_to_flu[0]
        beacon_imu_msg.orientation.y = q_rotation_enu_to_flu[1]
        beacon_imu_msg.orientation.z = q_rotation_enu_to_flu[2]
        beacon_imu_msg.orientation.w = q_rotation_enu_to_flu[3]
        beacon_imu_msg.orientation_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1,
        ]
        # Not populated (signified by -1 in first covariance element)
        beacon_imu_msg.angular_velocity.x = 0.0
        beacon_imu_msg.angular_velocity.y = 0.0
        beacon_imu_msg.angular_velocity.z = 0.0
        beacon_imu_msg.angular_velocity_covariance = [
            -1.0, 0.0, 0.0,
            0.0, -1.0, 0.0,
            0.0, 0.0, -1.0,
        ]
        beacon_imu_msg.linear_acceleration.x = 0.0
        beacon_imu_msg.linear_acceleration.y = 0.0
        beacon_imu_msg.linear_acceleration.z = 0.0
        beacon_imu_msg.linear_acceleration_covariance = [
            -1.0, 0.0, 0.0,
            0.0, -1.0, 0.0,
            0.0, 0.0, -1.0,
        ]
        # self._beacon_imu_publisher.publish(beacon_imu_msg)

        beacon_depth_msg = geometry_msgs.msg.PoseWithCovarianceStamped()
        beacon_depth_msg.header.stamp = self.get_clock().now().to_msg()
        beacon_depth_msg.header.frame_id = 'map'
        beacon_depth_msg.pose.pose.position.x = 0.0 # (Unpopulated)
        beacon_depth_msg.pose.pose.position.y = 0.0 # (Unpopulated)
        beacon_depth_msg.pose.pose.position.z = -data.env_depth
        beacon_depth_msg.pose.pose.orientation.x = q_rotation_enu_to_flu[0]
        beacon_depth_msg.pose.pose.orientation.y = q_rotation_enu_to_flu[1]
        beacon_depth_msg.pose.pose.orientation.z = q_rotation_enu_to_flu[2]
        beacon_depth_msg.pose.pose.orientation.w = q_rotation_enu_to_flu[3]
        beacon_depth_msg.pose.covariance = [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
        ]
        # self._beacon_depth_publisher.publish(beacon_depth_msg)

    # _data_receive_status subscriber
    def _on_data_receive_status(self, data):
        decrypted_msg = String()

        i = 0
        format_string = ""
        while i < data.packet_length:
            format_string += "b"
            i += 1
        data_received = list(struct.unpack(format_string, data.packet_data))

        decrypted_data = v_decrypt(data.packet_data, _ENCRYPTION_KEY)
        decrypted_msg.data = decrypted_data
        self._data_recieved_publisher.publish(decrypted_msg) # HERE IS THE UNENCRYPTED DATA, publishing it

        self._data_in_progress = False
        self._data_send_start_time = None
        self._last_receive_time = time.time()
        self.get_logger().info("Data received and processed. Resetting _data_in_progress.")
        



    def wait_for_message(self, topic, msg_type, timeout_sec=1.0, max_attempts=5):
        if topic not in self._temporary_subs:
            self._temporary_msg_buffers[topic] = None

            def _cb(msg, topic=topic):
                # Overwrite with latest only
                self._temporary_msg_buffers[topic] = msg

            self._temporary_subs[topic] = self.create_subscription(
                msg_type,
                topic,
                _cb,
                10,
                callback_group=self._temporary_cb_group,
            )

        self._temporary_msg_buffers[topic] = None
        last_seen = None

        for attempt in range(max_attempts):
            start = time.time()
            while time.time() - start < timeout_sec and rclpy.ok():
                if self._temporary_msg_buffers[topic] is not None:
                    last_seen = self._temporary_msg_buffers[topic]
                time.sleep(0.05)

            if last_seen is not None:
                return last_seen

        return None  # no valid message after all attempts

def main(args=None):
    rclpy.init(args=args)
    node = SeaTracMessagingHandler()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()  # spin forever
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C received. Exiting spin.")
    except Exception as e:
        node.get_logger().error(f"Unhandled exception: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()




if __name__ == '__main__':
    main()

