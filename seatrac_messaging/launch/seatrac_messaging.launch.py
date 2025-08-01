from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # Launch arguments
    self_beacon_id_arg = DeclareLaunchArgument(
        'self_beacon_id', default_value='15'
    )
    beacon_destination_id_arg = DeclareLaunchArgument(
        'beacon_destination_id', default_value='2'
    )

    # Path to the driver’s launch file
    seatrac_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('seatrac_driver'),
                'launch',
                'seatrac_driver.launch.py'
            )
        ),
        launch_arguments={
            'rate': '20.0',
            'device': '/dev/ttyUSB0',
            'baudrate': '115200'
        }.items()
    )

    # Messaging node
    messaging_node = Node(
        package='seatrac_messaging',
        executable='seatrac_messaging_handler_main.py',
        name='SeaTrac_Messaging_Handler',
        output='screen',
        parameters=[
            {'self_beacon_id': LaunchConfiguration('self_beacon_id')},
            {'beacon_destination_id': LaunchConfiguration('beacon_destination_id')}
        ]
    )

    return LaunchDescription([
        self_beacon_id_arg,
        beacon_destination_id_arg,
        GroupAction([
            seatrac_driver_launch,
            messaging_node
        ])
    ])
