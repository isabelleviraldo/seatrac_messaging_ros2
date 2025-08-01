from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Declare all arguments
    declared_args = [
        DeclareLaunchArgument('rate', default_value='20.0'),
        DeclareLaunchArgument('device', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('sim', default_value='false'),

        DeclareLaunchArgument('sim_comms_drop', default_value='false'),
        DeclareLaunchArgument('sim_tx_rate', default_value='false'),
        DeclareLaunchArgument('sim_water_baud', default_value='100'),
        DeclareLaunchArgument('sim_modem_type', default_value='x010'),
        DeclareLaunchArgument('sim_water_vos', default_value='1518'),

        DeclareLaunchArgument('sim_pose_src', default_value='pose_gt'),
        DeclareLaunchArgument('beacon_pose_ned', default_value='[]'),
        DeclareLaunchArgument('beacon_pose_pitch', default_value='0.0'),
        DeclareLaunchArgument('beacon_pose_roll', default_value='0.0'),
    ]

    # Normal driver node
    real_driver_node = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('sim')),
        actions=[
            Node(
                package='seatrac_driver',
                executable='seatrac_driver_node.py',
                name='seatrac',
                output='screen',
                parameters=[{
                    'rate': LaunchConfiguration('rate'),
                    'device': LaunchConfiguration('device'),
                    'baudrate': LaunchConfiguration('baudrate'),
                }]
            )
        ]
    )

    # Simulation driver node
    sim_driver_node = GroupAction(
        condition=IfCondition(LaunchConfiguration('sim')),
        actions=[
            Node(
                package='seatrac_driver',
                executable='seatrac_driver_stub_node.py',
                name='seatrac',
                output='screen',
                parameters=[{
                    'rate': LaunchConfiguration('rate'),
                    'device': LaunchConfiguration('device'),
                    'baudrate': LaunchConfiguration('baudrate'),
                    'sim_comms_drop': LaunchConfiguration('sim_comms_drop'),
                    'sim_tx_rate': LaunchConfiguration('sim_tx_rate'),
                    'sim_water_baud': LaunchConfiguration('sim_water_baud'),
                    'sim_modem_type': LaunchConfiguration('sim_modem_type'),
                    'sim_water_vos': LaunchConfiguration('sim_water_vos'),
                    'sim_pose_src': LaunchConfiguration('sim_pose_src'),
                    'beacon_pose_ned': LaunchConfiguration('beacon_pose_ned'),
                    'beacon_pose_pitch': LaunchConfiguration('beacon_pose_pitch'),
                    'beacon_pose_roll': LaunchConfiguration('beacon_pose_roll'),
                }]
            )
        ]
    )

    return LaunchDescription(declared_args + [real_driver_node, sim_driver_node])
