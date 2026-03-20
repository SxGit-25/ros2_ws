from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
            DeclareLaunchArgument('baudrate', default_value='500000'),
            DeclareLaunchArgument('timeout', default_value='0.01'),
            DeclareLaunchArgument('frame_info_enable', default_value='true'),
            DeclareLaunchArgument('cmd_resend_hz', default_value='30.0'),
            DeclareLaunchArgument('cmd_timeout_sec', default_value='0.5'),
            DeclareLaunchArgument('send_debug_enable', default_value='false'),
            Node(
                package='ano_receiver_bridge',
                executable='serial_node',
                name='ano_receiver_node',
                output='screen',
                parameters=[
                    {
                        'port': LaunchConfiguration('port'),
                        'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int),
                        'timeout': ParameterValue(LaunchConfiguration('timeout'), value_type=float),
                        'frame_info_enable': ParameterValue(
                            LaunchConfiguration('frame_info_enable'),
                            value_type=bool,
                        ),
                        'cmd_resend_hz': ParameterValue(
                            LaunchConfiguration('cmd_resend_hz'),
                            value_type=float,
                        ),
                        'cmd_timeout_sec': ParameterValue(
                            LaunchConfiguration('cmd_timeout_sec'),
                            value_type=float,
                        ),
                        'send_debug_enable': ParameterValue(
                            LaunchConfiguration('send_debug_enable'),
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
