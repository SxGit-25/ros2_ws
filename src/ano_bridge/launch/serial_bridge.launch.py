from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyUSB0')
    baudrate_arg = DeclareLaunchArgument('baudrate', default_value='500000')
    address_arg = DeclareLaunchArgument('address', default_value='255')
    frame_id_arg = DeclareLaunchArgument('frame_id', default_value='ano_link')
    resend_hz_arg = DeclareLaunchArgument('command_resend_hz', default_value='30.0')

    serial_node = Node(
        package='ano_bridge',
        executable='serial_node',
        name='ano_serial_node',
        output='screen',
        parameters=[
            {
                'port': LaunchConfiguration('port'),
                'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int),
                'address': ParameterValue(LaunchConfiguration('address'), value_type=int),
                'frame_id': LaunchConfiguration('frame_id'),
                'command_resend_hz': ParameterValue(
                    LaunchConfiguration('command_resend_hz'),
                    value_type=float,
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            port_arg,
            baudrate_arg,
            address_arg,
            frame_id_arg,
            resend_hz_arg,
            serial_node,
        ]
    )
