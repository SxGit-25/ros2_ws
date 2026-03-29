from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='radar_frontend',
                executable='private_observation_velocity_sender_node',
                name='private_observation_velocity_sender_node',
                output='screen',
                parameters=[
                    {
                        'candidate_topic': '/observation/radar_candidate',
                        'status_topic': '/observation/radar_status',
                        'send_hz': 10.0,
                        'dry_run': True,
                        'live_send': False,
                        'enable_send': False,
                        'send_invalid_frames': True,
                        'min_confidence_for_live_send': 0.85,
                        'max_speed_cms': 80,
                        'swap_xy': False,
                        'invert_x': False,
                        'invert_y': False,
                        'port': '/dev/ttyUSB0',
                        'baudrate': 500000,
                        'timeout': 0.01,
                        'output_mode': 'serial',
                        'output_file': '/tmp/private_observation_velocity_frames.bin',
                        'log_hex_enable': True,
                    }
                ],
            )
        ]
    )
