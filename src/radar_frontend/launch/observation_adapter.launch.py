from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='radar_frontend',
                executable='observation_adapter_node',
                name='observation_adapter_node',
                output='screen',
                parameters=[
                    {
                        'odom_topic': '/radar/odom_candidate',
                        'vel_topic': '/radar/vel_candidate',
                        'quality_topic': '/radar/match_quality',
                        'status_topic': '/radar/odom_status',
                        'source_name': 'radar_frontend',
                        'candidate_frame_id': 'body',
                        'min_quality_for_velocity': 0.72,
                        'max_speed_mps': 1.50,
                        'max_yaw_rate_rps': 1.50,
                        'reject_on_warnings': True,
                        'publish_hz': 10.0,
                    }
                ],
            )
        ]
    )
