from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='radar_frontend',
                executable='radar_scan_inspector_node',
                name='radar_scan_inspector_node',
                output='screen',
                parameters=[
                    {
                        'scan_topic': '/scan',
                        'obstacle_distance_threshold_m': 1.5,
                        'sector_mode': 'six',
                        'sector_count': 6,
                        'log_hz': 1.0,
                        'enable_stability_analysis': True,
                        'min_valid_ratio_threshold': 0.20,
                    }
                ],
            )
        ]
    )
