from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='radar_frontend',
                executable='radar_scan_odometry_node',
                name='radar_scan_odometry_node',
                output='screen',
                parameters=[
                    {
                        'scan_topic': '/scan',
                        'obstacle_distance_threshold_m': 1.5,
                        'sector_mode': 'six',
                        'sector_count': 6,
                        'log_hz': 1.0,
                        'min_valid_points': 80,
                        'min_valid_ratio': 0.20,
                        'min_overlap_ratio': 0.35,
                        'max_median_residual_m': 0.25,
                        'max_translation_norm_m': 0.80,
                        'max_translation_step_m': 0.80,
                        'max_yaw_search_deg': 12.0,
                        'yaw_search_step_deg': 1.0,
                        'min_quality': 0.60,
                        'low_confidence_quality': 0.40,
                        'min_dt_sec': 0.03,
                        'max_dt_sec': 0.25,
                        'sector_median_jump_threshold_m': 0.80,
                        'publish_debug': True,
                    }
                ],
            )
        ]
    )
