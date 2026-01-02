import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    fc_port = LaunchConfiguration('fc_port', default='/dev/ttyUSB0')

    return LaunchDescription([
        DeclareLaunchArgument('fc_port', default_value='/dev/ttyUSB0'),

        # 1. 匿名协议桥接节点 (STM32 <-> ROS2)
        Node(
            package='ano_drone_bringup',
            executable='ano_bridge_node',
            name='ano_bridge_node',
            output='screen',
            parameters=[{
                'serial_port': fc_port,
                'baud_rate': 500000,
                'imu_frame_id': 'imu_link',
                'flow_frame_id': 'flow_link'
            }]
        ),

        # 2. TF 静态变换 (构建无人机物理模型)
        # 机身 -> 雷达
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'laser_link']
        ),
        # 机身 -> IMU
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        ),
        # 机身 -> 光流
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_flow',
            arguments=['0', '0', '-0.02', '0', '0', '0', 'base_link', 'flow_link']
        ),
    ])