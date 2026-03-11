#!/usr/bin/env python3
"""
匿名V7协议硬件接口节点启动文件
用于启动树莓派与STM32飞控的串口通信节点
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 定义启动参数
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='串口设备路径，如 /dev/ttyUSB0 或 /dev/ttyACM0'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='500000',
        description='串口波特率，默认500000'
    )
    
    use_custom_msg_arg = DeclareLaunchArgument(
        'use_custom_msg',
        default_value='true',
        description='是否使用自定义消息格式，true使用自定义消息，false使用标准IMU消息'
    )
    
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='imu/data',
        description='IMU数据发布话题名称'
    )
    
    pose_topic_arg = DeclareLaunchArgument(
        'pose_topic',
        default_value='ekf/pose',
        description='位姿订阅话题名称'
    )
    
    velocity_topic_arg = DeclareLaunchArgument(
        'velocity_topic',
        default_value='cmd_vel',
        description='速度控制指令订阅话题名称'
    )
    
    # 创建节点
    ano_hardware_interface_node = Node(
        package='ano_drone_bringup',
        executable='ano_hardware_interface_node',
        name='ano_hardware_interface',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'use_custom_msg': LaunchConfiguration('use_custom_msg'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'pose_topic': LaunchConfiguration('pose_topic'),
            'velocity_topic': LaunchConfiguration('velocity_topic'),
        }],
        remappings=[
            ('imu/data', LaunchConfiguration('imu_topic')),
            ('ekf/pose', LaunchConfiguration('pose_topic')),
            ('cmd_vel', LaunchConfiguration('velocity_topic')),
        ]
    )
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加参数声明
    ld.add_action(serial_port_arg)
    ld.add_action(baud_rate_arg)
    ld.add_action(use_custom_msg_arg)
    ld.add_action(imu_topic_arg)
    ld.add_action(pose_topic_arg)
    ld.add_action(velocity_topic_arg)
    
    # 添加节点
    ld.add_action(ano_hardware_interface_node)
    
    return ld