import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取包路径
    ano_pkg_dir = get_package_share_directory('ano_drone_bringup')
    
    # 2. 获取配置文件路径
    # EKF 配置文件 (融合 IMU + 光流)
    ekf_config_path = os.path.join(ano_pkg_dir, 'config', 'ekf.yaml')
    # Cartographer 配置文件 (雷达 SLAM)
    cartographer_config_dir = os.path.join(ano_pkg_dir, 'config')
    cartographer_config_basename = 'cartographer.lua'

    # 3. 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true'),

        # ---------------------------------------------------------
        # 节点 1: Robot Localization (EKF) - 局部状态估计
        # 功能: 融合 IMU(0x01) 和 光流(0x51)，输出 /odometry/filtered
        # ---------------------------------------------------------
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', '/odometry/filtered')]
        ),

        # ---------------------------------------------------------
        # 节点 2: Cartographer Node - 全局定位/SLAM
        # 功能: 接收 /scan，发布 map -> odom 的 TF 变换
        # ---------------------------------------------------------
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', cartographer_config_basename,
            ],
            remappings=[
                ('echoes', 'horizontal_laser_2d'),
                ('scan', '/scan') # 确保雷达话题对齐 /scan
            ] 
        ),

        # ---------------------------------------------------------
        # 节点 3: Cartographer Occupancy Grid - 生成栅格地图
        # 功能: 将 SLAM 结果转为 Nav2 可用的栅格地图话题 /map
        # ---------------------------------------------------------
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'resolution': 0.05,
                'publish_period_sec': 1.0
            }],
        ),
    ])