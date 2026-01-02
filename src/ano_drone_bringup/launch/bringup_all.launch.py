import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. 获取包路径
    ano_pkg_dir = get_package_share_directory('ano_drone_bringup')
    lidar_pkg_dir = get_package_share_directory('LSLIDAR_X_ROS2') 
    
    # 2. 定义参数
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # 3. 声明参数
    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(ano_pkg_dir, 'maps', 'map.yaml'),
        description='Full path to map file to load')
        
    declare_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation clock')

    # ---------------------------------------------------------
    # 阶段 1: 基础硬件启动 (0秒时刻)
    # ---------------------------------------------------------
    
    # A. 飞控桥接 + TF (调用更新后的 hardware_driver)
    hardware_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ano_pkg_dir, 'launch', 'hardware_driver.launch.py')
        )
    )

    # B. 镭神雷达 N10P 
    # 该文件位于 ano_drone_bringup/launch/ 下
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            # 在当前LSLIDAR_X_ROS2中：
            os.path.join(lidar_pkg_dir, 'launch', 'lsn10p_launch.py')
        )
    )

    # ---------------------------------------------------------
    # 阶段 2: 定位系统 (延迟 3 秒)
    # 等待雷达转速稳定、IMU数据流建立
    # ---------------------------------------------------------
    localization_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ano_pkg_dir, 'launch', 'localization.launch.py')
                ),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    # ---------------------------------------------------------
    # 阶段 3: 导航系统 (延迟 4 秒)
    # 等待 EKF 收敛、Cartographer 构建出初始 map->odom 变换
    # ---------------------------------------------------------
    navigation_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ano_pkg_dir, 'launch', 'navigation.launch.py')
                ),
                launch_arguments={
                    'map': map_yaml_file,
                    'use_sim_time': use_sim_time,
                    'log_level': 'info'
                }.items()
            )
        ]
    )

    return LaunchDescription([
        declare_map_cmd,
        declare_sim_time_cmd,
        
        # 1. 硬件层
        hardware_driver_launch,
        lidar_launch,
        
        # 2. 算法层 (带延迟)
        localization_launch,
        navigation_launch
    ])