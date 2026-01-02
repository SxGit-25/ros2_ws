import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取功能包路径
    # 假设您的包名为 ano_drone_bringup，如果不同请修改此处
    ano_pkg_dir = get_package_share_directory('ano_drone_bringup')
    nav2_pkg_dir = get_package_share_directory('nav2_bringup')

    # 2. 定义 Launch 配置变量
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    log_level = LaunchConfiguration('log_level')

    # 3. 声明 Launch 参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup the nav2 stack')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        # 默认地图路径，请确保 maps 文件夹存在，或者在运行时指定 map:=/path/to/map.yaml
        default_value=os.path.join(ano_pkg_dir, 'maps', 'map.yaml'),
        description='Full path to map yaml file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        # 加载我们在上一轮生成的全向移动配置文件
        default_value=os.path.join(ano_pkg_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True', # 在树莓派上推荐使用组件以节省内存和通信开销
        description='Whether to use composed bringup')
        
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='log level')

    # 4. 包含 Nav2 的标准启动文件 (bringup_launch.py)
    # 这个文件会自动启动: map_server, amcl, bt_navigator, planner_server, controller_server 等
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': 'False',
            'log_level': log_level
        }.items()
    )

    # 5. 返回 LaunchDescription
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_use_composition_cmd,
        declare_log_level_cmd,
        nav2_bringup_launch
    ])