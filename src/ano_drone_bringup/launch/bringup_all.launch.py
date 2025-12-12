import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ================= 配置变量 =================
    
    # 1. 雷达配置 (N10P 串口版)
    # 依据镭神驱动说明 ，N10P 串口启动文件为 lsn10p_launch.py
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyAMA0') # 树莓派 GPIO 14/15
    lidar_frame_id = LaunchConfiguration('lidar_frame_id', default='laser_link')

    # 2. 飞控配置 (ANO Bridge)
    # 飞控通过 USB 连接到树莓派，如果是其他串口请修改这里
    fc_port = LaunchConfiguration('fc_port', default='/dev/ttyUSB0') 
    fc_baud = LaunchConfiguration('fc_baud', default='500000') # 匿名飞控常用波特率

    # ================= 节点定义 =================

    # 1. 镭神 N10P 雷达驱动
    # 引用官方驱动包中的 launch 文件
    lslidar_dir = get_package_share_directory('lslidar_driver')
    lslidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(lslidar_dir, 'launch', 'lsn10p_launch.py')),
        launch_arguments={
            'serial_port': lidar_port,       # 覆盖默认的 /dev/wheeltec_laser 
            'frame_id': lidar_frame_id,      # 设置坐标系
            'baud_rate': '460800'            # N10P 固定波特率 [cite: 17, 1288]
        }.items()
    )

    # 2. 匿名飞控桥接节点 (你编写的 C++ 节点)
    ano_bridge_node = Node(
        package='ano_drone_bringup',                # 请确保你的包名正确
        executable='ano_bridge_node',        # 请确保 CMakeLists.txt 中生成的可执行文件名正确
        name='ano_bridge_node',
        output='screen',
        parameters=[{
            'serial_port': fc_port,
            'baud_rate': fc_baud,
            'imu_frame_id': 'imu_link',
            'flow_frame_id': 'flow_link'
        }]
    )

    # 3. 静态 TF 变换 (Static Transform Publisher)
    # 定义机身(base_link) 到 雷达(laser_link) 的相对位置
    # 假设雷达安装在机身中心上方 5cm 处 (0.05m)，无旋转
    # 参数格式: x y z roll pitch yaw frame_id child_frame_id
    tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'laser_link']
    )

    # 定义机身(base_link) 到 飞控IMU(imu_link) 的相对位置
    # 假设飞控就在机身中心
    tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    )

    # 假设光流模块在机身下方 2cm，且朝下安装 
    tf_base_to_flow = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '-0.02', '0', '0', '0', 'base_link', 'flow_link']
)

    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyAMA0', description='Lidar serial port'),
        DeclareLaunchArgument('fc_port', default_value='/dev/ttyUSB0', description='Flight Controller serial port'),
        
        # 启动节点
        tf_base_to_laser,
        tf_base_to_imu,
        tf_base_to_flow,
        lslidar_launch,
        ano_bridge_node
    ])