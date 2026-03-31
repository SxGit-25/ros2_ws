import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events.matchers import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in {'1', 'true', 'yes', 'on'}


def _build_launch(context, *args, **kwargs):
    frontend_share = get_package_share_directory('radar_frontend')

    sender_mode = LaunchConfiguration('sender_mode').perform(context).strip().lower()
    if sender_mode not in {'safe_dry_run', 'live_send'}:
        raise RuntimeError("sender_mode must be either 'safe_dry_run' or 'live_send'")

    start_lidar_driver = _as_bool(
        LaunchConfiguration('start_lidar_driver').perform(context)
    )
    start_sender = _as_bool(LaunchConfiguration('start_sender').perform(context))
    rf2o_publish_tf = _as_bool(LaunchConfiguration('rf2o_publish_tf').perform(context))
    rf2o_base_frame_id = LaunchConfiguration('rf2o_base_frame_id').perform(context)
    rf2o_odom_frame_id = LaunchConfiguration('rf2o_odom_frame_id').perform(context)
    rf2o_scan_topic = LaunchConfiguration('rf2o_scan_topic').perform(context)
    sender_port = LaunchConfiguration('sender_port').perform(context)
    sender_output_mode = LaunchConfiguration('sender_output_mode').perform(context)
    lslidar_params_file = LaunchConfiguration('lslidar_params_file').perform(context)

    config_dir = os.path.join(frontend_share, 'config')
    entities = []

    if start_lidar_driver:
        lidar_node = LifecycleNode(
            package='lslidar_driver',
            executable='lslidar_driver_node',
            name='lslidar_driver_node',
            namespace='',
            output='log',
            emulate_tty=True,
            parameters=[lslidar_params_file],
        )
        entities.append(lidar_node)
        entities.extend(
            [
                RegisterEventHandler(
                    OnProcessStart(
                        target_action=lidar_node,
                        on_start=[
                            EmitEvent(
                                event=ChangeState(
                                    lifecycle_node_matcher=matches_action(lidar_node),
                                    transition_id=Transition.TRANSITION_CONFIGURE,
                                )
                            )
                        ],
                    )
                ),
                RegisterEventHandler(
                    OnStateTransition(
                        target_lifecycle_node=lidar_node,
                        goal_state='inactive',
                        entities=[
                            EmitEvent(
                                event=ChangeState(
                                    lifecycle_node_matcher=matches_action(lidar_node),
                                    transition_id=Transition.TRANSITION_ACTIVATE,
                                )
                            )
                        ],
                    )
                ),
            ]
        )

    entities.extend(
        [
            Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='log',
                emulate_tty=True,
                parameters=[
                    os.path.join(config_dir, 'rf2o_laser_odometry.flight.yaml'),
                    {
                        'laser_scan_topic': rf2o_scan_topic,
                        'publish_tf': rf2o_publish_tf,
                        'base_frame_id': rf2o_base_frame_id,
                        'odom_frame_id': rf2o_odom_frame_id,
                    },
                ],
            ),
            Node(
                package='radar_frontend',
                executable='rf2o_radar_bridge_node',
                name='rf2o_radar_bridge_node',
                output='log',
                emulate_tty=True,
                parameters=[
                    os.path.join(config_dir, 'rf2o_radar_bridge.flight.yaml'),
                    {
                        'scan_topic': rf2o_scan_topic,
                    },
                ],
            ),
            Node(
                package='radar_frontend',
                executable='observation_adapter_node',
                name='observation_adapter_node',
                output='log',
                emulate_tty=True,
                parameters=[os.path.join(config_dir, 'observation_adapter.flight.yaml')],
            ),
            Node(
                package='radar_frontend',
                executable='radar_trial_monitor_node',
                name='radar_trial_monitor_node',
                output='screen',
                emulate_tty=True,
                parameters=[os.path.join(config_dir, 'radar_trial_monitor.flight.yaml')],
            ),
        ]
    )

    if start_sender:
        sender_overrides = {
            'dry_run': sender_mode != 'live_send',
            'live_send': sender_mode == 'live_send',
            'enable_send': sender_mode == 'live_send',
            'port': sender_port,
            'output_mode': sender_output_mode,
        }
        entities.append(
            Node(
                package='radar_frontend',
                executable='private_observation_velocity_sender_node',
                name='private_observation_velocity_sender_node',
                output='log',
                emulate_tty=True,
                parameters=[
                    os.path.join(config_dir, 'private_observation_velocity_sender.flight.yaml'),
                    sender_overrides,
                ],
            )
        )

    return entities


def generate_launch_description():
    lslidar_share = get_package_share_directory('lslidar_driver')
    default_lidar_params = os.path.join(lslidar_share, 'params', 'lsx10.yaml')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'sender_mode',
                default_value='safe_dry_run',
                description='safe_dry_run or live_send',
            ),
            DeclareLaunchArgument(
                'start_lidar_driver',
                default_value='true',
                description='Whether to launch lslidar_driver_node in this entrypoint',
            ),
            DeclareLaunchArgument(
                'start_sender',
                default_value='true',
                description='Whether to start private_observation_velocity_sender_node',
            ),
            DeclareLaunchArgument(
                'sender_port',
                default_value='/dev/ttyUSB0',
                description='Serial port for private observation sender',
            ),
            DeclareLaunchArgument(
                'sender_output_mode',
                default_value='serial',
                description='serial, file, or stdout for the velocity sender',
            ),
            DeclareLaunchArgument(
                'rf2o_publish_tf',
                default_value='true',
                description='Whether rf2o publishes odom to base_link TF',
            ),
            DeclareLaunchArgument(
                'rf2o_base_frame_id',
                default_value='base_link',
                description='Base frame for rf2o odometry output',
            ),
            DeclareLaunchArgument(
                'rf2o_odom_frame_id',
                default_value='odom',
                description='Odom frame for rf2o odometry output',
            ),
            DeclareLaunchArgument(
                'rf2o_scan_topic',
                default_value='/scan',
                description='Laser scan topic consumed by rf2o',
            ),
            DeclareLaunchArgument(
                'lslidar_params_file',
                default_value=default_lidar_params,
                description='Path to the lslidar driver parameter file',
            ),
            OpaqueFunction(function=_build_launch),
        ]
    )
