include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  
  -- 坐标系配置
  map_frame = "map",
  tracking_frame = "base_link",    -- 机器人中心坐标系
  published_frame = "odom",        -- Cartographer 发布 map -> odom 的修正变换
  odom_frame = "odom",             -- 外部里程计(EKF)提供的坐标系
  
  -- 关键设置：是否发布里程计帧
  -- 设置为 false，因为这个工作已经由 robot_localization (EKF) 完成了
  -- 避免两个节点同时发布 odom -> base_link 导致 TF 树闪烁
  provide_odom_frame = false,
  
  -- 2D SLAM 模式设置
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  
  -- 传感器输入配置
  use_odometry = true,             -- 必须为 true，订阅 /odom 话题（来自 EKF）
  use_nav_sat = false,
  use_landmarks = false,
  
  --雷达数量
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  
  -- 处理频率与超时
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  
  -- 采样比率 (树莓派 CPU 有限，可适当降低采样率以减少负载)
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- -------------------------------------------------------------------
-- 2D 轨迹构建器参数调优 (针对 N10P 雷达和无人机特性)
-- -------------------------------------------------------------------

MAP_BUILDER.use_trajectory_builder_2d = true

-- 是否在 Cartographer 内部使用 IMU？
-- 建议: false。因为 IMU 已经融合进 EKF 生成的 odom 了。
-- 如果在这里也开启，Cartographer 会试图再次融合 IMU，可能导致冲突。
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- 雷达有效距离过滤
TRAJECTORY_BUILDER_2D.min_range = 0.2  -- N10P 盲区
TRAJECTORY_BUILDER_2D.max_range = 10.0 -- 室内环境建议限制在 10m-12m，太远的数据精度差且易受干扰

-- 丢失数据时的射线长度 (用于清除动态障碍物)
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0

-- 实时扫描匹配参数
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- 运动过滤器 (减少静止时的 CPU 消耗)
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05

-- -------------------------------------------------------------------
-- 后端位姿图优化参数 (Loop Closure)
-- -------------------------------------------------------------------

-- 降低优化频率以节省树莓派 CPU
POSE_GRAPH.optimize_every_n_nodes = 90 -- 每积累 90 个节点优化一次
POSE_GRAPH.constraint_builder.min_score = 0.65 -- 闭环检测的最低匹配分数
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

return options