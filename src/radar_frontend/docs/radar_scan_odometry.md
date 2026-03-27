# Radar Scan Odometry

`radar_scan_odometry_node` is a conservative first-pass lidar frontend candidate node.

Goals for this version:

- stay near zero when the aircraft is static
- avoid publishing misleading motion when quality is poor
- provide motion trend candidates, not final navigation truth

## Input

- `/scan` (`sensor_msgs/msg/LaserScan`)

This version does not require IMU input yet.

## Published Topics

- `/radar/odom_candidate` (`nav_msgs/msg/Odometry`)
- `/radar/vel_candidate` (`geometry_msgs/msg/TwistStamped`)
- `/radar/match_quality` (`std_msgs/msg/Float32`)
- `/radar/odom_status` (`std_msgs/msg/String`)
- `/radar/match_debug` (`std_msgs/msg/String`)

## Matching Strategy

This version uses a lightweight scan-to-scan candidate matcher:

1. build filtered scan summaries from consecutive `/scan` frames
2. search a small yaw window in discrete angle bins
3. choose the best candidate using overlap ratio and median range residual
4. estimate translation with matched point differences
5. apply strict quality gating before updating odometry

This is intentionally a conservative candidate frontend, not SLAM and not EKF.

## Gate States

- `VALID`: publish and integrate candidate motion
- `LOW_CONFIDENCE`: publish status and debug, hold odometry motion update
- `INVALID`: reject motion update

## Build

```bash
cd /Users/metro/code/ros2_ws-main
colcon build --packages-select radar_frontend
source install/setup.bash
```

## Run

```bash
cd /Users/metro/code/ros2_ws-main
source install/setup.bash
ros2 run radar_frontend radar_scan_odometry_node
```

Or:

```bash
cd /Users/metro/code/ros2_ws-main
source install/setup.bash
ros2 launch radar_frontend radar_scan_odometry.launch.py
```

## Useful Debug Commands

```bash
ros2 topic echo /radar/odom_candidate
ros2 topic echo /radar/vel_candidate
ros2 topic echo /radar/match_quality
ros2 topic echo /radar/odom_status
ros2 topic echo /radar/match_debug
```

## Expected Behavior

Static aircraft:

- `match_quality` should usually remain moderate to high
- `odom_status` should mostly be `VALID` or occasionally `LOW_CONFIDENCE`
- velocity should stay near zero
- odom should not walk away quickly

Small manual motion:

- `vel_candidate` should change in the expected direction
- yaw motion should appear on `angular.z`
- `match_quality` may drop slightly but should remain usable in structured scenes

Dynamic environment or poor data:

- `match_quality` should drop
- `odom_status` should move toward `LOW_CONFIDENCE` or `INVALID`
- motion integration should pause instead of forcing bad estimates
