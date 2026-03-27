# Observation Adapter

`observation_adapter_node` converts frontend candidate motion outputs into a unified external observation candidate state.

This node intentionally does **not** publish the private protocol and does **not** perform EKF or fusion.

## Purpose

- treat radar frontend outputs as candidates, not truth
- decide whether velocity is valid enough to move forward in the pipeline
- keep position and distance conservative at this stage

## Input Topics

- `/radar/odom_candidate` (`nav_msgs/msg/Odometry`)
- `/radar/vel_candidate` (`geometry_msgs/msg/TwistStamped`)
- `/radar/match_quality` (`std_msgs/msg/Float32`)
- `/radar/odom_status` (`std_msgs/msg/String`)

## Output Topics

- `/observation/radar_candidate` (`std_msgs/msg/String`)
- `/observation/radar_status` (`std_msgs/msg/String`)

Both topics publish compact JSON.

## Current Mapping Policy

- `vel_valid`:
  - true only when frontend status is `VALID`
  - `accept_motion` is true
  - quality exceeds the configured threshold
  - speed and yaw rate are inside configured limits
  - no blocking warnings or reasons remain
- `pos_valid`:
  - always false in this version
- `dist_valid`:
  - always false in this version

Rejected velocity candidates are currently zeroed in the unified state to avoid accidental downstream misuse.

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
ros2 run radar_frontend observation_adapter_node
```

Or:

```bash
cd /Users/metro/code/ros2_ws-main
source install/setup.bash
ros2 launch radar_frontend observation_adapter.launch.py
```

## Useful Debug Commands

```bash
ros2 topic echo /observation/radar_candidate
ros2 topic echo /observation/radar_status
ros2 topic echo /radar/odom_status
ros2 topic echo /radar/match_quality
ros2 topic echo /radar/vel_candidate
```
