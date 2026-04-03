# Observation Adapter

`observation_adapter_node` converts RF2O bridge candidate motion outputs into a unified external observation candidate state.

This node intentionally does **not** publish the private protocol and does **not** perform EKF or fusion.

## Purpose

- treat RF2O bridge outputs as candidates, not truth
- decide whether velocity is valid enough to move forward in the pipeline
- keep position and distance conservative at this stage

## Input Topics

- `/radar/odom_candidate` (`nav_msgs/msg/Odometry`)
- `/radar/vel_candidate` (`geometry_msgs/msg/TwistStamped`)
- `/radar/match_quality` (`std_msgs/msg/Float32`)
- `/radar/odom_status` (`std_msgs/msg/String`)

These `/radar/*` topics are expected to be produced by `rf2o_radar_bridge_node`.

## Output Topics

- `/observation/radar_candidate` (`std_msgs/msg/String`)
- `/observation/radar_status` (`std_msgs/msg/String`)
- `/observation/radar_validation_debug` (`std_msgs/msg/String`)

Both topics publish compact JSON.
The validation debug topic also publishes compact JSON.

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

## Current Coordinate Semantics

At this stage the adapter does not transform radar velocity into a separately calibrated aircraft body frame.

- candidate velocity is expressed in the current frontend frame id
- in practice this will usually be the lidar scan frame such as `laser_link`
- `+x` means forward in that radar/laser frame
- `+y` means left in that radar/laser frame
- positive yaw rate means counter-clockwise about `+z`

This is only equal to aircraft body semantics if `laser_link` is physically aligned with the aircraft body axes.

## Controlled Validation Support

`/observation/radar_validation_debug` includes:

- frame semantics
- candidate `vx / vy / yaw_rate`
- `vel_valid`
- `confidence`
- frontend status
- reject reason
- downstream recommendation

Downstream recommendation values:

- `BLOCK`
- `MANUAL_REVIEW`
- `ALLOW_FOR_NEXT_STAGE_REVIEW`

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
ros2 topic echo /observation/radar_validation_debug
ros2 topic echo /radar/odom_status
ros2 topic echo /radar/match_quality
ros2 topic echo /radar/vel_candidate
```
