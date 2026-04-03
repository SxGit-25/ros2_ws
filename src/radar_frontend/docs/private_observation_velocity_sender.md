# Private Observation Velocity Sender

`private_observation_velocity_sender_node` is a conservative velocity-only bridge from the RF2O-backed radar observation candidate layer into the existing private observation protocol chain.

This node intentionally:

- only uses the velocity portion of the private observation payload
- keeps `pos_valid = false`
- keeps `dist_valid = false`
- forces `vel_z_cms = 0`
- defaults to `dry_run = true`

## Input Topics

- `/observation/radar_candidate`
- `/observation/radar_status`

## Decision Fields Read

- `vel_valid`
- `vel_x_cms`
- `vel_y_cms`
- `vel_z_cms`
- `confidence`
- `status`
- `reject_reason`
- `downstream_recommendation`

## Send Policy

Live send is allowed only when all of the following are true:

- `vel_valid = true`
- `confidence >= min_confidence_for_live_send`
- candidate `status = VALID`
- downstream recommendation is `ALLOW_FOR_NEXT_STAGE_REVIEW`
- corrected speed norm is below `max_speed_cms`
- `reject_reason` is empty

Otherwise the node emits an explicit all-invalid private observation frame when `send_invalid_frames = true`.

## Safety Modes

- `dry_run = true`
  - never touches the real serial output path
- `live_send = true`
  - enables the live path only if `enable_send = true`
- `enable_send = false`
  - hard blocks live transmission even when `live_send = true`

Actual serial transmission requires:

- `dry_run = false`
- `live_send = true`
- `enable_send = true`

## Axis Correction Parameters

- `swap_xy`
- `invert_x`
- `invert_y`

These are applied before speed-limit gating and before serialization.

## Output Topics

- `/private_observation/velocity_sender_debug`
- `/private_observation/velocity_sender_status`

## Build

```bash
cd /Users/metro/code/ros2_ws-main
colcon build --packages-select radar_frontend ano_receiver_bridge
source install/setup.bash
```

## Dry-Run Start

```bash
cd /Users/metro/code/ros2_ws-main
source install/setup.bash
ros2 run radar_frontend private_observation_velocity_sender_node
```

## Launch File

```bash
cd /Users/metro/code/ros2_ws-main
source install/setup.bash
ros2 launch radar_frontend private_observation_velocity_sender.launch.py
```

## Useful Debug Commands

```bash
ros2 topic echo /private_observation/velocity_sender_status
ros2 topic echo /private_observation/velocity_sender_debug
ros2 topic echo /observation/radar_candidate
ros2 topic echo /observation/radar_status
```
