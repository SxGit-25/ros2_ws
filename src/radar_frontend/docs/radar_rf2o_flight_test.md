# Radar RF2O Flight Test

This entrypoint replaces the custom `radar_scan_odometry_node` in the main test chain with:

- `rf2o_laser_odometry_node`
- `rf2o_radar_bridge_node`

The downstream chain remains the same:

- `/radar/*`
- `observation_adapter_node`
- `radar_trial_monitor_node`
- `private_observation_velocity_sender_node`

The old frontend remains in the repository, but it is no longer used by this launch.

## Safe Dry Run

```bash
cd /Users/metro/code/ros2_ws-main
source install/setup.bash
ros2 launch radar_frontend radar_rf2o_flight_test.launch.py
```

## Live Send

```bash
cd /Users/metro/code/ros2_ws-main
source install/setup.bash
ros2 launch radar_frontend radar_rf2o_flight_test.launch.py sender_mode:=live_send
```

## Key Topics

```bash
ros2 topic echo /radar/odom_status
ros2 topic echo /radar/match_debug
ros2 topic echo /radar/vel_candidate
ros2 topic echo /observation/radar_status
ros2 topic echo /private_observation/velocity_sender_status
```
