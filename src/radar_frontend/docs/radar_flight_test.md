# Radar Flight Test Entry

`radar_flight_test.launch.py` is the one-command entry for the current 0x33 minimum-risk flight-test stage.

It is intentionally focused on:

- one-command startup
- safe-by-default sender mode
- a low-noise flight-test monitor panel
- preserving the ability to stop only the sender

## Start Safe Dry Run

```bash
cd /Users/metro/code/ros2_ws-main
source install/setup.bash
ros2 launch radar_frontend radar_flight_test.launch.py
```

## Start Live Send

```bash
cd /Users/metro/code/ros2_ws-main
source install/setup.bash
ros2 launch radar_frontend radar_flight_test.launch.py sender_mode:=live_send
```

## Common Options

```bash
ros2 launch radar_frontend radar_flight_test.launch.py enable_inspector:=true
ros2 launch radar_frontend radar_flight_test.launch.py start_lidar_driver:=false
ros2 launch radar_frontend radar_flight_test.launch.py pipeline_output:=screen
ros2 launch radar_frontend radar_flight_test.launch.py sender_mode:=live_send sender_port:=/dev/ttyUSB0
```

## Standalone Monitor

```bash
cd /Users/metro/code/ros2_ws-main
source install/setup.bash
ros2 run radar_frontend radar_trial_monitor_node
```

The monitor also publishes:

- `/radar/trial_monitor_summary` (`std_msgs/msg/String`)

## Sender Quick Rollback

Because the sender caches startup parameters, runtime parameter changes are not treated as a reliable rollback path.

Preferred rollback:

```bash
pkill -INT -f private_observation_velocity_sender_node
```

or stop only the sender terminal with `Ctrl-C`.
