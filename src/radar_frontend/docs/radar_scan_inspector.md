# Radar Scan Inspector

`radar_scan_inspector_node` is a minimal lidar frontend inspection node for the current project stage.

It intentionally does **not** generate private observation frames, `0x32`, `0x33`, `0x34`, radar odometry, or EKF outputs.

## Input

- `/scan` (`sensor_msgs/msg/LaserScan`)

## Published Topics

- `/radar/scan_quality` (`std_msgs/msg/String`)
- `/radar/sector_metrics` (`std_msgs/msg/String`)
- `/radar/scan_stability` (`std_msgs/msg/String`)
- `/radar/scan_stability_score` (`std_msgs/msg/Float32`)

All string topics publish compact JSON for easy CLI inspection.

## Valid Point Rule

A point is counted as valid only when:

- it is finite
- `range_min <= r <= range_max`

Handling:

- `NaN` / `Inf` -> counted in `nan_inf_points`, excluded from statistics
- `r < range_min` -> counted in `below_range_min_points`, excluded from statistics
- `r > range_max` -> counted in `above_range_max_points`, excluded from statistics
- boundary values equal to `range_min` or `range_max` are retained

## Sector Layout

Default `sector_mode := six` uses:

- `front`: `[-30 deg, 30 deg)`
- `left_front`: `[30 deg, 90 deg)`
- `left`: `[90 deg, 150 deg)`
- `rear`: `[150 deg, 180 deg)` and `[-180 deg, -150 deg)`
- `right`: `[-150 deg, -90 deg)`
- `right_front`: `[-90 deg, -30 deg)`

`sector_mode := uniform` uses `sector_count` equal-width sectors over `[-180 deg, 180 deg)`.

## Stability Metrics

This node performs only lightweight frame-to-frame inspection:

- valid point count delta
- valid ratio delta
- nearest valid distance delta
- per-sector median distance delta
- overall difference score in `[0, 1]`

The score is a simple engineering indicator, not scan matching and not motion estimation.

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
ros2 run radar_frontend radar_scan_inspector_node
```

Or:

```bash
cd /Users/metro/code/ros2_ws-main
source install/setup.bash
ros2 launch radar_frontend radar_scan_inspector.launch.py
```

## Useful Debug Commands

```bash
ros2 topic echo /radar/scan_quality
ros2 topic echo /radar/sector_metrics
ros2 topic echo /radar/scan_stability
ros2 topic echo /radar/scan_stability_score
ros2 topic hz /scan
ros2 topic echo /scan --once
```

## Minimal Checklist

1. Confirm the lidar driver is publishing `/scan`
2. Start `radar_scan_inspector_node`
3. Check `valid_ratio` remains above the configured threshold
4. Check sector medians are sensible for the environment
5. In a static scene, verify `overall_difference_score` stays low and timestamps increase monotonically
