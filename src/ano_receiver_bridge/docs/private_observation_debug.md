# Private Observation Offline Debug

## Protocol Summary

- Header: `BE EF`
- Length byte: `1F` (`31` payload bytes)
- Total frame length: `36` bytes
- Byte order:
  - `remote_tick_ms`: little-endian `u32`
  - `pos_*`: little-endian `s32`, unit `cm`
  - `vel_*`: little-endian `s16`, unit `cm/s`
  - `dist_angle_deg`: little-endian `u16`
  - `dist_cm`: little-endian `u32`, unit `cm`
- Checksum:
  - `u16`, little-endian
  - value = sum of bytes `offset 0..33`, truncated with `& 0xFFFF`

## Profiles

- `all_valid`: position, velocity, distance all valid
- `dist_only`: only distance valid, position and velocity converted to protocol invalid values
- `pos_only`: only position valid
- `all_invalid`: all valid bits clear, all three payload groups use protocol invalid values

## Offline Inspect

```bash
cd /Users/metro/code/ros2_ws-main
PYTHONPATH=src/ano_receiver_bridge python3 -m ano_receiver_bridge.inspect_private_frame --profile all_valid
```

Write the binary frame to a file:

```bash
cd /Users/metro/code/ros2_ws-main
PYTHONPATH=src/ano_receiver_bridge python3 -m ano_receiver_bridge.inspect_private_frame \
  --profile dist_only \
  --output-file /tmp/dist_only_frame.bin
```

## Dry Run Sender

Print generated frames without opening any serial port:

```bash
cd /Users/metro/code/ros2_ws-main
source install/setup.bash
ros2 run ano_receiver_bridge observation_sender --ros-args \
  -p dry_run:=true \
  -p output_mode:=stdout \
  -p profile:=all_valid \
  -p send_hz:=2.0
```

Append binary frames to a file:

```bash
cd /Users/metro/code/ros2_ws-main
source install/setup.bash
ros2 run ano_receiver_bridge observation_sender --ros-args \
  -p dry_run:=true \
  -p output_mode:=file \
  -p output_file:=/tmp/private_observation_frames.bin \
  -p profile:=dist_only \
  -p send_hz:=5.0
```

Serial mode remains unchanged:

```bash
cd /Users/metro/code/ros2_ws-main
source install/setup.bash
ros2 run ano_receiver_bridge observation_sender --ros-args \
  -p dry_run:=false \
  -p output_mode:=serial \
  -p port:=/dev/ttyUSB0 \
  -p baudrate:=500000
```

## Inspect Binary Output

```bash
xxd /tmp/private_observation_frames.bin
hexdump -C /tmp/private_observation_frames.bin
```

## Run Unit Tests

```bash
cd /Users/metro/code/ros2_ws-main/src/ano_receiver_bridge
PYTHONDONTWRITEBYTECODE=1 pytest tests/test_private_protocol.py
```
