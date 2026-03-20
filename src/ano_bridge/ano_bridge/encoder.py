import math
import struct

from ano_bridge.checksum import compute_checksums
from ano_bridge.protocol import (
    DEFAULT_ADDRESS,
    FRAME_HEADER,
    REALTIME_CONTROL_FRAME_ID,
    RealtimeControlCommand,
)


def saturate_int16(value: float) -> int:
    if math.isnan(value):
        return 0
    return max(-32768, min(32767, int(round(value))))


def encode_realtime_control(
    command: RealtimeControlCommand,
    address: int = DEFAULT_ADDRESS,
) -> bytes:
    payload = struct.pack(
        '<hhhhhhh',
        command.rol,
        command.pit,
        command.thr,
        command.ctrl_spd_x,
        command.ctrl_spd_y,
        command.ctrl_spd_z,
        command.ctrl_yaw_dps,
    )
    frame_without_checksums = bytes(
        [FRAME_HEADER, address & 0xFF, REALTIME_CONTROL_FRAME_ID, len(payload)]
    ) + payload
    sumcheck, addcheck = compute_checksums(frame_without_checksums)
    return frame_without_checksums + bytes([sumcheck, addcheck])
