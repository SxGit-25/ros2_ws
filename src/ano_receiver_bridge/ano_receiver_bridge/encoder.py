import math
import struct

from ano_receiver_bridge.checksum import compute_checksums
from ano_receiver_bridge.protocol_types import (
    CONTROL_ADDRESS,
    REALTIME_CONTROL_FRAME_ID,
    RealtimeControlCommand,
)


def clamp_int16(value: float) -> int:
    if math.isnan(value):
        return 0
    return max(-32768, min(32767, int(round(value))))


def encode_realtime_control_frame(command: RealtimeControlCommand) -> bytes:
    """Encode ANO 0x41 realtime control frame."""
    payload = struct.pack(
        '<hhhhhhh',
        command.ctrl_rol,
        command.ctrl_pit,
        command.ctrl_thr,
        command.ctrl_yawdps,
        command.ctrl_spd_x,
        command.ctrl_spd_y,
        command.ctrl_spd_z,
    )
    frame_without_checksums = bytes(
        [0xAA, CONTROL_ADDRESS, REALTIME_CONTROL_FRAME_ID, len(payload)]
    ) + payload
    sumcheck, addcheck = compute_checksums(frame_without_checksums)
    return frame_without_checksums + bytes([sumcheck, addcheck])
