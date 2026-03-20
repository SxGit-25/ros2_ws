from dataclasses import dataclass
from typing import Dict


FRAME_HEADER = 0xAA
DEFAULT_ADDRESS = 0xFF

ATTITUDE_FRAME_ID = 0x03
QUATERNION_FRAME_ID = 0x04
VELOCITY_FRAME_ID = 0x07
GENERAL_VELOCITY_FRAME_ID = 0x33
GENERAL_DISTANCE_FRAME_ID = 0x34
REALTIME_CONTROL_FRAME_ID = 0x41

FRAME_PAYLOAD_LENGTHS: Dict[int, int] = {
    ATTITUDE_FRAME_ID: 7,
    QUATERNION_FRAME_ID: 9,
    VELOCITY_FRAME_ID: 6,
    GENERAL_VELOCITY_FRAME_ID: 6,
    GENERAL_DISTANCE_FRAME_ID: 7,
}

GENERAL_VELOCITY_INVALID_RAW = -32768
GENERAL_DISTANCE_INVALID_RAW = 0xFFFFFFFF


@dataclass(frozen=True)
class AnoFrame:
    address: int
    frame_id: int
    payload: bytes


@dataclass(frozen=True)
class RealtimeControlCommand:
    rol: int = 0
    pit: int = 0
    thr: int = 0
    ctrl_spd_x: int = 0
    ctrl_spd_y: int = 0
    ctrl_spd_z: int = 0
    ctrl_yaw_dps: int = 0
