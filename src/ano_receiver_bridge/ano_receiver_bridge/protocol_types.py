from dataclasses import dataclass
from typing import Dict


FRAME_HEAD = 0xAA

ATTITUDE_FRAME_ID = 0x03
QUATERNION_FRAME_ID = 0x04
VELOCITY_FRAME_ID = 0x07
IMU_RAW_FRAME_ID = 0x08
FLOW_OBS_FRAME_ID = 0x09
GENERAL_VELOCITY_FRAME_ID = 0x33
GENERAL_DISTANCE_FRAME_ID = 0x34
REALTIME_CONTROL_FRAME_ID = 0x41

SUPPORTED_PAYLOAD_LENGTHS: Dict[int, int] = {
    ATTITUDE_FRAME_ID: 7,
    QUATERNION_FRAME_ID: 9,
    VELOCITY_FRAME_ID: 6,
    IMU_RAW_FRAME_ID: 12,
    FLOW_OBS_FRAME_ID: 10,
    GENERAL_VELOCITY_FRAME_ID: 6,
    GENERAL_DISTANCE_FRAME_ID: 7,
}

GENERAL_VELOCITY_INVALID = -32768
GENERAL_DISTANCE_INVALID = 0xFFFFFFFF
CONTROL_ADDRESS = 0xFF


@dataclass(frozen=True)
class AnoFrame:
    head: int
    address: int
    frame_id: int
    length: int
    payload: bytes
    sumcheck: int
    addcheck: int


@dataclass(frozen=True)
class AttitudeData:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    fusion_state: int


@dataclass(frozen=True)
class QuaternionData:
    w: float
    x: float
    y: float
    z: float
    fusion_state: int


@dataclass(frozen=True)
class VelocityData:
    x_mps: float
    y_mps: float
    z_mps: float


@dataclass(frozen=True)
class ImuRawData:
    acc_x: int
    acc_y: int
    acc_z: int
    gyr_x: int
    gyr_y: int
    gyr_z: int


@dataclass(frozen=True)
class FlowObsData:
    flow_vx: float
    flow_vy: float
    flow_state: int
    flow_quality: int
    alt_cm: int


@dataclass(frozen=True)
class GeneralDistanceData:
    direction: float
    angle_deg: float
    distance_m: float


@dataclass(frozen=True)
class RealtimeControlCommand:
    ctrl_rol: int = 0
    ctrl_pit: int = 0
    ctrl_thr: int = 0
    ctrl_yawdps: int = 0
    ctrl_spd_x: int = 0
    ctrl_spd_y: int = 0
    ctrl_spd_z: int = 0
