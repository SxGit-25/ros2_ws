import math
import struct
from typing import Optional, Union

from ano_receiver_bridge.protocol_types import (
    ATTITUDE_FRAME_ID,
    GENERAL_DISTANCE_FRAME_ID,
    GENERAL_DISTANCE_INVALID,
    GENERAL_VELOCITY_FRAME_ID,
    GENERAL_VELOCITY_INVALID,
    QUATERNION_FRAME_ID,
    SUPPORTED_PAYLOAD_LENGTHS,
    VELOCITY_FRAME_ID,
    AnoFrame,
    AttitudeData,
    GeneralDistanceData,
    QuaternionData,
    VelocityData,
)


DecodedType = Union[AttitudeData, QuaternionData, VelocityData, GeneralDistanceData]


def decode_frame(frame: AnoFrame) -> Optional[DecodedType]:
    expected_length = SUPPORTED_PAYLOAD_LENGTHS.get(frame.frame_id)
    if expected_length is None or frame.length != expected_length or len(frame.payload) != expected_length:
        return None

    if frame.frame_id == ATTITUDE_FRAME_ID:
        return _decode_attitude(frame.payload)
    if frame.frame_id == QUATERNION_FRAME_ID:
        return _decode_quaternion(frame.payload)
    if frame.frame_id == VELOCITY_FRAME_ID:
        return _decode_velocity(frame.payload)
    if frame.frame_id == GENERAL_VELOCITY_FRAME_ID:
        return _decode_general_velocity(frame.payload)
    if frame.frame_id == GENERAL_DISTANCE_FRAME_ID:
        return _decode_general_distance(frame.payload)
    return None


def _decode_attitude(payload: bytes) -> AttitudeData:
    roll_x100, pitch_x100, yaw_x100, fusion_state = struct.unpack('<hhhB', payload)
    return AttitudeData(
        roll_deg=roll_x100 / 100.0,
        pitch_deg=pitch_x100 / 100.0,
        yaw_deg=yaw_x100 / 100.0,
        fusion_state=fusion_state,
    )


def _decode_quaternion(payload: bytes) -> QuaternionData:
    w_x10000, x_x10000, y_x10000, z_x10000, fusion_state = struct.unpack('<hhhhB', payload)
    return QuaternionData(
        w=w_x10000 / 10000.0,
        x=x_x10000 / 10000.0,
        y=y_x10000 / 10000.0,
        z=z_x10000 / 10000.0,
        fusion_state=fusion_state,
    )


def _decode_velocity(payload: bytes) -> VelocityData:
    vx_cmps, vy_cmps, vz_cmps = struct.unpack('<hhh', payload)
    return VelocityData(
        x_mps=vx_cmps / 100.0,
        y_mps=vy_cmps / 100.0,
        z_mps=vz_cmps / 100.0,
    )


def _decode_general_velocity(payload: bytes) -> VelocityData:
    speed_x_cmps, speed_y_cmps, speed_z_cmps = struct.unpack('<hhh', payload)
    return VelocityData(
        x_mps=_speed_to_mps_or_nan(speed_x_cmps),
        y_mps=_speed_to_mps_or_nan(speed_y_cmps),
        z_mps=_speed_to_mps_or_nan(speed_z_cmps),
    )


def _decode_general_distance(payload: bytes) -> GeneralDistanceData:
    direction, angle_deg, distance_cm = struct.unpack('<BHI', payload)
    distance_m = math.nan if distance_cm == GENERAL_DISTANCE_INVALID else distance_cm / 100.0
    return GeneralDistanceData(
        direction=float(direction),
        angle_deg=float(angle_deg),
        distance_m=distance_m,
    )


def _speed_to_mps_or_nan(value_cmps: int) -> float:
    if value_cmps == GENERAL_VELOCITY_INVALID:
        return math.nan
    return value_cmps / 100.0
