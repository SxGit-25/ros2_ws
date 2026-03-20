import math
import struct
from typing import List, Optional

from ano_bridge.checksum import verify_checksums
from ano_bridge.messages import (
    AttitudeMessage,
    FloatArrayMessage,
    QuaternionMessage,
    Vector3Message,
)
from ano_bridge.protocol import (
    AnoFrame,
    ATTITUDE_FRAME_ID,
    FRAME_HEADER,
    FRAME_PAYLOAD_LENGTHS,
    GENERAL_DISTANCE_FRAME_ID,
    GENERAL_DISTANCE_INVALID_RAW,
    GENERAL_VELOCITY_FRAME_ID,
    GENERAL_VELOCITY_INVALID_RAW,
    QUATERNION_FRAME_ID,
    VELOCITY_FRAME_ID,
)


class AnoStreamParser:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def append(self, data: bytes) -> List[AnoFrame]:
        self._buffer.extend(data)
        frames: List[AnoFrame] = []

        while True:
            start = self._find_header()
            if start is None:
                self._buffer.clear()
                break

            if start > 0:
                del self._buffer[:start]

            if len(self._buffer) < 4:
                break

            payload_length = self._buffer[3]
            total_length = 4 + payload_length + 2
            if len(self._buffer) < total_length:
                break

            candidate = bytes(self._buffer[:total_length])
            sumcheck = candidate[-2]
            addcheck = candidate[-1]
            frame_without_checksums = candidate[:-2]

            if not verify_checksums(frame_without_checksums, sumcheck, addcheck):
                del self._buffer[0]
                continue

            frames.append(
                AnoFrame(
                    address=candidate[1],
                    frame_id=candidate[2],
                    payload=candidate[4:-2],
                )
            )
            del self._buffer[:total_length]

        return frames

    def _find_header(self) -> Optional[int]:
        try:
            return self._buffer.index(FRAME_HEADER)
        except ValueError:
            return None


def parse_payload(frame: AnoFrame):
    expected_length = FRAME_PAYLOAD_LENGTHS.get(frame.frame_id)
    if expected_length is None or len(frame.payload) != expected_length:
        return None

    if frame.frame_id == ATTITUDE_FRAME_ID:
        return parse_attitude_payload(frame.payload)
    if frame.frame_id == QUATERNION_FRAME_ID:
        return parse_quaternion_payload(frame.payload)
    if frame.frame_id == VELOCITY_FRAME_ID:
        return parse_velocity_payload(frame.payload)
    if frame.frame_id == GENERAL_VELOCITY_FRAME_ID:
        return parse_general_velocity_payload(frame.payload)
    if frame.frame_id == GENERAL_DISTANCE_FRAME_ID:
        return parse_general_distance_payload(frame.payload)
    return None


def parse_attitude_payload(payload: bytes) -> AttitudeMessage:
    roll_raw, pitch_raw, yaw_raw, _ = struct.unpack('<hhhB', payload)
    return AttitudeMessage(
        roll_deg=roll_raw / 100.0,
        pitch_deg=pitch_raw / 100.0,
        yaw_deg=yaw_raw / 100.0,
    )


def parse_quaternion_payload(payload: bytes) -> QuaternionMessage:
    w_raw, x_raw, y_raw, z_raw, _ = struct.unpack('<hhhhB', payload)
    return QuaternionMessage(
        w=w_raw / 10000.0,
        x=x_raw / 10000.0,
        y=y_raw / 10000.0,
        z=z_raw / 10000.0,
    )


def parse_velocity_payload(payload: bytes) -> Vector3Message:
    vx_raw, vy_raw, vz_raw = struct.unpack('<hhh', payload)
    return Vector3Message(
        x=vx_raw / 100.0,
        y=vy_raw / 100.0,
        z=vz_raw / 100.0,
    )


def parse_general_velocity_payload(payload: bytes) -> Vector3Message:
    raw_values = struct.unpack('<hhh', payload)
    parsed = []
    for raw in raw_values:
        if raw == GENERAL_VELOCITY_INVALID_RAW:
            parsed.append(math.nan)
        else:
            parsed.append(raw / 100.0)
    return Vector3Message(x=parsed[0], y=parsed[1], z=parsed[2])


def parse_general_distance_payload(payload: bytes) -> FloatArrayMessage:
    direction, angle_raw, distance_raw = struct.unpack('<BhI', payload)
    if distance_raw == GENERAL_DISTANCE_INVALID_RAW:
        distance_m = math.nan
    else:
        distance_m = distance_raw / 100.0
    return FloatArrayMessage(
        data=[float(direction), angle_raw / 100.0, distance_m]
    )
