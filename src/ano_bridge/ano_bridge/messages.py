from dataclasses import dataclass
from typing import List


@dataclass(frozen=True)
class AttitudeMessage:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float


@dataclass(frozen=True)
class QuaternionMessage:
    w: float
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class Vector3Message:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class FloatArrayMessage:
    data: List[float]
