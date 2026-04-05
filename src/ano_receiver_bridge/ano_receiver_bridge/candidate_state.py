from dataclasses import asdict, dataclass
from typing import Dict


@dataclass(frozen=True)
class ImuCandidateState:
    stamp_ms: int
    acc_x: int
    acc_y: int
    acc_z: int
    gyr_x: int
    gyr_y: int
    gyr_z: int
    valid: bool
    source: str

    def to_dict(self) -> Dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class FlowCandidateState:
    stamp_ms: int
    flow_vx: float
    flow_vy: float
    flow_state: int
    flow_quality: int
    alt_cm: int
    valid: bool
    source: str

    def to_dict(self) -> Dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class ImuHealthState:
    stamp_ms: int
    imu_stream_ok: bool
    imu_rate_hz: float
    imu_all_zero: bool
    imu_stale: bool
    imu_same_sample_count: int
    acc_unit_hint: str
    gyr_unit_hint: str
    decode_error_count: int

    def to_dict(self) -> Dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class FlowHealthState:
    stamp_ms: int
    flow_stream_ok: bool
    flow_rate_hz: float
    flow_state_recent: int
    flow_quality_recent: int
    flow_alt_recent: int
    flow_large_jump: bool
    flow_stale: bool
    decode_error_count: int

    def to_dict(self) -> Dict[str, object]:
        return asdict(self)
