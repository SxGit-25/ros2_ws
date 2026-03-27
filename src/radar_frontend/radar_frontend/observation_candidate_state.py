from dataclasses import asdict, dataclass
from typing import Dict


@dataclass(frozen=True)
class ObservationCandidateState:
    timestamp_ms: int
    source_name: str
    frame_id: str
    pos_x_cm: int
    pos_y_cm: int
    pos_z_cm: int
    vel_x_cms: int
    vel_y_cms: int
    vel_z_cms: int
    dist_direction: int
    dist_angle_deg: int
    dist_cm: int
    pos_valid: bool
    vel_valid: bool
    dist_valid: bool
    confidence: float
    status: str
    reject_reason: str
    debug_info: str

    def to_dict(self) -> Dict[str, object]:
        return asdict(self)
