import struct
from dataclasses import asdict, dataclass
from typing import Dict, List, Tuple

from ano_receiver_bridge.observation_state import ExternalObservationState


RPI_PRIVATE_HEAD_1 = 0xBE
RPI_PRIVATE_HEAD_2 = 0xEF
RPI_OBS_PAYLOAD_LEN = 31
RPI_OBS_FRAME_LEN = 36

RPI_OBS_VALID_POS = 0x01
RPI_OBS_VALID_SPEED = 0x02
RPI_OBS_VALID_DIST = 0x04

ANO_POS_INVALID_VALUE = -2147483648
ANO_SPEED_INVALID_VALUE = -32768
ANO_DIST_INVALID_VALUE = 0xFFFFFFFF

SEQUENCE_OFFSET = 3
VALID_FLAGS_OFFSET = 4
REMOTE_TICK_OFFSET = 5
POS_OFFSET = 9
VEL_OFFSET = 21
DIST_OFFSET = 27
CHECKSUM_OFFSET = 34


@dataclass(frozen=True)
class ExternalObservationFrame:
    sequence: int
    valid_flags: int
    remote_tick_ms: int
    pos_x_cm: int
    pos_y_cm: int
    pos_z_cm: int
    vel_x_cms: int
    vel_y_cms: int
    vel_z_cms: int
    dist_direction: int
    dist_angle_deg: int
    dist_cm: int


FIELD_LAYOUT = [
    {'offset': 0, 'size': 1, 'name': 'head1', 'type': 'u8', 'unit': '', 'endian': ''},
    {'offset': 1, 'size': 1, 'name': 'head2', 'type': 'u8', 'unit': '', 'endian': ''},
    {'offset': 2, 'size': 1, 'name': 'len', 'type': 'u8', 'unit': 'bytes', 'endian': ''},
    {'offset': 3, 'size': 1, 'name': 'sequence', 'type': 'u8', 'unit': 'count', 'endian': ''},
    {'offset': 4, 'size': 1, 'name': 'valid_flags', 'type': 'u8', 'unit': 'bitmask', 'endian': ''},
    {'offset': 5, 'size': 4, 'name': 'remote_tick_ms', 'type': 'u32', 'unit': 'ms', 'endian': 'little'},
    {'offset': 9, 'size': 4, 'name': 'pos_x_cm', 'type': 's32', 'unit': 'cm', 'endian': 'little'},
    {'offset': 13, 'size': 4, 'name': 'pos_y_cm', 'type': 's32', 'unit': 'cm', 'endian': 'little'},
    {'offset': 17, 'size': 4, 'name': 'pos_z_cm', 'type': 's32', 'unit': 'cm', 'endian': 'little'},
    {'offset': 21, 'size': 2, 'name': 'vel_x_cms', 'type': 's16', 'unit': 'cm/s', 'endian': 'little'},
    {'offset': 23, 'size': 2, 'name': 'vel_y_cms', 'type': 's16', 'unit': 'cm/s', 'endian': 'little'},
    {'offset': 25, 'size': 2, 'name': 'vel_z_cms', 'type': 's16', 'unit': 'cm/s', 'endian': 'little'},
    {'offset': 27, 'size': 1, 'name': 'dist_direction', 'type': 'u8', 'unit': '', 'endian': ''},
    {'offset': 28, 'size': 2, 'name': 'dist_angle_deg', 'type': 'u16', 'unit': 'deg', 'endian': 'little'},
    {'offset': 30, 'size': 4, 'name': 'dist_cm', 'type': 'u32', 'unit': 'cm', 'endian': 'little'},
    {'offset': 34, 'size': 2, 'name': 'checksum', 'type': 'u16', 'unit': '', 'endian': 'little'},
]


def compute_private_checksum(data: bytes) -> int:
    return sum(data) & 0xFFFF


def checksum_steps(data: bytes) -> List[Dict[str, int]]:
    running_sum = 0
    steps: List[Dict[str, int]] = []
    for offset, value in enumerate(data):
        running_sum = (running_sum + value) & 0xFFFF
        steps.append(
            {
                'offset': offset,
                'byte': value,
                'running_sum': running_sum,
            }
        )
    return steps


def state_valid_flags(state: ExternalObservationState) -> int:
    valid_flags = 0
    if state.pos_valid:
        valid_flags |= RPI_OBS_VALID_POS
    if state.vel_valid:
        valid_flags |= RPI_OBS_VALID_SPEED
    if state.dist_valid:
        valid_flags |= RPI_OBS_VALID_DIST
    return valid_flags


def state_to_frame(state: ExternalObservationState, sequence: int) -> ExternalObservationFrame:
    return ExternalObservationFrame(
        sequence=sequence & 0xFF,
        valid_flags=state_valid_flags(state),
        remote_tick_ms=state.timestamp_ms & 0xFFFFFFFF,
        pos_x_cm=state.pos_x_cm if state.pos_valid else ANO_POS_INVALID_VALUE,
        pos_y_cm=state.pos_y_cm if state.pos_valid else ANO_POS_INVALID_VALUE,
        pos_z_cm=state.pos_z_cm if state.pos_valid else ANO_POS_INVALID_VALUE,
        vel_x_cms=state.vel_x_cms if state.vel_valid else ANO_SPEED_INVALID_VALUE,
        vel_y_cms=state.vel_y_cms if state.vel_valid else ANO_SPEED_INVALID_VALUE,
        vel_z_cms=state.vel_z_cms if state.vel_valid else ANO_SPEED_INVALID_VALUE,
        dist_direction=state.dist_direction if state.dist_valid else 0,
        dist_angle_deg=state.dist_angle_deg if state.dist_valid else 0,
        dist_cm=state.dist_cm if state.dist_valid else ANO_DIST_INVALID_VALUE,
    )


def frame_to_payload(frame: ExternalObservationFrame) -> bytes:
    return struct.pack(
        '<BBI3i3hBHI',
        frame.sequence & 0xFF,
        frame.valid_flags & 0xFF,
        frame.remote_tick_ms & 0xFFFFFFFF,
        frame.pos_x_cm,
        frame.pos_y_cm,
        frame.pos_z_cm,
        frame.vel_x_cms,
        frame.vel_y_cms,
        frame.vel_z_cms,
        frame.dist_direction & 0xFF,
        frame.dist_angle_deg & 0xFFFF,
        frame.dist_cm & 0xFFFFFFFF,
    )


def encode_external_observation_frame(frame: ExternalObservationFrame) -> bytes:
    payload = frame_to_payload(frame)
    frame_without_checksum = bytes(
        [RPI_PRIVATE_HEAD_1, RPI_PRIVATE_HEAD_2, RPI_OBS_PAYLOAD_LEN]
    ) + payload
    checksum = compute_private_checksum(frame_without_checksum)
    return frame_without_checksum + struct.pack('<H', checksum)


def encode_state_packet(state: ExternalObservationState, sequence: int) -> bytes:
    return encode_external_observation_frame(state_to_frame(state, sequence))


def decode_external_observation_frame(packet: bytes) -> ExternalObservationFrame:
    if len(packet) != RPI_OBS_FRAME_LEN:
        raise ValueError(f'Expected {RPI_OBS_FRAME_LEN} bytes, got {len(packet)}')
    if packet[0] != RPI_PRIVATE_HEAD_1 or packet[1] != RPI_PRIVATE_HEAD_2:
        raise ValueError('Invalid private frame header')
    if packet[2] != RPI_OBS_PAYLOAD_LEN:
        raise ValueError('Invalid private frame payload length')

    checksum = int.from_bytes(packet[CHECKSUM_OFFSET:CHECKSUM_OFFSET + 2], 'little')
    expected_checksum = compute_private_checksum(packet[:CHECKSUM_OFFSET])
    if checksum != expected_checksum:
        raise ValueError(
            f'Checksum mismatch: got 0x{checksum:04X}, expected 0x{expected_checksum:04X}'
        )

    payload = packet[3:CHECKSUM_OFFSET]
    unpacked = struct.unpack('<BBI3i3hBHI', payload)
    return ExternalObservationFrame(
        sequence=unpacked[0],
        valid_flags=unpacked[1],
        remote_tick_ms=unpacked[2],
        pos_x_cm=unpacked[3],
        pos_y_cm=unpacked[4],
        pos_z_cm=unpacked[5],
        vel_x_cms=unpacked[6],
        vel_y_cms=unpacked[7],
        vel_z_cms=unpacked[8],
        dist_direction=unpacked[9],
        dist_angle_deg=unpacked[10],
        dist_cm=unpacked[11],
    )


def packet_to_hex(packet: bytes) -> str:
    return packet.hex(' ')


def frame_to_dict(frame: ExternalObservationFrame) -> Dict[str, int]:
    return asdict(frame)


def layout_rows(packet: bytes) -> List[Dict[str, object]]:
    frame = decode_external_observation_frame(packet)
    values = {
        'head1': RPI_PRIVATE_HEAD_1,
        'head2': RPI_PRIVATE_HEAD_2,
        'len': RPI_OBS_PAYLOAD_LEN,
        **frame_to_dict(frame),
        'checksum': int.from_bytes(packet[CHECKSUM_OFFSET:CHECKSUM_OFFSET + 2], 'little'),
    }
    rows: List[Dict[str, object]] = []
    for field in FIELD_LAYOUT:
        start = field['offset']
        end = start + field['size']
        rows.append(
            {
                **field,
                'hex': packet[start:end].hex(' '),
                'value': values[field['name']],
            }
        )
    return rows


def build_profile_frame(
    profile: str,
    sequence: int,
    remote_tick_ms: int,
    pos_x_cm: int,
    pos_y_cm: int,
    pos_z_cm: int,
    vel_x_cms: int,
    vel_y_cms: int,
    vel_z_cms: int,
    dist_direction: int,
    dist_angle_deg: int,
    dist_cm: int,
) -> ExternalObservationFrame:
    return state_to_frame(
        ExternalObservationState.build_profile(
            profile=profile,
            timestamp_ms=remote_tick_ms,
            pos_x_cm=pos_x_cm,
            pos_y_cm=pos_y_cm,
            pos_z_cm=pos_z_cm,
            vel_x_cms=vel_x_cms,
            vel_y_cms=vel_y_cms,
            vel_z_cms=vel_z_cms,
            dist_direction=dist_direction,
            dist_angle_deg=dist_angle_deg,
            dist_cm=dist_cm,
        ),
        sequence=sequence,
    )


def build_profile_packet(
    profile: str,
    sequence: int,
    remote_tick_ms: int,
    pos_x_cm: int,
    pos_y_cm: int,
    pos_z_cm: int,
    vel_x_cms: int,
    vel_y_cms: int,
    vel_z_cms: int,
    dist_direction: int,
    dist_angle_deg: int,
    dist_cm: int,
) -> bytes:
    return encode_external_observation_frame(
        build_profile_frame(
            profile=profile,
            sequence=sequence,
            remote_tick_ms=remote_tick_ms,
            pos_x_cm=pos_x_cm,
            pos_y_cm=pos_y_cm,
            pos_z_cm=pos_z_cm,
            vel_x_cms=vel_x_cms,
            vel_y_cms=vel_y_cms,
            vel_z_cms=vel_z_cms,
            dist_direction=dist_direction,
            dist_angle_deg=dist_angle_deg,
            dist_cm=dist_cm,
        )
    )


def profile_vector(profile: str) -> Dict[str, object]:
    packet = build_profile_packet(
        profile=profile,
        sequence=1,
        remote_tick_ms=1234,
        pos_x_cm=100,
        pos_y_cm=0,
        pos_z_cm=120,
        vel_x_cms=0,
        vel_y_cms=0,
        vel_z_cms=0,
        dist_direction=1,
        dist_angle_deg=270,
        dist_cm=120,
    )
    return {
        'profile': profile,
        'length': len(packet),
        'checksum': int.from_bytes(packet[-2:], 'little'),
        'hex': packet.hex(),
    }


def checksum_value_and_bytes(packet: bytes) -> Tuple[int, str]:
    checksum = int.from_bytes(packet[-2:], 'little')
    return checksum, packet[-2:].hex(' ')
