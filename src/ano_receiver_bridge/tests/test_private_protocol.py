import json
import unittest
from pathlib import Path

from ano_receiver_bridge.private_protocol import (
    CHECKSUM_OFFSET,
    DIST_OFFSET,
    POS_OFFSET,
    RPI_OBS_PAYLOAD_LEN,
    RPI_PRIVATE_HEAD_1,
    RPI_PRIVATE_HEAD_2,
    VEL_OFFSET,
    build_profile_frame,
    build_profile_packet,
    checksum_steps,
    compute_private_checksum,
    decode_external_observation_frame,
)


def _vector_path() -> Path:
    return Path(__file__).parent / 'test_vectors' / 'private_observation_vectors.json'


def _load_vectors():
    return json.loads(_vector_path().read_text())


class PrivateProtocolTests(unittest.TestCase):
    def test_packet_has_expected_fixed_header_and_length(self) -> None:
        packet = build_profile_packet('all_valid', 1, 1234, 100, 0, 120, 0, 0, 0, 1, 270, 120)
        self.assertEqual(len(packet), 36)
        self.assertEqual(packet[0], RPI_PRIVATE_HEAD_1)
        self.assertEqual(packet[1], RPI_PRIVATE_HEAD_2)
        self.assertEqual(packet[2], RPI_OBS_PAYLOAD_LEN)

    def test_checksum_matches_manual_sum(self) -> None:
        packet = build_profile_packet('all_valid', 1, 1234, 100, 0, 120, 0, 0, 0, 1, 270, 120)
        expected = compute_private_checksum(packet[:CHECKSUM_OFFSET])
        actual = int.from_bytes(packet[CHECKSUM_OFFSET:], 'little')
        self.assertEqual(actual, expected)
        self.assertEqual(checksum_steps(packet[:CHECKSUM_OFFSET])[-1]['running_sum'], expected)

    def test_profile_valid_flags(self) -> None:
        self.assertEqual(
            build_profile_frame('all_valid', 1, 1234, 100, 0, 120, 0, 0, 0, 1, 270, 120).valid_flags,
            0x07,
        )
        self.assertEqual(
            build_profile_frame('dist_only', 1, 1234, 100, 0, 120, 0, 0, 0, 1, 270, 120).valid_flags,
            0x04,
        )
        self.assertEqual(
            build_profile_frame('all_invalid', 1, 1234, 100, 0, 120, 0, 0, 0, 1, 270, 120).valid_flags,
            0x00,
        )

    def test_field_offsets_are_stable(self) -> None:
        packet = build_profile_packet('all_valid', 1, 1234, 100, 0, 120, 0, 0, 0, 1, 270, 120)
        self.assertEqual(int.from_bytes(packet[POS_OFFSET:POS_OFFSET + 4], 'little', signed=True), 100)
        self.assertEqual(int.from_bytes(packet[VEL_OFFSET:VEL_OFFSET + 2], 'little', signed=True), 0)
        self.assertEqual(packet[DIST_OFFSET], 1)
        self.assertEqual(int.from_bytes(packet[DIST_OFFSET + 1:DIST_OFFSET + 3], 'little'), 270)

    def test_vectors_match_expected_hex_and_checksum(self) -> None:
        vectors = _load_vectors()
        for vector in vectors.values():
            inputs = vector['inputs']
            packet = build_profile_packet(
                inputs['profile'],
                inputs['sequence'],
                inputs['remote_tick_ms'],
                inputs['pos_x_cm'],
                inputs['pos_y_cm'],
                inputs['pos_z_cm'],
                inputs['vel_x_cms'],
                inputs['vel_y_cms'],
                inputs['vel_z_cms'],
                inputs['dist_direction'],
                inputs['dist_angle_deg'],
                inputs['dist_cm'],
            )
            self.assertEqual(len(packet), vector['expected_length'])
            self.assertEqual(packet.hex(), vector['expected_hex'])
            self.assertEqual(int.from_bytes(packet[-2:], 'little'), vector['expected_checksum'])

    def test_round_trip_decode_matches_known_fields(self) -> None:
        packet = build_profile_packet('all_valid', 1, 1234, 100, 0, 120, 0, 0, 0, 1, 270, 120)
        frame = decode_external_observation_frame(packet)
        self.assertEqual(frame.sequence, 1)
        self.assertEqual(frame.remote_tick_ms, 1234)
        self.assertEqual(frame.pos_x_cm, 100)
        self.assertEqual(frame.pos_z_cm, 120)
        self.assertEqual(frame.dist_direction, 1)
        self.assertEqual(frame.dist_angle_deg, 270)


if __name__ == '__main__':
    unittest.main()
