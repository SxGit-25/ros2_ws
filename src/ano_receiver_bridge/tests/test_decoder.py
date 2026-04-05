import unittest

from ano_receiver_bridge.decoder import decode_frame
from ano_receiver_bridge.protocol_types import (
    FLOW_OBS_FRAME_ID,
    IMU_RAW_FRAME_ID,
    AnoFrame,
    FlowObsData,
    ImuRawData,
)


def _make_frame(frame_id: int, payload: bytes) -> AnoFrame:
    return AnoFrame(
        head=0xAA,
        address=0xFF,
        frame_id=frame_id,
        length=len(payload),
        payload=payload,
        sumcheck=0,
        addcheck=0,
    )


class DecoderTests(unittest.TestCase):
    def test_decode_imu_raw_frame(self) -> None:
        payload = bytes.fromhex('64009cff2c011000e0ff3000')
        decoded = decode_frame(_make_frame(IMU_RAW_FRAME_ID, payload))

        self.assertIsInstance(decoded, ImuRawData)
        self.assertEqual(decoded.acc_x, 100)
        self.assertEqual(decoded.acc_y, -100)
        self.assertEqual(decoded.acc_z, 300)
        self.assertEqual(decoded.gyr_x, 16)
        self.assertEqual(decoded.gyr_y, -32)
        self.assertEqual(decoded.gyr_z, 48)

    def test_decode_flow_obs_frame(self) -> None:
        payload = bytes.fromhex('d2042efb01c82c010000')
        decoded = decode_frame(_make_frame(FLOW_OBS_FRAME_ID, payload))

        self.assertIsInstance(decoded, FlowObsData)
        self.assertEqual(decoded.flow_vx, 12.34)
        self.assertEqual(decoded.flow_vy, -12.34)
        self.assertEqual(decoded.flow_state, 1)
        self.assertEqual(decoded.flow_quality, 200)
        self.assertEqual(decoded.alt_cm, 300)


if __name__ == '__main__':
    unittest.main()
