from typing import Optional

from ano_receiver_bridge.checksum import verify_checksums
from ano_receiver_bridge.protocol_types import AnoFrame, FRAME_HEAD


class AnoFrameParser:
    """Incremental parser that accepts bytes one by one and resynchronizes on header loss."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed_byte(self, value: int) -> Optional[AnoFrame]:
        self._buffer.append(value & 0xFF)

        while self._buffer and self._buffer[0] != FRAME_HEAD:
            del self._buffer[0]

        if len(self._buffer) < 4:
            return None

        payload_length = self._buffer[3]
        total_length = 4 + payload_length + 2
        if len(self._buffer) < total_length:
            return None

        raw_frame = bytes(self._buffer[:total_length])
        del self._buffer[:total_length]

        frame_without_checksums = raw_frame[:-2]
        sumcheck = raw_frame[-2]
        addcheck = raw_frame[-1]
        if not verify_checksums(frame_without_checksums, sumcheck, addcheck):
            self._resync_after_invalid_frame(raw_frame)
            return None

        return AnoFrame(
            head=raw_frame[0],
            address=raw_frame[1],
            frame_id=raw_frame[2],
            length=payload_length,
            payload=raw_frame[4:-2],
            sumcheck=sumcheck,
            addcheck=addcheck,
        )

    def _resync_after_invalid_frame(self, raw_frame: bytes) -> None:
        # Keep a sliding window so an embedded 0xAA can still seed the next frame.
        trailing = raw_frame[1:] + bytes(self._buffer)
        self._buffer = bytearray(trailing)
        while self._buffer and self._buffer[0] != FRAME_HEAD:
            del self._buffer[0]
