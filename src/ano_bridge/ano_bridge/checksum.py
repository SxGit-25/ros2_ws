from typing import Tuple


def compute_checksums(data: bytes) -> Tuple[int, int]:
    sumcheck = 0
    addcheck = 0
    for value in data:
        sumcheck = (sumcheck + value) & 0xFF
        addcheck = (addcheck + sumcheck) & 0xFF
    return sumcheck, addcheck


def verify_checksums(frame_without_checksums: bytes, sumcheck: int, addcheck: int) -> bool:
    expected_sumcheck, expected_addcheck = compute_checksums(frame_without_checksums)
    return expected_sumcheck == sumcheck and expected_addcheck == addcheck
