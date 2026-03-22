from typing import Tuple


def compute_checksums(data: bytes) -> Tuple[int, int]:
    """Compute ANO sumcheck and addcheck over HEAD..DATA."""
    sumcheck = 0
    addcheck = 0
    for value in data:
        sumcheck = (sumcheck + value) & 0xFF
        addcheck = (addcheck + sumcheck) & 0xFF
    return sumcheck, addcheck


def verify_checksums(data: bytes, sumcheck: int, addcheck: int) -> bool:
    expected_sumcheck, expected_addcheck = compute_checksums(data)
    return expected_sumcheck == sumcheck and expected_addcheck == addcheck
