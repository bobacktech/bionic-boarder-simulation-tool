import pytest
from bionic_boarder_simulation_tool.vesc import fw
from math import ldexp
import struct
import math


def bytes_to_float32(res: int) -> float:
    e = (res >> 23) & 0xFF
    sig_i = res & 0x7FFFFF
    neg = res & (1 << 31) != 0

    sig = 0.0
    if e != 0 or sig_i != 0:
        sig = sig_i / (8388608.0 * 2.0) + 0.5
        e -= 126

    if neg:
        sig = -sig

    return ldexp(sig, e)


def test_float32_to_bytes_and_back():
    # Test values including edge cases
    test_values = [0.0, 1.0, -1.0, 34.5, -34.5, 2.4567, -2.4567]

    for val in test_values:
        # Convert float to bytes
        bytes_val = fw.float32_to_bytes(val)

        # Ensure the result is bytes and has the correct length (4 bytes for a 32-bit float)
        assert isinstance(bytes_val, bytes)
        assert len(bytes_val) == 4

        # Convert back to float
        unpacked_val = struct.unpack(">I", bytes_val)[0]
        float_val = bytes_to_float32(unpacked_val)

        # Assert the conversion back to float is accurate within an acceptable error margin
        # Note: Due to floating-point arithmetic precision, a direct comparison might not always work
        assert math.isclose(val, float_val, rel_tol=1e-6), f"Original: {val}, Converted: {float_val}"
