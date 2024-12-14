import math
import struct
from enum import Enum

"""
Enum class for the names of the VESC firmware versions
"""


class FirmwareVersion(Enum):
    FW_6_00 = "6.00"


"""
The following methods are utilities that are available to all implemented firmware versions.
"""


def float32_to_bytes(number) -> bytes:
    """
    See [buffer.c](https://github.com/vedderb/bldc/blob/master/util/buffer.c)
    in VESC bldc source code on Github for an explanation of this method.
    """
    # Handle subnormal numbers
    if abs(number) < 1.5e-38:
        number = 0.0

    # Decompose number into significand and exponent
    sig, e = math.frexp(number)
    sig_abs = abs(sig)
    sig_i = 0
    if sig_abs >= 0.5:
        sig_i = int((sig_abs - 0.5) * 2.0 * 8388608.0)
        e += 126

    # Construct the 32-bit representation
    res = (e & 0xFF) << 23 | (sig_i & 0x7FFFFF)
    if sig < 0:
        res |= 1 << 31

    # Pack the result into bytes using struct.pack, assuming big-endian format
    packed_result = struct.pack(">I", res)
    return packed_result
