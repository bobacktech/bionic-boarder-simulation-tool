import pytest
from augmented_skateboarding_simulator.vesc.fw_6_00 import (
    FirmwareMessage,
)


def test_firmware_message_initialization():
    msg = FirmwareMessage()
    assert (
        len(msg.buffer) == FirmwareMessage.BYTE_LENGTH
    ), "Buffer length does not match expected length."

    # Test initial values set in buffer
    assert msg.buffer[0] == 0, "First byte of buffer should be 0."
    assert msg.buffer[1] == 6, "Second byte of buffer should be 6."
    assert msg.buffer[2] == 0, "Third byte of buffer should be 0."
    assert (
        msg.buffer[3:15] == b"HardwareName"
    ), "Bytes 3-15 should be encoded 'HardwareName'."


def test_firmware_message_buffer_property():
    msg = FirmwareMessage()
    buffer = msg.buffer
    assert isinstance(buffer, bytes), "Buffer property should return a bytes object."
    assert buffer == bytes(
        msg._FirmwareMessage__buffer
    ), "Buffer property does not return expected byte array."
