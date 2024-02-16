import pytest
import struct
from augmented_skateboarding_simulator.vesc.fw_6_00 import (
    FirmwareMessage,
    StateMessage,
    IMUStateMessage,
)
from math import ldexp
import struct
import math


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


class TestStateMessage:
    def test_initial_state(self):
        """Test initial state of StateMessage."""
        msg = StateMessage()
        assert msg.duty_cycle == 0
        assert msg.rpm == 0
        assert msg.motor_current == 0
        assert msg.input_voltage == 0

    def test_setting_properties(self):
        """Test setting properties of StateMessage."""
        msg = StateMessage()
        msg.duty_cycle = 0.5
        msg.rpm = 1200
        msg.motor_current = 1.5
        msg.input_voltage = 12.5

        assert msg.duty_cycle == 0.5
        assert msg.rpm == 1200
        assert msg.motor_current == 1.5
        assert msg.input_voltage == 12.5

    def test_buffer_property(self):
        """Test the buffer property to ensure correct byte structure."""
        msg = StateMessage()
        msg.duty_cycle = 0.5
        msg.rpm = 1200
        msg.motor_current = 1.5
        msg.input_voltage = 12.5

        buffer = msg.buffer
        assert len(buffer) == 76  # Check buffer size
        # Decode specific fields to verify correct packing
        unpacked_mc = struct.unpack(">I", buffer[9:13])[0]
        unpacked_dc = struct.unpack(">H", buffer[25:27])[0]
        unpacked_rpm = struct.unpack(">I", buffer[27:31])[0]
        unpacked_iv = struct.unpack(">H", buffer[31:33])[0]

        assert unpacked_mc == int(1.5 * 100)
        assert unpacked_dc == int(0.5 * 1000)
        assert unpacked_rpm == 1200
        assert unpacked_iv == int(12.5 * 10)


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


class TestIMUStateMessage:
    def test_buffer_property(self):
        # Create an instance of IMUStateMessage with some sample data
        message = IMUStateMessage()
        # Assuming setter methods or direct attribute access is possible to simulate real data
        message.rpy[0] = 1.0
        message.rpy[1] = 2.0
        message.rpy[2] = 3.0
        message.acc[0] = 0.1
        message.acc[1] = 0.2
        message.acc[2] = 0.3
        message.gyro[0] = 0.01
        message.gyro[1] = 0.02
        message.gyro[2] = 0.03
        message.mag[0] = 0.4
        message.mag[1] = 0.5
        message.mag[2] = 0.6
        message.q[0] = 1.25
        message.q[1] = 0
        message.q[2] = 0
        message.q[3] = 0

        # Fetch the buffer
        buf = message.buffer
        yaw = bytes_to_float32(struct.unpack(">I", buf[9:13])[0])
        assert yaw == message.rpy[2]
        acc_z = bytes_to_float32(struct.unpack(">I", buf[21:25])[0])
        assert math.isclose(acc_z, message.acc[2], rel_tol=1e-6)
        q1 = bytes_to_float32(struct.unpack(">I", buf[49:53])[0])
        assert math.isclose(q1, message.q[0], rel_tol=1e-6)
