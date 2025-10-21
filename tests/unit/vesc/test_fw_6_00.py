import pytest
from unittest import mock
import struct
from bionic_boarder_simulation_tool.riding.frictional_deceleration_model import FrictionalDecelerationModel
from bionic_boarder_simulation_tool.vesc.fw_6_00 import (
    FirmwareMessage,
    StateMessage,
    IMUStateMessage,
    BionicBoarderMessage,
    FW6_00CMP,
)
from math import ldexp
import struct
import math
from threading import Lock
import time
from bionic_boarder_simulation_tool.riding.battery_discharge_model import BatteryDischargeModel
from bionic_boarder_simulation_tool.riding.eboard_kinematic_state import EboardKinematicState
from bionic_boarder_simulation_tool.riding.motor_controller import MotorController
from bionic_boarder_simulation_tool.riding.eboard import EBoard


def test_firmware_message_initialization():
    msg = FirmwareMessage()
    assert len(msg.buffer) == FirmwareMessage.BYTE_LENGTH, "Buffer length does not match expected length."

    # Test initial values set in buffer
    assert msg.buffer[0] == 6, "Second byte of buffer should be 6."
    assert msg.buffer[1] == 0, "Third byte of buffer should be 0."
    assert msg.buffer[2:14] == b"HardwareName", "Bytes 3-15 should be encoded 'HardwareName'."


def test_firmware_message_buffer_property():
    msg = FirmwareMessage()
    buffer = msg.buffer
    assert isinstance(buffer, bytes), "Buffer property should return a bytes object."
    assert buffer == bytes(msg._FirmwareMessage__buffer), "Buffer property does not return expected byte array."


class TestStateMessage:
    def test_initial_state(self):
        """Test initial state of StateMessage."""
        msg = StateMessage()
        assert msg.rpm == 0
        assert msg.motor_current == 0
        assert msg.watt_hours == 0

    def test_setting_properties(self):
        """Test setting properties of StateMessage."""
        msg = StateMessage()
        msg.rpm = 1200
        msg.motor_current = 1.5
        msg.watt_hours = 12.5

        assert msg.rpm == 1200
        assert msg.motor_current == 1.5
        assert msg.watt_hours == 12.5

    def test_buffer_property(self):
        """Test the buffer property to ensure correct byte structure."""
        msg = StateMessage()
        msg.rpm = 1200
        msg.motor_current = 1.5
        msg.watt_hours = 12.5

        buffer = msg.buffer
        assert len(buffer) == 74  # Check buffer size
        # Decode specific fields to verify correct packing
        unpacked_mc = struct.unpack(">I", buffer[4:8])[0]
        unpacked_rpm = struct.unpack(">i", buffer[22:26])[0]
        unpacked_wh = struct.unpack(">I", buffer[36:40])[0]

        assert unpacked_mc == int(1.5 * 100)
        assert unpacked_rpm == 1200
        assert unpacked_wh == int(12.5 * 10000)


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
        """Test the buffer property to ensure correct byte structure."""
        message = IMUStateMessage()

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
        yaw = bytes_to_float32(struct.unpack(">I", buf[10:14])[0])
        assert yaw == message.rpy[2]
        acc_z = bytes_to_float32(struct.unpack(">I", buf[22:26])[0])
        assert math.isclose(acc_z, message.acc[2], rel_tol=1e-6)
        q0 = bytes_to_float32(struct.unpack(">I", buf[50:54])[0])
        assert math.isclose(q0, message.q[0], rel_tol=1e-6)


class TestBionicBoarderMessage:
    def test_buffer_property(self):
        """Test the buffer property to ensure correct byte structure."""
        message = BionicBoarderMessage()

        message.motor_current = 12.34
        message.duty_cycle = 0.567
        message.rpm = 1500
        message.acc[0] = 0.1
        message.acc[1] = 0.2
        message.acc[2] = 0.3
        message.rpy[0] = 1.0
        message.rpy[1] = 2.0
        message.rpy[2] = 3.0

        buf = message.buffer
        motor_current = struct.unpack(">i", buf[0:4])[0] / 100.0
        assert math.isclose(motor_current, message.motor_current, rel_tol=1e-6)
        duty_cycle = struct.unpack(">h", buf[4:6])[0] / 1000.0
        assert math.isclose(duty_cycle, message.duty_cycle, rel_tol=1e-6)
        rpm = struct.unpack(">i", buf[6:10])[0]
        assert rpm == message.rpm
        acc_z = bytes_to_float32(struct.unpack(">I", buf[18:22])[0])
        assert math.isclose(acc_z, message.acc[2], rel_tol=1e-6)
        yaw = bytes_to_float32(struct.unpack(">I", buf[30:34])[0])
        assert math.isclose(yaw, message.rpy[2], rel_tol=1e-6)


""" 
Unit tests for class FW6_00CMP
"""


@pytest.fixture
def mock_serial(mocker):
    return mocker.patch("serial.Serial", autospec=True)


def test_firmware_command(mock_serial):
    cmp = FW6_00CMP(
        "COM1",
        230400,
        8,
        None,
        None,
        None,
        None,
    )
    cmp._publish_firmware()
    data = b"\x02@\x00\x06\x00HardwareName" + bytes(50)
    mock_serial.return_value.write.assert_called_once_with(data)


def test_state_command(mock_serial):
    cmp = FW6_00CMP(
        "COM1",
        230400,
        8,
        EboardKinematicState(0, 0, 0, 0, 0, 0, 0, 0, 0),
        Lock(),
        BatteryDischargeModel(42.0),
        None,
    )
    cmp._publish_state()
    data = b"\x02J\x04" + bytes(74)
    mock_serial.return_value.write.assert_called_once_with(data)


def test_imu_state_command(mock_serial):
    cmp = FW6_00CMP(
        "COM1", 230400, 8, EboardKinematicState(0, 0, 0, 0, 0, 0, 0, 0, 0), Lock(), BatteryDischargeModel(42.0), None
    )
    cmp._publish_imu_state()
    data = b"\x02DA" + bytes(68)
    mock_serial.return_value.write.assert_called_once_with(data)


def test_update_rpm(mock_serial):
    eks = EboardKinematicState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    eks_lock = Lock()
    eb = EBoard(
        total_weight_with_rider_kg=80.0,
        frontal_area_of_rider_m2=0.5,
        wheel_diameter_m=0.1,
        battery_max_capacity_Ah=10.0,
        battery_max_voltage=36.0,
        gear_ratio=2.0,
        motor_kv=190,
        motor_max_torque=6.0,
        motor_max_amps=50.0,
        motor_max_power_watts=500.0,
        motor_pole_pairs=7,
    )
    fdm = FrictionalDecelerationModel(mu_rolling=0.01, c_drag=0.8, eboard=eb)
    mc = MotorController(eb, eks, eks_lock, fdm)
    mc.control_time_step_ms = 20
    mc.start()
    assert eks.erpm == 0
    cmp = FW6_00CMP("COM1", 230400, 8, eks, eks_lock, BatteryDischargeModel(42.0), mc)

    # Set the RPM to 1000 to create a speed increase
    rpm = 1000
    command = bytes(3) + rpm.to_bytes(4, "big")
    cmp._update_rpm(command)
    assert rpm == mc.target_erpm
    while eks.erpm < mc.target_erpm:
        pass
    assert eks.erpm >= mc.target_erpm

    # Now set RPM to 500 to create a speed decrease
    rpm = 500
    assert rpm < mc.target_erpm
    assert rpm < eks.erpm
    command = bytes(3) + rpm.to_bytes(4, "big")
    cmp._update_rpm(command)
    assert rpm == mc.target_erpm
    while eks.erpm > mc.target_erpm:
        pass
    assert eks.erpm <= mc.target_erpm
    mc.stop()
