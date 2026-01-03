import pytest
import struct
from bionic_boarder_simulation_tool.riding.frictional_deceleration_model import FrictionalDecelerationModel
from bionic_boarder_simulation_tool.vesc.fw_6_00 import (
    FirmwareMessage,
    MotorControllerConfigurationMessage,
    StateMessage,
    BionicBoarderMessage,
    FW6_00CMP,
)
from math import ldexp
import struct
import math
from threading import Lock
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


class TestMotorControllerConfigurationMessage:
    def test_buffer_property(self):
        """
        Verifies that the buffer property returns a bytes object of the correct length.
        """
        message = MotorControllerConfigurationMessage()
        buffer = message.buffer
        assert isinstance(buffer, bytes), "Buffer property should return a bytes object."
        assert (
            len(buffer) == MotorControllerConfigurationMessage.BYTE_LENGTH
        ), f"Buffer length should be {MotorControllerConfigurationMessage.BYTE_LENGTH}, got {len(buffer)}."

    def test_buffer_encoding(self):
        """
        Verifies that setting properties on the object results in the correct
        byte sequences at the specific offsets defined in the class.
        """
        message = MotorControllerConfigurationMessage()

        # 1. Set arbitrary test values
        test_current_max = 123.45
        test_max_vin = 56.78
        test_watt_max = 999.99
        test_flux_linkage = 0.0025
        test_motor_poles = 14  # Integer
        test_gear_ratio = 3.5
        test_wheel_diameter = 0.083
        test_battery_ah = 12.5

        message.l_current_max = test_current_max
        message.l_max_vin = test_max_vin
        message.l_watt_max = test_watt_max
        message.foc_motor_flux_linkage = test_flux_linkage
        message.si_motor_poles = test_motor_poles
        message.si_gear_ratio = test_gear_ratio
        message.si_wheel_diameter = test_wheel_diameter
        message.si_battery_ah = test_battery_ah

        # 2. Generate the buffer
        buf = message.buffer

        # 3. Assert that the bytes at specific offsets match the struct.pack expectation
        # Note: The class uses Big-Endian (>f) for floats

        # l_current_max: 0-3
        assert buf[0:4] == struct.pack(">f", test_current_max), "l_current_max encoding failed"

        # l_max_vin: 44-47
        assert buf[44:48] == struct.pack(">f", test_max_vin), "l_max_vin encoding failed"

        # l_watt_max: 85-88
        assert buf[85:89] == struct.pack(">f", test_watt_max), "l_watt_max encoding failed"

        # foc_motor_flux_linkage: 222-225
        assert buf[222:226] == struct.pack(">f", test_flux_linkage), "foc_motor_flux_linkage encoding failed"

        # si_motor_poles: 644 (Single byte integer)
        assert buf[644] == test_motor_poles, "si_motor_poles encoding failed"

        # si_gear_ratio: 645-648
        assert buf[645:649] == struct.pack(">f", test_gear_ratio), "si_gear_ratio encoding failed"

        # si_wheel_diameter: 649-652
        assert buf[649:653] == struct.pack(">f", test_wheel_diameter), "si_wheel_diameter encoding failed"

        # si_battery_ah: 661-664
        assert buf[661:665] == struct.pack(">f", test_battery_ah), "si_battery_ah encoding failed"


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
        None,
    )
    cmp._publish_firmware()
    data = b"\x02@\x00\x06\x00HardwareName" + bytes(50)
    mock_serial.return_value.write.assert_called_once_with(data)


def test_motor_controller_configuration_command(mock_serial):
    cmp = FW6_00CMP(
        "COM1",
        230400,
        8,
        EBoard(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        None,
        None,
        None,
        None,
    )
    cmp._publish_motor_controller_configuration()
    data = b"\x03\x02\xb8\x0e" + bytes(696)
    mock_serial.return_value.write.assert_called_once_with(data)


def test_state_command(mock_serial):
    cmp = FW6_00CMP(
        "COM1",
        230400,
        8,
        None,
        EboardKinematicState(0, 0, 0, 0, 0, 0, 0, 0, 0),
        Lock(),
        BatteryDischargeModel(42.0),
        None,
    )
    cmp._publish_state()
    data = b"\x02J\x04" + bytes(74)
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
    cmp = FW6_00CMP("COM1", 230400, 8, None, eks, eks_lock, BatteryDischargeModel(42.0), mc)

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
