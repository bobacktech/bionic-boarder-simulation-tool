import pytest
import struct
from bionic_boarder_simulation_tool.riding.frictional_deceleration_model import FrictionalDecelerationModel
from bionic_boarder_simulation_tool.vesc.fw_6_00 import (
    FirmwareMessage,
    MotorControllerConfigurationMessage,
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

    # Test initial values set in buffer
    buf = msg.buffer[1:]  # Skip the first byte which is the message type
    assert buf[0] == 6, "Second byte of buffer should be 6."
    assert buf[1] == 0, "Third byte of buffer should be 0."
    assert buf[2:14] == b"HardwareName", "Bytes 3-15 should be encoded 'HardwareName'."


def test_firmware_message_buffer_property():
    msg = FirmwareMessage()
    buffer = msg.buffer[1:]  # Skip the first byte which is the message type
    assert isinstance(buffer, bytes), "Buffer property should return a bytes object."
    assert buffer == bytes(msg._FirmwareMessage__buffer), "Buffer property does not return expected byte array."


class TestBionicBoarderMessage:
    def test_buffer_property(self):
        """Test the buffer property to ensure correct byte structure."""
        message = BionicBoarderMessage()

        message.avg_motor_current = 12.34
        message.duty_cycle = 0.567
        message.rpm = 1500
        message.acc[0] = 0.1
        message.acc[1] = 0.2
        message.acc[2] = 0.3
        message.rpy[0] = 1.0
        message.rpy[1] = 2.0
        message.rpy[2] = 3.0

        buf = message.buffer[1:]
        motor_current = struct.unpack(">i", buf[4:8])[0] / 100.0
        assert math.isclose(motor_current, message.avg_motor_current, rel_tol=1e-6)
        duty_cycle = struct.unpack(">h", buf[20:22])[0] / 1000.0
        assert math.isclose(duty_cycle, message.duty_cycle, rel_tol=1e-6)
        rpm = struct.unpack(">i", buf[22:26])[0]
        assert rpm == message.rpm
        acc_z = struct.unpack(">f", buf[73:77])[0]
        assert math.isclose(acc_z, message.acc[2], rel_tol=1e-6)
        yaw = struct.unpack(">f", buf[85:89])[0]
        assert math.isclose(yaw, message.rpy[2], rel_tol=1e-6)


MCCONF_SIGNATURE = 776184161  # 0x2E4B7631


class TestMotorControllerConfigurationMessage:

    @staticmethod
    def _parse_buffer(data: bytes) -> dict:
        """Unpack the raw buffer into a flat dict for easy assertion."""
        offset = 0

        def read(fmt):
            nonlocal offset
            size = struct.calcsize(fmt)
            values = struct.unpack_from(fmt, data, offset)
            offset += size
            return values

        result = {}

        result["cmd_id"] = data[0]
        offset = 1

        (result["signature"],) = read(">I")
        result["pwm_mode"], result["comm_mode"], result["motor_type"], result["sensor_mode"] = read(">BBBB")

        (
            result["l_current_max"],
            result["l_current_min"],
            result["l_in_current_max"],
            result["l_in_current_min"],
            result["l_abs_current_max"],
            result["l_min_erpm"],
            result["l_max_erpm"],
        ) = read(">fffffff")

        (result["l_erpm_start_raw"],) = read(">h")
        result["l_max_erpm_fbrake"], result["l_max_erpm_fbrake_cc"] = read(">ff")
        result["l_min_vin"], result["l_max_vin"] = read(">ff")
        result["l_battery_cut_start"], result["l_battery_cut_end"] = read(">ff")
        (result["l_slow_abs_current"],) = read(">B")

        (
            result["l_temp_fet_start_raw"],
            result["l_temp_fet_end_raw"],
            result["l_temp_motor_start_raw"],
            result["l_temp_motor_end_raw"],
        ) = read(">hhhh")

        result["l_temp_accel_dec_raw"], result["l_min_duty_raw"], result["l_max_duty_raw"] = read(">hhh")

        result["l_watt_max"], result["l_watt_min"] = read(">ff")

        result["l_current_max_scale_raw"], result["l_current_min_scale_raw"], result["l_duty_start_raw"] = read(">hhh")

        return result  # extend as needed for deeper tests

    # ------------------------------------------------------------------
    # Basic structural tests
    # ------------------------------------------------------------------

    def test_buffer_returns_bytes(self):
        msg = MotorControllerConfigurationMessage()
        assert isinstance(msg.buffer, bytes)

    def test_buffer_first_byte_is_command_id(self):
        msg = MotorControllerConfigurationMessage()
        assert msg.buffer[0] == MotorControllerConfigurationMessage.ID  # 14

    def test_buffer_signature(self):
        msg = MotorControllerConfigurationMessage()
        parsed = self._parse_buffer(msg.buffer)
        assert parsed["signature"] == MCCONF_SIGNATURE

    def test_buffer_minimum_length(self):
        """Buffer must be at least long enough to hold the signature + cmd byte."""
        msg = MotorControllerConfigurationMessage()
        assert len(msg.buffer) > 5

    def test_buffer_is_deterministic(self):
        """Calling buffer twice on the same object returns identical bytes."""
        msg = MotorControllerConfigurationMessage()
        assert msg.buffer == msg.buffer

    # ------------------------------------------------------------------
    # Default-value encoding tests
    # ------------------------------------------------------------------

    def test_default_pwm_mode_encoded(self):
        msg = MotorControllerConfigurationMessage()
        parsed = self._parse_buffer(msg.buffer)
        assert parsed["pwm_mode"] == int(MotorControllerConfigurationMessage.mc_pwm_mode.PWM_MODE_SYNCHRONOUS)

    def test_default_comm_mode_encoded(self):
        msg = MotorControllerConfigurationMessage()
        parsed = self._parse_buffer(msg.buffer)
        assert parsed["comm_mode"] == int(MotorControllerConfigurationMessage.mc_comm_mode.COMM_MODE_INTEGRATE)

    def test_default_motor_type_encoded(self):
        msg = MotorControllerConfigurationMessage()
        parsed = self._parse_buffer(msg.buffer)
        assert parsed["motor_type"] == int(MotorControllerConfigurationMessage.mc_motor_type.MOTOR_TYPE_FOC)

    def test_default_sensor_mode_encoded(self):
        msg = MotorControllerConfigurationMessage()
        parsed = self._parse_buffer(msg.buffer)
        assert parsed["sensor_mode"] == int(MotorControllerConfigurationMessage.mc_sensor_mode.SENSOR_MODE_SENSORLESS)

    def test_default_floats_are_zero(self):
        msg = MotorControllerConfigurationMessage()
        parsed = self._parse_buffer(msg.buffer)
        for key in (
            "l_current_max",
            "l_current_min",
            "l_in_current_max",
            "l_in_current_min",
            "l_abs_current_max",
            "l_min_erpm",
            "l_max_erpm",
        ):
            assert parsed[key] == pytest.approx(0.0), f"{key} should be 0.0"

    def test_default_l_slow_abs_current_false(self):
        msg = MotorControllerConfigurationMessage()
        parsed = self._parse_buffer(msg.buffer)
        assert parsed["l_slow_abs_current"] == 0

    # ------------------------------------------------------------------
    # Round-trip / mutation tests
    # ------------------------------------------------------------------

    def test_l_erpm_start_scaled_encoding(self):
        """l_erpm_start is packed as int16 * 10000."""
        msg = MotorControllerConfigurationMessage()
        msg.l_erpm_start = 0.75
        parsed = self._parse_buffer(msg.buffer)
        assert parsed["l_erpm_start_raw"] == int(0.75 * 10000)

    def test_l_slow_abs_current_true_encodes_as_1(self):
        msg = MotorControllerConfigurationMessage()
        msg.l_slow_abs_current = True
        parsed = self._parse_buffer(msg.buffer)
        assert parsed["l_slow_abs_current"] == 1

    def test_pwm_mode_change_reflected_in_buffer(self):
        msg = MotorControllerConfigurationMessage()
        msg.pwm_mode = MotorControllerConfigurationMessage.mc_pwm_mode.PWM_MODE_BIPOLAR
        parsed = self._parse_buffer(msg.buffer)
        assert parsed["pwm_mode"] == int(MotorControllerConfigurationMessage.mc_pwm_mode.PWM_MODE_BIPOLAR)

    def test_motor_type_change_reflected_in_buffer(self):
        msg = MotorControllerConfigurationMessage()
        msg.motor_type = MotorControllerConfigurationMessage.mc_motor_type.MOTOR_TYPE_DC
        parsed = self._parse_buffer(msg.buffer)
        assert parsed["motor_type"] == int(MotorControllerConfigurationMessage.mc_motor_type.MOTOR_TYPE_DC)

    # ------------------------------------------------------------------
    # BMS sub-struct tests
    # ------------------------------------------------------------------

    def test_bms_type_default_encoded(self):
        msg = MotorControllerConfigurationMessage()
        # BMS type byte is near the end; just verify the buffer ends with
        # the expected BMS_TYPE_NONE (0) and limit_mode (0) bytes.
        buf = msg.buffer
        # Find bms section: last 11 bytes = BB + hhhh + B
        bms_type, limit_mode = struct.unpack_from(">BB", buf, len(buf) - 11)
        assert bms_type == int(MotorControllerConfigurationMessage.BMS_TYPE.BMS_TYPE_NONE)
        assert limit_mode == 0

    def test_bms_fwd_can_mode_default_encoded(self):
        msg = MotorControllerConfigurationMessage()
        buf = msg.buffer
        (fwd_can_mode,) = struct.unpack_from(">B", buf, len(buf) - 1)
        assert fwd_can_mode == int(MotorControllerConfigurationMessage.BMS_FWD_CAN_MODE.BMS_FWD_CAN_MODE_DISABLED)


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
        256,
        None,
        None,
        None,
        None,
        None,
    )
    buffer = FirmwareMessage().buffer
    crc = cmp.crc16(buffer)
    cmp._publish_firmware()
    data = int.to_bytes(2) + int.to_bytes(len(buffer)) + buffer + int.to_bytes(crc, 2) + int.to_bytes(0x03)
    mock_serial.return_value.write.assert_called_once_with(data)


def test_motor_controller_configuration_command(mock_serial):
    cmp = FW6_00CMP(
        "COM1",
        230400,
        256,
        EBoard(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        None,
        None,
        None,
        None,
    )
    buffer = MotorControllerConfigurationMessage().buffer
    crc = cmp.crc16(buffer)
    cmp._publish_motor_controller_configuration()
    data = int.to_bytes(3) + int.to_bytes(len(buffer), 2) + buffer + int.to_bytes(crc, 2) + int.to_bytes(0x03)
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
