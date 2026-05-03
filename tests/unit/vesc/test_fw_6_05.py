from filecmp import cmp

import pytest
from bionic_boarder_simulation_tool.riding.frictional_deceleration_model import FrictionalDecelerationModel
from bionic_boarder_simulation_tool.vesc.fw_6_05 import (
    FirmwareMessage,
    MotorControllerConfigurationMessage,
    BionicBoarderMessage,
    FW6_05CMP,
)
from bionic_boarder_simulation_tool.riding.battery_discharge_model import BatteryDischargeModel
from bionic_boarder_simulation_tool.riding.eboard_kinematic_state import EboardKinematicState
from bionic_boarder_simulation_tool.riding.motor_controller import MotorController
from bionic_boarder_simulation_tool.riding.eboard import EBoard
from math import ldexp
from threading import Lock
import struct
import math


class TestFirmwareMessage:
    def test_initialization(self):
        msg = FirmwareMessage()

        # Test initial values set in buffer
        buf = msg.buffer[1:]  # Skip the first byte which is the message type
        assert buf[0] == 6, "Second byte of buffer should be 6."
        assert buf[1] == 5, "Third byte of buffer should be 5."
        assert buf[2:14] == b"HardwareName", "Bytes 3-15 should be encoded 'HardwareName'."


class TestBionicBoarderMessage:
    def test_buffer_property(self):
        msg = BionicBoarderMessage()
        msg.motor_current = 12.34
        msg.duty_cycle = 0.567
        msg.rpm = 1500
        msg.acc[0] = 0.1
        msg.acc[1] = 0.2
        msg.acc[2] = 0.3
        msg.rpy[0] = 1.0
        msg.rpy[1] = 2.0
        msg.rpy[2] = 3.0

        buf = msg.buffer[1:]  # Skip the first byte which is the message type
        motor_current = struct.unpack(">i", buf[4:8])[0] / 100.0
        assert math.isclose(motor_current, msg.avg_motor_current, rel_tol=1e-6)
        duty_cycle = struct.unpack(">h", buf[20:22])[0] / 1000.0
        assert math.isclose(duty_cycle, msg.duty_cycle, rel_tol=1e-6)
        rpm = struct.unpack(">i", buf[22:26])[0]
        assert rpm == msg.rpm
        acc_z = struct.unpack(">f", buf[73:77])[0]
        assert math.isclose(acc_z, msg.acc[2], rel_tol=1e-6)
        yaw = struct.unpack(">f", buf[85:89])[0]
        assert math.isclose(yaw, msg.rpy[2], rel_tol=1e-6)


class TestMotorControllerConfigurationMessage:
    """Unit tests for MotorControllerConfigurationMessage."""

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @pytest.fixture
    def msg(self):
        """Return a fresh default-constructed message."""
        return MotorControllerConfigurationMessage()

    MCCONF_SIGNATURE = 1065524471  # 0x3F6A2C57

    # ------------------------------------------------------------------
    # buffer property – structural checks
    # ------------------------------------------------------------------

    def test_buffer_returns_bytes(self, msg):
        assert isinstance(msg.buffer, bytes)

    def test_buffer_first_byte_is_message_id(self, msg):
        assert msg.buffer[0] == MotorControllerConfigurationMessage.ID

    def test_buffer_contains_mcconf_signature(self, msg):
        buf = msg.buffer
        # Signature starts at byte 1 (after the 1-byte message ID)
        sig = struct.unpack(">I", buf[1:5])[0]
        assert sig == self.MCCONF_SIGNATURE

    def test_buffer_is_deterministic(self, msg):
        assert msg.buffer == msg.buffer

    def test_buffer_changes_when_field_modified(self, msg):
        original = msg.buffer
        msg.l_current_max = 100.0
        assert msg.buffer != original

    def test_buffer_pwm_mode_byte(self, msg):
        msg.pwm_mode = MotorControllerConfigurationMessage.mc_pwm_mode.PWM_MODE_BIPOLAR
        buf = msg.buffer
        # byte 5 = pwm_mode (after 1-byte ID + 4-byte signature)
        assert buf[5] == MotorControllerConfigurationMessage.mc_pwm_mode.PWM_MODE_BIPOLAR

    def test_buffer_comm_mode_byte(self, msg):
        msg.comm_mode = MotorControllerConfigurationMessage.mc_comm_mode.COMM_MODE_DELAY
        buf = msg.buffer
        assert buf[6] == MotorControllerConfigurationMessage.mc_comm_mode.COMM_MODE_DELAY

    def test_buffer_motor_type_byte(self, msg):
        msg.motor_type = MotorControllerConfigurationMessage.mc_motor_type.MOTOR_TYPE_BLDC
        buf = msg.buffer
        assert buf[7] == MotorControllerConfigurationMessage.mc_motor_type.MOTOR_TYPE_BLDC

    def test_buffer_sensor_mode_byte(self, msg):
        msg.sensor_mode = MotorControllerConfigurationMessage.mc_sensor_mode.SENSOR_MODE_SENSORED
        buf = msg.buffer
        assert buf[8] == MotorControllerConfigurationMessage.mc_sensor_mode.SENSOR_MODE_SENSORED

    def test_buffer_l_current_max_encoding(self, msg):
        msg.l_current_max = 42.5
        buf = msg.buffer
        # l_current_max is the first float after the 4-byte mode block (offset 9)
        (val,) = struct.unpack(">f", buf[9:13])
        assert abs(val - 42.5) < 1e-4

    def test_buffer_l_current_min_encoding(self, msg):
        msg.l_current_min = -10.0
        buf = msg.buffer
        (val,) = struct.unpack(">f", buf[13:17])
        assert abs(val - (-10.0)) < 1e-4

    def test_buffer_bms_fwd_can_mode_last_byte(self, msg):
        msg.bms.fwd_can_mode = MotorControllerConfigurationMessage.BMS_FWD_CAN_MODE.BMS_FWD_CAN_MODE_ANY
        buf = msg.buffer
        assert buf[-1] == MotorControllerConfigurationMessage.BMS_FWD_CAN_MODE.BMS_FWD_CAN_MODE_ANY


@pytest.fixture
def mock_serial(mocker):
    return mocker.patch("serial.Serial", autospec=True)


class TestFW6_05CMP:
    def test_firmware_command(self, mock_serial):
        cmp = FW6_05CMP(
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

    def test_motor_controller_configuration_command(self, mock_serial):
        cmp = FW6_05CMP(
            "COM1",
            230400,
            8,
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

    def test_bionic_boarder_command(self, mock_serial):
        cmp = FW6_05CMP(
            "COM1",
            230400,
            256,
            None,
            EboardKinematicState(0, 0, 0, 0, 0, 0, 0, 0, 0),
            Lock(),
            None,
            None,
        )
        buffer = BionicBoarderMessage().buffer
        crc = cmp.crc16(buffer)
        cmp._publish_bionic_boarder()
        data = int.to_bytes(2) + int.to_bytes(len(buffer)) + buffer + int.to_bytes(crc, 2) + int.to_bytes(0x03)
        mock_serial.return_value.write.assert_called_once_with(data)

    def test_update_rpm(self, mock_serial):
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
        cmp = FW6_05CMP("COM1", 230400, 8, None, eks, eks_lock, BatteryDischargeModel(42.0), mc)

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
