import pytest
from bionic_boarder_simulation_tool.riding.frictional_deceleration_model import FrictionalDecelerationModel
from bionic_boarder_simulation_tool.vesc.fw_6_02 import (
    FirmwareMessage,
    MotorControllerConfigurationMessage,
    StateMessage,
    BionicBoarderMessage,
    FW6_02CMP,
)
from bionic_boarder_simulation_tool.riding.battery_discharge_model import BatteryDischargeModel
from bionic_boarder_simulation_tool.riding.eboard_kinematic_state import EboardKinematicState
from bionic_boarder_simulation_tool.riding.motor_controller import MotorController
from bionic_boarder_simulation_tool.riding.eboard import EBoard
from math import ldexp
from threading import Lock
import struct


class TestFirmwareMessage:
    def test_initialization(self):
        msg = FirmwareMessage()
        assert len(msg.buffer) == FirmwareMessage.BYTE_LENGTH, "Buffer length does not match expected length."

        # Test initial values set in buffer
        assert msg.buffer[0] == 6, "Second byte of buffer should be 6."
        assert msg.buffer[1] == 2, "Third byte of buffer should be 2."
        assert msg.buffer[2:14] == b"HardwareName", "Bytes 3-15 should be encoded 'HardwareName'."


class TestStateMessage:
    def test_initialization(self):
        msg = StateMessage()
        assert msg.temp_fet == 0
        assert msg.temp_motor == 0
        assert msg.avg_motor_current == 0.0
        assert msg.avg_input_current == 0.0
        assert msg.avg_id == 0.0
        assert msg.avg_iq == 0.0
        assert msg.duty_cycle_now == 0.0
        assert msg.vin == 0.0
        assert msg.amp_hours == 0.0
        assert msg.amp_hours_charged == 0.0
        assert msg.watt_hours == 0.0
        assert msg.watt_hours_charged == 0.0
        assert msg.tachometer == 0
        assert msg.tachometer_abs == 0
        assert msg.fault_code == 0
        assert msg.pid_pos_now == 0.0
        assert msg.current_controller_id == 0.0
        assert msg.mos1_temp == 0
        assert msg.mos2_temp == 0
        assert msg.mos3_temp == 0
        assert msg.avg_vd == 0.0
        assert msg.avg_vq == 0.0
        assert msg.status == 0

    def test_buffer_property(self):
        msg = StateMessage()
        msg.temp_fet = 45
        msg.temp_motor = 50
        msg.avg_motor_current = 12.34
        msg.avg_input_current = 23.45
        msg.avg_id = 1.23
        msg.avg_iq = 4.56
        msg.duty_cycle_now = 0.78
        msg.rpm = 1500
        msg.vin = 36.5
        msg.amp_hours = 2.34
        msg.amp_hours_charged = 3.45
        msg.watt_hours = 45.67
        msg.watt_hours_charged = 56.78
        msg.tachometer = 123456
        msg.tachometer_abs = 654321
        msg.fault_code = 2
        msg.pid_pos_now = 7.89
        msg.current_controller_id = 14
        msg.mos1_temp = 40
        msg.mos2_temp = 41
        msg.mos3_temp = 42
        msg.avg_vd = 0.34
        msg.avg_vq = 0.56
        msg.status = 1

        buffer = msg.buffer
        assert isinstance(buffer, bytes), "Buffer property should return a bytes object."
        assert len(buffer) == 74, "Buffer length does not match expected length."
        assert buffer[0:2] == int(45 * 1e1).to_bytes(
            2, byteorder="big", signed=True
        ), "FET temperature bytes incorrect."
        assert buffer[2:4] == int(50 * 1e1).to_bytes(
            2, byteorder="big", signed=True
        ), "Motor temperature bytes incorrect."
        assert buffer[4:8] == int(12.34 * 1e2).to_bytes(
            4, byteorder="big", signed=True
        ), "Average motor current bytes incorrect."
        assert buffer[8:12] == int(23.45 * 1e2).to_bytes(
            4, byteorder="big", signed=True
        ), "Average input current bytes incorrect."
        assert buffer[12:16] == int(1.23 * 1e2).to_bytes(4, byteorder="big", signed=True), "Average Id bytes incorrect."
        assert buffer[16:20] == int(4.56 * 1e2).to_bytes(4, byteorder="big", signed=True), "Average Iq bytes incorrect."
        assert buffer[20:22] == int(0.78 * 1e3).to_bytes(2, byteorder="big", signed=True), "Duty cycle bytes incorrect."
        assert buffer[22:26] == int(1500 * 1e0).to_bytes(4, byteorder="big", signed=True), "RPM bytes incorrect."
        assert buffer[26:28] == int(36.5 * 1e1).to_bytes(2, byteorder="big", signed=True), "Vin bytes incorrect."
        assert buffer[28:32] == int(2.34 * 1e4).to_bytes(4, byteorder="big", signed=True), "Amp hours bytes incorrect."
        assert buffer[32:36] == int(3.45 * 1e4).to_bytes(
            4, byteorder="big", signed=True
        ), "Amp hours charged bytes incorrect."
        assert buffer[36:40] == int(45.67 * 1e4).to_bytes(
            4, byteorder="big", signed=True
        ), "Watt hours bytes incorrect."
        assert buffer[40:44] == int(56.78 * 1e4).to_bytes(
            4, byteorder="big", signed=True
        ), "Watt hours charged bytes incorrect."
        assert buffer[44:48] == int(123456 * 1e0).to_bytes(
            4, byteorder="big", signed=True
        ), "Tachometer bytes incorrect."
        assert buffer[48:52] == int(654321 * 1e0).to_bytes(
            4, byteorder="big", signed=True
        ), "Tachometer abs bytes incorrect."
        assert buffer[52:53] == int(2 * 1e0).to_bytes(1, byteorder="big", signed=True), "Fault code bytes incorrect."
        assert buffer[53:57] == int(7.89 * 1e6).to_bytes(
            4, byteorder="big", signed=True
        ), "PID position now bytes incorrect."
        assert buffer[57:58] == int(14 * 1e0).to_bytes(
            1, byteorder="big", signed=True
        ), "Controller ID bytes incorrect."
        assert buffer[58:60] == int(40 * 1e1).to_bytes(
            2, byteorder="big", signed=True
        ), "MOS1 temperature bytes incorrect."
        assert buffer[60:62] == int(41 * 1e1).to_bytes(
            2, byteorder="big", signed=True
        ), "MOS2 temperature bytes incorrect."
        assert buffer[62:64] == int(42 * 1e1).to_bytes(
            2, byteorder="big", signed=True
        ), "MOS3 temperature bytes incorrect."
        assert buffer[64:68] == int(0.34 * 1e3).to_bytes(4, byteorder="big", signed=True), "Average Vd bytes incorrect."
        assert buffer[68:72] == int(0.56 * 1e3).to_bytes(4, byteorder="big", signed=True), "Average Vq bytes incorrect."
        assert buffer[72:73] == int(1 * 1e0).to_bytes(1, byteorder="big", signed=True), "Status bytes incorrect."


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
    def test_initialization(self):
        msg = BionicBoarderMessage()
        assert msg.motor_current == 0.0
        assert msg.duty_cycle == 0.0
        assert msg.rpm == 0
        assert msg.acc == [0.0, 0.0, 0.0]
        assert msg.rpy == [0.0, 0.0, 0.0]

    def test_buffer_property(self):
        msg = BionicBoarderMessage()
        msg.motor_current = 12.34
        msg.duty_cycle = 0.567
        msg.rpm = 1500
        msg.acc = [0.1, 0.2, 0.3]
        msg.rpy = [1.0, 2.0, 3.0]

        assert isinstance(msg.buffer, bytes), "Buffer property should return a bytes object."
        assert len(msg.buffer) == 34, "Buffer length does not match expected length."
        assert msg.buffer[0:4] == int(12.34 * 1e2).to_bytes(
            4, byteorder="big", signed=True
        ), "Motor current bytes incorrect."
        assert msg.buffer[4:6] == int(0.567 * 1e3).to_bytes(
            2, byteorder="big", signed=True
        ), "Duty cycle bytes incorrect."
        assert msg.buffer[6:10] == int(1500 * 1e0).to_bytes(4, byteorder="big", signed=True), "RPM bytes incorrect."
        assert bytes_to_float32(int.from_bytes(msg.buffer[10:14])) == pytest.approx(
            0.1, rel=1e-5
        ), "Acceleration X bytes incorrect."
        assert bytes_to_float32(int.from_bytes(msg.buffer[14:18])) == pytest.approx(
            0.2, rel=1e-5
        ), "Acceleration Y bytes incorrect."
        assert bytes_to_float32(int.from_bytes(msg.buffer[18:22])) == pytest.approx(
            0.3, rel=1e-5
        ), "Acceleration Z bytes incorrect."
        assert bytes_to_float32(int.from_bytes(msg.buffer[22:26])) == pytest.approx(
            1.0, rel=1e-5
        ), "Roll bytes incorrect."
        assert bytes_to_float32(int.from_bytes(msg.buffer[26:30])) == pytest.approx(
            2.0, rel=1e-5
        ), "Pitch bytes incorrect."
        assert bytes_to_float32(int.from_bytes(msg.buffer[30:34])) == pytest.approx(
            3.0, rel=1e-5
        ), "Yaw bytes incorrect."


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


@pytest.fixture
def mock_serial(mocker):
    return mocker.patch("serial.Serial", autospec=True)


class TestFW6_02CMP:
    def test_firmware_command(self, mock_serial):
        cmp = FW6_02CMP(
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
        assert mock_serial.return_value.write.called, "Firmware command did not write to serial port."

    def test_state_command(self, mock_serial):
        cmp = FW6_02CMP(
            "COM1",
            230400,
            8,
            None,
            None,
            None,
            None,
            None,
        )
        cmp._publish_state()
        assert mock_serial.return_value.write.called, "State command did not write to serial port."

    def test_bionic_boarder_command(self, mock_serial):
        cmp = FW6_02CMP(
            "COM1",
            230400,
            8,
            None,
            EboardKinematicState(0, 0, 0, 0, 0, 0, 0, 0, 0),
            Lock(),
            None,
            None,
        )
        cmp._publish_bionic_boarder()
        assert mock_serial.return_value.write.called, "Bionic Boarder command did not write to serial port."

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
        cmp = FW6_02CMP("COM1", 230400, 8, None, eks, eks_lock, BatteryDischargeModel(42.0), mc)

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
