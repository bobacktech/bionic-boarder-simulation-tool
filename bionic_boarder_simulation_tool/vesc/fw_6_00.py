from . import fw
from .command_message_processor import CommandMessageProcessor
import struct
from threading import Lock
import math
from bionic_boarder_simulation_tool.riding.battery_discharge_model import BatteryDischargeModel
from bionic_boarder_simulation_tool.riding.eboard_kinematic_state import EboardKinematicState
from bionic_boarder_simulation_tool.riding.motor_controller import MotorController
from bionic_boarder_simulation_tool.logger import Logger
from bionic_boarder_simulation_tool.riding.eboard import EBoard


class FirmwareMessage:
    """
    See the message specification in [commands.c](https://github.com/vedderb/bldc/blob/6.00/comm/commands.c)
    in VESC bldc-6.00 source code on Github.
    """

    ID = 0

    def __init__(self) -> None:
        """
        Initializes a new instance of the FirmwareMessage class.

        Sets up the firmware message buffer according to the specified format.
        The buffer is initialized with predefined values, and a section is filled with an encoded string.
        """
        self.__buffer = bytearray(64)
        self.__buffer[0] = 6
        self.__buffer[1] = 0
        self.__buffer[2:14] = "HardwareName".encode("utf-8")

    @property
    def buffer(self):
        """
        The buffer property representing the immutable form of the firmware message.

        Returns:
            bytes: An immutable bytes object representing the current state of the firmware message buffer.
        """
        return FirmwareMessage.ID.to_bytes(1) + bytes(self.__buffer)


class MotorControllerConfigurationMessage:
    """
    See the "COMM_GET_MCCONF" message specification in [commands.c](https://github.com/vedderb/bldc/blob/6.00/comm/commands.c)
    in VESC bldc-6.00 source code on Github.
    """

    BYTE_LENGTH = 696

    def __init__(self) -> None:
        self.__buffer = bytearray(MotorControllerConfigurationMessage.BYTE_LENGTH)
        self.si_wheel_diameter: float = 0.0
        self.si_battery_ah: float = 0.0
        self.si_gear_ratio: float = 0.0
        self.si_motor_poles: int = 0
        self.l_current_max: float = 0.0
        self.l_watt_max: float = 0.0
        self.l_max_vin: float = 0.0
        self.foc_motor_flux_linkage: float = 0.0

    @property
    def buffer(self) -> bytes:
        """
        Generates a byte representation of the motor controller configuration message based on the current properties of the object.

        Byte Positions:
            l_current_max: 0-3
            l_max_vin: 44-47
            l_watt_max: 85-88
            foc_motor_flux_linkage: 222-225
            si_motor_poles: 644
            si_gear_ratio: 645-648
            si_wheel_diameter: 649-652
            si_battery_ah: 661-664

        Returns:
            bytes: A bytes object representing the encoded motor controller configuration message.
        """
        self.__buffer[0:4] = struct.pack(">f", self.l_current_max)
        self.__buffer[44:48] = struct.pack(">f", self.l_max_vin)
        self.__buffer[85:89] = struct.pack(">f", self.l_watt_max)
        self.__buffer[222:226] = struct.pack(">f", self.foc_motor_flux_linkage)
        self.__buffer[644] = self.si_motor_poles
        self.__buffer[645:649] = struct.pack(">f", self.si_gear_ratio)
        self.__buffer[649:653] = struct.pack(">f", self.si_wheel_diameter)
        self.__buffer[661:665] = struct.pack(">f", self.si_battery_ah)
        return bytes(self.__buffer)


class BionicBoarderMessage:
    """
    See the "COMM_BIONIC_BOARDER_DATA" message specification in [commands.c](https://github.com/vedderb/bldc/blob/6.00/comm/commands.c)
    in VESC bldc-6.00 source code on Github.

    Note: This message is not defined yet in this version of theVESC BLDC firmware; this class is created for simulation purposes, for now.
    """

    ID = 152

    def __init__(self) -> None:
        # VESC State Data
        self.__temp_fet: float = 0.0
        self.__temp_motor: float = 0.0
        self.__avg_motor_current: float = 0.0
        self.__avg_input_current: float = 0.0
        self.__avg_id: float = 0.0
        self.__avg_iq: float = 0.0
        self.__duty_cycle: float = 0.0
        self.__rpm: float = 0.0
        self.__input_voltage: float = 0.0
        self.__amp_hours: float = 0.0
        self.__amp_hours_charged: float = 0.0
        self.__watt_hours: float = 0.0
        self.__watt_hours_charged: float = 0.0
        self.__tachometer: int = 0
        self.__tachometer_abs: int = 0
        self.__fault: int = 0
        self.__pid_pos: float = 0.0
        self.__avg_vd: float = 0.0
        self.__avg_vq: float = 0.0

        # IMU Data
        self.__rpy = [0.0, 0.0, 0.0]  # Roll, pitch, yaw in radians
        self.__acc = [0.0, 0.0, 0.0]  # Accelerometer data (x, y, z) in m/s^2
        self.__gyro = [0.0, 0.0, 0.0]  # Gyroscope data (x, y, z) in rad/s
        self.__mag = [0.0, 0.0, 0.0]  # Magnetometer data (x, y, z) in micro teslas
        self.__q = [0.0, 0.0, 0.0, 0.0]  # Quaternion data (w, x, y, z)

    @property
    def buffer(self) -> bytes:
        """
        Serializes the Bionic Boarder message into a bytes object.

        This serialization combines the VESC state data and IMU data into a single byte array according to the specified format in
        the VESC custom firmware for the Bionic Boarder application. The VESC state data is encoded with specific scaling factors,
        and the IMU data is encoded as IEEE 754 floats.

        Returns:
            bytes: A bytes object containing the serialized Bionic Boarder message data.
        """
        buffer = bytearray(129)

        # float16: temp_fet (scale 1e1) -> 2 bytes
        buffer[0:2] = struct.pack(">h", int(self.__temp_fet * 1e1))
        # float16: temp_motor (scale 1e1) -> 2 bytes
        buffer[2:4] = struct.pack(">h", int(self.__temp_motor * 1e1))
        # float32: avg_motor_current (scale 1e2) -> 4 bytes
        buffer[4:8] = struct.pack(">i", int(self.__avg_motor_current * 1e2))
        # float32: avg_input_current (scale 1e2) -> 4 bytes
        buffer[8:12] = struct.pack(">i", int(self.__avg_input_current * 1e2))
        # float32: avg_id (scale 1e2) -> 4 bytes
        buffer[12:16] = struct.pack(">i", int(self.__avg_id * 1e2))
        # float32: avg_iq (scale 1e2) -> 4 bytes
        buffer[16:20] = struct.pack(">i", int(self.__avg_iq * 1e2))
        # float16: duty_cycle (scale 1e3) -> 2 bytes
        buffer[20:22] = struct.pack(">h", int(self.__duty_cycle * 1e3))
        # float32: rpm (scale 1e0) -> 4 bytes
        buffer[22:26] = struct.pack(">i", int(self.__rpm * 1e0))
        # float16: input_voltage (scale 1e1) -> 2 bytes
        buffer[26:28] = struct.pack(">h", int(self.__input_voltage * 1e1))
        # float32: amp_hours (scale 1e4) -> 4 bytes
        buffer[28:32] = struct.pack(">i", int(self.__amp_hours * 1e4))
        # float32: amp_hours_charged (scale 1e4) -> 4 bytes
        buffer[32:36] = struct.pack(">i", int(self.__amp_hours_charged * 1e4))
        # float32: watt_hours (scale 1e4) -> 4 bytes
        buffer[36:40] = struct.pack(">i", int(self.__watt_hours * 1e4))
        # float32: watt_hours_charged (scale 1e4) -> 4 bytes
        buffer[40:44] = struct.pack(">i", int(self.__watt_hours_charged * 1e4))
        # int32: tachometer -> 4 bytes
        buffer[44:48] = struct.pack(">i", self.__tachometer)
        # int32: tachometer_abs -> 4 bytes
        buffer[48:52] = struct.pack(">i", self.__tachometer_abs)
        # uint8: fault -> 1 byte
        buffer[52] = self.__fault & 0xFF
        # float32: pid_pos (scale 1e6) -> 4 bytes
        buffer[53:57] = struct.pack(">i", int(self.__pid_pos * 1e6))
        # float32: avg_vd (scale 1e3) -> 4 bytes
        buffer[57:61] = struct.pack(">i", int(self.__avg_vd * 1e3))
        # float32: avg_vq (scale 1e3) -> 4 bytes
        buffer[61:65] = struct.pack(">i", int(self.__avg_vq * 1e3))

        # IMU Data — buffer_append_float32_auto uses IEEE 754 float ('>f')
        # acc (x, y, z)
        buffer[65:69] = struct.pack(">f", self.__acc[0])
        buffer[69:73] = struct.pack(">f", self.__acc[1])
        buffer[73:77] = struct.pack(">f", self.__acc[2])
        # rpy (roll, pitch, yaw)
        buffer[77:81] = struct.pack(">f", self.__rpy[0])
        buffer[81:85] = struct.pack(">f", self.__rpy[1])
        buffer[85:89] = struct.pack(">f", self.__rpy[2])
        # gyro (x, y, z)
        buffer[89:93] = struct.pack(">f", self.__gyro[0])
        buffer[93:97] = struct.pack(">f", self.__gyro[1])
        buffer[97:101] = struct.pack(">f", self.__gyro[2])
        # mag (x, y, z)
        buffer[101:105] = struct.pack(">f", self.__mag[0])
        buffer[105:109] = struct.pack(">f", self.__mag[1])
        buffer[109:113] = struct.pack(">f", self.__mag[2])
        # quaternion (w, x, y, z)
        buffer[113:117] = struct.pack(">f", self.__q[0])
        buffer[117:121] = struct.pack(">f", self.__q[1])
        buffer[121:125] = struct.pack(">f", self.__q[2])
        buffer[125:129] = struct.pack(">f", self.__q[3])

        return BionicBoarderMessage.ID.to_bytes(1) + bytes(buffer)

    @property
    def temp_fet(self) -> float:
        return self.__temp_fet

    @temp_fet.setter
    def temp_fet(self, value: float) -> None:
        self.__temp_fet = value

    @property
    def temp_motor(self) -> float:
        return self.__temp_motor

    @temp_motor.setter
    def temp_motor(self, value: float) -> None:
        self.__temp_motor = value

    @property
    def avg_motor_current(self) -> float:
        return self.__avg_motor_current

    @avg_motor_current.setter
    def avg_motor_current(self, value: float) -> None:
        self.__avg_motor_current = value

    @property
    def avg_input_current(self) -> float:
        return self.__avg_input_current

    @avg_input_current.setter
    def avg_input_current(self, value: float) -> None:
        self.__avg_input_current = value

    @property
    def avg_id(self) -> float:
        return self.__avg_id

    @avg_id.setter
    def avg_id(self, value: float) -> None:
        self.__avg_id = value

    @property
    def avg_iq(self) -> float:
        return self.__avg_iq

    @avg_iq.setter
    def avg_iq(self, value: float) -> None:
        self.__avg_iq = value

    @property
    def duty_cycle(self) -> float:
        return self.__duty_cycle

    @duty_cycle.setter
    def duty_cycle(self, value: float) -> None:
        self.__duty_cycle = value

    @property
    def rpm(self) -> float:
        return self.__rpm

    @rpm.setter
    def rpm(self, value: float) -> None:
        self.__rpm = value

    @property
    def input_voltage(self) -> float:
        return self.__input_voltage

    @input_voltage.setter
    def input_voltage(self, value: float) -> None:
        self.__input_voltage = value

    @property
    def amp_hours(self) -> float:
        return self.__amp_hours

    @amp_hours.setter
    def amp_hours(self, value: float) -> None:
        self.__amp_hours = value

    @property
    def amp_hours_charged(self) -> float:
        return self.__amp_hours_charged

    @amp_hours_charged.setter
    def amp_hours_charged(self, value: float) -> None:
        self.__amp_hours_charged = value

    @property
    def watt_hours(self) -> float:
        return self.__watt_hours

    @watt_hours.setter
    def watt_hours(self, value: float) -> None:
        self.__watt_hours = value

    @property
    def watt_hours_charged(self) -> float:
        return self.__watt_hours_charged

    @watt_hours_charged.setter
    def watt_hours_charged(self, value: float) -> None:
        self.__watt_hours_charged = value

    @property
    def tachometer(self) -> int:
        return self.__tachometer

    @tachometer.setter
    def tachometer(self, value: int) -> None:
        self.__tachometer = value

    @property
    def tachometer_abs(self) -> int:
        return self.__tachometer_abs

    @tachometer_abs.setter
    def tachometer_abs(self, value: int) -> None:
        self.__tachometer_abs = value

    @property
    def fault(self) -> int:
        return self.__fault

    @fault.setter
    def fault(self, value: int) -> None:
        self.__fault = value

    @property
    def pid_pos(self) -> float:
        return self.__pid_pos

    @pid_pos.setter
    def pid_pos(self, value: float) -> None:
        self.__pid_pos = value

    @property
    def avg_vd(self) -> float:
        return self.__avg_vd

    @avg_vd.setter
    def avg_vd(self, value: float) -> None:
        self.__avg_vd = value

    @property
    def avg_vq(self) -> float:
        return self.__avg_vq

    @avg_vq.setter
    def avg_vq(self, value: float) -> None:
        self.__avg_vq = value

    @property
    def rpy(self) -> list:
        return self.__rpy

    @property
    def acc(self) -> list:
        return self.__acc

    @property
    def gyro(self) -> list:
        return self.__gyro

    @property
    def mag(self) -> list:
        return self.__mag

    @property
    def q(self) -> list:
        return self.__q


class FW6_00CMP(CommandMessageProcessor):
    def __init__(
        self,
        com_port,
        baud_rate,
        command_byte_size,
        eboard: EBoard,
        eks: EboardKinematicState,
        eks_lock: Lock,
        bdm: BatteryDischargeModel,
        mc: MotorController,
    ):
        super().__init__(com_port, baud_rate, command_byte_size)
        self.__cmd_id_name = {
            6: CommandMessageProcessor.CURRENT,
            8: CommandMessageProcessor.RPM,
            30: CommandMessageProcessor.HEARTBEAT,
            0: CommandMessageProcessor.FIRMWARE,
            14: CommandMessageProcessor.MOTOR_CONTROLLER_CONFIGURATION,
            152: CommandMessageProcessor.BIONIC_BOARDER,
        }
        self.__packet_header = lambda l: int.to_bytes(2) + int.to_bytes(l)
        # crc value + end_byte
        self.__packet_footer = lambda payload: int.to_bytes(self.crc16(payload), 2) + int.to_bytes(0x03)
        self.__eks = eks
        self.__eks_lock = eks_lock
        self.__bdm = bdm
        self.__mc = mc
        self.__eboard = eboard

    @property
    def _command_id_name(self):
        """
        Returns a dictionary of the command id associated to the name of the command.
        """
        return self.__cmd_id_name

    def _get_command_id(self, command: bytes) -> int:
        """
        The command ID for the received command is in the third byte of the [command] data buffer.
        """
        return command[2]

    def _publish_bionic_boarder(self):
        bb = BionicBoarderMessage()
        with self.__eks_lock:
            bb.motor_current = self.__eks.motor_current
            bb.rpm = self.__eks.erpm
            bb.acc[0] = self.__eks.acceleration_x
            bb.rpy[1] = self.__eks.pitch * (math.pi / 180.0)
        msg_data = bb.buffer
        packet = self.__packet_header(len(msg_data)) + msg_data + self.__packet_footer(msg_data)
        self.serial.write(packet)
        Logger().logger.info(
            "Publishing Bionic Boarder message",
            motor_current=bb.motor_current,
            duty_cycle=bb.duty_cycle,
            rpm=bb.rpm,
            imu_acc=bb.acc,
            imu_rpy=bb.rpy,
            CMP=self.__class__.__name__,
        )

    def _publish_firmware(self):
        fw = FirmwareMessage()
        packet = self.__packet_header(len(fw.buffer)) + fw.buffer + self.__packet_footer(fw.buffer)
        self.serial.write(packet)

    def _publish_motor_controller_configuration(self):
        mcc = MotorControllerConfigurationMessage()
        mcc.si_wheel_diameter = self.__eboard.wheel_diameter_m
        mcc.si_battery_ah = self.__eboard.battery_max_capacity_Ah
        mcc.si_gear_ratio = self.__eboard.gear_ratio
        mcc.si_motor_poles = self.__eboard.motor_pole_pairs * 2
        mcc.l_current_max = self.__eboard.motor_max_amps
        mcc.l_watt_max = self.__eboard.motor_max_power_watts
        mcc.l_max_vin = self.__eboard.battery_max_voltage
        msg_data = mcc.buffer
        packet = (
            int.to_bytes(3)
            + int.to_bytes(MotorControllerConfigurationMessage.BYTE_LENGTH, 2)
            + int.to_bytes(14)
            + msg_data
            + self.__packet_footer(msg_data)
        )
        self.serial.write(packet)
        Logger().logger.info(
            "Publishing motor controller configuration message",
            wheel_diameter_m=mcc.si_wheel_diameter,
            battery_capacity_Ah=mcc.si_battery_ah,
            gear_ratio=mcc.si_gear_ratio,
            motor_poles=mcc.si_motor_poles,
            current_max=mcc.l_current_max,
            watt_max=mcc.l_watt_max,
            max_vin=mcc.l_max_vin,
            CMP=self.__class__.__name__,
        )

    def _update_current(self, command):
        motor_current_commanded = int.from_bytes(command[3:7], byteorder="big") / 1000.0
        self.__mc.target_current = motor_current_commanded
        try:
            self.__mc.current_sem.release()
        except ValueError as e:
            pass
        Logger().logger.info(
            "Processing set current command", motor_current=motor_current_commanded, CMP=self.__class__.__name__
        )

    def _update_rpm(self, command):
        erpm_commanded = int.from_bytes(command[3:7], byteorder="big")
        self.__mc.target_erpm = erpm_commanded
        try:
            self.__mc.erpm_sem.release()
        except ValueError as e:
            pass
        Logger().logger.info("Processing set ERPM command", erpm=erpm_commanded, CMP=self.__class__.__name__)
