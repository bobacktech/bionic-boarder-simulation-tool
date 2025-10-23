from . import fw
from .command_message_processor import CommandMessageProcessor
import struct
from threading import Lock
import math
from bionic_boarder_simulation_tool.riding.battery_discharge_model import BatteryDischargeModel
from bionic_boarder_simulation_tool.riding.eboard_kinematic_state import EboardKinematicState
from bionic_boarder_simulation_tool.riding.motor_controller import MotorController
from bionic_boarder_simulation_tool.logger import Logger


class FirmwareMessage:
    """
    See the message specification in [commands.c](https://github.com/vedderb/bldc/blob/release_6_02/comm/commands.c)
    in VESC bldc-6.02 source code on Github.
    """

    BYTE_LENGTH = 64

    def __init__(self) -> None:
        """
        Initializes a new instance of the FirmwareMessage class.

        Sets up the firmware message buffer according to the specified format.
        The buffer is initialized with predefined values, and a section is filled with an encoded string.
        """
        self.__buffer = bytearray(FirmwareMessage.BYTE_LENGTH)
        self.__buffer[0] = 6
        self.__buffer[1] = 0x02
        self.__buffer[2:14] = "HardwareName".encode("utf-8")

    @property
    def buffer(self):
        """
        The buffer property representing the immutable form of the firmware message.

        Returns:
            bytes: An immutable bytes object representing the current state of the firmware message buffer.
        """
        return bytes(self.__buffer)


class StateMessage:
    """
    See the "COMM_GET_VALUES" message specification in [commands.c](https://github.com/vedderb/bldc/blob/release_6_02/comm/commands.c)
    in VESC bldc-6.02 source code on Github.
    """

    def __init__(self) -> None:
        self.__temp_fet: float = 0.0
        self.__temp_motor: float = 0.0
        self.__avg_motor_current: float = 0.0
        self.__avg_input_current: float = 0.0
        self.__avg_id: float = 0.0
        self.__avg_iq: float = 0.0
        self.__duty_cycle_now: float = 0.0
        self.__rpm: int = 0
        self.__vin: float = 0.0
        self.__amp_hours: float = 0.0
        self.__amp_hours_charged: float = 0.0
        self.__watt_hours: float = 0.0
        self.__watt_hours_charged: float = 0.0
        self.__tachometer: int = 0
        self.__tachometer_abs: int = 0
        self.__fault_code: int = 0
        self.__pid_pos_now: float = 0.0
        self.__current_controller_id: int = 0
        self.__mos1_temp: float = 0.0
        self.__mos2_temp: float = 0.0
        self.__mos3_temp: float = 0.0
        self.__avg_vd: float = 0.0
        self.__avg_vq: float = 0.0
        self.__status: int = 0

    @property
    def buffer(self) -> bytes:
        """
        Generates a byte representation of the state message based on the current properties of the object.

        Returns:
            bytes: A bytes object representing the encoded state message, suitable for transmission or processing
                in accordance with the "COMM_GET_VALUES" message specification of the VESC firmware.
        """
        buffer = bytearray(74)
        buffer[0:2] = struct.pack(">h", int(self.__temp_fet * 1e1))
        buffer[2:4] = struct.pack(">h", int(self.__temp_motor * 1e1))
        buffer[4:8] = struct.pack(">i", int(self.__avg_motor_current * 1e2))
        buffer[8:12] = struct.pack(">i", int(self.__avg_input_current * 1e2))
        buffer[12:16] = struct.pack(">i", int(self.__avg_id * 1e2))
        buffer[16:20] = struct.pack(">i", int(self.__avg_iq * 1e2))
        buffer[20:22] = struct.pack(">h", int(self.__duty_cycle_now * 1e3))
        buffer[22:26] = struct.pack(">i", int(self.__rpm * 1.0))
        buffer[26:28] = struct.pack(">h", int(self.__vin * 1e1))
        buffer[28:32] = struct.pack(">i", int(self.__amp_hours * 1e4))
        buffer[32:36] = struct.pack(">i", int(self.__amp_hours_charged * 1e4))
        buffer[36:40] = struct.pack(">i", int(self.__watt_hours * 1e4))
        buffer[40:44] = struct.pack(">i", int(self.__watt_hours_charged * 1e4))
        buffer[44:48] = struct.pack(">i", int(self.__tachometer))
        buffer[48:52] = struct.pack(">i", int(self.__tachometer_abs))
        buffer[52:53] = struct.pack("B", int(self.__fault_code) & 0xFF)
        buffer[53:57] = struct.pack(">i", int(self.__pid_pos_now * 1e6))
        buffer[57:58] = struct.pack("B", int(self.__current_controller_id) & 0xFF)
        buffer[58:60] = struct.pack(">h", int(self.__mos1_temp * 1e1))
        buffer[60:62] = struct.pack(">h", int(self.__mos2_temp * 1e1))
        buffer[62:64] = struct.pack(">h", int(self.__mos3_temp * 1e1))
        buffer[64:68] = struct.pack(">i", int(self.__avg_vd * 1e3))
        buffer[68:72] = struct.pack(">i", int(self.__avg_vq * 1e3))
        buffer[72:73] = struct.pack("B", int(self.__status) & 0xFF)
        return bytes(buffer)

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
    def duty_cycle_now(self) -> float:
        return self.__duty_cycle_now

    @duty_cycle_now.setter
    def duty_cycle_now(self, value: float) -> None:
        self.__duty_cycle_now = value

    @property
    def vin(self) -> float:
        return self.__vin

    @vin.setter
    def vin(self, value: float) -> None:
        self.__vin = value

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
    def fault_code(self) -> int:
        return self.__fault_code

    @fault_code.setter
    def fault_code(self, value: int) -> None:
        self.__fault_code = value

    @property
    def pid_pos_now(self) -> float:
        return self.__pid_pos_now

    @pid_pos_now.setter
    def pid_pos_now(self, value: float) -> None:
        self.__pid_pos_now = value

    @property
    def current_controller_id(self) -> int:
        return self.__current_controller_id

    @current_controller_id.setter
    def current_controller_id(self, value: int) -> None:
        self.__current_controller_id = value

    @property
    def mos1_temp(self) -> float:
        return self.__mos1_temp

    @mos1_temp.setter
    def mos1_temp(self, value: float) -> None:
        self.__mos1_temp = value

    @property
    def mos2_temp(self) -> float:
        return self.__mos2_temp

    @mos2_temp.setter
    def mos2_temp(self, value: float) -> None:
        self.__mos2_temp = value

    @property
    def mos3_temp(self) -> float:
        return self.__mos3_temp

    @mos3_temp.setter
    def mos3_temp(self, value: float) -> None:
        self.__mos3_temp = value

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
    def status(self) -> int:
        return self.__status

    @status.setter
    def status(self, value: int) -> None:
        self.__status = value


class BionicBoarderMessage:
    """
    See the "COMM_BIONIC_BOARDER_DATA" message specification in [commands.c](https://github.com/vedderb/bldc/blob/release_6_02/comm/commands.c)
    in VESC bldc-6.02 source code on Github.

    Note: This message is not defined yet in the VESC BLDC firmware; this class is created for simulation purposes, for now.
    """

    def __init__(self) -> None:
        # Motor Dynamics Data
        self.__motor_current: float = 0
        self.__duty_cycle: float = 0.0
        self.__rpm: int = 0

        # IMU Data
        self.__rpy = [0.0, 0.0, 0.0]  # Roll, pitch, yaw in radians
        self.__acc = [0.0, 0.0, 0.0]  # Accelerometer data (x, y, z) in m/s^2

    @property
    def buffer(self) -> bytes:
        """
        Serializes the Bionic Boarder message into a bytes object.

        This serialization includes motor dynamics data (motor current, duty cycle, RPM)
        and IMU data (roll, pitch, yaw, accelerometer data), formatted according to the
        specifications in the VESC bldc-6.02 source code.

        Returns:
            bytes: A bytes object containing the serialized Bionic Boarder message data.
        """
        buffer = bytearray(34)
        mc = int(self.__motor_current * 100.0)
        buffer[0:4] = struct.pack(">i", mc)
        dc = int(self.__duty_cycle * 1000.0)
        buffer[4:6] = struct.pack(">h", dc)
        buffer[6:10] = struct.pack(">i", self.__rpm)
        buffer[10:14] = fw.float32_to_bytes(self.__acc[0])
        buffer[14:18] = fw.float32_to_bytes(self.__acc[1])
        buffer[18:22] = fw.float32_to_bytes(self.__acc[2])
        buffer[22:26] = fw.float32_to_bytes(self.__rpy[0])
        buffer[26:30] = fw.float32_to_bytes(self.__rpy[1])
        buffer[30:34] = fw.float32_to_bytes(self.__rpy[2])
        return bytes(buffer)

    @property
    def rpm(self) -> int:
        return self.__rpm

    @rpm.setter
    def rpm(self, value: int) -> None:
        self.__rpm = value

    @property
    def motor_current(self) -> float:
        return self.__motor_current

    @motor_current.setter
    def motor_current(self, value: float) -> None:
        self.__motor_current = value

    @property
    def duty_cycle(self) -> float:
        return self.__duty_cycle

    @duty_cycle.setter
    def duty_cycle(self, value: float) -> None:
        self.__duty_cycle = value

    @property
    def rpy(self):
        return self.__rpy

    @property
    def acc(self):
        return self.__acc


class FW6_02CMP(CommandMessageProcessor):
    def __init__(
        self,
        com_port,
        baud_rate,
        command_byte_size,
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
            4: CommandMessageProcessor.STATE,
            164: CommandMessageProcessor.BIONIC_BOARDER,
        }
        self.__packet_header = lambda id, l: int.to_bytes(2) + int.to_bytes(l) + int.to_bytes(id)
        self.__eks = eks
        self.__eks_lock = eks_lock
        self.__bdm = bdm
        self.__mc = mc

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

    def _publish_state(self):
        sm = StateMessage()
        # Create the state message in the future. It's TBD for now.
        msg_data = sm.buffer
        packet = self.__packet_header(4, len(msg_data)) + msg_data
        self.serial.write(packet)
        Logger().logger.info(
            "Publishing state message",
            CMP=self.__class__.__name__,
        )

    def _publish_bionic_boarder(self):
        bb = BionicBoarderMessage()
        with self.__eks_lock:
            bb.motor_current = self.__eks.motor_current
            bb.rpm = self.__eks.erpm
            bb.acc[0] = self.__eks.acceleration_x
            bb.rpy[1] = self.__eks.pitch * (math.pi / 180.0)
        msg_data = bb.buffer
        packet = self.__packet_header(66, len(msg_data)) + msg_data
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
        packet = self.__packet_header(0, FirmwareMessage.BYTE_LENGTH) + fw.buffer
        self.serial.write(packet)

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
