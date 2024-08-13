from . import fw
from .command_message_processor import CommandMessageProcessor
import struct
from threading import Lock
import time
import math
from augmented_skateboarding_simulator.riding.battery_discharge_model import BatteryDischargeModel
from augmented_skateboarding_simulator.riding.eboard_kinematic_state import EboardKinematicState
from augmented_skateboarding_simulator.riding.motor_controller import MotorController


class FirmwareMessage:
    """
    See the message specification in [commands.c](https://github.com/vedderb/bldc/blob/6.00/comm/commands.c)
    in VESC bldc-6.00 source code on Github.
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
        self.__buffer[1] = 0
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
    See the "COMM_GET_VALUES" message specification in [commands.c](https://github.com/vedderb/bldc/blob/6.00/comm/commands.c)
    in VESC bldc-6.00 source code on Github.
    """

    def __init__(self) -> None:
        self.__rpm: int = 0
        self.__motor_current: float = 0
        self.__watt_hours: float = 0

    @property
    def buffer(self) -> bytes:
        """
        Generates a byte representation of the state message based on the current properties of the object.

        The state message is structured as a 76-byte array, with specific portions of the array dedicated to
        representing the input voltage, motor current, and RPM, encoded in specific formats.

        The encoding is as follows:
        - Motor current (mc) is stored from bytes 9 to 12, represented as an unsigned int (">I"), scaled by 100.
        - RPM is stored from bytes 27 to 30, represented directly as an unsigned int (">I") without scaling.
        - Watt hours (Wh) is stored from bytes 41 to 44, represented as an unsigned short (">H"), scaled by 10000.

        Returns:
            bytes: A bytes object representing the encoded state message, suitable for transmission or processing
                in accordance with the "COMM_GET_VALUES" message specification of the VESC firmware.
        """
        buffer = bytearray(76)
        wh = int(self.__watt_hours * 10000.0)
        mc = int(self.__motor_current * 100.0)
        buffer[9:13] = struct.pack(">I", mc)
        buffer[27:31] = struct.pack(">I", self.__rpm)
        buffer[41:45] = struct.pack(">I", wh)
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
    def watt_hours(self) -> float:
        return self.__watt_hours

    @watt_hours.setter
    def watt_hours(self, value: float) -> None:
        self.__watt_hours = value


class IMUStateMessage:
    """
    See the "COMM_GET_IMU_DATA" message specification in [commands.c](https://github.com/vedderb/bldc/blob/6.00/comm/commands.c)
    in VESC bldc-6.00 source code on Github.
    """

    def __init__(self) -> None:
        self.__rpy = [0.0, 0.0, 0.0]  # Roll, pitch, yaw
        self.__acc = [0.0, 0.0, 0.0]  # Accelerometer data
        self.__gyro = [0.0, 0.0, 0.0]  # Gyroscope data
        self.__mag = [0.0, 0.0, 0.0]  # Magnetometer data
        self.__q = [0.0, 0.0, 0.0, 0.0]  # Quaternion data

    @property
    def buffer(self) -> bytes:
        """
        Serializes the IMU state into a bytes object.

        The serialization format includes roll, pitch, yaw, accelerometer data,
        gyroscope data, magnetometer data, and quaternion data, each converted
        to bytes using the floating-point to bytes conversion method provided
        by the `fw` module. This format is compliant with the "COMM_GET_IMU_DATA"
        message specification in the VESC bldc-6.00 source code.

        Returns:
            bytes: A bytes object containing the serialized IMU state data.
        """
        buffer = bytearray(68)

        buffer[1:5] = fw.float32_to_bytes(self.__rpy[0])
        buffer[5:9] = fw.float32_to_bytes(self.__rpy[1])
        buffer[9:13] = fw.float32_to_bytes(self.__rpy[2])

        buffer[13:17] = fw.float32_to_bytes(self.__acc[0])
        buffer[17:21] = fw.float32_to_bytes(self.__acc[1])
        buffer[21:25] = fw.float32_to_bytes(self.__acc[2])

        buffer[25:29] = fw.float32_to_bytes(self.__gyro[0])
        buffer[29:33] = fw.float32_to_bytes(self.__gyro[1])
        buffer[33:37] = fw.float32_to_bytes(self.__gyro[2])

        buffer[37:41] = fw.float32_to_bytes(self.__mag[0])
        buffer[41:45] = fw.float32_to_bytes(self.__mag[1])
        buffer[45:49] = fw.float32_to_bytes(self.__mag[2])

        buffer[49:53] = fw.float32_to_bytes(self.__q[0])
        buffer[53:57] = fw.float32_to_bytes(self.__q[1])
        buffer[57:61] = fw.float32_to_bytes(self.__q[2])
        buffer[61:65] = fw.float32_to_bytes(self.__q[3])

        return bytes(buffer)

    @property
    def rpy(self):
        return self.__rpy

    @property
    def acc(self):
        return self.__acc

    @property
    def gyro(self):
        return self.__gyro

    @property
    def mag(self):
        return self.__mag

    @property
    def q(self):
        return self.__q


class FW6_00CMP(CommandMessageProcessor):
    PUBLISH_STATE_MESSAGE_DELAY_SEC = 0.05
    PUBLISH_IMU_STATE_MESSAGE_DELAY_SEC = 0.05

    def __init__(
        self,
        com_port,
        command_byte_size,
        eks: EboardKinematicState,
        eks_lock: Lock,
        bdm: BatteryDischargeModel,
        mc: MotorController,
    ):
        super().__init__(com_port, command_byte_size)
        self.__cmd_id_name = {
            6: CommandMessageProcessor.CURRENT,
            8: CommandMessageProcessor.RPM,
            30: CommandMessageProcessor.HEARTBEAT,
            0: CommandMessageProcessor.FIRMWARE,
            4: CommandMessageProcessor.STATE,
            65: CommandMessageProcessor.IMU_STATE,
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
        self.__eks_lock.acquire()
        sm.motor_current = self.__eks.input_current
        sm.rpm = self.__eks.erpm
        self.__eks_lock.release()
        sm.watt_hours = self.__bdm.get_watt_hours_consumed()
        msg_data = sm.buffer
        packet = self.__packet_header(4, len(msg_data)) + msg_data
        start = time.perf_counter()
        while time.perf_counter() - start < FW6_00CMP.PUBLISH_STATE_MESSAGE_DELAY_SEC:
            pass
        self.serial.write(packet)

    def _publish_imu_state(self):
        imu = IMUStateMessage()
        with self.__eks_lock:
            imu.acc[0] = self.__eks.acceleration_x
            imu.rpy[1] = self.__eks.pitch * (math.pi / 180.0)
        msg_data = imu.buffer
        packet = self.__packet_header(65, len(msg_data)) + msg_data
        start = time.perf_counter()
        while time.perf_counter() - start < FW6_00CMP.PUBLISH_IMU_STATE_MESSAGE_DELAY_SEC:
            pass
        self.serial.write(packet)

    def _publish_firmware(self):
        fw = FirmwareMessage()
        packet = self.__packet_header(0, FirmwareMessage.BYTE_LENGTH) + fw.buffer
        self.serial.write(packet)

    def _update_current(self, command):
        motor_current_commanded = int.from_bytes(command[3:7], byteorder="big") / 1000.0
        self.__mc.target_current = motor_current_commanded
        self.__mc.current_sem.release()

    def _update_rpm(self, command):
        erpm_commanded = int.from_bytes(command[3:7], byteorder="big")
        self.__mc.target_erpm = erpm_commanded
        self.__mc.erpm_sem.release()
