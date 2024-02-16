from . import fw
import struct


class FirmwareMessage:
    """
    See the message specification in [commands.c](https://github.com/vedderb/bldc/blob/6.00/comm/commands.c)
    in VESC bldc-6.00 source code on Github.
    """

    BYTE_LENGTH = 65

    def __init__(self) -> None:
        """
        Initializes a new instance of the FirmwareMessage class.

        Sets up the firmware message buffer according to the specified format.
        The buffer is initialized with predefined values, and a section is filled with an encoded string.
        """
        self.__buffer = bytearray(FirmwareMessage.BYTE_LENGTH)
        self.__buffer[0] = 0
        self.__buffer[1] = 6
        self.__buffer[2] = 0
        self.__buffer[3:15] = "HardwareName".encode("utf-8")

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
        self.__duty_cycle: float = 0
        self.__rpm: int = 0
        self.__motor_current: float = 0
        self.__input_voltage: float = 0

    @property
    def buffer(self) -> bytes:
        """
        Generates a byte representation of the state message based on the current properties of the object.

        The state message is structured as a 76-byte array, with specific portions of the array dedicated to
        representing the duty cycle, input voltage, motor current, and RPM, encoded in specific formats.

        The encoding is as follows:
        - Motor current (mc) is stored from bytes 9 to 12, represented as an unsigned int (">I"), scaled by 100.
        - Duty cycle (dc) is stored from bytes 25 to 26, represented as an unsigned short (">H"), scaled by 1000.
        - RPM is stored from bytes 27 to 30, represented directly as an unsigned int (">I") without scaling.
        - Input voltage (iv) is stored from bytes 31 to 32, represented as an unsigned short (">H"), scaled by 10.

        Returns:
            bytes: A bytes object representing the encoded state message, suitable for transmission or processing
                in accordance with the "COMM_GET_VALUES" message specification of the VESC firmware.
        """
        buffer = bytearray(76)
        dc = int(self.duty_cycle * 1000)
        iv = int(self.__input_voltage * 10.0)
        mc = int(self.__motor_current * 100.0)
        buffer[9:13] = struct.pack(">I", mc)
        buffer[25:27] = struct.pack(">H", dc)
        buffer[27:31] = struct.pack(">I", self.__rpm)
        buffer[31:33] = struct.pack(">H", iv)
        return bytes(buffer)

    @property
    def duty_cycle(self) -> float:
        return self.__duty_cycle

    @duty_cycle.setter
    def duty_cycle(self, value: float) -> None:
        self.__duty_cycle = value

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
    def input_voltage(self) -> float:
        return self.__input_voltage

    @input_voltage.setter
    def input_voltage(self, value: float) -> None:
        self.__input_voltage = value


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
