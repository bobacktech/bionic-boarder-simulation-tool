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
    def buffer(self) -> bytearray:
        """State message size is 76 bytes."""
        buffer = bytearray(76)
        dc = int(self.duty_cycle * 1000)
        iv = int(self.__input_voltage * 10.0)
        mc = int(self.__motor_current * 100.0)
        buffer[9:13] = struct.pack(">H", mc)
        buffer[25:27] = struct.pack(">H", dc)
        buffer[27:31] = struct.pack(">H", self.__rpm)
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
