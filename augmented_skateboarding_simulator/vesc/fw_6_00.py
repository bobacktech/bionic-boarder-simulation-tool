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
    BYTE_LENGTH = 76

    def __init__(self) -> None:
        self.__duty_cycle = 0
        self.__rpm = 0
        self.__motor_current = 0
        self.__input_voltage = 0

    @property
    def duty_cycle(self):
        return self.__duty_cycle

    @duty_cycle.setter
    def duty_cycle(self, value):
        self.__duty_cycle = value

    @property
    def rpm(self):
        return self.__rpm

    @rpm.setter
    def rpm(self, value):
        self.__rpm = value

    @property
    def motor_current(self):
        return self.__motor_current

    @motor_current.setter
    def motor_current(self, value):
        self.__motor_current = value

    @property
    def input_voltage(self):
        return self.__input_voltage

    @input_voltage.setter
    def input_voltage(self, value):
        self.__input_voltage = value
