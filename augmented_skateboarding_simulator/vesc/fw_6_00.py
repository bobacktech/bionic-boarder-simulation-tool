class FirmwareMessage:
    """
    See comm/commands.c in VESC bldc-6.00 source code on Github for message specification.
    """

    BYTE_LENGTH = 65

    def __init__(self) -> None:
        self.__buffer = bytearray(FirmwareMessage.BYTE_LENGTH)
        self.__buffer[0] = 0
        self.__buffer[1] = 6
        self.__buffer[2] = 0
        self.__buffer[3:15] = "HardwareName".encode("utf-8")

    @property
    def buffer(self):
        return bytes(self.__buffer)
