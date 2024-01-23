from abc import ABC, abstractmethod
import serial


class CommandMessageProcessor(ABC):
    def __init__(self, com_port, command_byte_size):
        self.__serial = serial.Serial(
            port=com_port,
            baudrate=230400,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )
        self.__command_byte_size = command_byte_size

    def receive_command(self):
        while True:
            command = self.__serial.read(self.__command_byte_size)
            self._handle_command(command)

    @abstractmethod
    def _handle_command(self, command: bytes):
        pass
