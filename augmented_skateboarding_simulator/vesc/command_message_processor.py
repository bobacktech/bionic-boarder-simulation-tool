from abc import ABC, abstractmethod
import serial


class CommandMessageProcessor(ABC):
    # State change commands
    DUTY_CYCLE = "DUTY CYCLE"
    CURRENT = "CURRENT"
    RPM = "RPM"
    HEARTBEAT = "HEARTBEAT"

    # Message request commands
    FIRMWARE = "FIRMWARE"
    STATE = "STATE"
    IMU_STATE = "IMU STATE"

    @property
    @abstractmethod
    def __state_change_command_ids(self):
        """
        Returns a dictionary of the state change command ids associated with a lambda to change the particular
        value in the state message.
        """
        pass

    @property
    @abstractmethod
    def __message_request_commands(self):
        """
        Returns a dictionary of the message request command id associated with an instance of the message type.
        """
        pass

    @property
    @abstractmethod
    def __command_id_names(self):
        """
        Returns a dictionary of the command id associated to the name of the command.
        """
        pass

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
