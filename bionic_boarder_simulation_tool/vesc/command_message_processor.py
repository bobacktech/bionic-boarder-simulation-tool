from abc import ABC, abstractmethod
from threading import Timer
import serial
from bionic_boarder_simulation_tool.logger import Logger
import os


class CommandMessageProcessor(ABC):
    """
    Abstract base class for processing command messages.

    This class defines an interface for handling various command messages received
    through a serial port. It includes both state change commands and message request commands.
    """

    # State change commands
    CURRENT = "CURRENT"
    RPM = "RPM"
    HEARTBEAT = "HEARTBEAT"
    HEARTBEAT_TIMEOUT_SEC = 1.5

    # Message request commands
    FIRMWARE = "FIRMWARE"
    STATE = "STATE"
    BIONIC_BOARDER = "BIONIC BOARDER"

    @property
    @abstractmethod
    def _command_id_name(self):
        """
        Returns a dictionary mapping command IDs to their respective command names.

        This abstract property should be implemented in subclasses to provide
        the specific mapping of command IDs to command names.
        """
        pass

    def __init__(
        self,
        com_port,
        baud_rate,
        command_byte_size,
    ):
        """
        Initializes a new instance of CommandMessageProcessor.

        Args:
            com_port (str): The COM port to use for serial communication.
            command_byte_size (int): The size of the command byte.
        """
        self.serial = serial.Serial(
            port=com_port,
            baudrate=baud_rate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )
        self.__command_byte_size = command_byte_size
        self.__heartbeat_timer = None

    def handle_command(self):
        """
        Continuously reads command bytes from the serial port and handles them using
        the appropriate method based on the command type.
        """
        command_bytes = None
        handler = {
            CommandMessageProcessor.STATE: lambda: self._publish_state(),
            CommandMessageProcessor.BIONIC_BOARDER: lambda: self._publish_bionic_boarder(),
            CommandMessageProcessor.FIRMWARE: lambda: self._publish_firmware(),
            CommandMessageProcessor.CURRENT: lambda: self._update_current(command_bytes),
            CommandMessageProcessor.RPM: lambda: self._update_rpm(command_bytes),
            CommandMessageProcessor.HEARTBEAT: lambda: self.heartbeat(),
        }
        while True:
            command_bytes = self.serial.read(self.__command_byte_size)
            try:
                command_id = self._get_command_id(command_bytes)
                command_name = self._command_id_name[command_id]
                Logger().logger.info("VESC received command", command=command_name)
                handler[command_name]()
            except Exception as e:
                Logger().logger.error("Received command was not processed correctly", error=e, command=command_name)

    @abstractmethod
    def _get_command_id(self, command: bytes) -> int:
        """
        Abstract method to get the command ID from the command bytes.

        Args:
            command (bytes): The command bytes from which to extract the command ID.

        Returns:
            int: The command ID.
        """
        pass

    @abstractmethod
    def _publish_state(self):
        """
        Abstract method to publish the 'state' message.
        """
        pass

    @abstractmethod
    def _publish_bionic_boarder(self):
        """
        Abstract method to publish the 'bionic boarder' message.
        """
        pass

    @abstractmethod
    def _publish_firmware(self):
        """
        Abstract method to publish the 'firmware' message.
        """
        pass

    @abstractmethod
    def _update_current(self, command):
        """
        Abstract method to update the current in the state data based on the provided command.

        Args:
            command: The command containing the information to update the current.
        """
        pass

    @abstractmethod
    def _update_rpm(self, command):
        """
        Abstract method to update the RPM in the state data based on the provided command.

        Args:
            command: The command containing the information to update the RPM.
        """
        pass

    def heartbeat(self):
        """
        Handle the 'heartbeat' command.
        """
        if self.__heartbeat_timer == None:
            self.__heartbeat_timer = Timer(
                CommandMessageProcessor.HEARTBEAT_TIMEOUT_SEC,
                self.__heartbeat_not_receieved,
            )
            self.__heartbeat_timer.start()
        else:
            self.__heartbeat_timer.cancel()
            self.__heartbeat_timer = Timer(
                CommandMessageProcessor.HEARTBEAT_TIMEOUT_SEC,
                self.__heartbeat_not_receieved,
            )
            self.__heartbeat_timer.start()

    def __heartbeat_not_receieved(self):
        """
        If heartbeat message has not been received within the specified timeout, terminate the simulation.
        """
        Logger().logger.error(
            "Heartbeat command was not received in time. Simulation has terminated.",
            command=CommandMessageProcessor.HEARTBEAT,
        )
        os._exit(1)
