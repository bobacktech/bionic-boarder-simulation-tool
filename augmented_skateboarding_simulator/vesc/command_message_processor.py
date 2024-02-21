from abc import ABC, abstractmethod
from threading import Lock
import serial


class CommandMessageProcessor(ABC):
    """
    Abstract base class for processing command messages.

    This class defines an interface for handling various command messages received
    through a serial port. It includes both state change commands and message request commands.
    """

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
    def _command_id_name(self):
        """
        Returns a dictionary mapping command IDs to their respective command names.

        This abstract property should be implemented in subclasses to provide
        the specific mapping of command IDs to command names.
        """
        pass

    def __init__(
        self, com_port, command_byte_size, state_lock: Lock, imu_state_lock: Lock
    ):
        """
        Initializes a new instance of CommandMessageProcessor.

        Args:
            com_port (str): The COM port to use for serial communication.
            command_byte_size (int): The size of the command byte.
        """
        self.__serial = serial.Serial(
            port=com_port,
            baudrate=230400,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )
        self.__command_byte_size = command_byte_size
        self.__sl = state_lock
        self.__isl = imu_state_lock

    def handle_command(self):
        """
        Continuously reads command bytes from the serial port and handles them using
        the appropriate method based on the command type.
        """
        command_bytes = None
        handler = {
            CommandMessageProcessor.STATE: lambda: (
                self.__sl.acquire(),
                self._publish_state(),
                self.__sl.release(),
            ),
            CommandMessageProcessor.IMU_STATE: lambda: (
                self.__isl.acquire(),
                self._publish_imu_state(),
                self.__isl.release(),
            ),
            CommandMessageProcessor.FIRMWARE: lambda: self._publish_firmware(),
            CommandMessageProcessor.DUTY_CYCLE: lambda: (
                self.__sl.acquire(),
                self._update_duty_cycle(command_bytes),
                self.__sl.release(),
            ),
            CommandMessageProcessor.CURRENT: lambda: (
                self.__sl.acquire(),
                self._update_current(command_bytes),
                self.__sl.release(),
            ),
            CommandMessageProcessor.RPM: lambda: (
                self.__sl.acquire(),
                self._update_rpm(command_bytes),
                self.__sl.release(),
            ),
            CommandMessageProcessor.HEARTBEAT: lambda: self._heartbeat(),
        }
        while True:
            command_bytes = self.__serial.read(self.__command_byte_size)
            command_id = self._get_command_id(command_bytes)
            command_name = self._command_id_name[command_id]
            handler[command_name]()

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
    def _publish_imu_state(self):
        """
        Abstract method to publish the 'IMU state' message.
        """
        pass

    @abstractmethod
    def _publish_firmware(self):
        """
        Abstract method to publish the 'firmware' message.
        """
        pass

    @abstractmethod
    def _update_duty_cycle(self, command):
        """
        Abstract method to update the duty cycle in the state data based on the provided command.

        Args:
            command: The command containing the information to update the duty cycle.
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

    @abstractmethod
    def _heartbeat(self):
        """
        Abstract method to handle the 'heartbeat' command.
        """
        pass
