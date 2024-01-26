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
    def _command_id_name(self):
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

    def handle_command(self):
        command_bytes = None
        handler = {
            CommandMessageProcessor.STATE: self._publish_state(),
            CommandMessageProcessor.IMU_STATE: self._publish_imu_state(),
            CommandMessageProcessor.FIRMWARE: self._publish_firmware(),
            CommandMessageProcessor.DUTY_CYCLE: self._update_duty_cycle(command_bytes),
            CommandMessageProcessor.CURRENT: self._update_current(command_bytes),
            CommandMessageProcessor.RPM: self._update_rpm(command_bytes),
            CommandMessageProcessor.HEARTBEAT: self._heartbeat(),
        }
        while True:
            command_bytes = self.__serial.read(self.__command_byte_size)
            command_id = self._get_command_id(command_bytes)
            command_name = self._command_id_name[command_id]
            handler[command_name]

    @abstractmethod
    def _get_command_id(self, command: bytes) -> int:
        pass

    @abstractmethod
    def _publish_state(self):
        pass

    @abstractmethod
    def _publish_imu_state(self):
        pass

    @abstractmethod
    def _publish_firmware(self):
        pass

    @abstractmethod
    def _update_duty_cycle(self, command):
        pass

    @abstractmethod
    def _update_current(self, command):
        pass

    @abstractmethod
    def _update_rpm(self, command):
        pass

    @abstractmethod
    def _heartbeat(self):
        pass
