from augmented_skateboarding_simulator.vesc.command_message_processor import (
    CommandMessageProcessor,
)
import threading
from threading import Lock
import sys
from PyQt6.QtBluetooth import (
    QBluetoothSocket,
    QBluetoothServiceInfo,
    QBluetoothAddress,
    QBluetoothUuid,
)
from PyQt6.QtCore import QCoreApplication, QObject


class TestCommandMessageProcessor(CommandMessageProcessor):
    def __init__(self, com_port, command_byte_size):
        super().__init__(com_port, command_byte_size, Lock(), Lock())
        self.__cmd_id_name = {
            1: CommandMessageProcessor.DUTY_CYCLE,
            2: CommandMessageProcessor.CURRENT,
            3: CommandMessageProcessor.RPM,
            4: CommandMessageProcessor.HEARTBEAT,
            5: CommandMessageProcessor.FIRMWARE,
            6: CommandMessageProcessor.STATE,
            7: CommandMessageProcessor.IMU_STATE,
        }

    @property
    def _command_id_name(self):
        """
        Returns a dictionary of the command id associated to the name of the command.
        """
        return self.__cmd_id_name

    def _get_command_id(self, command: bytes) -> int:
        return 1

    def _publish_state(self):
        pass

    def _publish_imu_state(self):
        pass

    def _publish_firmware(self):
        pass

    TEST_NUMBER = 777

    def _update_duty_cycle(self, command):
        result = int.from_bytes(command[0:4])
        assert result == TestCommandMessageProcessor.TEST_NUMBER
        data = int.to_bytes(result, 4, byteorder="big") + bytes(96)
        self.serial.write(data)
        self.serial.flush()
        self.serial.close()

    def _update_current(self, command):
        pass

    def _update_rpm(self, command):
        pass


class BluetoothClient(QObject):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.socket = QBluetoothSocket(QBluetoothServiceInfo.Protocol.RfcommProtocol)
        self.socket.connected.connect(self.on_connected)
        self.socket.readyRead.connect(self.on_data_received)
        self.socket.errorOccurred.connect(self.on_error_occurred)
        self.socket.disconnected.connect(self.on_disconnected)

    def on_connected(self):
        data = int.to_bytes(TestCommandMessageProcessor.TEST_NUMBER, 4) + bytes(96)
        bytes_sent = self.socket.write(data)
        assert bytes_sent == 100

    def on_disconnected(self):
        self.socket.close()
        QCoreApplication.instance().quit()

    def on_data_received(self):
        raw_bytes = self.socket.read(100)
        result = int.from_bytes(raw_bytes[0:4])
        assert result == TestCommandMessageProcessor.TEST_NUMBER
        self.socket.disconnectFromService()

    def connect_to_device(self, address):
        SERIAL_PORT_PROFILE_UUID = "00001101-0000-1000-8000-00805F9B34FB"
        self.socket.connectToService(
            QBluetoothAddress(address),
            QBluetoothUuid(SERIAL_PORT_PROFILE_UUID),
        )

    def on_error_occurred(socket_error):
        assert False, "Connection could not be established with Bluetooth HC06 module."


def test_bluetooth_to_serial_comms():
    """
    This unit test case sends a data packet over the laptop's Bluetooth radio to the HC06 module
    and then receives a data packet from the HC06 module.
    """
    cmp = None
    try:
        cmp = TestCommandMessageProcessor("/dev/ttyUSB0", 100)
    except:
        assert (
            False
        ), "Test Command Message Processor did not establish connection with FTDI USB adapter."

    def handle():
        try:
            cmp.handle_command()
        except:
            pass

    cmp_thread = threading.Thread(target=handle)
    cmp_thread.daemon = True
    cmp_thread.start()

    app = QCoreApplication(sys.argv)
    client = BluetoothClient()
    simHC06_address = "00:14:03:05:59:CE"
    client.connect_to_device(simHC06_address)
    try:
        sys.exit(app.exec())
    except:
        pass
