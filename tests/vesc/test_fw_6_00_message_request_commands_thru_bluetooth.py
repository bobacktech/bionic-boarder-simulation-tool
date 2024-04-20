from augmented_skateboarding_simulator.vesc import fw_6_00
import threading
from threading import Lock
import sys
from PyQt6.QtBluetooth import (
    QBluetoothSocket,
    QBluetoothServiceInfo,
    QBluetoothAddress,
    QBluetoothUuid,
)
from PyQt6.QtCore import QCoreApplication, QObject, QByteArray
import time
from augmented_skateboarding_simulator.riding.motor_state import MotorState


class BluetoothMessageRequestClient(QObject):

    def packetize(self, data: bytearray):
        packet = bytearray(256)
        i = 0
        packet[i] = 2
        i += 1
        packet[i] = len(data)
        i += 1
        packet[i : i + len(data)] = data
        i += len(data)
        # Sim doesn't check crc. So any value can be used.
        crcValue = 1111
        packet[i] = (crcValue >> 8) & 0xFF
        i += 1
        packet[i] = crcValue & 0xFF
        i += 1
        packet[i] = 3
        i += 1
        packet[i] = 0
        return bytes(packet)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.socket = QBluetoothSocket(QBluetoothServiceInfo.Protocol.RfcommProtocol)
        self.socket.connected.connect(self.on_connected)
        self.socket.readyRead.connect(self.on_data_received)
        self.socket.errorOccurred.connect(self.on_error_occurred)
        self.socket.disconnected.connect(self.on_disconnected)
        self.in_data = QByteArray()
        self.state_msg_received = False

    def on_connected(self):
        # VESC state message request command
        b = bytearray(1)
        b[0] = 4
        d1 = self.packetize(b)
        bytes_sent = self.socket.write(d1)
        assert bytes_sent == 256
        # VESC imu state message request command
        b = bytearray(1)
        b[0] = 65
        d2 = self.packetize(b)
        bytes_sent = self.socket.write(d2)
        assert bytes_sent == 256

    def connect_to_device(self, address):
        SERIAL_PORT_PROFILE_UUID = "00001101-0000-1000-8000-00805F9B34FB"
        self.socket.connectToService(
            QBluetoothAddress(address),
            QBluetoothUuid(SERIAL_PORT_PROFILE_UUID),
        )

    def on_disconnected(self):
        self.socket.close()
        QCoreApplication.instance().quit()

    def on_error_occurred(socket_error):
        assert False, "Connection could not be established with Bluetooth HC06 module."

    def on_data_received(self):
        if self.socket.bytesAvailable():
            self.in_data += self.socket.readAll()
        if len(self.in_data) == 79:
            id = int.from_bytes(self.in_data[2])
            assert id == 4
            self.in_data.clear()
            self.state_msg_received = True
        if self.state_msg_received == True and len(self.in_data) == 71:
            id = int.from_bytes(self.in_data[2])
            assert id == 65
            self.in_data.clear()
            self.socket.disconnectFromService()


def test_handle_message_request_commands():
    state_msg = fw_6_00.StateMessage()
    cmp = None
    try:
        cmp = fw_6_00.FW6_00CMP(
            "/dev/ttyUSB0",
            256,
            state_msg,
            Lock(),
            fw_6_00.IMUStateMessage(),
            Lock(),
            MotorState(),
        )
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
    client = BluetoothMessageRequestClient()
    simHC06_address = "00:14:03:05:59:CE"
    client.connect_to_device(simHC06_address)
    try:
        sys.exit(app.exec())
    except:
        pass
    cmp.serial.close()
    # Allow for Bluetooth connection to fully disconnect before running the next test case with Bluetooth connection
    time.sleep(5)
