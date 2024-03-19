import pytest
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
from PyQt6.QtCore import QCoreApplication, QObject
import time


@pytest.fixture
def state_msg():
    return fw_6_00.StateMessage()


@pytest.fixture
def imu_state_msg():
    return fw_6_00.IMUStateMessage()


@pytest.fixture
def cmp(state_msg, imu_state_msg):
    try:
        return fw_6_00.FW6_00CMP(
            "/dev/ttyUSB0",
            256,
            state_msg,
            Lock(),
            imu_state_msg,
            Lock(),
        )
    except:
        return None


class BluetoothClient(QObject):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.socket = QBluetoothSocket(QBluetoothServiceInfo.Protocol.RfcommProtocol)
        self.socket.connected.connect(self.on_connected)
        self.socket.readyRead.connect(self.on_data_received)
        self.socket.errorOccurred.connect(self.on_error_occurred)
        self.socket.disconnected.connect(self.on_disconnected)
        self.result_raw_bytes: bytes = None
        self.send_data: bytes = None

    def on_connected(self):
        bytes_sent = self.socket.write(self.send_data)
        l = len(self.send_data)
        assert bytes_sent == l

    def on_disconnected(self):
        self.socket.close()
        QCoreApplication.instance().quit()

    def on_data_received(self):
        self.result_raw_bytes = self.socket.read(100)
        self.socket.disconnectFromService()

    def connect_to_device(self, address):
        SERIAL_PORT_PROFILE_UUID = "00001101-0000-1000-8000-00805F9B34FB"
        self.socket.connectToService(
            QBluetoothAddress(address),
            QBluetoothUuid(SERIAL_PORT_PROFILE_UUID),
        )

    def on_error_occurred(socket_error):
        assert False, "Connection could not be established with Bluetooth HC06 module."


def packetize(data: bytearray):
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


def test_handle_command_duty_cycle(cmp, state_msg):
    if cmp == None:
        assert (
            False
        ), "fw_6_00.FW6_00CMP did not establish connection with FTDI USB adapter."

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

    # Create a VESC duty cycle command
    duty_cycle = 0.555
    b = bytearray(5)
    b[0] = 5
    d = int(duty_cycle * 100000)
    b[1] = (d >> 24) & 0xFF
    b[2] = (d >> 16) & 0xFF
    b[3] = (d >> 8) & 0xFF
    b[4] = d & 0xFF
    client.send_data = packetize(b)
    simHC06_address = "00:14:03:05:59:CE"
    client.connect_to_device(simHC06_address)

    def duty_check():
        while state_msg.duty_cycle == 0:
            time.sleep(0.5)
        assert state_msg.duty_cycle == duty_cycle
        cmp.serial.close()
        QCoreApplication.instance().quit()

    duty_thread = threading.Thread(target=duty_check)
    duty_thread.daemon = True
    duty_thread.start()
    try:
        sys.exit(app.exec())
    except:
        pass
