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


class BluetoothStateChangeClient(QObject):
    duty_cycle = 0.455
    current = 24.57
    rpm = 14560

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
        self.socket.errorOccurred.connect(self.on_error_occurred)

    def on_connected(self):
        # VESC duty cycle command
        b = bytearray(5)
        b[0] = 5
        d = int(BluetoothStateChangeClient.duty_cycle * 100000)
        b[1] = (d >> 24) & 0xFF
        b[2] = (d >> 16) & 0xFF
        b[3] = (d >> 8) & 0xFF
        b[4] = d & 0xFF
        d1 = self.packetize(b)
        bytes_sent = self.socket.write(d1)
        assert bytes_sent == 256
        # VESC current command
        b = bytearray(5)
        b[0] = 6
        d = int(BluetoothStateChangeClient.current * 1000)
        b[1] = (d >> 24) & 0xFF
        b[2] = (d >> 16) & 0xFF
        b[3] = (d >> 8) & 0xFF
        b[4] = d & 0xFF
        d2 = self.packetize(b)
        bytes_sent = self.socket.write(d2)
        assert bytes_sent == 256
        b = bytearray(5)
        b[0] = 8
        d = BluetoothStateChangeClient.rpm
        b[1] = (d >> 24) & 0xFF
        b[2] = (d >> 16) & 0xFF
        b[3] = (d >> 8) & 0xFF
        b[4] = d & 0xFF
        d3 = self.packetize(b)
        bytes_sent = self.socket.write(d3)
        assert bytes_sent == 256

    def connect_to_device(self, address):
        SERIAL_PORT_PROFILE_UUID = "00001101-0000-1000-8000-00805F9B34FB"
        self.socket.connectToService(
            QBluetoothAddress(address),
            QBluetoothUuid(SERIAL_PORT_PROFILE_UUID),
        )

    def on_error_occurred(socket_error):
        assert False, "Connection could not be established with Bluetooth HC06 module."


def test_handle_state_change_commands():
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
    client = BluetoothStateChangeClient()
    simHC06_address = "00:14:03:05:59:CE"
    client.connect_to_device(simHC06_address)

    def state_change_check():
        while state_msg.duty_cycle == 0:
            time.sleep(0.4)
        assert state_msg.duty_cycle == BluetoothStateChangeClient.duty_cycle
        while state_msg.motor_current == 0:
            time.sleep(0.4)
        assert state_msg.motor_current == BluetoothStateChangeClient.current
        while state_msg.rpm == 0:
            time.sleep(0.4)
        assert state_msg.rpm == BluetoothStateChangeClient.rpm
        cmp.serial.close()
        QCoreApplication.instance().quit()

    state_change_thread = threading.Thread(target=state_change_check)
    state_change_thread.daemon = True
    state_change_thread.start()
    try:
        sys.exit(app.exec())
    except:
        pass
