from PyQt6.QtBluetooth import (
    QBluetoothSocket,
    QBluetoothServiceInfo,
    QBluetoothAddress,
    QBluetoothUuid,
)
from PyQt6.QtCore import QCoreApplication, QObject, QByteArray, QTimer
import time
from typing import List, Tuple


class VescIMUStateMsgRequester(QObject):
    def __init__(self, imu_state_request_time_duration_ms: int):
        super().__init__()
        self.socket = QBluetoothSocket(QBluetoothServiceInfo.Protocol.RfcommProtocol)
        self.socket.connected.connect(self.on_connected)
        self.socket.readyRead.connect(self.on_data_received)
        self.socket.errorOccurred.connect(self.on_error_occurred)
        self.socket.disconnected.connect(self.on_disconnected)
        self.in_data = QByteArray()
        self.imu_state_msg_buffer: List[Tuple[int, QByteArray]] = []
        self.imu_state_request_time_duration_ms = imu_state_request_time_duration_ms

    def connect_to_device(self, address):
        SERIAL_PORT_PROFILE_UUID = "00001101-0000-1000-8000-00805F9B34FB"
        self.socket.connectToService(
            QBluetoothAddress(address),
            QBluetoothUuid(SERIAL_PORT_PROFILE_UUID),
        )

    def on_connected(self):
        QTimer.singleShot(self.imu_state_request_time_duration_ms, self.socket.disconnectFromService)
        self.start_time = int(time.time() * 1000)
        # VESC state message request command
        bytes_sent = self.socket.write(self.packetize(bytearray([65])))

    def on_data_received(self):
        if self.socket.bytesAvailable():
            self.in_data += self.socket.readAll()
        if len(self.in_data) == 71:
            id = int.from_bytes(self.in_data[2])
            assert id == 65
            temp: Tuple[int, QByteArray] = (int(time.time() * 1000) - self.start_time, QByteArray(self.in_data))
            self.imu_state_msg_buffer.append(temp)
            self.in_data.clear()
            # VESC state message request command
            bytes_sent = self.socket.write(self.packetize(bytearray([65])))

    def on_error_occurred(self, error):
        print(f"Error occurred: {error}")

    def on_disconnected(self):
        self.socket.close()
        QCoreApplication.instance().quit()

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
