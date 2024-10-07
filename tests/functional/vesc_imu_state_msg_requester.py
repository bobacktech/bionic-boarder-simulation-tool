from PyQt6.QtCore import QObject, QByteArray
import time
from typing import List, Tuple


class VescIMUStateMsgRequester(QObject):
    def __init__(self, socket):
        super().__init__()
        self.socket = socket
        self.socket.readyRead.connect(self.on_data_received)
        self.in_data = QByteArray()
        self.imu_state_msg_buffer: List[Tuple[int, QByteArray]] = []
        self.imu_state_msg_received = False

    def send_imu_state_msg_request(self):
        bytes_sent = self.socket.write(self.packetize(bytearray([65])))
        self.imu_state_msg_received = False

    def on_data_received(self):
        if self.socket.bytesAvailable():
            self.in_data += self.socket.readAll()
        if len(self.in_data) == 71:
            id = int.from_bytes(self.in_data[2])
            assert id == 65
            temp: Tuple[int, QByteArray] = (int(time.time() * 1000), QByteArray(self.in_data))
            self.imu_state_msg_buffer.append(temp)
            self.in_data.clear()
            self.imu_state_msg_received = True

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
