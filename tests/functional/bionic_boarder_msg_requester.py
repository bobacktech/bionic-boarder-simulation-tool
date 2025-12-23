from PyQt6.QtCore import QByteArray
import time
from typing import List, Tuple
from bleak.backends.characteristic import BleakGATTCharacteristic
from .conftest import UART_RX_CHAR_UUID, UART_TX_CHAR_UUID


class VescBionicBoarderMsgRequester:
    def __init__(self, socket):
        super().__init__()
        self.in_data = QByteArray()
        self.bionic_boarder_msg_buffer: List[Tuple[int, QByteArray]] = []
        self.socket = socket
        socket.readyRead.connect(self.on_data_received)
        self.bionic_boarder_msg_received = False

    def send_bionic_boarder_msg_request(self):
        # VESC bionic boarder message request command
        bytes_sent = self.socket.write(self.packetize(bytearray([164])))
        self.bionic_boarder_msg_received = False

    def on_data_received(self):
        if self.socket.bytesAvailable():
            self.in_data += self.socket.readAll()
        if len(self.in_data) == 37:
            id = int.from_bytes(self.in_data[2])
            assert id == 164
            temp: Tuple[int, QByteArray] = (int(time.time() * 1000), QByteArray(self.in_data))
            self.bionic_boarder_msg_buffer.append(temp)
            self.in_data.clear()
            self.bionic_boarder_msg_received = True

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


class VescBionicBoarderMsgRequesterBLE:
    def __init__(self, client):
        self.client = client
        self.response_buffer: List[Tuple[int, bytearray]] = []
        self.response_received = False

    async def set_up_response_handler(self):
        characteristic = self.client.services.get_characteristic(UART_TX_CHAR_UUID)
        await self.client.start_notify(characteristic, self.on_data_received)

    def on_data_received(self, sender: BleakGATTCharacteristic, data: bytearray):
        if len(data) == 37:
            id = data[2]
            assert id == 164
            temp: Tuple[int, bytearray] = (int(time.time() * 1000), data)
            self.response_buffer.append(temp)
            self.response_received = True

    async def send_command(self):
        data = self.packetize(bytearray([164]))
        await self.client.write_gatt_char(UART_RX_CHAR_UUID, data, response=False)
        self.response_received = False

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
