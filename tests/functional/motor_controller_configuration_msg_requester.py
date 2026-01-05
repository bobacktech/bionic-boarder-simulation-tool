import time
from typing import List, Tuple
from bleak.backends.characteristic import BleakGATTCharacteristic
from .conftest import UART_RX_CHAR_UUID, UART_TX_CHAR_UUID


class MotorControllerConfigurationMsgRequester:
    def __init__(self, client):
        self.client = client
        self.response_buffer: List[Tuple[int, bytearray]] = []
        self.response_received = False
        self.__received_data: bytearray = bytearray()

    async def set_up_response_handler(self):
        characteristic = self.client.services.get_characteristic(UART_TX_CHAR_UUID)
        await self.client.start_notify(characteristic, self.on_data_received)

    def on_data_received(self, sender: BleakGATTCharacteristic, data: bytearray):
        self.__received_data += data
        if len(self.__received_data) == 700:
            id = self.__received_data[3]
            assert id == 14
            temp: Tuple[int, bytearray] = (int(time.time() * 1000), self.__received_data.copy())
            self.response_buffer.append(temp)
            self.response_received = True
            self.__received_data.clear()

    async def send_command(self):
        data = self.packetize(bytearray([14]))
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
