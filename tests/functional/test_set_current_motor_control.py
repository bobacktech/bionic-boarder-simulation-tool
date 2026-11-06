from tests.functional import bionic_boarder_msg_requester
from tests.functional.conftest import UART_RX_CHAR_UUID
import struct
import pytest
import asyncio
import time


def packetize(data: bytearray) -> bytes:
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


@pytest.mark.skip(reason="Skip is the default behavior. Remove this line to enable the test.")
@pytest.mark.asyncio
async def test_current_motor_control(activate_sim_and_ble_client):
    client = activate_sim_and_ble_client
    bb_requester = bionic_boarder_msg_requester.VescBionicBoarderMsgRequester(client)
    await bb_requester.set_up_response_handler()
    count = 0
    while count < 2:        
        target_current = 25.0 * (count+1) 
        scaled_current = int(target_current * 100)
        set_current_command = bytearray([6]) + bytearray(scaled_current.to_bytes(4, "big"))
        data = packetize(set_current_command)
        await client.write_gatt_char(UART_RX_CHAR_UUID, data, response=False)
        time.sleep(1)
        await bb_requester.send_command()
        while bb_requester.response_received == False:
            await asyncio.sleep(0.1)
        bb_data = bb_requester.response_buffer.pop(0)[1]    
        latest_current = (struct.unpack(">i", bb_data[7:11])[0])/100.0
        assert latest_current == target_current
        count += 1
    zero_current_command = bytearray([6]) + bytearray((0).to_bytes(4, "big"))
    data = packetize(zero_current_command)
    await client.write_gatt_char(UART_RX_CHAR_UUID, data, response=False)
    time.sleep(1)
    await bb_requester.send_command()
    while bb_requester.response_received == False:
        await asyncio.sleep(0.1)
    bb_data = bb_requester.response_buffer.pop(0)[1]    
    latest_current = (struct.unpack(">i", bb_data[7:11])[0])/100.0
    assert latest_current == 0.0
    
        
