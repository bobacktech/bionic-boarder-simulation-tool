from . import vesc_state_msg_requester
import struct
import pytest
import time
from PyQt6.QtCore import QByteArray


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


@pytest.mark.skip(reason="This test is currently disabled.")
def test_increase_decrease_motor_erpm(activate_sim_and_bluetooth_socket):
    app, socket = activate_sim_and_bluetooth_socket
    vsmr = vesc_state_msg_requester.VescStateMsgRequester(socket)
    latest_erpm = 0
    vsmr.send_state_msg_request()
    while vsmr.state_msg_received == False:
        app.processEvents()
    state_data: QByteArray = vsmr.state_msg_buffer.pop(0)[1]
    latest_erpm = struct.unpack(">i", state_data[25:29])[0]

    target_erpm = latest_erpm + 20000
    command = bytearray([8]) + bytearray(target_erpm.to_bytes(4, "big"))
    socket.write(packetize(command))
    app.processEvents()
    time.sleep(5.0)

    command = bytearray([8]) + bytearray(latest_erpm.to_bytes(4, "big"))
    socket.write(packetize(command))
    app.processEvents()
    time.sleep(5.0)
