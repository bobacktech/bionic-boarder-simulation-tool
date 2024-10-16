import pytest
from . import vesc_imu_state_msg_requester, vesc_state_msg_requester
import struct
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


max_negative_64bit = -(2**63)


@pytest.mark.skip(reason="This test is currently disabled.")
def test_vesc_imu_state_erpm_query_command_loop(activate_sim_and_bluetooth_socket):
    app, socket = activate_sim_and_bluetooth_socket
    vsmr = vesc_state_msg_requester.VescStateMsgRequester(socket)
    socket.readyRead.disconnect(vsmr.on_data_received)
    vismr = vesc_imu_state_msg_requester.VescIMUStateMsgRequester(socket)
    socket.readyRead.disconnect(vismr.on_data_received)
    erpm: int = 0
    x_accel: float = 0.0
    last_x_accel_value_after_erpm_command_sent: float = 0.0
    commanded_erpm: int = 0
    for i in range(30):
        socket.readyRead.connect(vsmr.on_data_received)
        vsmr.send_state_msg_request()
        while vsmr.state_msg_received == False:
            app.processEvents()
        socket.readyRead.disconnect(vsmr.on_data_received)
        socket.readyRead.connect(vismr.on_data_received)
        vismr.send_imu_state_msg_request()
        while vismr.imu_state_msg_received == False:
            app.processEvents()
        socket.readyRead.disconnect(vismr.on_data_received)
        state_data: QByteArray = vsmr.state_msg_buffer.pop(0)[1]
        imu_state_data: QByteArray = vismr.imu_state_msg_buffer.pop(0)[1]
        erpm = struct.unpack(">i", state_data[30:34])[0]
        x_accel = struct.unpack(">f", imu_state_data[16:20])[0]
        if i == 9:
            last_x_accel_value_after_erpm_command_sent = x_accel
            commanded_erpm: int = erpm + 5000
            command = bytearray([8]) + bytearray(commanded_erpm.to_bytes(4, "big"))
            socket.write(packetize(command))
            app.processEvents()
    assert x_accel > last_x_accel_value_after_erpm_command_sent
    assert abs(erpm - commanded_erpm) < 1000
