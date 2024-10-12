import sys
from PyQt6.QtCore import QCoreApplication
from PyQt6.QtBluetooth import (
    QBluetoothSocket,
    QBluetoothServiceInfo,
    QBluetoothAddress,
    QBluetoothUuid,
)
from . import vesc_state_msg_requester
from .start_sim_fixture import start_sim_process
from .bluetooth_socket_fixture import bluetooth_socket as socket
import struct
import matplotlib.pyplot as plt
import os
from datetime import datetime
import pytest
import time

NUMBER_VESC_STATE_MSG_REQUESTS = 40


@pytest.mark.skip(reason="This test is currently disabled.")
def test_erpm_values_without_commanding_motor(start_sim_process, socket):
    app = QCoreApplication(sys.argv)
    while socket.state() != QBluetoothSocket.SocketState.ConnectedState:
        app.processEvents()
    start_time = int(time.time() * 1000)
    vsmr = vesc_state_msg_requester.VescStateMsgRequester(socket)
    vsmr.send_state_msg_request()
    count = 0
    while count < NUMBER_VESC_STATE_MSG_REQUESTS:
        while vsmr.state_msg_received == False:
            app.processEvents()
        vsmr.send_state_msg_request()
        count += 1
    socket.disconnectFromService()
    while socket.state() == QBluetoothSocket.SocketState.ConnectedState:
        app.processEvents()
    socket.close()
    while socket.state() == QBluetoothSocket.SocketState.ClosingState:
        app.processEvents()
    app.quit()
    times = []
    erpms = []
    assert len(vsmr.state_msg_buffer) > 0
    for timestamp, byte_array in vsmr.state_msg_buffer:
        current = struct.unpack(">I", byte_array[12:16])[0]
        assert current == 0
        erpms.append(struct.unpack(">i", byte_array[30:34])[0])
        times.append(timestamp - start_time)
    assert len(times) == len(erpms)
    plt.figure(figsize=(10, 10))
    plt.plot(times, erpms, marker="o")
    plt.title("Times vs ERPMs")
    plt.xlabel("Time (ms)")
    plt.ylabel("ERPM")
    plt.grid(True)
    current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
    plt.savefig(os.path.expanduser(f"~/erpm_plot_{current_time}.png"))
    plt.close()
