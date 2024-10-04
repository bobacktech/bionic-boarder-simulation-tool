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
import struct
import matplotlib.pyplot as plt
import os
from datetime import datetime
import pytest
import time

NUMBER_VESC_STATE_MSG_REQUESTS = 40


@pytest.mark.skip(reason="This test is currently disabled.")
def test_erpm_values_without_commanding_motor(start_sim_process):
    app = QCoreApplication(sys.argv)
    socket = QBluetoothSocket(QBluetoothServiceInfo.Protocol.RfcommProtocol)
    mac_address = os.getenv("SIMHC06_MAC_ADDRESS")
    if mac_address is None:
        pytest.skip("Environment variable for SIMHC06 bluetooth module MAC address is not set. Test is skipped.")
    SERIAL_PORT_PROFILE_UUID = "00001101-0000-1000-8000-00805F9B34FB"
    socket.connectToService(
        QBluetoothAddress(mac_address),
        QBluetoothUuid(SERIAL_PORT_PROFILE_UUID),
    )
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
    socket.close()
    app.quit()
    times = []
    erpms = []
    assert len(vsmr.state_msg_buffer) > 0
    for timestamp, byte_array in vsmr.state_msg_buffer:
        current = struct.unpack(">I", byte_array[12:16])[0]
        assert current == 0
        erpms.append(struct.unpack(">I", byte_array[30:34])[0])
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
