import subprocess
import os
import pytest
import time
from pyftdi.ftdi import Ftdi
from PyQt6.QtBluetooth import (
    QBluetoothSocket,
    QBluetoothServiceInfo,
    QBluetoothAddress,
    QBluetoothUuid,
)
from PyQt6.QtCore import QCoreApplication
import sys


@pytest.fixture
def activate_sim_and_bluetooth_socket():
    try:
        # List all connected FTDI devices
        devices = Ftdi.list_devices()
        if not devices:
            pytest.skip("USB FTDI adapter for HC06 bluetooth is not connected to PC. Test case is skipped.")
    except pytest.skip.Exception:
        raise
    except:
        # An exception will be thrown if the FTDI USB adapter is connected to the PC due to an access privilege issue.
        # This verifies that the FTDI USB adapter is connected and accessible to the simulation.
        pass
    command = [
        "poetry",
        "run",
        "python",
        "main.py",
        os.path.expanduser("~/git/augmented-skateboarding-simulator/tests/app_input_arguments_example.json"),
        "--enable-logging",
    ]
    sim_process = subprocess.Popen(
        command, cwd=os.path.expanduser("~/git/augmented-skateboarding-simulator/augmented_skateboarding_simulator")
    )
    while sim_process.poll() is not None:
        time.sleep(0.2)

    socket = QBluetoothSocket(QBluetoothServiceInfo.Protocol.RfcommProtocol)
    mac_address = os.getenv("SIMHC06_MAC_ADDRESS")
    if mac_address is None:
        pytest.skip("Environment variable for SIMHC06 bluetooth module MAC address is not set. Test is skipped.")
    SERIAL_PORT_PROFILE_UUID = "00001101-0000-1000-8000-00805F9B34FB"
    socket.connectToService(
        QBluetoothAddress(mac_address),
        QBluetoothUuid(SERIAL_PORT_PROFILE_UUID),
    )
    app = QCoreApplication(sys.argv)
    while socket.state() != QBluetoothSocket.SocketState.ConnectedState:
        app.processEvents()
    yield app, socket
    socket.disconnectFromService()
    while socket.state() == QBluetoothSocket.SocketState.ConnectedState:
        app.processEvents()
    socket.close()
    sim_process.kill()
