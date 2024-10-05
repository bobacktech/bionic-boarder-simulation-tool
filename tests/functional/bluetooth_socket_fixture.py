import os
import pytest
from PyQt6.QtBluetooth import (
    QBluetoothSocket,
    QBluetoothServiceInfo,
    QBluetoothAddress,
    QBluetoothUuid,
)


@pytest.fixture(scope="function")
def bluetooth_socket() -> QBluetoothSocket:
    socket = QBluetoothSocket(QBluetoothServiceInfo.Protocol.RfcommProtocol)
    mac_address = os.getenv("SIMHC06_MAC_ADDRESS")
    if mac_address is None:
        pytest.skip("Environment variable for SIMHC06 bluetooth module MAC address is not set. Test is skipped.")
    SERIAL_PORT_PROFILE_UUID = "00001101-0000-1000-8000-00805F9B34FB"
    socket.connectToService(
        QBluetoothAddress(mac_address),
        QBluetoothUuid(SERIAL_PORT_PROFILE_UUID),
    )
    return socket
