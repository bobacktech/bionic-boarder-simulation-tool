import subprocess
import os
import time
import pytest
from bleak import BleakScanner, BleakClient

UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # Write to device
UART_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # Notify from device


@pytest.fixture
async def activate_sim_and_ble_client():
    command = [
        "poetry",
        "run",
        "python",
        "main.py",
        os.path.expanduser("~/git/bionic-boarder-simulation-tool/tests/app_input_arguments_example.json"),
        "--enable-logging",
    ]
    sim_process = subprocess.Popen(
        command, cwd=os.path.expanduser("~/git/bionic-boarder-simulation-tool/bionic_boarder_simulation_tool")
    )
    while sim_process.poll() is not None:
        time.sleep(0.2)

    sim_ble_device_name = os.getenv("SIM_BLE_DEVICE_NAME")
    if sim_ble_device_name is None:
        pytest.fail("Environment variable for SIM_BLE_DEVICE_NAME is not set. Test is skipped.")
    device = await BleakScanner.find_device_by_name(sim_ble_device_name, timeout=10.0)
    if device is None:
        pytest.skip(f"BLE device '{sim_ble_device_name}' not found during scan.")
    client = BleakClient(device)
    await client.connect()
    uart_service = client.services.get_service(UART_SERVICE_UUID)
    if uart_service is None:
        pytest.fail("UART service not found on the BLE device.")
    if client._backend.__class__.__name__ == "BleakClientBlueZDBus":
        await client._backend._acquire_mtu()
    assert client.mtu_size > 256
    yield client
    await client.disconnect()
    sim_process.kill()
