import sys
from PyQt6.QtCore import QCoreApplication
from . import vesc_imu_state_msg_requester
from .start_sim_fixture import start_sim_process
import struct
import matplotlib.pyplot as plt
import os
from datetime import datetime
import pytest
import math


# Set this time value to whatever duration is desired
VESC_IMU_STATE_MSG_REQUEST_DURATION_SEC = 5


@pytest.mark.skip(reason="This test is currently disabled.")
def test_erpm_values_without_commanding_motor(start_sim_process):
    vismr = vesc_imu_state_msg_requester.VescIMUStateMsgRequester(VESC_IMU_STATE_MSG_REQUEST_DURATION_SEC * 1000)
    app = QCoreApplication(sys.argv)
    mac_address = os.getenv("SIMHC06_MAC_ADDRESS")
    if mac_address is None:
        pytest.skip("Environment variable for SIMHC06 bluetooth module MAC address is not set. Test is skipped.")
    vismr.connect_to_device(mac_address)
    try:
        sys.exit(app.exec())
    except:
        pass
    times = []
    x_accels = []
    pitchs = []
    assert len(vismr.imu_state_msg_buffer) > 0
    for timestamp, byte_array in vismr.imu_state_msg_buffer:
        x_accels.append(struct.unpack(">f", byte_array[16:20])[0])
        pitchs.append(struct.unpack(">f", byte_array[8:12])[0] * 180.0 / math.pi)
        times.append(timestamp)
    assert len(times) == len(pitchs)
    assert len(times) == len(x_accels)

    plt.figure(figsize=(10, 10))
    plt.plot(times, pitchs, marker="o")
    plt.title("Time vs Pitch")
    plt.xlabel("Time (ms)")
    plt.ylabel("Pitch (deg)")
    plt.grid(True)
    current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
    plt.savefig(os.path.expanduser(f"~/pitch_plot_{current_time}.png"))

    plt.figure(figsize=(10, 10))
    plt.plot(times, x_accels, marker="o")
    plt.title("Time vs X Accel")
    plt.xlabel("Time (ms)")
    plt.ylabel("X Accel (m/s^2)")
    plt.grid(True)
    plt.savefig(os.path.expanduser(f"~/x_accel_plot_{current_time}.png"))
    plt.close()
