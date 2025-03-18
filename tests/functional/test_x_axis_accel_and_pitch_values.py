from . import vesc_imu_state_msg_requester
import struct
import matplotlib.pyplot as plt
import os
from datetime import datetime
import pytest
import math
import time

NUMBER_VESC_IMU_STATE_MSG_REQUESTS = 30


@pytest.mark.skip(reason="This test is currently disabled.")
def test_x_axis_accel_and_pitch_values(activate_sim_and_bluetooth_socket):
    app, socket = activate_sim_and_bluetooth_socket
    start_time = int(time.time() * 1000)
    vismr = vesc_imu_state_msg_requester.VescIMUStateMsgRequester(socket)
    vismr.send_imu_state_msg_request()
    count = 0
    while count < NUMBER_VESC_IMU_STATE_MSG_REQUESTS:
        while vismr.imu_state_msg_received == False:
            app.processEvents()
        vismr.send_imu_state_msg_request()
        count += 1
    times = []
    x_accels = []
    pitchs = []
    assert len(vismr.imu_state_msg_buffer) > 0
    for timestamp, byte_array in vismr.imu_state_msg_buffer:
        x_accels.append(struct.unpack(">f", byte_array[17:21])[0])
        pitchs.append(struct.unpack(">f", byte_array[9:13])[0] * 180.0 / math.pi)
        times.append(timestamp - start_time)
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
