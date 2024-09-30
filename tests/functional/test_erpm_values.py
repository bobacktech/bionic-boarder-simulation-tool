import sys
from PyQt6.QtCore import QCoreApplication
from . import vesc_state_msg_requester
from .start_sim_fixture import start_sim_process
import struct
import matplotlib.pyplot as plt
import os
from datetime import datetime
import pytest

# Set this time value to whatever duration is desired
VESC_STATE_MSG_REQUEST_DURATION_SEC = 5


@pytest.mark.skip(reason="This test is currently disabled.")
def test_erpm_values(start_sim_process):
    vsmr = vesc_state_msg_requester.VescStateMsgRequester(VESC_STATE_MSG_REQUEST_DURATION_SEC * 1000)
    app = QCoreApplication(sys.argv)
    mac_address = os.getenv("SIMHC06_MAC_ADDRESS")
    if mac_address is None:
        pytest.skip("Environment variable for SIMHC06 bluetooth module MAC address is not set. Test is skipped.")
    vsmr.connect_to_device(mac_address)
    try:
        sys.exit(app.exec())
    except:
        pass
    times = []
    erpms = []
    assert len(vsmr.state_msg_buffer) > 0
    for timestamp, byte_array in vsmr.state_msg_buffer:
        erpms.append(struct.unpack(">I", byte_array[30:34])[0])
        times.append(timestamp)
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
