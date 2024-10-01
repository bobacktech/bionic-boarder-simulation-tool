import subprocess
import os
import pytest
import time
from pyftdi.ftdi import Ftdi


@pytest.fixture(scope="function")
def start_sim_process():
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
    ]
    sim_process = subprocess.Popen(
        command, cwd=os.path.expanduser("~/git/augmented-skateboarding-simulator/augmented_skateboarding_simulator")
    )
    while sim_process.poll() is not None:
        time.sleep(0.2)
    yield sim_process
    sim_process.kill()
