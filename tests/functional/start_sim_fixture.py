import subprocess
import os
import pytest
import time


@pytest.fixture(scope="function")
def start_sim_process():
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
