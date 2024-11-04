from augmented_skateboarding_simulator.riding.eboard_state_recorder import EboardStateRecorder
from threading import Lock
from augmented_skateboarding_simulator.riding.eboard_kinematic_state import EboardKinematicState
import time
import os
import struct


def test_record():
    """
    Test the record method of the EboardStateRecorder class.
    """
    eks = EboardKinematicState()
    recording_period_ms = 20
    duration_sec = 1.0
    eboard_state_recorder = EboardStateRecorder(Lock(), eks, recording_period_ms)
    eboard_state_recorder.start_recording()
    time.sleep(duration_sec)
    eboard_state_recorder.stop_recording()
    time.sleep(0.02)
    assert os.path.exists(eboard_state_recorder.record_file_name)
    format_string = "d f f f f f f f i f"
    previous_timestamp = 0.0
    with open(eboard_state_recorder.record_file_name, "rb") as file:
        while True:
            # Read the number of bytes corresponding to the format string
            data = file.read(struct.calcsize(format_string))

            if not data:
                break
            # Unpack the data
            unpacked_data = struct.unpack(format_string, data)

            # Extract the timestamp
            timestamp = unpacked_data[0]
            assert timestamp > previous_timestamp, "Timestamps are not in increasing order"
            previous_timestamp = timestamp
    assert previous_timestamp <= (duration_sec + recording_period_ms / 1000.0)
    os.remove(eboard_state_recorder.record_file_name)
