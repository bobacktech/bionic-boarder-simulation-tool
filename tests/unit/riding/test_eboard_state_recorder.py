from bionic_boarder_simulation_tool.riding.eboard_state_recorder import EboardStateRecorder
from bionic_boarder_simulation_tool.riding.eboard_kinematic_state import EboardKinematicState
import time
import os
import struct
from collections import deque 


def test_record():
    """
    Test the record method of the EboardStateRecorder class.
    """
    eks = EboardKinematicState()
    recording_period_ms = 10
    time_step_s = 0.01
    data_buffer: deque[tuple[float, EboardKinematicState]] = deque()
    for i in range(1, 1000):
        timestamp = i * time_step_s
        eks.velocity = i * 0.1
        eks.acceleration_x = i * 0.01
        eks.acceleration_y = i * 0.02
        eks.acceleration_z = i * 0.03
        eks.pitch = i * 0.1
        eks.roll = i * 0.2
        eks.yaw = i * 0.3
        eks.w = i * 0.4
        eks.x = i * 0.5
        eks.y = i * 0.6
        eks.z = i * 0.7
        eks.erpm = i * 10
        eks.motor_current = i * 1.5
        eks.input_current = i * 2.5
        data_buffer.append((timestamp, eks))
    eboard_state_recorder = EboardStateRecorder(data_buffer, recording_start_delay_s=1.0, time_step_ms=recording_period_ms)
    eboard_state_recorder.start_recording()
    time.sleep(3.0)
    eboard_state_recorder.stop_recording()
    time.sleep(1)
    assert os.path.exists(eboard_state_recorder.record_file_name)
    format_string = "d f f f f f f f f f f f i f f"
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
    os.remove(eboard_state_recorder.record_file_name)
