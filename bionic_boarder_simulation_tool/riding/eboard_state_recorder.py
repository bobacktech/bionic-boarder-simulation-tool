from datetime import datetime
from .eboard_kinematic_state import EboardKinematicState
from threading import Thread
import time
import struct
from collections import deque

class EboardStateRecorder:

    def __init__(self, data: deque[tuple[float, EboardKinematicState]], recording_start_delay_s: float, time_step_ms: int):
        self.__data = data
        self.__time_delay_s = recording_start_delay_s
        self.__time_step_s = time_step_ms/1000.0
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.__record_file_name = f"sim_data_recording_{timestamp}.bin"
        self.__recording_thread = Thread(target=self.record, daemon=True)
        self.__stop_recording = False

    def start_recording(self) -> None:
        self.__recording_thread.start()

    def stop_recording(self):
        self.__stop_recording = True

    @property
    def record_file_name(self) -> str:
        return self.__record_file_name

    def record(self) -> None:
        f = open(self.__record_file_name, "wb")
        time.sleep(self.__time_delay_s)
        while True:
            item = self.__data.popleft()
            timestamp, eks = item
            eks_bytes = struct.pack(
                "d f f f f f f f f f f f i f f",
                timestamp,
                eks.velocity,
                eks.acceleration_x,
                eks.acceleration_y,
                eks.acceleration_z,
                eks.pitch,
                eks.roll,
                eks.yaw,
                eks.w,
                eks.x,
                eks.y,
                eks.z,
                eks.erpm,
                eks.motor_current,
                eks.input_current,
                )
            f.write(eks_bytes)
            if self.__stop_recording:
                f.close()
                break
            time.sleep(self.__time_step_s)
