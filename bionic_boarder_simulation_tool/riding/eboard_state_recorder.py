from datetime import datetime
from .eboard_kinematic_state import EboardKinematicState
from threading import Lock, Thread
import time
import struct
from bionic_boarder_simulation_tool.mission_elapsed_time import MissionElapsedTime


class EboardStateRecorder:

    def __init__(self, eks_lock: Lock, eks: EboardKinematicState, recording_period_ms: int):
        self.__eks: EboardKinematicState = eks
        self.__eks_lock: Lock = eks_lock
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.__record_file_name = f"sim_data_recording_{timestamp}.bin"
        self.__recording_period_s: float = recording_period_ms / 1000.0
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
        while True:
            eks_bytes = None
            with self.__eks_lock:
                timestamp = MissionElapsedTime().elapsed_time_sec
                eks_bytes = struct.pack(
                    "d f f f f f f f i f",
                    timestamp,
                    self.__eks.velocity,
                    self.__eks.acceleration_x,
                    self.__eks.acceleration_y,
                    self.__eks.acceleration_z,
                    self.__eks.pitch,
                    self.__eks.roll,
                    self.__eks.yaw,
                    self.__eks.erpm,
                    self.__eks.motor_current,
                )
            f.write(eks_bytes)
            if self.__stop_recording:
                f.close()
                break
            time.sleep(self.__recording_period_s)
