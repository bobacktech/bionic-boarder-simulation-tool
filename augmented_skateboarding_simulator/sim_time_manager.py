import time


class SimTimeManager:
    __time_step_ms = 0
    __sim_start_time_ms = 0

    def __init__(self):
        pass

    def set_sim_time_step(self, step_ms):
        if SimTimeManager.__time_step_ms == 0:
            SimTimeManager.__time_step_ms = step_ms

    def sim_time_step(self) -> int:
        return SimTimeManager.__time_step_ms

    def start_sim(self):
        if SimTimeManager.__sim_start_time_ms == 0:
            SimTimeManager.__sim_start_time_ms = int(time.time_ns() // 1000000)

    def sim_elapsed_time(self) -> int:
        return int(time.time_ns() // 1000000) - SimTimeManager.__sim_start_time_ms
