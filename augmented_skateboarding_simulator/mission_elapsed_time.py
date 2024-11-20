import time


class MissionElapsedTime:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._start_time_sec = time.time()
        return cls._instance

    @property
    def elapsed_time_sec(self) -> float:
        return time.time() - self._start_time_sec
