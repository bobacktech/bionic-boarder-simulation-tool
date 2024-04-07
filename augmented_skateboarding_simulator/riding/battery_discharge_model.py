from threading import Lock


class BatteryDischargeModel:
    def __init__(self, battery_max_nominal_voltage: float) -> None:
        self.__current_draw = 1.0
        self.__nominal_voltage = battery_max_nominal_voltage
        self.__total_energy_consumed_Wh = 0
        self.__total_energy_consumed_lock = Lock()

    def get_watt_hours_consumed(self) -> float:
        self.__total_energy_consumed_lock.acquire()
        e = self.__total_energy_consumed_Wh
        self.__total_energy_consumed_lock.release()
        return e

    def discharge(self, time_duration_ms: int):
        power_watts = self.__nominal_voltage * self.__current_draw
        step = time_duration_ms / 1000
        energy_consumed = power_watts * (step / 3600)
        self.__total_energy_consumed_lock.acquire()
        self.__total_energy_consumed_Wh += energy_consumed
        self.__total_energy_consumed_lock.release()

    @property
    def current_draw(self) -> float:
        return self.__current_draw

    @current_draw.setter
    def current_draw(self, value: float) -> None:
        self.__current_draw = value
