from dataclasses import dataclass


@dataclass(frozen=True)
class EBoard:
    total_weight_with_rider_kg: float
    wheel_diameter_m: float
    battery_max_capacity_Ah: float
    battery_max_voltage: float
    gear_ratio: float
    motor_kv: int
