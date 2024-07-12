from dataclasses import dataclass


@dataclass(frozen=True)
class EBoard:
    total_weight_with_rider_kg: float
    frontal_area_of_rider_m2: float
    wheel_diameter_m: float
    battery_max_capacity_Ah: float
    battery_max_voltage: float
    gear_ratio: float
    motor_kv: int
    motor_max_torque: float
    motor_max_amps: float
    motor_max_power_watts: float
    motor_pole_pairs: int
