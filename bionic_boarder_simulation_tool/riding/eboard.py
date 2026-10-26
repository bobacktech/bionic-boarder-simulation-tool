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
    
    # VESC configuration parameters
    imu_rotation_roll_deg: float
    imu_rotation_pitch_deg: float
    imu_rotation_yaw_deg: float
    accel_offset_x_g: float
    accel_offset_y_g: float
    accel_offset_z_g: float
    gyro_offset_x_deg_per_s: float
    gyro_offset_y_deg_per_s: float
    gyro_offset_z_deg_per_s: float
    motor_current_max_A: float
    battery_current_max_A: float    
