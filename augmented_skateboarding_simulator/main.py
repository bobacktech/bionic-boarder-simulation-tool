from dataclasses import dataclass
import argparse
import threading
import sys
import json
from jsonschema import validate, ValidationError
import os
from augmented_skateboarding_simulator.riding import *


@dataclass(frozen=True)
class AppInputArguments:

    # Electric Skateboard Specifics
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

    # Serial I/O
    com_port: str
    baud_rate: int

    # Kinematic loop
    fixed_time_step_ms: int
    push_period_sec: float
    theta_slope_period_sec: float
    slope_range_bound_deg: float

    # Friction model
    mu_rolling: float
    c_drag: float

    # Motor Controller
    control_time_step_sec: float

    # VESC
    vesc_fw: str
    heartbeat_timeout_sec: float


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("app_inputs_json", type=str, help="This is the path to the simulation app inputs file.")
    args = parser.parse_args()
    if len(sys.argv) != 2:
        print("Error: Exactly one argument is required.")
        sys.exit(1)

    script_dir = os.path.dirname(__file__)
    schema_path = os.path.join(script_dir, "./app_input_arguments.schema.json")
    app_input_json = None
    with open(args.app_inputs_json, "r") as file:
        app_input_json = json.load(file)
        schema = None
        with open(schema_path, "r") as schema_file:
            schema = json.load(schema_file)
        try:
            validate(instance=app_input_json, schema=schema)
        except ValidationError as e:
            print("Error: App inputs data file did not validate against the app input schema.")
            sys.exit(1)

    app_input_arguments = AppInputArguments(**app_input_json)
    eboard_kinematic_state = eboard_kinematic_state.EboardKinematicState()
    eboard_kinematic_state_lock = threading.Lock()
    battery_discharge_model = battery_discharge_model.BatteryDischargeModel(app_input_arguments.battery_max_voltage)
    eboard = eboard.EBoard(
        app_input_arguments.total_weight_with_rider_kg,
        app_input_arguments.frontal_area_of_rider_m2,
        app_input_arguments.wheel_diameter_m,
        app_input_arguments.battery_max_capacity_Ah,
        app_input_arguments.battery_max_voltage,
        app_input_arguments.gear_ratio,
        app_input_arguments.motor_kv,
        app_input_arguments.motor_max_torque,
        app_input_arguments.motor_max_amps,
        app_input_arguments.motor_max_power_watts,
        app_input_arguments.motor_pole_pairs,
    )
    frictional_deceleration_model = frictional_deceleration_model.FrictionalDecelerationModel(
        app_input_arguments.mu_rolling, app_input_arguments.c_drag, eboard
    )
    push_model = push_model.PushModel(eboard)
    motor_controller = motor_controller.MotorController(eboard, eboard_kinematic_state, eboard_kinematic_state_lock)
    kinematic_loop = kinematic_loop.KinematicLoop(
        eboard, eboard_kinematic_state, eboard_kinematic_state_lock, frictional_deceleration_model, push_model
    )
