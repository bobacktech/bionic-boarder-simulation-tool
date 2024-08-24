from dataclasses import dataclass
from sim_time_manager import SimTimeManager
import argparse
import re
import threading
from vesc import fw_6_00, fw
import sys
from riding import motor_state


@dataclass
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


def firmwareRegex(argValue, pattern=re.compile(r"^\d*[.]\d*$")):
    if not pattern.match(argValue):
        raise argparse.ArgumentTypeError("VESC firmware version specified as $MajorVersion.$MinorVersion, e.g 2.18")
    return argValue


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--vescFW",
        help="Specifies the VESC firmware version to be used in the simulation.",
        type=firmwareRegex,
    )
    parser.add_argument("--comPort", help="The com port for the attached USB FTDI module.")
    args = parser.parse_args()
    com_port = args.comPort
    vesc_fw = args.vescFW
    motor_state = motor_state.MotorState(0.0, 0.0, 0.0)
    motor_state_lock = threading.Lock()
    cmp_thread = None
    if vesc_fw == fw.FirmwareVersion.FW_6_00.value:
        cmp = fw_6_00.FW6_00CMP(com_port, 100, motor_state, motor_state_lock)
        cmp_thread = threading.Thread(target=cmp.handle_command())
        cmp_thread.daemon = True
        cmp_thread.start()
    else:
        print("No VESC firmware specified. Exiting simulation.")
        sys.exit(1)

    cmp_thread.join()

    # stm = SimTimeManager()
    # stm.set_sim_time_step(10)
    # stm.start_sim()
