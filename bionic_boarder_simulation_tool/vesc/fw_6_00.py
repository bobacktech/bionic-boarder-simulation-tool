from enum import IntEnum
from . import fw
from .command_message_processor import CommandMessageProcessor
import struct
from threading import Lock
import math
from bionic_boarder_simulation_tool.riding.battery_discharge_model import BatteryDischargeModel
from bionic_boarder_simulation_tool.riding.eboard_kinematic_state import EboardKinematicState
from bionic_boarder_simulation_tool.riding.motor_controller import MotorController
from bionic_boarder_simulation_tool.logger import Logger
from bionic_boarder_simulation_tool.riding.eboard import EBoard


class FirmwareMessage:
    """
    See the message specification in [commands.c](https://github.com/vedderb/bldc/blob/6.00/comm/commands.c)
    in VESC bldc-6.00 source code on Github.
    """

    ID = 0

    def __init__(self) -> None:
        """
        Initializes a new instance of the FirmwareMessage class.

        Sets up the firmware message buffer according to the specified format.
        The buffer is initialized with predefined values, and a section is filled with an encoded string.
        """
        self.__buffer = bytearray(64)
        self.__buffer[0] = 6
        self.__buffer[1] = 0
        self.__buffer[2:14] = "HardwareName".encode("utf-8")

    @property
    def buffer(self):
        """
        The buffer property representing the immutable form of the firmware message.

        Returns:
            bytes: An immutable bytes object representing the current state of the firmware message buffer.
        """
        return FirmwareMessage.ID.to_bytes(1) + bytes(self.__buffer)


class MotorControllerConfigurationMessage:
    """
    See the "COMM_GET_MCCONF" message specification in [commands.c](https://github.com/vedderb/bldc/blob/6.00/comm/commands.c)
    in VESC bldc-6.00 source code on Github.
    """

    ID = 14

    # The following enums are defined based on the VESC BLDC firmware source code, specifically in the "datatypes.h" file.
    class mc_pwm_mode(IntEnum):
        PWM_MODE_NONSYNCHRONOUS_HISW = 0
        PWM_MODE_SYNCHRONOUS = 1
        PWM_MODE_BIPOLAR = 2

    class mc_comm_mode(IntEnum):
        COMM_MODE_INTEGRATE = 0
        COMM_MODE_DELAY = 1

    class mc_sensor_mode(IntEnum):
        SENSOR_MODE_SENSORLESS = 0
        SENSOR_MODE_SENSORED = 1
        SENSOR_MODE_HYBRID = 2

    class mc_motor_type(IntEnum):
        MOTOR_TYPE_BLDC = 0
        MOTOR_TYPE_DC = 1
        MOTOR_TYPE_FOC = 2
        MOTOR_TYPE_GPD = 3

    class mc_foc_sensor_mode(IntEnum):
        FOC_SENSOR_MODE_SENSORLESS = 0
        FOC_SENSOR_MODE_ENCODER = 1
        FOC_SENSOR_MODE_HALL = 2
        FOC_SENSOR_MODE_HFI = 3
        FOC_SENSOR_MODE_HFI_START = 4
        FOC_SENSOR_MODE_HFI_V2 = 5
        FOC_SENSOR_MODE_HFI_V3 = 6
        FOC_SENSOR_MODE_HFI_V4 = 7
        FOC_SENSOR_MODE_HFI_V5 = 8

    class mc_foc_cc_decoupling_mode(IntEnum):
        FOC_CC_DECOUPLING_DISABLED = 0
        FOC_CC_DECOUPLING_CROSS = 1
        FOC_CC_DECOUPLING_BEMF = 2
        FOC_CC_DECOUPLING_CROSS_BEMF = 3

    class mc_foc_observer_type(IntEnum):
        FOC_OBSERVER_ORTEGA_ORIGINAL = 0
        FOC_OBSERVER_MXLEMMING = 1
        FOC_OBSERVER_ORTEGA_LAMBDA_COMP = 2
        FOC_OBSERVER_MXLEMMING_LAMBDA_COMP = 3

    class foc_hfi_samples(IntEnum):
        HFI_SAMPLES_8 = 0
        HFI_SAMPLES_16 = 1
        HFI_SAMPLES_32 = 2

    class SAT_COMP_MODE(IntEnum):
        SAT_COMP_DISABLED = 0
        SAT_COMP_FACTOR = 1
        SAT_COMP_LAMBDA = 2
        SAT_COMP_LAMBDA_AND_FACTOR = 3

    class MTPA_MODE(IntEnum):
        MTPA_MODE_OFF = 0
        MTPA_MODE_IQ_TARGET = 1
        MTPA_MODE_IQ_MEASURED = 2

    class SPEED_SRC(IntEnum):
        SPEED_SRC_CORRECTED = 0
        SPEED_SRC_OBSERVER = 1

    class sensor_port_mode(IntEnum):
        SENSOR_PORT_MODE_HALL = 0
        SENSOR_PORT_MODE_ABI = 1
        SENSOR_PORT_MODE_AS5047_SPI = 2
        SENSOR_PORT_MODE_AD2S1205 = 3
        SENSOR_PORT_MODE_SINCOS = 4
        SENSOR_PORT_MODE_TS5700N8501 = 5
        SENSOR_PORT_MODE_TS5700N8501_MULTITURN = 6
        SENSOR_PORT_MODE_MT6816_SPI_HW = 7
        SENSOR_PORT_MODE_AS5x47U_SPI = 8
        SENSOR_PORT_MODE_BISSC = 9
        SENSOR_PORT_MODE_TLE5012_SSC_SW = 10
        SENSOR_PORT_MODE_TLE5012_SSC_HW = 11
        SENSOR_PORT_MODE_CUSTOM_ENCODER = 12

    class drv8301_oc_mode(IntEnum):
        DRV8301_OC_LIMIT = 0
        DRV8301_OC_LATCH_SHUTDOWN = 1
        DRV8301_OC_REPORT_ONLY = 2
        DRV8301_OC_DISABLED = 3

    class out_aux_mode(IntEnum):
        OUT_AUX_MODE_OFF = 0
        OUT_AUX_MODE_ON_AFTER_2S = 1
        OUT_AUX_MODE_ON_AFTER_5S = 2
        OUT_AUX_MODE_ON_AFTER_10S = 3
        OUT_AUX_MODE_UNUSED = 4
        OUT_AUX_MODE_ON_WHEN_RUNNING = 5
        OUT_AUX_MODE_ON_WHEN_NOT_RUNNING = 6
        OUT_AUX_MODE_MOTOR_50 = 7
        OUT_AUX_MODE_MOSFET_50 = 8
        OUT_AUX_MODE_MOTOR_70 = 9
        OUT_AUX_MODE_MOSFET_70 = 10
        OUT_AUX_MODE_MOTOR_MOSFET_50 = 11
        OUT_AUX_MODE_MOTOR_MOSFET_70 = 12

    class temp_sensor_type(IntEnum):
        TEMP_SENSOR_NTC_10K_25C = 0
        TEMP_SENSOR_PTC_1K_100C = 1
        TEMP_SENSOR_KTY83_122 = 2
        TEMP_SENSOR_NTC_100K_25C = 3
        TEMP_SENSOR_KTY84_130 = 4
        TEMP_SENSOR_NTCX = 5
        TEMP_SENSOR_PTCX = 6
        TEMP_SENSOR_PT1000 = 7
        TEMP_SENSOR_DISABLED = 8

    class BATTERY_TYPE(IntEnum):
        BATTERY_TYPE_LIION_3_0__4_2 = 0
        BATTERY_TYPE_LIIRON_2_6__3_6 = 1
        BATTERY_TYPE_LEAD_ACID = 2

    class PID_RATE(IntEnum):
        PID_RATE_25_HZ = 0
        PID_RATE_50_HZ = 1
        PID_RATE_100_HZ = 2
        PID_RATE_250_HZ = 3
        PID_RATE_500_HZ = 4
        PID_RATE_1000_HZ = 5
        PID_RATE_2500_HZ = 6
        PID_RATE_5000_HZ = 7
        PID_RATE_10000_HZ = 8

    class BMS_TYPE(IntEnum):
        BMS_TYPE_NONE = 0
        BMS_TYPE_VESC = 1

    class BMS_FWD_CAN_MODE(IntEnum):
        BMS_FWD_CAN_MODE_DISABLED = 0
        BMS_FWD_CAN_MODE_USB_ONLY = 1
        BMS_FWD_CAN_MODE_ANY = 2

    class bms_config:
        def __init__(self):
            self.type: MotorControllerConfigurationMessage.BMS_TYPE = (
                MotorControllerConfigurationMessage.BMS_TYPE.BMS_TYPE_NONE
            )
            self.limit_mode: int = 0  # uint8_t
            self.t_limit_start: float = 0.0
            self.t_limit_end: float = 0.0
            self.soc_limit_start: float = 0.0
            self.soc_limit_end: float = 0.0
            self.fwd_can_mode: MotorControllerConfigurationMessage.BMS_FWD_CAN_MODE = (
                MotorControllerConfigurationMessage.BMS_FWD_CAN_MODE.BMS_FWD_CAN_MODE_DISABLED
            )

    def __init__(self) -> None:
        self.l_current_max: float = 0.0
        self.l_current_min: float = 0.0
        self.l_in_current_max: float = 0.0
        self.l_in_current_min: float = 0.0
        self.l_abs_current_max: float = 0.0
        self.l_min_erpm: float = 0.0
        self.l_max_erpm: float = 0.0
        self.l_erpm_start: float = 0.0
        self.l_max_erpm_fbrake: float = 0.0
        self.l_max_erpm_fbrake_cc: float = 0.0
        self.l_min_vin: float = 0.0
        self.l_max_vin: float = 0.0
        self.l_battery_cut_start: float = 0.0
        self.l_battery_cut_end: float = 0.0
        self.l_slow_abs_current: bool = False
        self.l_temp_fet_start: float = 0.0
        self.l_temp_fet_end: float = 0.0
        self.l_temp_motor_start: float = 0.0
        self.l_temp_motor_end: float = 0.0
        self.l_temp_accel_dec: float = 0.0
        self.l_min_duty: float = 0.0
        self.l_max_duty: float = 0.0
        self.l_watt_max: float = 0.0
        self.l_watt_min: float = 0.0
        self.l_current_max_scale: float = 0.0
        self.l_current_min_scale: float = 0.0
        self.l_duty_start: float = 0.0

        # Overridden limits (runtime)
        self.lo_current_max: float = 0.0
        self.lo_current_min: float = 0.0
        self.lo_in_current_max: float = 0.0
        self.lo_in_current_min: float = 0.0
        self.lo_current_motor_max_now: float = 0.0
        self.lo_current_motor_min_now: float = 0.0

        # BLDC switching and drive
        self.pwm_mode: MotorControllerConfigurationMessage.mc_pwm_mode = (
            MotorControllerConfigurationMessage.mc_pwm_mode.PWM_MODE_SYNCHRONOUS
        )
        self.comm_mode: MotorControllerConfigurationMessage.mc_comm_mode = (
            MotorControllerConfigurationMessage.mc_comm_mode.COMM_MODE_INTEGRATE
        )
        self.motor_type: MotorControllerConfigurationMessage.mc_motor_type = (
            MotorControllerConfigurationMessage.mc_motor_type.MOTOR_TYPE_FOC
        )
        self.sensor_mode: MotorControllerConfigurationMessage.mc_sensor_mode = (
            MotorControllerConfigurationMessage.mc_sensor_mode.SENSOR_MODE_SENSORLESS
        )

        # Sensorless (BLDC)
        self.sl_min_erpm: float = 0.0
        self.sl_min_erpm_cycle_int_limit: float = 0.0
        self.sl_max_fullbreak_current_dir_change: float = 0.0
        self.sl_cycle_int_limit: float = 0.0
        self.sl_phase_advance_at_br: float = 0.0
        self.sl_cycle_int_rpm_br: float = 0.0
        self.sl_bemf_coupling_k: float = 0.0

        # Hall sensor
        self.hall_table: list = [0] * 8  # int8_t[8]
        self.hall_sl_erpm: float = 0.0

        # FOC
        self.foc_current_kp: float = 0.0
        self.foc_current_ki: float = 0.0
        self.foc_f_zv: float = 0.0
        self.foc_dt_us: float = 0.0
        self.foc_encoder_offset: float = 0.0
        self.foc_encoder_inverted: bool = False
        self.foc_encoder_ratio: float = 0.0
        self.foc_motor_l: float = 0.0
        self.foc_motor_ld_lq_diff: float = 0.0
        self.foc_motor_r: float = 0.0
        self.foc_motor_flux_linkage: float = 0.0
        self.foc_observer_gain: float = 0.0
        self.foc_observer_gain_slow: float = 0.0
        self.foc_observer_offset: float = 0.0
        self.foc_pll_kp: float = 0.0
        self.foc_pll_ki: float = 0.0
        self.foc_duty_dowmramp_kp: float = 0.0
        self.foc_duty_dowmramp_ki: float = 0.0
        self.foc_start_curr_dec: float = 0.0
        self.foc_start_curr_dec_rpm: float = 0.0
        self.foc_openloop_rpm: float = 0.0
        self.foc_openloop_rpm_low: float = 0.0
        self.foc_d_gain_scale_start: float = 0.0
        self.foc_d_gain_scale_max_mod: float = 0.0
        self.foc_sl_openloop_hyst: float = 0.0
        self.foc_sl_openloop_time: float = 0.0
        self.foc_sl_openloop_time_lock: float = 0.0
        self.foc_sl_openloop_time_ramp: float = 0.0
        self.foc_sl_openloop_boost_q: float = 0.0
        self.foc_sl_openloop_max_q: float = 0.0
        self.foc_sensor_mode: MotorControllerConfigurationMessage.mc_foc_sensor_mode = (
            MotorControllerConfigurationMessage.mc_foc_sensor_mode.FOC_SENSOR_MODE_SENSORLESS
        )
        self.foc_hall_table: list = [0] * 8  # uint8_t[8]
        self.foc_hall_interp_erpm: float = 0.0
        self.foc_sl_erpm: float = 0.0
        self.foc_sample_v0_v7: bool = False
        self.foc_sample_high_current: bool = False
        self.foc_sat_comp_mode: MotorControllerConfigurationMessage.SAT_COMP_MODE = (
            MotorControllerConfigurationMessage.SAT_COMP_MODE.SAT_COMP_DISABLED
        )
        self.foc_sat_comp: float = 0.0
        self.foc_temp_comp: bool = False
        self.foc_temp_comp_base_temp: float = 0.0
        self.foc_current_filter_const: float = 0.0
        self.foc_cc_decoupling: MotorControllerConfigurationMessage.mc_foc_cc_decoupling_mode = (
            MotorControllerConfigurationMessage.mc_foc_cc_decoupling_mode.FOC_CC_DECOUPLING_DISABLED
        )
        self.foc_observer_type: MotorControllerConfigurationMessage.mc_foc_observer_type = (
            MotorControllerConfigurationMessage.mc_foc_observer_type.FOC_OBSERVER_ORTEGA_ORIGINAL
        )
        self.foc_hfi_voltage_start: float = 0.0
        self.foc_hfi_voltage_run: float = 0.0
        self.foc_hfi_voltage_max: float = 0.0
        self.foc_hfi_gain: float = 0.0
        self.foc_hfi_hyst: float = 0.0
        self.foc_sl_erpm_hfi: float = 0.0
        self.foc_hfi_start_samples: int = 0  # uint16_t
        self.foc_hfi_obs_ovr_sec: float = 0.0
        self.foc_hfi_sample: MotorControllerConfigurationMessage.foc_hfi_samples = (
            MotorControllerConfigurationMessage.foc_hfi_samples.HFI_SAMPLES_8
        )
        self.foc_offsets_cal_on_boot: bool = False
        self.foc_offsets_current: list = [0.0] * 3
        self.foc_offsets_voltage: list = [0.0] * 3
        self.foc_offsets_voltage_undriven: list = [0.0] * 3
        self.foc_phase_filter_enable: bool = False
        self.foc_phase_filter_disable_fault: bool = False
        self.foc_phase_filter_max_erpm: float = 0.0
        self.foc_mtpa_mode: MotorControllerConfigurationMessage.MTPA_MODE = (
            MotorControllerConfigurationMessage.MTPA_MODE.MTPA_MODE_OFF
        )

        # Field Weakening
        self.foc_fw_current_max: float = 0.0
        self.foc_fw_duty_start: float = 0.0
        self.foc_fw_ramp_time: float = 0.0
        self.foc_fw_q_current_factor: float = 0.0
        self.foc_speed_soure: MotorControllerConfigurationMessage.SPEED_SRC = (
            MotorControllerConfigurationMessage.SPEED_SRC.SPEED_SRC_CORRECTED
        )  # typo preserved from C

        # GPDrive
        self.gpd_buffer_notify_left: int = 0
        self.gpd_buffer_interpol: int = 0
        self.gpd_current_filter_const: float = 0.0
        self.gpd_current_kp: float = 0.0
        self.gpd_current_ki: float = 0.0

        self.sp_pid_loop_rate: MotorControllerConfigurationMessage.PID_RATE = (
            MotorControllerConfigurationMessage.PID_RATE.PID_RATE_1000_HZ
        )

        # Speed PID
        self.s_pid_kp: float = 0.0
        self.s_pid_ki: float = 0.0
        self.s_pid_kd: float = 0.0
        self.s_pid_kd_filter: float = 0.0
        self.s_pid_min_erpm: float = 0.0
        self.s_pid_allow_braking: bool = False
        self.s_pid_ramp_erpms_s: float = 0.0

        # Position PID
        self.p_pid_kp: float = 0.0
        self.p_pid_ki: float = 0.0
        self.p_pid_kd: float = 0.0
        self.p_pid_kd_proc: float = 0.0
        self.p_pid_kd_filter: float = 0.0
        self.p_pid_ang_div: float = 0.0
        self.p_pid_gain_dec_angle: float = 0.0
        self.p_pid_offset: float = 0.0

        # Current controller
        self.cc_startup_boost_duty: float = 0.0
        self.cc_min_current: float = 0.0
        self.cc_gain: float = 0.0
        self.cc_ramp_step_max: float = 0.0

        # Misc
        self.m_fault_stop_time_ms: int = 0  # int32_t
        self.m_duty_ramp_step: float = 0.0
        self.m_current_backoff_gain: float = 0.0
        self.m_encoder_counts: int = 0  # uint32_t
        self.m_encoder_sin_offset: float = 0.0
        self.m_encoder_sin_amp: float = 0.0
        self.m_encoder_cos_offset: float = 0.0
        self.m_encoder_cos_amp: float = 0.0
        self.m_encoder_sincos_filter_constant: float = 0.0
        self.m_encoder_sincos_phase_correction: float = 0.0
        self.m_sensor_port_mode: MotorControllerConfigurationMessage.sensor_port_mode = (
            MotorControllerConfigurationMessage.sensor_port_mode.SENSOR_PORT_MODE_HALL
        )
        self.m_invert_direction: bool = False
        self.m_drv8301_oc_mode: MotorControllerConfigurationMessage.drv8301_oc_mode = (
            MotorControllerConfigurationMessage.drv8301_oc_mode.DRV8301_OC_LIMIT
        )
        self.m_drv8301_oc_adj: int = 0
        self.m_bldc_f_sw_min: float = 0.0
        self.m_bldc_f_sw_max: float = 0.0
        self.m_dc_f_sw: float = 0.0
        self.m_ntc_motor_beta: float = 0.0
        self.m_out_aux_mode: MotorControllerConfigurationMessage.out_aux_mode = (
            MotorControllerConfigurationMessage.out_aux_mode.OUT_AUX_MODE_OFF
        )
        self.m_motor_temp_sens_type: MotorControllerConfigurationMessage.temp_sensor_type = (
            MotorControllerConfigurationMessage.temp_sensor_type.TEMP_SENSOR_NTC_10K_25C
        )
        self.m_ptc_motor_coeff: float = 0.0
        self.m_hall_extra_samples: int = 0
        self.m_batt_filter_const: int = 0
        self.m_ntcx_ptcx_temp_base: float = 0.0
        self.m_ntcx_ptcx_res: float = 0.0

        # Setup info
        self.si_motor_poles: int = 0  # uint8_t
        self.si_gear_ratio: float = 0.0
        self.si_wheel_diameter: float = 0.0
        self.si_battery_type: MotorControllerConfigurationMessage.BATTERY_TYPE = (
            MotorControllerConfigurationMessage.BATTERY_TYPE.BATTERY_TYPE_LIION_3_0__4_2
        )
        self.si_battery_cells: int = 0
        self.si_battery_ah: float = 0.0
        self.si_motor_nl_current: float = 0.0

        # BMS Configuration
        self.bms: MotorControllerConfigurationMessage.bms_config = MotorControllerConfigurationMessage.bms_config()

    @property
    def buffer(self) -> bytes:
        MCCONF_SIGNATURE = 776184161  # 0x2E4B7631, defined in VESC BLDC firmware source code
        data = b""
        data += struct.pack(">I", MCCONF_SIGNATURE)
        data += struct.pack(">BBBB", self.pwm_mode, self.comm_mode, self.motor_type, self.sensor_mode)
        data += struct.pack(
            ">fffffff",
            self.l_current_max,
            self.l_current_min,
            self.l_in_current_max,
            self.l_in_current_min,
            self.l_abs_current_max,
            self.l_min_erpm,
            self.l_max_erpm,
        )
        data += struct.pack(">h", int(self.l_erpm_start * 10000))
        data += struct.pack(">ff", self.l_max_erpm_fbrake, self.l_max_erpm_fbrake_cc)
        data += struct.pack(">ff", self.l_min_vin, self.l_max_vin)
        data += struct.pack(">ff", self.l_battery_cut_start, self.l_battery_cut_end)
        data += struct.pack(">B", int(self.l_slow_abs_current))
        data += struct.pack(
            ">hhhh",
            int(self.l_temp_fet_start * 10),
            int(self.l_temp_fet_end * 10),
            int(self.l_temp_motor_start * 10),
            int(self.l_temp_motor_end * 10),
        )
        data += struct.pack(
            ">hhh", int(self.l_temp_accel_dec * 10000), int(self.l_min_duty * 10000), int(self.l_max_duty * 10000)
        )
        data += struct.pack(">ff", self.l_watt_max, self.l_watt_min)
        data += struct.pack(
            ">hhh",
            int(self.l_current_max_scale * 10000),
            int(self.l_current_min_scale * 10000),
            int(self.l_duty_start * 10000),
        )
        data += struct.pack(
            ">fff", self.sl_min_erpm, self.sl_min_erpm_cycle_int_limit, self.sl_max_fullbreak_current_dir_change
        )
        data += struct.pack(">hh", int(self.sl_cycle_int_limit * 10), int(self.sl_phase_advance_at_br * 10000))
        data += struct.pack(">ff", self.sl_cycle_int_rpm_br, self.sl_bemf_coupling_k)
        data += struct.pack(">8B", *[self.hall_table[i] & 0xFF for i in range(8)])
        data += struct.pack(">f", self.hall_sl_erpm)
        data += struct.pack(">ffff", self.foc_current_kp, self.foc_current_ki, self.foc_f_zv, self.foc_dt_us)
        data += struct.pack(">B", int(self.foc_encoder_inverted))
        data += struct.pack(">ff", self.foc_encoder_offset, self.foc_encoder_ratio)
        data += struct.pack(">B", self.foc_sensor_mode)
        data += struct.pack(
            ">fffff", self.foc_pll_kp, self.foc_pll_ki, self.foc_motor_l, self.foc_motor_ld_lq_diff, self.foc_motor_r
        )
        data += struct.pack(">fff", self.foc_motor_flux_linkage, self.foc_observer_gain, self.foc_observer_gain_slow)
        data += struct.pack(">h", int(self.foc_observer_offset * 1000))
        data += struct.pack(">ff", self.foc_duty_dowmramp_kp, self.foc_duty_dowmramp_ki)
        data += struct.pack(">h", int(self.foc_start_curr_dec * 10000))
        data += struct.pack(">ff", self.foc_start_curr_dec_rpm, self.foc_openloop_rpm)
        data += struct.pack(
            ">hhhh",
            int(self.foc_openloop_rpm_low * 1000),
            int(self.foc_d_gain_scale_start * 1000),
            int(self.foc_d_gain_scale_max_mod * 1000),
            int(self.foc_sl_openloop_hyst * 100),
        )
        data += struct.pack(
            ">hhhhh",
            int(self.foc_sl_openloop_time_lock * 100),
            int(self.foc_sl_openloop_time_ramp * 100),
            int(self.foc_sl_openloop_time * 100),
            int(self.foc_sl_openloop_boost_q * 100),
            int(self.foc_sl_openloop_max_q * 100),
        )
        data += struct.pack(">8B", *[self.foc_hall_table[i] & 0xFF for i in range(8)])
        data += struct.pack(">ff", self.foc_hall_interp_erpm, self.foc_sl_erpm)
        data += struct.pack(
            ">BBB", int(self.foc_sample_v0_v7), int(self.foc_sample_high_current), self.foc_sat_comp_mode
        )
        data += struct.pack(">h", int(self.foc_sat_comp * 1000))
        data += struct.pack(">B", int(self.foc_temp_comp))
        data += struct.pack(">hh", int(self.foc_temp_comp_base_temp * 100), int(self.foc_current_filter_const * 10000))
        data += struct.pack(">BB", self.foc_cc_decoupling, self.foc_observer_type)
        data += struct.pack(
            ">hhhhh",
            int(self.foc_hfi_voltage_start * 10),
            int(self.foc_hfi_voltage_run * 10),
            int(self.foc_hfi_voltage_max * 10),
            int(self.foc_hfi_gain * 1000),
            int(self.foc_hfi_hyst * 100),
        )
        data += struct.pack(">f", self.foc_sl_erpm_hfi)
        data += struct.pack(">H", self.foc_hfi_start_samples)
        data += struct.pack(">f", self.foc_hfi_obs_ovr_sec)
        data += struct.pack(">BB", self.foc_hfi_sample, int(self.foc_offsets_cal_on_boot))
        data += struct.pack(
            ">fff", self.foc_offsets_current[0], self.foc_offsets_current[1], self.foc_offsets_current[2]
        )
        data += struct.pack(
            ">hhhhhh",
            int(self.foc_offsets_voltage[0] * 10000),
            int(self.foc_offsets_voltage[1] * 10000),
            int(self.foc_offsets_voltage[2] * 10000),
            int(self.foc_offsets_voltage_undriven[0] * 10000),
            int(self.foc_offsets_voltage_undriven[1] * 10000),
            int(self.foc_offsets_voltage_undriven[2] * 10000),
        )
        data += struct.pack(">BB", int(self.foc_phase_filter_enable), int(self.foc_phase_filter_disable_fault))
        data += struct.pack(">f", self.foc_phase_filter_max_erpm)
        data += struct.pack(">B", self.foc_mtpa_mode)
        data += struct.pack(">f", self.foc_fw_current_max)
        data += struct.pack(
            ">hhh",
            int(self.foc_fw_duty_start * 10000),
            int(self.foc_fw_ramp_time * 1000),
            int(self.foc_fw_q_current_factor * 10000),
        )
        data += struct.pack(">B", self.foc_speed_soure)
        data += struct.pack(">hh", self.gpd_buffer_notify_left, self.gpd_buffer_interpol)
        data += struct.pack(">h", int(self.gpd_current_filter_const * 10000))
        data += struct.pack(">ff", self.gpd_current_kp, self.gpd_current_ki)
        data += struct.pack(">B", self.sp_pid_loop_rate)
        data += struct.pack(">fff", self.s_pid_kp, self.s_pid_ki, self.s_pid_kd)
        data += struct.pack(">h", int(self.s_pid_kd_filter * 10000))
        data += struct.pack(">f", self.s_pid_min_erpm)
        data += struct.pack(">B", int(self.s_pid_allow_braking))
        data += struct.pack(">f", self.s_pid_ramp_erpms_s)
        data += struct.pack(">ffff", self.p_pid_kp, self.p_pid_ki, self.p_pid_kd, self.p_pid_kd_proc)
        data += struct.pack(">h", int(self.p_pid_kd_filter * 10000))
        data += struct.pack(">f", self.p_pid_ang_div)
        data += struct.pack(">h", int(self.p_pid_gain_dec_angle * 10))
        data += struct.pack(">f", self.p_pid_offset)
        data += struct.pack(">h", int(self.cc_startup_boost_duty * 10000))
        data += struct.pack(">ff", self.cc_min_current, self.cc_gain)
        data += struct.pack(">h", int(self.cc_ramp_step_max * 10000))
        data += struct.pack(">i", self.m_fault_stop_time_ms)
        data += struct.pack(">h", int(self.m_duty_ramp_step * 10000))
        data += struct.pack(">f", self.m_current_backoff_gain)
        data += struct.pack(">I", self.m_encoder_counts)
        data += struct.pack(
            ">hhhhhh",
            int(self.m_encoder_sin_amp * 1000),
            int(self.m_encoder_cos_amp * 1000),
            int(self.m_encoder_sin_offset * 1000),
            int(self.m_encoder_cos_offset * 1000),
            int(self.m_encoder_sincos_filter_constant * 1000),
            int(self.m_encoder_sincos_phase_correction * 1000),
        )
        data += struct.pack(
            ">BBBB",
            self.m_sensor_port_mode,
            int(self.m_invert_direction),
            self.m_drv8301_oc_mode,
            self.m_drv8301_oc_adj & 0xFF,
        )
        data += struct.pack(">ffff", self.m_bldc_f_sw_min, self.m_bldc_f_sw_max, self.m_dc_f_sw, self.m_ntc_motor_beta)
        data += struct.pack(">BB", self.m_out_aux_mode, self.m_motor_temp_sens_type)
        data += struct.pack(">f", self.m_ptc_motor_coeff)
        data += struct.pack(">hh", int(self.m_ntcx_ptcx_res * 0.1), int(self.m_ntcx_ptcx_temp_base * 10))
        data += struct.pack(
            ">BBB", self.m_hall_extra_samples & 0xFF, self.m_batt_filter_const & 0xFF, self.si_motor_poles & 0xFF
        )
        data += struct.pack(">ff", self.si_gear_ratio, self.si_wheel_diameter)
        data += struct.pack(">BB", self.si_battery_type, self.si_battery_cells & 0xFF)
        data += struct.pack(">ff", self.si_battery_ah, self.si_motor_nl_current)
        data += struct.pack(">BB", self.bms.type, self.bms.limit_mode)
        data += struct.pack(
            ">hhhh",
            int(self.bms.t_limit_start * 100),
            int(self.bms.t_limit_end * 100),
            int(self.bms.soc_limit_start * 1000),
            int(self.bms.soc_limit_end * 1000),
        )
        data += struct.pack(">B", self.bms.fwd_can_mode)
        return MotorControllerConfigurationMessage.ID.to_bytes(1) + data


class BionicBoarderMessage:
    """
    See the "COMM_BIONIC_BOARDER_DATA" message specification in [commands.c](https://github.com/vedderb/bldc/blob/6.00/comm/commands.c)
    in VESC bldc-6.00 source code on Github.

    Note: This message is not defined yet in this version of theVESC BLDC firmware; this class is created for simulation purposes, for now.
    """

    ID = 152

    def __init__(self) -> None:
        # VESC State Data
        self.__temp_fet: float = 0.0
        self.__temp_motor: float = 0.0
        self.__avg_motor_current: float = 0.0
        self.__avg_input_current: float = 0.0
        self.__avg_id: float = 0.0
        self.__avg_iq: float = 0.0
        self.__duty_cycle: float = 0.0
        self.__rpm: float = 0.0
        self.__input_voltage: float = 0.0
        self.__amp_hours: float = 0.0
        self.__amp_hours_charged: float = 0.0
        self.__watt_hours: float = 0.0
        self.__watt_hours_charged: float = 0.0
        self.__tachometer: int = 0
        self.__tachometer_abs: int = 0
        self.__fault: int = 0
        self.__pid_pos: float = 0.0
        self.__avg_vd: float = 0.0
        self.__avg_vq: float = 0.0

        # IMU Data
        self.__rpy = [0.0, 0.0, 0.0]  # Roll, pitch, yaw in radians
        self.__acc = [0.0, 0.0, 0.0]  # Accelerometer data (x, y, z) in m/s^2
        self.__gyro = [0.0, 0.0, 0.0]  # Gyroscope data (x, y, z) in rad/s
        self.__mag = [0.0, 0.0, 0.0]  # Magnetometer data (x, y, z) in micro teslas
        self.__q = [0.0, 0.0, 0.0, 0.0]  # Quaternion data (w, x, y, z)

    @property
    def buffer(self) -> bytes:
        """
        Serializes the Bionic Boarder message into a bytes object.

        This serialization combines the VESC state data and IMU data into a single byte array according to the specified format in
        the VESC custom firmware for the Bionic Boarder application. The VESC state data is encoded with specific scaling factors,
        and the IMU data is encoded as IEEE 754 floats.

        Returns:
            bytes: A bytes object containing the serialized Bionic Boarder message data.
        """
        buffer = bytearray(129)

        # float16: temp_fet (scale 1e1) -> 2 bytes
        buffer[0:2] = struct.pack(">h", int(self.__temp_fet * 1e1))
        # float16: temp_motor (scale 1e1) -> 2 bytes
        buffer[2:4] = struct.pack(">h", int(self.__temp_motor * 1e1))
        # float32: avg_motor_current (scale 1e2) -> 4 bytes
        buffer[4:8] = struct.pack(">i", int(self.__avg_motor_current * 1e2))
        # float32: avg_input_current (scale 1e2) -> 4 bytes
        buffer[8:12] = struct.pack(">i", int(self.__avg_input_current * 1e2))
        # float32: avg_id (scale 1e2) -> 4 bytes
        buffer[12:16] = struct.pack(">i", int(self.__avg_id * 1e2))
        # float32: avg_iq (scale 1e2) -> 4 bytes
        buffer[16:20] = struct.pack(">i", int(self.__avg_iq * 1e2))
        # float16: duty_cycle (scale 1e3) -> 2 bytes
        buffer[20:22] = struct.pack(">h", int(self.__duty_cycle * 1e3))
        # float32: rpm (scale 1e0) -> 4 bytes
        buffer[22:26] = struct.pack(">i", int(self.__rpm * 1e0))
        # float16: input_voltage (scale 1e1) -> 2 bytes
        buffer[26:28] = struct.pack(">h", int(self.__input_voltage * 1e1))
        # float32: amp_hours (scale 1e4) -> 4 bytes
        buffer[28:32] = struct.pack(">i", int(self.__amp_hours * 1e4))
        # float32: amp_hours_charged (scale 1e4) -> 4 bytes
        buffer[32:36] = struct.pack(">i", int(self.__amp_hours_charged * 1e4))
        # float32: watt_hours (scale 1e4) -> 4 bytes
        buffer[36:40] = struct.pack(">i", int(self.__watt_hours * 1e4))
        # float32: watt_hours_charged (scale 1e4) -> 4 bytes
        buffer[40:44] = struct.pack(">i", int(self.__watt_hours_charged * 1e4))
        # int32: tachometer -> 4 bytes
        buffer[44:48] = struct.pack(">i", self.__tachometer)
        # int32: tachometer_abs -> 4 bytes
        buffer[48:52] = struct.pack(">i", self.__tachometer_abs)
        # uint8: fault -> 1 byte
        buffer[52] = self.__fault & 0xFF
        # float32: pid_pos (scale 1e6) -> 4 bytes
        buffer[53:57] = struct.pack(">i", int(self.__pid_pos * 1e6))
        # float32: avg_vd (scale 1e3) -> 4 bytes
        buffer[57:61] = struct.pack(">i", int(self.__avg_vd * 1e3))
        # float32: avg_vq (scale 1e3) -> 4 bytes
        buffer[61:65] = struct.pack(">i", int(self.__avg_vq * 1e3))

        # IMU Data — buffer_append_float32_auto uses IEEE 754 float ('>f')
        # acc (x, y, z)
        buffer[65:69] = struct.pack(">f", self.__acc[0])
        buffer[69:73] = struct.pack(">f", self.__acc[1])
        buffer[73:77] = struct.pack(">f", self.__acc[2])
        # rpy (roll, pitch, yaw)
        buffer[77:81] = struct.pack(">f", self.__rpy[0])
        buffer[81:85] = struct.pack(">f", self.__rpy[1])
        buffer[85:89] = struct.pack(">f", self.__rpy[2])
        # gyro (x, y, z)
        buffer[89:93] = struct.pack(">f", self.__gyro[0])
        buffer[93:97] = struct.pack(">f", self.__gyro[1])
        buffer[97:101] = struct.pack(">f", self.__gyro[2])
        # mag (x, y, z)
        buffer[101:105] = struct.pack(">f", self.__mag[0])
        buffer[105:109] = struct.pack(">f", self.__mag[1])
        buffer[109:113] = struct.pack(">f", self.__mag[2])
        # quaternion (w, x, y, z)
        buffer[113:117] = struct.pack(">f", self.__q[0])
        buffer[117:121] = struct.pack(">f", self.__q[1])
        buffer[121:125] = struct.pack(">f", self.__q[2])
        buffer[125:129] = struct.pack(">f", self.__q[3])

        return BionicBoarderMessage.ID.to_bytes(1) + bytes(buffer)

    @property
    def temp_fet(self) -> float:
        return self.__temp_fet

    @temp_fet.setter
    def temp_fet(self, value: float) -> None:
        self.__temp_fet = value

    @property
    def temp_motor(self) -> float:
        return self.__temp_motor

    @temp_motor.setter
    def temp_motor(self, value: float) -> None:
        self.__temp_motor = value

    @property
    def avg_motor_current(self) -> float:
        return self.__avg_motor_current

    @avg_motor_current.setter
    def avg_motor_current(self, value: float) -> None:
        self.__avg_motor_current = value

    @property
    def avg_input_current(self) -> float:
        return self.__avg_input_current

    @avg_input_current.setter
    def avg_input_current(self, value: float) -> None:
        self.__avg_input_current = value

    @property
    def avg_id(self) -> float:
        return self.__avg_id

    @avg_id.setter
    def avg_id(self, value: float) -> None:
        self.__avg_id = value

    @property
    def avg_iq(self) -> float:
        return self.__avg_iq

    @avg_iq.setter
    def avg_iq(self, value: float) -> None:
        self.__avg_iq = value

    @property
    def duty_cycle(self) -> float:
        return self.__duty_cycle

    @duty_cycle.setter
    def duty_cycle(self, value: float) -> None:
        self.__duty_cycle = value

    @property
    def rpm(self) -> float:
        return self.__rpm

    @rpm.setter
    def rpm(self, value: float) -> None:
        self.__rpm = value

    @property
    def input_voltage(self) -> float:
        return self.__input_voltage

    @input_voltage.setter
    def input_voltage(self, value: float) -> None:
        self.__input_voltage = value

    @property
    def amp_hours(self) -> float:
        return self.__amp_hours

    @amp_hours.setter
    def amp_hours(self, value: float) -> None:
        self.__amp_hours = value

    @property
    def amp_hours_charged(self) -> float:
        return self.__amp_hours_charged

    @amp_hours_charged.setter
    def amp_hours_charged(self, value: float) -> None:
        self.__amp_hours_charged = value

    @property
    def watt_hours(self) -> float:
        return self.__watt_hours

    @watt_hours.setter
    def watt_hours(self, value: float) -> None:
        self.__watt_hours = value

    @property
    def watt_hours_charged(self) -> float:
        return self.__watt_hours_charged

    @watt_hours_charged.setter
    def watt_hours_charged(self, value: float) -> None:
        self.__watt_hours_charged = value

    @property
    def tachometer(self) -> int:
        return self.__tachometer

    @tachometer.setter
    def tachometer(self, value: int) -> None:
        self.__tachometer = value

    @property
    def tachometer_abs(self) -> int:
        return self.__tachometer_abs

    @tachometer_abs.setter
    def tachometer_abs(self, value: int) -> None:
        self.__tachometer_abs = value

    @property
    def fault(self) -> int:
        return self.__fault

    @fault.setter
    def fault(self, value: int) -> None:
        self.__fault = value

    @property
    def pid_pos(self) -> float:
        return self.__pid_pos

    @pid_pos.setter
    def pid_pos(self, value: float) -> None:
        self.__pid_pos = value

    @property
    def avg_vd(self) -> float:
        return self.__avg_vd

    @avg_vd.setter
    def avg_vd(self, value: float) -> None:
        self.__avg_vd = value

    @property
    def avg_vq(self) -> float:
        return self.__avg_vq

    @avg_vq.setter
    def avg_vq(self, value: float) -> None:
        self.__avg_vq = value

    @property
    def rpy(self) -> list:
        return self.__rpy

    @property
    def acc(self) -> list:
        return self.__acc

    @property
    def gyro(self) -> list:
        return self.__gyro

    @property
    def mag(self) -> list:
        return self.__mag

    @property
    def q(self) -> list:
        return self.__q


class AppConfigurationMessage:

    ID = 17

    class AppUse(IntEnum):
        APP_NONE = 0
        APP_PPM = 1
        APP_ADC = 2
        APP_UART = 3
        APP_PPM_UART = 4
        APP_ADC_UART = 5
        APP_NUNCHUK = 6
        APP_NRF = 7
        APP_CUSTOM = 8
        APP_BALANCE = 9
        APP_PAS = 10
        APP_ADC_PAS = 11

    class CanBaud(IntEnum):
        CAN_BAUD_125K = 0
        CAN_BAUD_250K = 1
        CAN_BAUD_500K = 2
        CAN_BAUD_1M = 3
        CAN_BAUD_10K = 4
        CAN_BAUD_20K = 5
        CAN_BAUD_50K = 6
        CAN_BAUD_75K = 7
        CAN_BAUD_100K = 8

    class CanMode(IntEnum):
        CAN_MODE_VESC = 0
        CAN_MODE_UAVCAN = 1
        CAN_MODE_COMM_BRIDGE = 2
        CAN_MODE_UNUSED = 3

    class ShutdownMode(IntEnum):
        SHUTDOWN_MODE_ALWAYS_OFF = 0
        SHUTDOWN_MODE_ALWAYS_ON = 1
        SHUTDOWN_MODE_TOGGLE_BUTTON = 2
        SHUTDOWN_MODE_OFF_AFTER_10S = 3
        SHUTDOWN_MODE_OFF_AFTER_1M = 4
        SHUTDOWN_MODE_OFF_AFTER_5M = 5
        SHUTDOWN_MODE_OFF_AFTER_10M = 6
        SHUTDOWN_MODE_OFF_AFTER_30M = 7
        SHUTDOWN_MODE_OFF_AFTER_1H = 8
        SHUTDOWN_MODE_OFF_AFTER_5H = 9

    class KillSwMode(IntEnum):
        KILL_SW_MODE_DISABLED = 0
        KILL_SW_MODE_PPM_LOW = 1
        KILL_SW_MODE_PPM_HIGH = 2
        KILL_SW_MODE_ADC2_LOW = 3
        KILL_SW_MODE_ADC2_HIGH = 4

    class UavcanRawMode(IntEnum):
        UAVCAN_RAW_MODE_CURRENT = 0
        UAVCAN_RAW_MODE_CURRENT_NO_REV_BRAKE = 1
        UAVCAN_RAW_MODE_DUTY = 2
        UAVCAN_RAW_MODE_RPM = 3

    class UavcanStatusCurrentMode(IntEnum):
        UAVCAN_STATUS_CURRENT_MODE_MOTOR = 0
        UAVCAN_STATUS_CURRENT_MODE_INPUT = 1

    class SafeStartMode(IntEnum):
        SAFE_START_DISABLED = 0
        SAFE_START_REGULAR = 1
        SAFE_START_NO_FAULT = 2

    class ThrExpMode(IntEnum):
        THR_EXP_EXPO = 0
        THR_EXP_NATURAL = 1
        THR_EXP_POLY = 2

    class PpmCtrlType(IntEnum):
        PPM_CTRL_TYPE_NONE = 0
        PPM_CTRL_TYPE_CURRENT = 1
        PPM_CTRL_TYPE_CURRENT_NOREV = 2
        PPM_CTRL_TYPE_CURRENT_NOREV_BRAKE = 3
        PPM_CTRL_TYPE_DUTY = 4
        PPM_CTRL_TYPE_DUTY_NOREV = 5
        PPM_CTRL_TYPE_PID = 6
        PPM_CTRL_TYPE_PID_NOREV = 7
        PPM_CTRL_TYPE_CURRENT_BRAKE_REV_HYST = 8
        PPM_CTRL_TYPE_CURRENT_SMART_REV = 9
        PPM_CTRL_TYPE_PID_POSITION_180 = 10
        PPM_CTRL_TYPE_PID_POSITION_360 = 11

    class AdcCtrlType(IntEnum):
        ADC_CTRL_TYPE_NONE = 0
        ADC_CTRL_TYPE_CURRENT = 1
        ADC_CTRL_TYPE_CURRENT_REV_CENTER = 2
        ADC_CTRL_TYPE_CURRENT_REV_BUTTON = 3
        ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC = 4
        ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER = 5
        ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER = 6
        ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON = 7
        ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC = 8
        ADC_CTRL_TYPE_DUTY = 9
        ADC_CTRL_TYPE_DUTY_REV_CENTER = 10
        ADC_CTRL_TYPE_DUTY_REV_BUTTON = 11
        ADC_CTRL_TYPE_PID = 12
        ADC_CTRL_TYPE_PID_REV_CENTER = 13
        ADC_CTRL_TYPE_PID_REV_BUTTON = 14

    class ChukCtrlType(IntEnum):
        CHUK_CTRL_TYPE_NONE = 0
        CHUK_CTRL_TYPE_CURRENT = 1
        CHUK_CTRL_TYPE_CURRENT_NOREV = 2
        CHUK_CTRL_TYPE_CURRENT_BIDIRECTIONAL = 3

    class PasCtrlType(IntEnum):
        PAS_CTRL_TYPE_NONE = 0
        PAS_CTRL_TYPE_CADENCE = 1
        PAS_CTRL_TYPE_TORQUE = 2
        PAS_CTRL_TYPE_TORQUE_WITH_CADENCE_TIMEOUT = 3

    class PasSensorType(IntEnum):
        PAS_SENSOR_TYPE_QUADRATURE = 0

    class NrfSpeed(IntEnum):
        NRF_SPEED_250K = 0
        NRF_SPEED_1M = 1
        NRF_SPEED_2M = 2

    class NrfPower(IntEnum):
        NRF_POWER_M18DBM = 0
        NRF_POWER_M12DBM = 1
        NRF_POWER_M6DBM = 2
        NRF_POWER_0DBM = 3
        NRF_POWER_OFF = 4

    class NrfAw(IntEnum):
        NRF_AW_3 = 0
        NRF_AW_4 = 1
        NRF_AW_5 = 2

    class NrfCrc(IntEnum):
        NRF_CRC_DISABLED = 0
        NRF_CRC_1B = 1
        NRF_CRC_2B = 2

    class NrfRetrDelay(IntEnum):
        NRF_RETR_DELAY_250US = 0
        NRF_RETR_DELAY_500US = 1
        NRF_RETR_DELAY_750US = 2
        NRF_RETR_DELAY_1000US = 3
        NRF_RETR_DELAY_1250US = 4
        NRF_RETR_DELAY_1500US = 5
        NRF_RETR_DELAY_1750US = 6
        NRF_RETR_DELAY_2000US = 7
        NRF_RETR_DELAY_2250US = 8
        NRF_RETR_DELAY_2500US = 9
        NRF_RETR_DELAY_2750US = 10
        NRF_RETR_DELAY_3000US = 11
        NRF_RETR_DELAY_3250US = 12
        NRF_RETR_DELAY_3500US = 13
        NRF_RETR_DELAY_3750US = 14
        NRF_RETR_DELAY_4000US = 15

    class BalancePidMode(IntEnum):
        BALANCE_PID_MODE_ANGLE = 0
        BALANCE_PID_MODE_ANGLE_RATE_CASCADE = 1

    class ImuType(IntEnum):
        IMU_TYPE_OFF = 0
        IMU_TYPE_INTERNAL = 1
        IMU_TYPE_EXTERNAL_MPU9X50 = 2
        IMU_TYPE_EXTERNAL_ICM20948 = 3
        IMU_TYPE_EXTERNAL_BMI160 = 4
        IMU_TYPE_EXTERNAL_LSM6DS3 = 5

    class AhrsMode(IntEnum):
        AHRS_MODE_MADGWICK = 0
        AHRS_MODE_MAHONY = 1
        AHRS_MODE_MADGWICK_FUSION = 2

    class ImuFilter(IntEnum):
        IMU_FILTER_LOW = 0
        IMU_FILTER_MEDIUM = 1
        IMU_FILTER_HIGH = 2

    class BmsType(IntEnum):
        BMS_TYPE_NONE = 0
        BMS_TYPE_VESC = 1

    class BmsFwdCanMode(IntEnum):
        BMS_FWD_CAN_MODE_DISABLED = 0
        BMS_FWD_CAN_MODE_USB_ONLY = 1
        BMS_FWD_CAN_MODE_ANY = 2

    class PpmConfig:
        def __init__(self):
            self.ctrl_type = AppConfigurationMessage.PpmCtrlType.PPM_CTRL_TYPE_NONE
            self.pid_max_erpm = 0.0
            self.hyst = 0.0
            self.pulse_start = 0.0
            self.pulse_end = 0.0
            self.pulse_center = 0.0
            self.median_filter = False
            self.safe_start = AppConfigurationMessage.SafeStartMode.SAFE_START_DISABLED
            self.throttle_exp = 0.0
            self.throttle_exp_brake = 0.0
            self.throttle_exp_mode = AppConfigurationMessage.ThrExpMode.THR_EXP_EXPO
            self.ramp_time_pos = 0.0
            self.ramp_time_neg = 0.0
            self.multi_esc = False
            self.tc = False
            self.tc_max_diff = 0.0
            self.max_erpm_for_dir = 0.0
            self.smart_rev_max_duty = 0.0
            self.smart_rev_ramp_time = 0.0

    class AdcConfig:
        def __init__(self):
            self.ctrl_type = AppConfigurationMessage.AdcCtrlType.ADC_CTRL_TYPE_NONE
            self.hyst = 0.0
            self.voltage_start = 0.0
            self.voltage_end = 0.0
            self.voltage_min = 0.0
            self.voltage_max = 0.0
            self.voltage_center = 0.0
            self.voltage2_start = 0.0
            self.voltage2_end = 0.0
            self.use_filter = False
            self.safe_start = AppConfigurationMessage.SafeStartMode.SAFE_START_DISABLED
            self.buttons = 0  # uint8_t
            self.voltage_inverted = False
            self.voltage2_inverted = False
            self.throttle_exp = 0.0
            self.throttle_exp_brake = 0.0
            self.throttle_exp_mode = AppConfigurationMessage.ThrExpMode.THR_EXP_EXPO
            self.ramp_time_pos = 0.0
            self.ramp_time_neg = 0.0
            self.multi_esc = False
            self.tc = False
            self.tc_max_diff = 0.0
            self.update_rate_hz = 0  # uint32_t

    class ChukConfig:
        def __init__(self):
            self.ctrl_type = AppConfigurationMessage.ChukCtrlType.CHUK_CTRL_TYPE_NONE
            self.hyst = 0.0
            self.ramp_time_pos = 0.0
            self.ramp_time_neg = 0.0
            self.stick_erpm_per_s_in_cc = 0.0
            self.throttle_exp = 0.0
            self.throttle_exp_brake = 0.0
            self.throttle_exp_mode = AppConfigurationMessage.ThrExpMode.THR_EXP_EXPO
            self.multi_esc = False
            self.tc = False
            self.tc_max_diff = 0.0
            self.use_smart_rev = False
            self.smart_rev_max_duty = 0.0
            self.smart_rev_ramp_time = 0.0

    class NrfConfig:
        def __init__(self):
            self.speed = AppConfigurationMessage.NrfSpeed.NRF_SPEED_250K
            self.power = AppConfigurationMessage.NrfPower.NRF_POWER_0DBM
            self.crc_type = AppConfigurationMessage.NrfCrc.NRF_CRC_1B
            self.retry_delay = AppConfigurationMessage.NrfRetrDelay.NRF_RETR_DELAY_250US
            self.retries = 0  # unsigned char
            self.channel = 0  # unsigned char
            self.address = [0, 0, 0]  # unsigned char[3]
            self.send_crc_ack = False

    class BalanceConfig:
        def __init__(self):
            self.pid_mode = AppConfigurationMessage.BalancePidMode.BALANCE_PID_MODE_ANGLE
            self.kp = 0.0
            self.ki = 0.0
            self.kd = 0.0
            self.kp2 = 0.0
            self.ki2 = 0.0
            self.kd2 = 0.0
            self.hertz = 0  # uint16_t
            self.loop_time_filter = 0  # uint16_t
            self.fault_pitch = 0.0
            self.fault_roll = 0.0
            self.fault_duty = 0.0
            self.fault_adc1 = 0.0
            self.fault_adc2 = 0.0
            self.fault_delay_pitch = 0  # uint16_t
            self.fault_delay_roll = 0  # uint16_t
            self.fault_delay_duty = 0  # uint16_t
            self.fault_delay_switch_half = 0  # uint16_t
            self.fault_delay_switch_full = 0  # uint16_t
            self.fault_adc_half_erpm = 0  # uint16_t
            self.fault_is_dual_switch = False
            self.tiltback_duty_angle = 0.0
            self.tiltback_duty_speed = 0.0
            self.tiltback_duty = 0.0
            self.tiltback_hv_angle = 0.0
            self.tiltback_hv_speed = 0.0
            self.tiltback_hv = 0.0
            self.tiltback_lv_angle = 0.0
            self.tiltback_lv_speed = 0.0
            self.tiltback_lv = 0.0
            self.tiltback_return_speed = 0.0
            self.tiltback_constant = 0.0
            self.tiltback_constant_erpm = 0  # uint16_t
            self.tiltback_variable = 0.0
            self.tiltback_variable_max = 0.0
            self.noseangling_speed = 0.0
            self.startup_pitch_tolerance = 0.0
            self.startup_roll_tolerance = 0.0
            self.startup_speed = 0.0
            self.deadzone = 0.0
            self.multi_esc = False
            self.yaw_kp = 0.0
            self.yaw_ki = 0.0
            self.yaw_kd = 0.0
            self.roll_steer_kp = 0.0
            self.roll_steer_erpm_kp = 0.0
            self.brake_current = 0.0
            self.brake_timeout = 0  # uint16_t
            self.yaw_current_clamp = 0.0
            self.ki_limit = 0.0
            self.kd_pt1_lowpass_frequency = 0  # uint16_t
            self.kd_pt1_highpass_frequency = 0  # uint16_t
            self.booster_angle = 0.0
            self.booster_ramp = 0.0
            self.booster_current = 0.0
            self.torquetilt_start_current = 0.0
            self.torquetilt_angle_limit = 0.0
            self.torquetilt_on_speed = 0.0
            self.torquetilt_off_speed = 0.0
            self.torquetilt_strength = 0.0
            self.torquetilt_filter = 0.0
            self.turntilt_strength = 0.0
            self.turntilt_angle_limit = 0.0
            self.turntilt_start_angle = 0.0
            self.turntilt_start_erpm = 0  # uint16_t
            self.turntilt_speed = 0.0
            self.turntilt_erpm_boost = 0  # uint16_t
            self.turntilt_erpm_boost_end = 0  # uint16_t

    class PasConfig:
        def __init__(self):
            self.ctrl_type = AppConfigurationMessage.PasCtrlType.PAS_CTRL_TYPE_NONE
            self.sensor_type = AppConfigurationMessage.PasSensorType.PAS_SENSOR_TYPE_QUADRATURE
            self.current_scaling = 0.0
            self.pedal_rpm_start = 0.0
            self.pedal_rpm_end = 0.0
            self.invert_pedal_direction = False
            self.magnets = 0  # uint8_t
            self.use_filter = False
            self.ramp_time_pos = 0.0
            self.ramp_time_neg = 0.0
            self.update_rate_hz = 0  # uint32_t

    class ImuConfig:
        def __init__(self):
            self.type = AppConfigurationMessage.ImuType.IMU_TYPE_OFF
            self.mode = AppConfigurationMessage.AhrsMode.AHRS_MODE_MADGWICK
            self.filter = AppConfigurationMessage.ImuFilter.IMU_FILTER_MEDIUM
            self.accel_lowpass_filter_x = 0.0
            self.accel_lowpass_filter_y = 0.0
            self.accel_lowpass_filter_z = 0.0
            self.gyro_lowpass_filter = 0.0
            self.sample_rate_hz = 0
            self.use_magnetometer = False
            self.accel_confidence_decay = 0.0
            self.mahony_kp = 0.0
            self.mahony_ki = 0.0
            self.madgwick_beta = 0.0
            self.rot_roll = 0.0
            self.rot_pitch = 0.0
            self.rot_yaw = 0.0
            self.accel_offsets = [0.0, 0.0, 0.0]  # float[3]
            self.gyro_offsets = [0.0, 0.0, 0.0]  # float[3]

    def __init__(self):
        # ── Settings ──────────────────────────────────────────────────────────
        self.controller_id = 0  # uint8_t
        self.timeout_msec = 0  # uint32_t
        self.timeout_brake_current = 0.0
        self.can_status_rate_1 = 0  # uint16_t
        self.can_status_msgs_r1 = 0  # uint8_t
        self.can_status_rate_2 = 0  # uint16_t
        self.can_status_msgs_r2 = 0  # uint8_t
        self.can_baud_rate = AppConfigurationMessage.CanBaud.CAN_BAUD_500K
        self.pairing_done = False
        self.permanent_uart_enabled = False
        self.shutdown_mode = AppConfigurationMessage.ShutdownMode.SHUTDOWN_MODE_ALWAYS_OFF
        self.servo_out_enable = False
        self.kill_sw_mode = AppConfigurationMessage.KillSwMode.KILL_SW_MODE_DISABLED

        # ── CAN modes ─────────────────────────────────────────────────────────
        self.can_mode = AppConfigurationMessage.CanMode.CAN_MODE_VESC
        self.uavcan_esc_index = 0  # uint8_t
        self.uavcan_raw_mode = AppConfigurationMessage.UavcanRawMode.UAVCAN_RAW_MODE_CURRENT
        self.uavcan_raw_rpm_max = 0.0
        self.uavcan_status_current_mode = (
            AppConfigurationMessage.UavcanStatusCurrentMode.UAVCAN_STATUS_CURRENT_MODE_MOTOR
        )

        # ── Application to use ────────────────────────────────────────────────
        self.app_to_use = AppConfigurationMessage.AppUse.APP_NONE

        # ── PPM application settings ──────────────────────────────────────────
        self.app_ppm_conf = AppConfigurationMessage.PpmConfig()

        # ── ADC application settings ──────────────────────────────────────────
        self.app_adc_conf = AppConfigurationMessage.AdcConfig()

        # ── UART application settings ─────────────────────────────────────────
        self.app_uart_baudrate = 0  # uint32_t

        # ── Nunchuk application settings ──────────────────────────────────────
        self.app_chuk_conf = AppConfigurationMessage.ChukConfig()

        # ── NRF application settings ──────────────────────────────────────────
        self.app_nrf_conf = AppConfigurationMessage.NrfConfig()

        # ── Balance application settings ──────────────────────────────────────
        self.app_balance_conf = AppConfigurationMessage.BalanceConfig()

        # ── Pedal Assist application settings ─────────────────────────────────
        self.app_pas_conf = AppConfigurationMessage.PasConfig()

        # ── IMU settings ──────────────────────────────────────────────────────
        self.imu_conf = AppConfigurationMessage.ImuConfig()

        # ── Flash corruption protection ───────────────────────────────────────
        self.crc = 0  # uint16_t

    @property
    def buffer(self) -> bytes:
        APPCONF_SIGNATURE = 486554156
        data = b""

        # ── Signature ─────────────────────────────────────────────────────────────
        data += struct.pack(">I", APPCONF_SIGNATURE)

        # ── Settings ──────────────────────────────────────────────────────────────
        data += struct.pack("B", self.controller_id)
        data += struct.pack(">I", self.timeout_msec)
        data += struct.pack(">f", self.timeout_brake_current)
        data += struct.pack(">H", self.can_status_rate_1)
        data += struct.pack(">H", self.can_status_rate_2)
        data += struct.pack("B", self.can_status_msgs_r1)
        data += struct.pack("B", self.can_status_msgs_r2)
        data += struct.pack("B", int(self.can_baud_rate))
        data += struct.pack("B", int(self.pairing_done))
        data += struct.pack("B", int(self.permanent_uart_enabled))
        data += struct.pack("B", int(self.shutdown_mode))

        # ── CAN modes ─────────────────────────────────────────────────────────────
        data += struct.pack("B", int(self.can_mode))
        data += struct.pack("B", self.uavcan_esc_index)
        data += struct.pack("B", int(self.uavcan_raw_mode))
        data += struct.pack(">f", self.uavcan_raw_rpm_max)
        data += struct.pack("B", int(self.uavcan_status_current_mode))
        data += struct.pack("B", int(self.servo_out_enable))
        data += struct.pack("B", int(self.kill_sw_mode))

        # ── Application to use ────────────────────────────────────────────────────
        data += struct.pack("B", int(self.app_to_use))

        # ── PPM ───────────────────────────────────────────────────────────────────
        data += struct.pack("B", int(self.app_ppm_conf.ctrl_type))
        data += struct.pack(">f", self.app_ppm_conf.pid_max_erpm)
        data += struct.pack(">f", self.app_ppm_conf.hyst)
        data += struct.pack(">f", self.app_ppm_conf.pulse_start)
        data += struct.pack(">f", self.app_ppm_conf.pulse_end)
        data += struct.pack(">f", self.app_ppm_conf.pulse_center)
        data += struct.pack("B", int(self.app_ppm_conf.median_filter))
        data += struct.pack("B", int(self.app_ppm_conf.safe_start))
        data += struct.pack(">f", self.app_ppm_conf.throttle_exp)
        data += struct.pack(">f", self.app_ppm_conf.throttle_exp_brake)
        data += struct.pack("B", int(self.app_ppm_conf.throttle_exp_mode))
        data += struct.pack(">f", self.app_ppm_conf.ramp_time_pos)
        data += struct.pack(">f", self.app_ppm_conf.ramp_time_neg)
        data += struct.pack("B", int(self.app_ppm_conf.multi_esc))
        data += struct.pack("B", int(self.app_ppm_conf.tc))
        data += struct.pack(">f", self.app_ppm_conf.tc_max_diff)
        data += struct.pack(">h", int(self.app_ppm_conf.max_erpm_for_dir * 1))
        data += struct.pack(">f", self.app_ppm_conf.smart_rev_max_duty)
        data += struct.pack(">f", self.app_ppm_conf.smart_rev_ramp_time)

        # ── ADC ───────────────────────────────────────────────────────────────────
        data += struct.pack("B", int(self.app_adc_conf.ctrl_type))
        data += struct.pack(">f", self.app_adc_conf.hyst)
        data += struct.pack(">h", int(self.app_adc_conf.voltage_start * 1000))
        data += struct.pack(">h", int(self.app_adc_conf.voltage_end * 1000))
        data += struct.pack(">h", int(self.app_adc_conf.voltage_min * 1000))
        data += struct.pack(">h", int(self.app_adc_conf.voltage_max * 1000))
        data += struct.pack(">h", int(self.app_adc_conf.voltage_center * 1000))
        data += struct.pack(">h", int(self.app_adc_conf.voltage2_start * 1000))
        data += struct.pack(">h", int(self.app_adc_conf.voltage2_end * 1000))
        data += struct.pack("B", int(self.app_adc_conf.use_filter))
        data += struct.pack("B", int(self.app_adc_conf.safe_start))
        data += struct.pack("B", self.app_adc_conf.buttons)
        data += struct.pack("B", int(self.app_adc_conf.voltage_inverted))
        data += struct.pack("B", int(self.app_adc_conf.voltage2_inverted))
        data += struct.pack(">f", self.app_adc_conf.throttle_exp)
        data += struct.pack(">f", self.app_adc_conf.throttle_exp_brake)
        data += struct.pack("B", int(self.app_adc_conf.throttle_exp_mode))
        data += struct.pack(">f", self.app_adc_conf.ramp_time_pos)
        data += struct.pack(">f", self.app_adc_conf.ramp_time_neg)
        data += struct.pack("B", int(self.app_adc_conf.multi_esc))
        data += struct.pack("B", int(self.app_adc_conf.tc))
        data += struct.pack(">f", self.app_adc_conf.tc_max_diff)
        data += struct.pack(">H", self.app_adc_conf.update_rate_hz)

        # ── UART ──────────────────────────────────────────────────────────────────
        data += struct.pack(">I", self.app_uart_baudrate)

        # ── Nunchuk ───────────────────────────────────────────────────────────────
        data += struct.pack("B", int(self.app_chuk_conf.ctrl_type))
        data += struct.pack(">f", self.app_chuk_conf.hyst)
        data += struct.pack(">f", self.app_chuk_conf.ramp_time_pos)
        data += struct.pack(">f", self.app_chuk_conf.ramp_time_neg)
        data += struct.pack(">f", self.app_chuk_conf.stick_erpm_per_s_in_cc)
        data += struct.pack(">f", self.app_chuk_conf.throttle_exp)
        data += struct.pack(">f", self.app_chuk_conf.throttle_exp_brake)
        data += struct.pack("B", int(self.app_chuk_conf.throttle_exp_mode))
        data += struct.pack("B", int(self.app_chuk_conf.multi_esc))
        data += struct.pack("B", int(self.app_chuk_conf.tc))
        data += struct.pack(">f", self.app_chuk_conf.tc_max_diff)
        data += struct.pack("B", int(self.app_chuk_conf.use_smart_rev))
        data += struct.pack(">f", self.app_chuk_conf.smart_rev_max_duty)
        data += struct.pack(">f", self.app_chuk_conf.smart_rev_ramp_time)

        # ── NRF ───────────────────────────────────────────────────────────────────
        data += struct.pack("B", int(self.app_nrf_conf.speed))
        data += struct.pack("B", int(self.app_nrf_conf.power))
        data += struct.pack("B", int(self.app_nrf_conf.crc_type))
        data += struct.pack("B", int(self.app_nrf_conf.retry_delay))
        data += struct.pack("B", self.app_nrf_conf.retries)
        data += struct.pack("B", self.app_nrf_conf.channel)
        data += struct.pack("B", self.app_nrf_conf.address[0])
        data += struct.pack("B", self.app_nrf_conf.address[1])
        data += struct.pack("B", self.app_nrf_conf.address[2])
        data += struct.pack("B", int(self.app_nrf_conf.send_crc_ack))

        # ── Balance ───────────────────────────────────────────────────────────────
        data += struct.pack("B", int(self.app_balance_conf.pid_mode))
        data += struct.pack(">f", self.app_balance_conf.kp)
        data += struct.pack(">f", self.app_balance_conf.ki)
        data += struct.pack(">f", self.app_balance_conf.kd)
        data += struct.pack(">f", self.app_balance_conf.kp2)
        data += struct.pack(">f", self.app_balance_conf.ki2)
        data += struct.pack(">f", self.app_balance_conf.kd2)
        data += struct.pack(">H", self.app_balance_conf.hertz)
        data += struct.pack(">H", self.app_balance_conf.loop_time_filter)
        data += struct.pack(">f", self.app_balance_conf.fault_pitch)
        data += struct.pack(">f", self.app_balance_conf.fault_roll)
        data += struct.pack(">f", self.app_balance_conf.fault_duty)
        data += struct.pack(">f", self.app_balance_conf.fault_adc1)
        data += struct.pack(">f", self.app_balance_conf.fault_adc2)
        data += struct.pack(">H", self.app_balance_conf.fault_delay_pitch)
        data += struct.pack(">H", self.app_balance_conf.fault_delay_roll)
        data += struct.pack(">H", self.app_balance_conf.fault_delay_duty)
        data += struct.pack(">H", self.app_balance_conf.fault_delay_switch_half)
        data += struct.pack(">H", self.app_balance_conf.fault_delay_switch_full)
        data += struct.pack(">H", self.app_balance_conf.fault_adc_half_erpm)
        data += struct.pack("B", int(self.app_balance_conf.fault_is_dual_switch))
        data += struct.pack(">h", int(self.app_balance_conf.tiltback_duty_angle * 100))
        data += struct.pack(">h", int(self.app_balance_conf.tiltback_duty_speed * 100))
        data += struct.pack(">h", int(self.app_balance_conf.tiltback_duty * 1000))
        data += struct.pack(">h", int(self.app_balance_conf.tiltback_hv_angle * 100))
        data += struct.pack(">h", int(self.app_balance_conf.tiltback_hv_speed * 100))
        data += struct.pack(">f", self.app_balance_conf.tiltback_hv)
        data += struct.pack(">h", int(self.app_balance_conf.tiltback_lv_angle * 100))
        data += struct.pack(">h", int(self.app_balance_conf.tiltback_lv_speed * 100))
        data += struct.pack(">f", self.app_balance_conf.tiltback_lv)
        data += struct.pack(">h", int(self.app_balance_conf.tiltback_return_speed * 100))
        data += struct.pack(">f", self.app_balance_conf.tiltback_constant)
        data += struct.pack(">H", self.app_balance_conf.tiltback_constant_erpm)
        data += struct.pack(">f", self.app_balance_conf.tiltback_variable)
        data += struct.pack(">f", self.app_balance_conf.tiltback_variable_max)
        data += struct.pack(">h", int(self.app_balance_conf.noseangling_speed * 100))
        data += struct.pack(">f", self.app_balance_conf.startup_pitch_tolerance)
        data += struct.pack(">f", self.app_balance_conf.startup_roll_tolerance)
        data += struct.pack(">f", self.app_balance_conf.startup_speed)
        data += struct.pack(">f", self.app_balance_conf.deadzone)
        data += struct.pack("B", int(self.app_balance_conf.multi_esc))
        data += struct.pack(">f", self.app_balance_conf.yaw_kp)
        data += struct.pack(">f", self.app_balance_conf.yaw_ki)
        data += struct.pack(">f", self.app_balance_conf.yaw_kd)
        data += struct.pack(">f", self.app_balance_conf.roll_steer_kp)
        data += struct.pack(">f", self.app_balance_conf.roll_steer_erpm_kp)
        data += struct.pack(">f", self.app_balance_conf.brake_current)
        data += struct.pack(">H", self.app_balance_conf.brake_timeout)
        data += struct.pack(">f", self.app_balance_conf.yaw_current_clamp)
        data += struct.pack(">f", self.app_balance_conf.ki_limit)
        data += struct.pack(">H", self.app_balance_conf.kd_pt1_lowpass_frequency)
        data += struct.pack(">H", self.app_balance_conf.kd_pt1_highpass_frequency)
        data += struct.pack(">f", self.app_balance_conf.booster_angle)
        data += struct.pack(">f", self.app_balance_conf.booster_ramp)
        data += struct.pack(">f", self.app_balance_conf.booster_current)
        data += struct.pack(">f", self.app_balance_conf.torquetilt_start_current)
        data += struct.pack(">f", self.app_balance_conf.torquetilt_angle_limit)
        data += struct.pack(">f", self.app_balance_conf.torquetilt_on_speed)
        data += struct.pack(">f", self.app_balance_conf.torquetilt_off_speed)
        data += struct.pack(">f", self.app_balance_conf.torquetilt_strength)
        data += struct.pack(">f", self.app_balance_conf.torquetilt_filter)
        data += struct.pack(">f", self.app_balance_conf.turntilt_strength)
        data += struct.pack(">f", self.app_balance_conf.turntilt_angle_limit)
        data += struct.pack(">f", self.app_balance_conf.turntilt_start_angle)
        data += struct.pack(">H", self.app_balance_conf.turntilt_start_erpm)
        data += struct.pack(">f", self.app_balance_conf.turntilt_speed)
        data += struct.pack(">H", self.app_balance_conf.turntilt_erpm_boost)
        data += struct.pack(">H", self.app_balance_conf.turntilt_erpm_boost_end)

        # ── PAS ───────────────────────────────────────────────────────────────────
        data += struct.pack("B", int(self.app_pas_conf.ctrl_type))
        data += struct.pack("B", int(self.app_pas_conf.sensor_type))
        data += struct.pack(">h", int(self.app_pas_conf.current_scaling * 1000))
        data += struct.pack(">h", int(self.app_pas_conf.pedal_rpm_start * 10))
        data += struct.pack(">h", int(self.app_pas_conf.pedal_rpm_end * 10))
        data += struct.pack("B", int(self.app_pas_conf.invert_pedal_direction))
        data += struct.pack(">H", self.app_pas_conf.magnets)
        data += struct.pack("B", int(self.app_pas_conf.use_filter))
        data += struct.pack(">h", int(self.app_pas_conf.ramp_time_pos * 100))
        data += struct.pack(">h", int(self.app_pas_conf.ramp_time_neg * 100))
        data += struct.pack(">H", self.app_pas_conf.update_rate_hz)

        # ── IMU ───────────────────────────────────────────────────────────────────
        data += struct.pack("B", int(self.imu_conf.type))
        data += struct.pack("B", int(self.imu_conf.mode))
        data += struct.pack("B", int(self.imu_conf.filter))
        data += struct.pack(">h", int(self.imu_conf.accel_lowpass_filter_x * 1))
        data += struct.pack(">h", int(self.imu_conf.accel_lowpass_filter_y * 1))
        data += struct.pack(">h", int(self.imu_conf.accel_lowpass_filter_z * 1))
        data += struct.pack(">h", int(self.imu_conf.gyro_lowpass_filter * 1))
        data += struct.pack(">H", self.imu_conf.sample_rate_hz)
        data += struct.pack("B", int(self.imu_conf.use_magnetometer))
        data += struct.pack(">f", self.imu_conf.accel_confidence_decay)
        data += struct.pack(">f", self.imu_conf.mahony_kp)
        data += struct.pack(">f", self.imu_conf.mahony_ki)
        data += struct.pack(">f", self.imu_conf.madgwick_beta)
        data += struct.pack(">f", self.imu_conf.rot_roll)
        data += struct.pack(">f", self.imu_conf.rot_pitch)
        data += struct.pack(">f", self.imu_conf.rot_yaw)
        data += struct.pack(">f", self.imu_conf.accel_offsets[0])
        data += struct.pack(">f", self.imu_conf.accel_offsets[1])
        data += struct.pack(">f", self.imu_conf.accel_offsets[2])
        data += struct.pack(">f", self.imu_conf.gyro_offsets[0])
        data += struct.pack(">f", self.imu_conf.gyro_offsets[1])
        data += struct.pack(">f", self.imu_conf.gyro_offsets[2])

        return AppConfigurationMessage.ID.to_bytes(1) + data


class FW6_00CMP(CommandMessageProcessor):
    def __init__(
        self,
        com_port,
        baud_rate,
        command_byte_size,
        eboard: EBoard,
        eks: EboardKinematicState,
        eks_lock: Lock,
        bdm: BatteryDischargeModel,
        mc: MotorController,
    ):
        super().__init__(com_port, baud_rate, command_byte_size)
        self.__cmd_id_name = {
            6: CommandMessageProcessor.CURRENT,
            8: CommandMessageProcessor.RPM,
            30: CommandMessageProcessor.HEARTBEAT,
            0: CommandMessageProcessor.FIRMWARE,
            14: CommandMessageProcessor.MOTOR_CONTROLLER_CONFIGURATION,
            17: CommandMessageProcessor.APP_CONFIGURATION,
            152: CommandMessageProcessor.BIONIC_BOARDER,
        }
        self.__packet_header = lambda l: int.to_bytes(2) + int.to_bytes(l)
        # crc value + end_byte
        self.__packet_footer = lambda payload: int.to_bytes(self.crc16(payload), 2) + int.to_bytes(0x03)
        self.__eks = eks
        self.__eks_lock = eks_lock
        self.__bdm = bdm
        self.__mc = mc
        self.__eboard = eboard

    @property
    def _command_id_name(self):
        """
        Returns a dictionary of the command id associated to the name of the command.
        """
        return self.__cmd_id_name

    def _get_command_id(self, command: bytes) -> int:
        """
        The command ID for the received command is in the third byte of the [command] data buffer.
        """
        return command[2]

    def _publish_bionic_boarder(self):
        bb = BionicBoarderMessage()
        with self.__eks_lock:
            bb.motor_current = self.__eks.motor_current
            bb.rpm = self.__eks.erpm
            bb.acc[0] = self.__eks.acceleration_x
            bb.rpy[1] = self.__eks.pitch * (math.pi / 180.0)
        msg_data = bb.buffer
        packet = self.__packet_header(len(msg_data)) + msg_data + self.__packet_footer(msg_data)
        self.serial.write(packet)
        Logger().logger.info(
            "Publishing Bionic Boarder message",
            motor_current=bb.motor_current,
            duty_cycle=bb.duty_cycle,
            rpm=bb.rpm,
            imu_acc=bb.acc,
            imu_rpy=bb.rpy,
            CMP=self.__class__.__name__,
        )

    def _publish_firmware(self):
        fw = FirmwareMessage()
        packet = self.__packet_header(len(fw.buffer)) + fw.buffer + self.__packet_footer(fw.buffer)
        self.serial.write(packet)

    def _publish_motor_controller_configuration(self):
        mcc = MotorControllerConfigurationMessage()
        mcc.si_wheel_diameter = self.__eboard.wheel_diameter_m
        mcc.si_battery_ah = self.__eboard.battery_max_capacity_Ah
        mcc.si_gear_ratio = self.__eboard.gear_ratio
        mcc.si_motor_poles = self.__eboard.motor_pole_pairs * 2
        mcc.l_current_max = self.__eboard.motor_max_amps
        mcc.l_watt_max = self.__eboard.motor_max_power_watts
        mcc.l_max_vin = self.__eboard.battery_max_voltage
        msg_data = mcc.buffer
        packet = int.to_bytes(3) + int.to_bytes(len(msg_data), 2) + msg_data + self.__packet_footer(msg_data)
        self.serial.write(packet)
        Logger().logger.info(
            "Publishing motor controller configuration message",
            wheel_diameter_m=mcc.si_wheel_diameter,
            battery_capacity_Ah=mcc.si_battery_ah,
            gear_ratio=mcc.si_gear_ratio,
            motor_poles=mcc.si_motor_poles,
            current_max=mcc.l_current_max,
            watt_max=mcc.l_watt_max,
            max_vin=mcc.l_max_vin,
            CMP=self.__class__.__name__,
        )

    def _publish_app_configuration(self):
        app_conf = AppConfigurationMessage()
        msg_data = app_conf.buffer
        packet = int.to_bytes(3) + int.to_bytes(len(msg_data), 2) + msg_data + self.__packet_footer(msg_data)
        self.serial.write(packet)
        Logger().logger.info("Publishing application configuration message", CMP=self.__class__.__name__)

    def _update_current(self, command):
        motor_current_commanded = int.from_bytes(command[3:7], byteorder="big") / 1000.0
        self.__mc.target_current = motor_current_commanded
        try:
            self.__mc.current_sem.release()
        except ValueError as e:
            pass
        Logger().logger.info(
            "Processing set current command", motor_current=motor_current_commanded, CMP=self.__class__.__name__
        )

    def _update_rpm(self, command):
        erpm_commanded = int.from_bytes(command[3:7], byteorder="big")
        self.__mc.target_erpm = erpm_commanded
        try:
            self.__mc.erpm_sem.release()
        except ValueError as e:
            pass
        Logger().logger.info("Processing set ERPM command", erpm=erpm_commanded, CMP=self.__class__.__name__)
