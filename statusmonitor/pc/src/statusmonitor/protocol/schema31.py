from __future__ import annotations

import numpy as np

LOG_MAGIC = 0x374D3030
LOG_SCHEMA_MAJOR = 3
LOG_SCHEMA_MINOR = 1
LOG_FRAME_BYTES = 1536
MOTORS_PER_SIDE = 3
MAX_TRACKING_WHEELS = 3

FILE_MAGIC = 0x324C3556
BLOCK_MAGIC = 0x314B4C42
FOOTER_MAGIC = 0x31444E45
ENDIAN_MARKER = 0x01020304
FILE_HEADER_BYTES = 160
BLOCK_HEADER_BYTES = 28
FILE_FOOTER_BYTES = 48

TRACE_RAW_INPUTS = 1 << 0
TRACE_VALIDATED_STATE = 1 << 1
TRACE_POSE_STATE = 1 << 2
TRACE_DRIVER_MAPPING = 1 << 3
TRACE_REQUEST_CANDIDATE = 1 << 4
TRACE_ARBITRATION = 1 << 5
TRACE_ACTUATOR_INTENT = 1 << 6
TRACE_OUTPUT_STATUS = 1 << 7
TRACE_FAULT_MANAGER = 1 << 8
TRACE_PID_TERMS = 1 << 9
TRACE_MECHANISMS = 1 << 10
TRACE_PNEUMATICS = 1 << 11
TRACE_AUTONOMOUS_PROGRAM = 1 << 12
TRACE_COMPETITION_INPUT = 1 << 13

QUALITY_NAMES = {0: "Good", 1: "Degraded", 2: "Invalid"}


def _dtype(fields: list[tuple], *, align: bool = True) -> np.dtype:
    return np.dtype(fields, align=align)


log_header_dtype = _dtype(
    [
        ("magic", "<u4"),
        ("schema_major", "<u2"),
        ("schema_minor", "<u2"),
        ("frame_size_bytes", "<u4"),
        ("time_us", "<u8"),
        ("sequence", "<u4"),
        ("mode_epoch", "<u4"),
        ("run_id_hash", "<u4"),
    ]
)

motor_dtype = _dtype(
    [
        ("smart_port", "u1"),
        ("api_ok_mask", "u1"),
        ("quality", "u1"),
        ("reserved", "u1"),
        ("position_rad", "<f8"),
        ("velocity_radps", "<f8"),
        ("current_A", "<f8"),
        ("temperature_C", "<f8"),
        ("applied_voltage_V", "<f8"),
        ("position_time_us", "<u8"),
        ("velocity_time_us", "<u8"),
        ("current_time_us", "<u8"),
        ("temperature_time_us", "<u8"),
        ("applied_voltage_time_us", "<u8"),
        ("position_status", "<u4"),
        ("velocity_status", "<u4"),
        ("current_status", "<u4"),
        ("temperature_status", "<u4"),
        ("applied_voltage_status", "<u4"),
        ("api_faults", "<u4"),
        ("reject_bits", "<u4"),
    ]
)

tracking_dtype = _dtype(
    [
        ("position_rad", "<f8"),
        ("velocity_radps", "<f8"),
        ("position_time_us", "<u8"),
        ("velocity_time_us", "<u8"),
        ("position_status", "<u4"),
        ("velocity_status", "<u4"),
        ("configured", "?"),
        ("position_api_ok", "?"),
        ("velocity_api_ok", "?"),
        ("reserved", "u1"),
    ]
)

raw_io_dtype = _dtype(
    [
        ("acquisition_end_us", "<u8"),
        ("imu_rotation_time_us", "<u8"),
        ("imu_rate_time_us", "<u8"),
        ("battery_time_us", "<u8"),
        ("imu_rotation_status", "<u4"),
        ("imu_rate_status", "<u4"),
        ("battery_status", "<u4"),
        ("imu_rotation_api_ok", "?"),
        ("imu_rate_api_ok", "?"),
        ("imu_status_api_ok", "?"),
        ("imu_calibrating", "?"),
        ("battery_api_ok", "?"),
        ("reserved", "u1", (3,)),
    ]
)

controller_dtype = _dtype(
    [
        ("left_x", "<f8"),
        ("left_y", "<f8"),
        ("right_x", "<f8"),
        ("right_y", "<f8"),
        ("buttons", "<u4"),
        ("mode", "u1"),
        ("connected", "?"),
        ("api_ok", "?"),
        ("enabled", "?"),
        ("field_connected", "?"),
        ("reserved", "u1", (3,)),
    ]
)

system_event_dtype = _dtype(
    [
        ("event_sequence", "<u4"),
        ("event_bits", "<u4"),
        ("mechanism_motor_request_mask", "<u4"),
        ("pneumatic_request_mask", "<u4"),
        ("automatic_program_id", "<u4"),
        ("drive_consumed", "?"),
        ("mechanisms_valid", "?"),
        ("pneumatics_valid", "?"),
        ("automatic_program_valid", "?"),
    ]
)

recording_dtype = _dtype(
    [
        ("session_sequence", "<u4"),
        ("event_bits", "<u4"),
        ("state", "u1"),
        ("error", "u1"),
        ("reserved", "<u2"),
    ]
)

request_dtype = _dtype(
    [
        ("source", "u1"),
        ("payload_kind", "u1"),
        ("owner_id", "<u2"),
        ("owner_lease", "<u4"),
        ("reject_bits", "<u4"),
        ("request_time_us", "<u8"),
        ("ttl_us", "<u8"),
        ("forward", "<f8"),
        ("steering", "<f8"),
        ("vx_mps", "<f8"),
        ("omega_radps", "<f8"),
        ("requested_left_V", "<f8"),
        ("requested_right_V", "<f8"),
    ]
)

actuator_dtype = _dtype(
    [
        ("unallocated_left_V", "<f8"),
        ("unallocated_right_V", "<f8"),
        ("allocated_left_V", "<f8"),
        ("allocated_right_V", "<f8"),
        ("final_left_V", "<f8"),
        ("final_right_V", "<f8"),
        ("derate_target", "<f8"),
        ("derate_applied", "<f8"),
        ("final_motor_voltage_V", "<f8", (MOTORS_PER_SIDE * 2,)),
        ("final_motor_valid_mask", "<u4"),
        ("applied_limits", "<u4"),
        ("write_reject_bits", "<u4"),
        ("last_written_sequence", "<u4"),
        ("write_attempted", "?"),
        ("write_ok", "?"),
    ]
)

timing_dtype = _dtype(
    [
        ("raw_dt_s", "<f8"),
        ("math_dt_s", "<f8"),
        ("exec_s", "<f8"),
        ("jitter_s", "<f8"),
        ("sensor_age_us", "<u8"),
        ("request_age_us", "<u8"),
        ("actuator_age_us", "<u8"),
        ("consecutive_overruns", "<u4"),
        ("overrun_total", "<u4"),
        ("ring_depth", "<u4"),
        ("log_dropped_total", "<u4"),
    ]
)

fault_dtype = _dtype(
    [
        ("active_fault_bits", "<u8"),
        ("latched_fault_bits", "<u8"),
        ("enter_fault_bits", "<u8"),
        ("exit_fault_bits", "<u8"),
        ("affected_motor_mask", "<u4"),
        ("severity", "u1"),
        ("safety_state", "u1"),
        ("clear_authorized", "?"),
        ("reserved", "u1"),
    ]
)

trace_dtype = _dtype(
    [
        ("mapped_throttle", "<f8"),
        ("mapped_turn", "<f8"),
        ("output_derate", "<f8"),
        ("request_age_us", "<u8"),
        ("mode_transition_time_us", "<u8"),
        ("availability_bits", "<u4"),
        ("arbitration_reject_bits", "<u4"),
        ("arbitration_rejected_count", "<u4"),
        ("selected_request_sequence", "<u4"),
        ("selected_owner_lease", "<u4"),
        ("mode_fault_bits", "<u4"),
        ("ring_high_watermark", "<u4"),
        ("selected_source", "u1"),
        ("stop_mode", "u1"),
        ("output_action", "u1"),
        ("drive_state", "u1"),
        ("quick_turn_active", "?"),
        ("request_candidate_present", "?"),
        ("request_selected", "?"),
        ("deadline_missed", "?"),
        ("competition_disabled", "?"),
        ("competition_autonomous", "?"),
        ("competition_api_ok", "?"),
        ("reserved", "u1"),
    ]
)

log_frame_dtype = _dtype(
    [
        ("header", log_header_dtype),
        ("left_motor", motor_dtype, (MOTORS_PER_SIDE,)),
        ("right_motor", motor_dtype, (MOTORS_PER_SIDE,)),
        ("tracking", tracking_dtype, (MAX_TRACKING_WHEELS,)),
        ("raw_io", raw_io_dtype),
        ("imu_rotation_rad", "<f8"),
        ("imu_rate_radps", "<f8"),
        ("battery_V", "<f8"),
        ("pose_x_m", "<f8"),
        ("pose_y_m", "<f8"),
        ("pose_theta_rad", "<f8"),
        ("body_vx_mps", "<f8"),
        ("body_vy_mps", "<f8"),
        ("body_omega_radps", "<f8"),
        ("translation_quality", "u1"),
        ("heading_quality", "u1"),
        ("reserved", "<u2"),
        ("controller", controller_dtype),
        ("system_event", system_event_dtype),
        ("recording", recording_dtype),
        ("request", request_dtype),
        ("actuator", actuator_dtype),
        ("timing", timing_dtype),
        ("fault", fault_dtype),
        ("trace", trace_dtype),
    ]
)

file_header_dtype = _dtype(
    [
        ("magic", "<u4"),
        ("format_major", "<u2"),
        ("format_minor", "<u2"),
        ("log_schema_major", "<u2"),
        ("log_schema_minor", "<u2"),
        ("endian_marker", "<u4"),
        ("header_size_bytes", "<u4"),
        ("frame_size_bytes", "<u4"),
        ("boot_id", "<u8"),
        ("session_sequence", "<u4"),
        ("storage_sequence", "<u4"),
        ("start_time_ms", "<u4"),
        ("run_id_hash", "<u4"),
        ("robot_id_hash", "<u4"),
        ("config_hash", "<u4"),
        ("software_identity_hash", "<u4"),
        ("hardware_revision", "<u4"),
        ("config_schema", "<u4"),
        ("calibration_revision", "<u4"),
        ("hardware_verification", "u1"),
        ("dirty_build", "?"),
        ("reserved", "<u2"),
        ("robot_id", "S16"),
        ("software_version", "S16"),
        ("source_commit", "S41"),
        ("reserved_tail", "u1", (3,)),
        ("header_crc32", "<u4"),
    ]
)

block_header_dtype = _dtype(
    [
        ("magic", "<u4"),
        ("block_sequence", "<u4"),
        ("first_frame_sequence", "<u4"),
        ("frame_count", "<u4"),
        ("payload_bytes", "<u4"),
        ("payload_crc32", "<u4"),
        ("header_crc32", "<u4"),
    ],
    align=False,
)

footer_dtype = _dtype(
    [
        ("magic", "<u4"),
        ("total_frames", "<u4"),
        ("first_frame_sequence", "<u4"),
        ("last_frame_sequence", "<u4"),
        ("first_time_us", "<u8"),
        ("last_time_us", "<u8"),
        ("producer_drops", "<u4"),
        ("block_count", "<u4"),
        ("footer_crc32", "<u4"),
    ]
)


def assert_abi() -> None:
    expected = {
        "LogHeader": (log_header_dtype.itemsize, 40),
        "MotorLogSample": (motor_dtype.itemsize, 120),
        "TrackingLogSample": (tracking_dtype.itemsize, 48),
        "RawIoLog": (raw_io_dtype.itemsize, 56),
        "ControllerLog": (controller_dtype.itemsize, 48),
        "SystemEventLog": (system_event_dtype.itemsize, 24),
        "RecordingLog": (recording_dtype.itemsize, 12),
        "RequestLog": (request_dtype.itemsize, 80),
        "ActuatorLog": (actuator_dtype.itemsize, 136),
        "TimingLog": (timing_dtype.itemsize, 72),
        "FaultLog": (fault_dtype.itemsize, 40),
        "ControlTraceLog": (trace_dtype.itemsize, 80),
        "LogFrame": (log_frame_dtype.itemsize, LOG_FRAME_BYTES),
        "RecordingFileHeader": (file_header_dtype.itemsize, FILE_HEADER_BYTES),
        "RecordingBlockHeader": (block_header_dtype.itemsize, BLOCK_HEADER_BYTES),
        "RecordingFileFooter": (footer_dtype.itemsize, FILE_FOOTER_BYTES),
    }
    bad = [f"{name}={actual}, expected {wanted}" for name, (actual, wanted) in expected.items() if actual != wanted]
    if bad:
        raise RuntimeError("V5L ABI mismatch: " + "; ".join(bad))


assert_abi()
