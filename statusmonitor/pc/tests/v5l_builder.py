from __future__ import annotations

import zlib
from pathlib import Path

import numpy as np

from statusmonitor.protocol.schema31 import (
    BLOCK_MAGIC,
    ENDIAN_MARKER,
    FILE_HEADER_BYTES,
    FILE_MAGIC,
    FOOTER_MAGIC,
    LOG_FRAME_BYTES,
    LOG_MAGIC,
    LOG_SCHEMA_MAJOR,
    LOG_SCHEMA_MINOR,
    TRACE_ACTUATOR_INTENT,
    TRACE_ARBITRATION,
    TRACE_COMPETITION_INPUT,
    TRACE_DRIVER_MAPPING,
    TRACE_OUTPUT_STATUS,
    TRACE_RAW_INPUTS,
    TRACE_REQUEST_CANDIDATE,
    block_header_dtype,
    file_header_dtype,
    footer_dtype,
    log_frame_dtype,
)


def crc32(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF


def synthetic_frames(count: int = 500, pose: bool = False) -> np.ndarray:
    frames = np.zeros(count, dtype=log_frame_dtype)
    time_s = np.arange(count, dtype=float) * 0.01
    sequence = np.arange(10, 10 + count, dtype=np.uint32)
    run_hash = 0xA1B2C3D4
    frames["header"]["magic"] = LOG_MAGIC
    frames["header"]["schema_major"] = LOG_SCHEMA_MAJOR
    frames["header"]["schema_minor"] = LOG_SCHEMA_MINOR
    frames["header"]["frame_size_bytes"] = LOG_FRAME_BYTES
    frames["header"]["time_us"] = (time_s * 1e6).astype(np.uint64) + 1_000_000
    frames["header"]["sequence"] = sequence
    frames["header"]["mode_epoch"] = 7
    frames["header"]["run_id_hash"] = run_hash
    frames["raw_io"]["acquisition_end_us"] = frames["header"]["time_us"]
    frames["raw_io"]["battery_api_ok"] = True
    frames["battery_V"] = 12.4 - 0.15 * np.sin(2 * np.pi * 0.2 * time_s) ** 2
    frames["controller"]["connected"] = True
    frames["controller"]["api_ok"] = True
    frames["controller"]["enabled"] = True
    frames["controller"]["left_y"] = np.sin(2 * np.pi * 0.4 * time_s)
    frames["trace"]["availability_bits"] = (
        TRACE_RAW_INPUTS
        | TRACE_DRIVER_MAPPING
        | TRACE_REQUEST_CANDIDATE
        | TRACE_ARBITRATION
        | TRACE_ACTUATOR_INTENT
        | TRACE_OUTPUT_STATUS
        | TRACE_COMPETITION_INPUT
    )
    if pose:
        from statusmonitor.protocol.schema31 import TRACE_POSE_STATE, TRACE_VALIDATED_STATE

        frames["trace"]["availability_bits"] |= TRACE_POSE_STATE | TRACE_VALIDATED_STATE
        frames["translation_quality"] = 0
        frames["heading_quality"] = 0
        frames["pose_x_m"] = 0.3 * time_s
        frames["pose_y_m"] = 0.05 * np.sin(2 * np.pi * 0.2 * time_s)
        frames["pose_theta_rad"] = 0.1 * time_s
        frames["body_vx_mps"] = 0.3
        frames["body_omega_radps"] = 0.1
    else:
        frames["translation_quality"] = 2
        frames["heading_quality"] = 2
    frames["request"]["source"] = 1
    frames["request"]["request_time_us"] = frames["header"]["time_us"]
    frames["request"]["ttl_us"] = 100_000
    frames["request"]["forward"] = frames["controller"]["left_y"]
    frames["request"]["requested_left_V"] = 6 * frames["controller"]["left_y"]
    frames["request"]["requested_right_V"] = 6 * frames["controller"]["left_y"]
    frames["actuator"]["final_left_V"] = frames["request"]["requested_left_V"]
    frames["actuator"]["final_right_V"] = frames["request"]["requested_right_V"]
    frames["actuator"]["derate_target"] = 1.0
    frames["actuator"]["derate_applied"] = 1.0
    frames["actuator"]["write_attempted"] = True
    frames["actuator"]["write_ok"] = True
    frames["timing"]["raw_dt_s"] = 0.01
    frames["timing"]["math_dt_s"] = 0.01
    frames["timing"]["exec_s"] = 0.001 + 0.0001 * np.sin(2 * np.pi * 1.1 * time_s)
    frames["timing"]["jitter_s"] = frames["timing"]["raw_dt_s"] - 0.01
    frames["trace"]["mapped_throttle"] = frames["controller"]["left_y"]
    frames["trace"]["output_derate"] = 1.0
    for side_name, phase in (("left_motor", 0.0), ("right_motor", 0.03)):
        for index in range(3):
            motor = frames[side_name][:, index]
            motor["smart_port"] = index + 1 + (0 if side_name == "left_motor" else 3)
            motor["api_ok_mask"] = 0x3F
            motor["quality"] = 0
            motor["velocity_radps"] = (
                8.0 * np.sin(2 * np.pi * 0.4 * time_s + phase) + 0.03 * index
            )
            motor["position_rad"] = np.cumsum(motor["velocity_radps"]) * 0.01
            motor["current_A"] = 0.4 + 0.2 * np.abs(motor["velocity_radps"]) / 8
            motor["temperature_C"] = 28 + 0.2 * time_s + 0.1 * index
            motor["applied_voltage_V"] = frames["actuator"]["final_left_V"]
            motor["position_time_us"] = frames["header"]["time_us"]
            motor["velocity_time_us"] = frames["header"]["time_us"]
            motor["current_time_us"] = frames["header"]["time_us"]
            motor["temperature_time_us"] = frames["header"]["time_us"]
            motor["applied_voltage_time_us"] = frames["header"]["time_us"]
    return frames


def write_v5l(path: Path, frames: np.ndarray, *, footer: bool = True, producer_drops: int = 0) -> Path:
    header = np.zeros(1, dtype=file_header_dtype)
    h = header[0]
    h["magic"] = FILE_MAGIC
    h["format_major"] = 2
    h["format_minor"] = 0
    h["log_schema_major"] = LOG_SCHEMA_MAJOR
    h["log_schema_minor"] = LOG_SCHEMA_MINOR
    h["endian_marker"] = ENDIAN_MARKER
    h["header_size_bytes"] = FILE_HEADER_BYTES
    h["frame_size_bytes"] = LOG_FRAME_BYTES
    h["boot_id"] = 0x1122334455667788
    h["session_sequence"] = 3
    h["storage_sequence"] = 17
    h["start_time_ms"] = 9000
    h["run_id_hash"] = 0xA1B2C3D4
    h["robot_id_hash"] = 0x01020304
    h["config_hash"] = 0x55667788
    h["software_identity_hash"] = 0x99AABBCC
    h["hardware_revision"] = 1
    h["config_schema"] = 2
    h["calibration_revision"] = 0
    h["hardware_verification"] = 1
    h["dirty_build"] = False
    h["robot_id"] = b"1690X"
    h["software_version"] = b"test"
    h["source_commit"] = b"0123456789abcdef0123456789abcdef01234567"
    header_bytes = bytearray(header.tobytes())
    crc_offset = file_header_dtype.fields["header_crc32"][1]
    h["header_crc32"] = crc32(header_bytes[:crc_offset])
    header_bytes = header.tobytes()

    payload = frames.tobytes()
    block = np.zeros(1, dtype=block_header_dtype)
    b = block[0]
    b["magic"] = BLOCK_MAGIC
    b["block_sequence"] = 1
    b["first_frame_sequence"] = int(frames["header"]["sequence"][0]) if len(frames) else 0
    b["frame_count"] = len(frames)
    b["payload_bytes"] = len(payload)
    b["payload_crc32"] = crc32(payload)
    block_bytes = bytearray(block.tobytes())
    crc_offset = block_header_dtype.fields["header_crc32"][1]
    b["header_crc32"] = crc32(block_bytes[:crc_offset])

    pieces = [header_bytes, block.tobytes(), payload]
    if footer:
        end = np.zeros(1, dtype=footer_dtype)
        f = end[0]
        f["magic"] = FOOTER_MAGIC
        f["total_frames"] = len(frames)
        f["first_frame_sequence"] = int(frames["header"]["sequence"][0]) if len(frames) else 0
        f["last_frame_sequence"] = int(frames["header"]["sequence"][-1]) if len(frames) else 0
        f["first_time_us"] = int(frames["header"]["time_us"][0]) if len(frames) else 0
        f["last_time_us"] = int(frames["header"]["time_us"][-1]) if len(frames) else 0
        f["producer_drops"] = producer_drops
        f["block_count"] = 1
        footer_bytes = bytearray(end.tobytes())
        crc_offset = footer_dtype.fields["footer_crc32"][1]
        f["footer_crc32"] = crc32(footer_bytes[:crc_offset])
        pieces.append(end.tobytes())
    path.write_bytes(b"".join(pieces))
    return path

def write_long_v5l(
    path: Path, total_frames: int, frames_per_block: int = 256
) -> Path:
    seed = write_v5l(path, synthetic_frames(1))
    template = seed.read_bytes()
    header_bytes = template[: file_header_dtype.itemsize]
    with path.open("wb") as stream:
        stream.write(header_bytes)
        sequence = 10
        block_sequence = 1
        remaining = total_frames
        while remaining:
            count = min(frames_per_block, remaining)
            frames = np.zeros(count, dtype=log_frame_dtype)
            frames["header"]["magic"] = LOG_MAGIC
            frames["header"]["schema_major"] = LOG_SCHEMA_MAJOR
            frames["header"]["schema_minor"] = LOG_SCHEMA_MINOR
            frames["header"]["frame_size_bytes"] = LOG_FRAME_BYTES
            frames["header"]["sequence"] = np.arange(sequence, sequence + count)
            frames["header"]["time_us"] = (
                np.arange(sequence - 10, sequence - 10 + count, dtype=np.uint64)
                * 10_000
                + 1_000_000
            )
            frames["header"]["mode_epoch"] = 7
            frames["header"]["run_id_hash"] = 0xA1B2C3D4
            frames["translation_quality"] = 2
            frames["heading_quality"] = 2
            frames["battery_V"] = 12.0
            frames["timing"]["raw_dt_s"] = 0.01
            frames["timing"]["math_dt_s"] = 0.01
            payload = frames.tobytes()
            block = np.zeros(1, dtype=block_header_dtype)
            b = block[0]
            b["magic"] = BLOCK_MAGIC
            b["block_sequence"] = block_sequence
            b["first_frame_sequence"] = sequence
            b["frame_count"] = count
            b["payload_bytes"] = len(payload)
            b["payload_crc32"] = crc32(payload)
            raw_block = bytearray(block.tobytes())
            crc_offset = block_header_dtype.fields["header_crc32"][1]
            b["header_crc32"] = crc32(raw_block[:crc_offset])
            stream.write(block.tobytes())
            stream.write(payload)
            sequence += count
            block_sequence += 1
            remaining -= count
        end = np.zeros(1, dtype=footer_dtype)
        f = end[0]
        f["magic"] = FOOTER_MAGIC
        f["total_frames"] = total_frames
        f["first_frame_sequence"] = 10
        f["last_frame_sequence"] = 9 + total_frames
        f["first_time_us"] = 1_000_000
        f["last_time_us"] = 1_000_000 + (total_frames - 1) * 10_000
        f["producer_drops"] = 0
        f["block_count"] = block_sequence - 1
        raw_footer = bytearray(end.tobytes())
        crc_offset = footer_dtype.fields["footer_crc32"][1]
        f["footer_crc32"] = crc32(raw_footer[:crc_offset])
        stream.write(end.tobytes())
    return path
