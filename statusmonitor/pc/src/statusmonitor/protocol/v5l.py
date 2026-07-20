from __future__ import annotations

import os
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

import numpy as np

from statusmonitor.models import (
    IntegrityIssue,
    IntegrityReport,
    RobotIdentity,
    Verdict,
)

from .schema31 import (
    BLOCK_HEADER_BYTES,
    BLOCK_MAGIC,
    ENDIAN_MARKER,
    FILE_FOOTER_BYTES,
    FILE_HEADER_BYTES,
    FILE_MAGIC,
    FOOTER_MAGIC,
    LOG_FRAME_BYTES,
    LOG_MAGIC,
    LOG_SCHEMA_MAJOR,
    LOG_SCHEMA_MINOR,
    block_header_dtype,
    file_header_dtype,
    footer_dtype,
    log_frame_dtype,
)


def _crc32(data: bytes | memoryview) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF


def _cstring(value: np.bytes_) -> str:
    return bytes(value).split(b"\0", 1)[0].decode("utf-8", errors="replace")


def _scalar(record: np.void, name: str) -> int:
    return int(record[name])


@dataclass(frozen=True)
class DecodedRecording:
    path: Path
    frames: np.ndarray
    report: IntegrityReport


class V5LReader:
    """Streaming verifier and decoder for V5L2 / LogFrame schema 3.1."""

    def __init__(self, path: str | os.PathLike[str]):
        self.path = Path(path)

    def _issue(
        self,
        issues: list[IntegrityIssue],
        code: str,
        severity: str,
        message: str,
        *,
        offset: int | None = None,
        frame_sequence: int | None = None,
    ) -> None:
        issues.append(
            IntegrityIssue(
                code=code,
                severity=severity,
                message=message,
                offset=offset,
                frame_sequence=frame_sequence,
            )
        )

    def read(self, *, load_frames: bool = True) -> DecodedRecording:
        issues: list[IntegrityIssue] = []
        file_size = self.path.stat().st_size
        if file_size < FILE_HEADER_BYTES:
            report = IntegrityReport(
                path=str(self.path),
                verdict=Verdict.FAIL,
                complete=False,
                file_bytes=file_size,
                issues=[
                    IntegrityIssue(
                        code="BAD_HEADER",
                        severity="error",
                        message=f"file is shorter than {FILE_HEADER_BYTES}-byte V5L2 header",
                        offset=0,
                    )
                ],
            )
            return DecodedRecording(self.path, np.empty(0, dtype=log_frame_dtype), report)

        arrays: list[np.ndarray] = []
        valid_bytes = 0
        valid_blocks = 0
        complete = False
        footer: np.void | None = None
        expected_block = 1
        expected_sequence: int | None = None
        first_sequence: int | None = None
        last_sequence: int | None = None
        first_time: int | None = None
        last_time: int | None = None
        gaps = 0
        duplicates = 0
        regressions = 0
        invalid_numeric = 0
        total_frames = 0
        usable_raw: list[tuple[int, int]] = []
        current_window_start: int | None = None
        current_window_end: int | None = None

        with self.path.open("rb") as stream:
            header_bytes = stream.read(FILE_HEADER_BYTES)
            header = np.frombuffer(header_bytes, dtype=file_header_dtype, count=1)[0]
            identity = RobotIdentity(
                robot_id=_cstring(header["robot_id"]),
                robot_id_hash=_scalar(header, "robot_id_hash"),
                software_version=_cstring(header["software_version"]),
                source_commit=_cstring(header["source_commit"]),
                dirty_build=bool(header["dirty_build"]),
                config_hash=_scalar(header, "config_hash"),
                software_identity_hash=_scalar(header, "software_identity_hash"),
                hardware_revision=_scalar(header, "hardware_revision"),
                config_schema=_scalar(header, "config_schema"),
                calibration_revision=_scalar(header, "calibration_revision"),
                hardware_verification=_scalar(header, "hardware_verification"),
                log_schema_major=_scalar(header, "log_schema_major"),
                log_schema_minor=_scalar(header, "log_schema_minor"),
                frame_size_bytes=_scalar(header, "frame_size_bytes"),
                boot_id=_scalar(header, "boot_id"),
                session_sequence=_scalar(header, "session_sequence"),
                storage_sequence=_scalar(header, "storage_sequence"),
                start_time_ms=_scalar(header, "start_time_ms"),
                run_id_hash=_scalar(header, "run_id_hash"),
            )
            stored_header_crc = _scalar(header, "header_crc32")
            header_crc_offset = file_header_dtype.fields["header_crc32"][1]
            if _crc32(header_bytes[:header_crc_offset]) != stored_header_crc:
                self._issue(issues, "BAD_HEADER_CRC", "error", "file header CRC32 mismatch", offset=0)
            if _scalar(header, "magic") != FILE_MAGIC:
                self._issue(issues, "BAD_MAGIC", "error", "not a V5L2 file", offset=0)
            if _scalar(header, "endian_marker") != ENDIAN_MARKER:
                self._issue(issues, "BAD_ENDIAN", "error", "unsupported endian marker", offset=12)
            if _scalar(header, "header_size_bytes") != FILE_HEADER_BYTES:
                self._issue(issues, "BAD_HEADER_SIZE", "error", "unsupported header size", offset=16)
            if _scalar(header, "frame_size_bytes") != LOG_FRAME_BYTES:
                self._issue(issues, "BAD_FRAME_SIZE", "error", "unsupported LogFrame byte size", offset=20)
            if identity.log_schema_major != LOG_SCHEMA_MAJOR:
                self._issue(
                    issues,
                    "UNSUPPORTED_SCHEMA",
                    "error",
                    f"schema {identity.log_schema_major}.{identity.log_schema_minor} is not supported",
                    offset=8,
                )
            elif identity.log_schema_minor > LOG_SCHEMA_MINOR:
                self._issue(
                    issues,
                    "NEWER_SCHEMA_MINOR",
                    "warning",
                    f"schema minor {identity.log_schema_minor} is newer than decoder {LOG_SCHEMA_MINOR}",
                    offset=10,
                )
            valid_bytes = FILE_HEADER_BYTES

            while stream.tell() < file_size:
                offset = stream.tell()
                magic_bytes = stream.read(4)
                if len(magic_bytes) < 4:
                    self._issue(issues, "TRUNCATED", "error", "partial record magic at file end", offset=offset)
                    break
                magic = int.from_bytes(magic_bytes, "little")
                stream.seek(offset)

                if magic == FOOTER_MAGIC:
                    footer_bytes = stream.read(FILE_FOOTER_BYTES)
                    if len(footer_bytes) != FILE_FOOTER_BYTES:
                        self._issue(issues, "TRUNCATED_FOOTER", "error", "partial footer", offset=offset)
                        break
                    footer = np.frombuffer(footer_bytes, dtype=footer_dtype, count=1)[0]
                    footer_crc_offset = footer_dtype.fields["footer_crc32"][1]
                    if _crc32(footer_bytes[:footer_crc_offset]) != _scalar(footer, "footer_crc32"):
                        self._issue(issues, "BAD_FOOTER_CRC", "error", "footer CRC32 mismatch", offset=offset)
                        break
                    complete = True
                    valid_bytes = stream.tell()
                    if stream.tell() != file_size:
                        self._issue(
                            issues,
                            "TRAILING_DATA",
                            "error",
                            f"{file_size - stream.tell()} byte(s) follow the footer",
                            offset=stream.tell(),
                        )
                    break

                if magic != BLOCK_MAGIC:
                    self._issue(
                        issues,
                        "BAD_BLOCK_MAGIC",
                        "error",
                        f"unexpected record magic 0x{magic:08x}",
                        offset=offset,
                    )
                    break

                block_bytes = stream.read(BLOCK_HEADER_BYTES)
                if len(block_bytes) != BLOCK_HEADER_BYTES:
                    self._issue(issues, "TRUNCATED_BLOCK_HEADER", "error", "partial block header", offset=offset)
                    break
                block = np.frombuffer(block_bytes, dtype=block_header_dtype, count=1)[0]
                if _crc32(block_bytes[:-4]) != _scalar(block, "header_crc32"):
                    self._issue(issues, "BAD_BLOCK_HEADER_CRC", "error", "block header CRC32 mismatch", offset=offset)
                    break
                block_seq = _scalar(block, "block_sequence")
                if block_seq != expected_block:
                    self._issue(
                        issues,
                        "BLOCK_SEQUENCE_GAP",
                        "error",
                        f"expected block {expected_block}, found {block_seq}",
                        offset=offset,
                    )
                    break
                frame_count = _scalar(block, "frame_count")
                payload_bytes = _scalar(block, "payload_bytes")
                if frame_count == 0 or payload_bytes != frame_count * LOG_FRAME_BYTES:
                    self._issue(
                        issues,
                        "BAD_BLOCK_LENGTH",
                        "error",
                        f"frame_count={frame_count}, payload_bytes={payload_bytes}",
                        offset=offset,
                    )
                    break
                payload = stream.read(payload_bytes)
                if len(payload) != payload_bytes:
                    self._issue(
                        issues,
                        "TRUNCATED_PAYLOAD",
                        "error",
                        f"wanted {payload_bytes} bytes, found {len(payload)}",
                        offset=offset + BLOCK_HEADER_BYTES,
                    )
                    break
                if _crc32(payload) != _scalar(block, "payload_crc32"):
                    self._issue(issues, "BAD_PAYLOAD_CRC", "error", "block payload CRC32 mismatch", offset=offset)
                    break

                block_frames = np.frombuffer(payload, dtype=log_frame_dtype, count=frame_count).copy()
                if load_frames:
                    arrays.append(block_frames)
                total_frames += frame_count
                valid_blocks += 1
                expected_block += 1
                valid_bytes = stream.tell()

                block_first = int(block_frames["header"]["sequence"][0])
                if block_first != _scalar(block, "first_frame_sequence"):
                    self._issue(
                        issues,
                        "BLOCK_FRAME_SEQUENCE_MISMATCH",
                        "error",
                        "block first-frame sequence disagrees with payload",
                        offset=offset,
                        frame_sequence=block_first,
                    )

                sequences = block_frames["header"]["sequence"].astype(np.uint64)
                times = block_frames["header"]["time_us"].astype(np.uint64)
                headers = block_frames["header"]
                bad_header = (
                    (headers["magic"] != LOG_MAGIC)
                    | (headers["schema_major"] != LOG_SCHEMA_MAJOR)
                    | (headers["schema_minor"] != LOG_SCHEMA_MINOR)
                    | (headers["frame_size_bytes"] != LOG_FRAME_BYTES)
                    | (headers["run_id_hash"] != identity.run_id_hash)
                )
                if np.any(bad_header):
                    first_bad = int(np.flatnonzero(bad_header)[0])
                    self._issue(
                        issues,
                        "FRAME_IDENTITY_MISMATCH",
                        "error",
                        "frame magic/schema/size/run identity mismatch",
                        offset=offset + BLOCK_HEADER_BYTES + first_bad * LOG_FRAME_BYTES,
                        frame_sequence=int(sequences[first_bad]),
                    )

                finite_fields = [
                    "imu_rotation_rad",
                    "imu_rate_radps",
                    "battery_V",
                    "pose_x_m",
                    "pose_y_m",
                    "pose_theta_rad",
                    "body_vx_mps",
                    "body_vy_mps",
                    "body_omega_radps",
                ]
                for field in finite_fields:
                    invalid_numeric += int(np.count_nonzero(~np.isfinite(block_frames[field])))
                for side in ("left_motor", "right_motor"):
                    for field in (
                        "position_rad",
                        "velocity_radps",
                        "current_A",
                        "temperature_C",
                        "applied_voltage_V",
                    ):
                        invalid_numeric += int(np.count_nonzero(~np.isfinite(block_frames[side][field])))

                for seq_value, time_value in zip(sequences.tolist(), times.tolist(), strict=True):
                    seq = int(seq_value)
                    timestamp = int(time_value)
                    discontinuity = False
                    if expected_sequence is not None:
                        if seq == expected_sequence - 1:
                            duplicates += 1
                            discontinuity = True
                        elif seq != expected_sequence:
                            if seq > expected_sequence:
                                gaps += seq - expected_sequence
                            else:
                                duplicates += 1
                            discontinuity = True
                        if last_time is not None and timestamp <= last_time:
                            regressions += 1
                            discontinuity = True
                    if discontinuity and current_window_start is not None and current_window_end is not None:
                        usable_raw.append((current_window_start, current_window_end))
                        current_window_start = timestamp
                    elif current_window_start is None:
                        current_window_start = timestamp
                    current_window_end = timestamp
                    expected_sequence = seq + 1
                    if first_sequence is None:
                        first_sequence = seq
                        first_time = timestamp
                    last_sequence = seq
                    last_time = timestamp

        if current_window_start is not None and current_window_end is not None:
            usable_raw.append((current_window_start, current_window_end))
        frames = (
            np.concatenate(arrays)
            if load_frames and arrays
            else np.empty(0, dtype=log_frame_dtype)
        )
        producer_drops = _scalar(footer, "producer_drops") if footer is not None else 0

        if footer is not None:
            footer_checks = [
                (_scalar(footer, "total_frames") == total_frames, "footer frame count"),
                (_scalar(footer, "block_count") == valid_blocks, "footer block count"),
                (
                    first_sequence is None or _scalar(footer, "first_frame_sequence") == first_sequence,
                    "footer first sequence",
                ),
                (
                    last_sequence is None or _scalar(footer, "last_frame_sequence") == last_sequence,
                    "footer last sequence",
                ),
                (
                    first_time is None or _scalar(footer, "first_time_us") == first_time,
                    "footer first time",
                ),
                (
                    last_time is None or _scalar(footer, "last_time_us") == last_time,
                    "footer last time",
                ),
            ]
            for ok, name in footer_checks:
                if not ok:
                    self._issue(issues, "FOOTER_MISMATCH", "error", f"{name} does not match payload")
        else:
            self._issue(issues, "MISSING_FOOTER", "error", "recording is truncated or was not closed")
        if total_frames == 0:
            self._issue(issues, "NO_FRAMES", "error", "recording contains no recoverable frames")
        if gaps:
            self._issue(issues, "SEQUENCE_GAP", "error", f"{gaps} frame sequence value(s) missing")
        if duplicates:
            self._issue(issues, "DUPLICATE_OR_REORDERED", "error", f"{duplicates} duplicate/reordered frame(s)")
        if regressions:
            self._issue(issues, "TIME_REGRESSION", "error", f"{regressions} non-increasing timestamp(s)")
        if producer_drops:
            self._issue(issues, "PRODUCER_DROPS", "error", f"producer reported {producer_drops} dropped frame(s)")
        if invalid_numeric:
            self._issue(
                issues,
                "NONFINITE_VALUES",
                "warning",
                f"{invalid_numeric} non-finite scalar value(s); per-signal availability/quality gates apply",
            )

        hard_codes = {
            "BAD_HEADER_CRC",
            "BAD_MAGIC",
            "BAD_ENDIAN",
            "BAD_HEADER_SIZE",
            "BAD_FRAME_SIZE",
            "UNSUPPORTED_SCHEMA",
        }
        if any(issue.code in hard_codes for issue in issues):
            verdict = Verdict.FAIL
        elif any(issue.severity == "error" for issue in issues):
            verdict = Verdict.REPEAT
        elif issues:
            verdict = Verdict.CONDITIONAL_PASS
        else:
            verdict = Verdict.PASS

        origin = first_time or 0
        usable_windows = [
            ((start - origin) * 1e-6, (end - origin) * 1e-6)
            for start, end in usable_raw
            if end >= start
        ]
        report = IntegrityReport(
            path=str(self.path),
            verdict=verdict,
            complete=complete,
            identity=identity,
            file_bytes=file_size,
            valid_bytes=valid_bytes,
            valid_blocks=valid_blocks,
            recoverable_frames=total_frames,
            first_frame_sequence=first_sequence,
            last_frame_sequence=last_sequence,
            first_time_us=first_time,
            last_time_us=last_time,
            producer_drops=producer_drops,
            sequence_gaps=gaps,
            duplicate_sequences=duplicates,
            time_regressions=regressions,
            invalid_numeric_values=invalid_numeric,
            issues=issues,
            usable_windows_s=usable_windows,
        )
        return DecodedRecording(self.path, frames, report)

    def iter_blocks(self) -> Iterator[np.ndarray]:
        """Yield one verified payload block at a time without retaining the whole run."""
        report = self.read(load_frames=False).report
        if report.verdict == Verdict.FAIL:
            raise ValueError("; ".join(issue.message for issue in report.issues))
        with self.path.open("rb") as stream:
            stream.seek(FILE_HEADER_BYTES)
            while stream.tell() < report.valid_bytes:
                offset = stream.tell()
                magic_bytes = stream.read(4)
                if len(magic_bytes) < 4:
                    return
                magic = int.from_bytes(magic_bytes, "little")
                stream.seek(offset)
                if magic == FOOTER_MAGIC:
                    return
                if magic != BLOCK_MAGIC:
                    raise ValueError(f"invalid block magic at byte {offset}")
                block_bytes = stream.read(BLOCK_HEADER_BYTES)
                block = np.frombuffer(block_bytes, dtype=block_header_dtype, count=1)[0]
                if _crc32(block_bytes[:-4]) != _scalar(block, "header_crc32"):
                    raise ValueError(f"invalid block header CRC at byte {offset}")
                payload_bytes = _scalar(block, "payload_bytes")
                frame_count = _scalar(block, "frame_count")
                payload = stream.read(payload_bytes)
                if len(payload) != payload_bytes or _crc32(payload) != _scalar(
                    block, "payload_crc32"
                ):
                    raise ValueError(f"invalid block payload at byte {offset}")
                yield np.frombuffer(
                    payload, dtype=log_frame_dtype, count=frame_count
                ).copy()

    def frame_at(self, index: int) -> np.void:
        if index < 0:
            raise IndexError(index)
        remaining = index
        for block in self.iter_blocks():
            if remaining < len(block):
                return block[remaining]
            remaining -= len(block)
        raise IndexError(index)


def decode_recording(path: str | os.PathLike[str]) -> DecodedRecording:
    return V5LReader(path).read()


def flatten_frames(frames: np.ndarray) -> dict[str, np.ndarray]:
    """Return stable dot-named columns suitable for Parquet and analysis."""
    if len(frames) == 0:
        return {}
    columns: dict[str, np.ndarray] = {
        "time_us": frames["header"]["time_us"],
        "time_s": (frames["header"]["time_us"] - frames["header"]["time_us"][0]) * 1e-6,
        "sequence": frames["header"]["sequence"],
        "mode_epoch": frames["header"]["mode_epoch"],
        "run_id_hash": frames["header"]["run_id_hash"],
        "raw.imu_rotation_rad": frames["imu_rotation_rad"],
        "raw.imu_rate_radps": frames["imu_rate_radps"],
        "raw.battery_V": frames["battery_V"],
        "state.pose_x_m": frames["pose_x_m"],
        "state.pose_y_m": frames["pose_y_m"],
        "state.pose_theta_rad": frames["pose_theta_rad"],
        "state.body_vx_mps": frames["body_vx_mps"],
        "state.body_vy_mps": frames["body_vy_mps"],
        "state.body_omega_radps": frames["body_omega_radps"],
        "state.translation_quality": frames["translation_quality"],
        "state.heading_quality": frames["heading_quality"],
        "controller.left_x": frames["controller"]["left_x"],
        "controller.left_y": frames["controller"]["left_y"],
        "controller.right_x": frames["controller"]["right_x"],
        "controller.right_y": frames["controller"]["right_y"],
        "controller.buttons": frames["controller"]["buttons"],
        "controller.connected": frames["controller"]["connected"],
        "controller.enabled": frames["controller"]["enabled"],
        "controller.field_connected": frames["controller"]["field_connected"],
        "recording.session_sequence": frames["recording"]["session_sequence"],
        "recording.event_bits": frames["recording"]["event_bits"],
        "recording.state": frames["recording"]["state"],
        "recording.error": frames["recording"]["error"],
        "event.sequence": frames["system_event"]["event_sequence"],
        "event.bits": frames["system_event"]["event_bits"],
        "request.source": frames["request"]["source"],
        "request.payload_kind": frames["request"]["payload_kind"],
        "request.owner_id": frames["request"]["owner_id"],
        "request.owner_lease": frames["request"]["owner_lease"],
        "request.reject_bits": frames["request"]["reject_bits"],
        "request.time_us": frames["request"]["request_time_us"],
        "request.ttl_us": frames["request"]["ttl_us"],
        "request.forward": frames["request"]["forward"],
        "request.steering": frames["request"]["steering"],
        "request.vx_mps": frames["request"]["vx_mps"],
        "request.omega_radps": frames["request"]["omega_radps"],
        "request.left_V": frames["request"]["requested_left_V"],
        "request.right_V": frames["request"]["requested_right_V"],
        "actuator.unallocated_left_V": frames["actuator"]["unallocated_left_V"],
        "actuator.unallocated_right_V": frames["actuator"]["unallocated_right_V"],
        "actuator.allocated_left_V": frames["actuator"]["allocated_left_V"],
        "actuator.allocated_right_V": frames["actuator"]["allocated_right_V"],
        "actuator.final_left_V": frames["actuator"]["final_left_V"],
        "actuator.final_right_V": frames["actuator"]["final_right_V"],
        "actuator.derate_target": frames["actuator"]["derate_target"],
        "actuator.derate_applied": frames["actuator"]["derate_applied"],
        "actuator.final_motor_valid_mask": frames["actuator"]["final_motor_valid_mask"],
        "actuator.applied_limits": frames["actuator"]["applied_limits"],
        "actuator.write_reject_bits": frames["actuator"]["write_reject_bits"],
        "actuator.last_written_sequence": frames["actuator"]["last_written_sequence"],
        "actuator.write_attempted": frames["actuator"]["write_attempted"],
        "actuator.write_ok": frames["actuator"]["write_ok"],
        "timing.raw_dt_s": frames["timing"]["raw_dt_s"],
        "timing.math_dt_s": frames["timing"]["math_dt_s"],
        "timing.exec_s": frames["timing"]["exec_s"],
        "timing.jitter_s": frames["timing"]["jitter_s"],
        "timing.sensor_age_us": frames["timing"]["sensor_age_us"],
        "timing.request_age_us": frames["timing"]["request_age_us"],
        "timing.actuator_age_us": frames["timing"]["actuator_age_us"],
        "timing.consecutive_overruns": frames["timing"]["consecutive_overruns"],
        "timing.overrun_total": frames["timing"]["overrun_total"],
        "timing.ring_depth": frames["timing"]["ring_depth"],
        "timing.log_dropped_total": frames["timing"]["log_dropped_total"],
        "fault.active_bits": frames["fault"]["active_fault_bits"],
        "fault.latched_bits": frames["fault"]["latched_fault_bits"],
        "fault.enter_bits": frames["fault"]["enter_fault_bits"],
        "fault.exit_bits": frames["fault"]["exit_fault_bits"],
        "fault.affected_motor_mask": frames["fault"]["affected_motor_mask"],
        "fault.severity": frames["fault"]["severity"],
        "fault.safety_state": frames["fault"]["safety_state"],
        "trace.mapped_throttle": frames["trace"]["mapped_throttle"],
        "trace.mapped_turn": frames["trace"]["mapped_turn"],
        "trace.output_derate": frames["trace"]["output_derate"],
        "trace.request_age_us": frames["trace"]["request_age_us"],
        "trace.availability_bits": frames["trace"]["availability_bits"],
        "trace.arbitration_reject_bits": frames["trace"]["arbitration_reject_bits"],
        "trace.selected_request_sequence": frames["trace"]["selected_request_sequence"],
        "trace.ring_high_watermark": frames["trace"]["ring_high_watermark"],
        "trace.selected_source": frames["trace"]["selected_source"],
        "trace.stop_mode": frames["trace"]["stop_mode"],
        "trace.output_action": frames["trace"]["output_action"],
        "trace.drive_state": frames["trace"]["drive_state"],
        "trace.quick_turn_active": frames["trace"]["quick_turn_active"],
        "trace.request_selected": frames["trace"]["request_selected"],
        "trace.deadline_missed": frames["trace"]["deadline_missed"],
        "trace.competition_disabled": frames["trace"]["competition_disabled"],
        "trace.competition_autonomous": frames["trace"]["competition_autonomous"],
    }
    for side_name in ("left_motor", "right_motor"):
        prefix = "L" if side_name == "left_motor" else "R"
        for index in range(frames[side_name].shape[1]):
            motor = frames[side_name][:, index]
            label = f"motor.{prefix}{index + 1}"
            for field in (
                "smart_port",
                "api_ok_mask",
                "quality",
                "position_rad",
                "velocity_radps",
                "current_A",
                "temperature_C",
                "applied_voltage_V",
                "api_faults",
                "reject_bits",
            ):
                columns[f"{label}.{field}"] = motor[field]
            columns[f"actuator.{prefix}{index + 1}.final_voltage_V"] = frames["actuator"][
                "final_motor_voltage_V"
            ][:, index if prefix == "L" else index + 3]
    for index in range(frames["tracking"].shape[1]):
        tracking = frames["tracking"][:, index]
        for field in (
            "position_rad",
            "velocity_radps",
            "configured",
            "position_api_ok",
            "velocity_api_ok",
        ):
            columns[f"tracking.{index + 1}.{field}"] = tracking[field]
    return columns
