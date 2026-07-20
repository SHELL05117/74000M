from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq

from statusmonitor.models import AnalysisResult, IntegrityReport, SessionStatus, Verdict
from statusmonitor.protocol.schema31 import (
    TRACE_ACTUATOR_INTENT,
    TRACE_PID_TERMS,
    TRACE_POSE_STATE,
    TRACE_RAW_INPUTS,
    TRACE_VALIDATED_STATE,
)
from statusmonitor.protocol.v5l import V5LReader
from statusmonitor.repository import Repository
from statusmonitor.segmentation import segment_motion
from statusmonitor.version import ANALYSIS_VERSION

from .signals import (
    cross_spectral_metrics,
    filter_ab,
    finite_stats,
    frequency_metrics,
    periodic_fourier_series,
    robust_derivative,
)


def _as_float(columns: dict[str, np.ndarray], name: str) -> np.ndarray:
    return np.asarray(columns[name], dtype=float)


def _read_parquet(path: Path) -> dict[str, np.ndarray]:
    table = pq.read_table(path)
    return {
        name: table[name].combine_chunks().to_numpy(zero_copy_only=False)
        for name in table.column_names
    }


def _event_edges(time_s: np.ndarray, values: np.ndarray, kind: str) -> list[dict]:
    values = np.asarray(values)
    changed = np.flatnonzero(np.r_[values[0] != 0, values[1:] != values[:-1]])
    return [
        {"time_s": float(time_s[index]), "kind": kind, "value": int(values[index])}
        for index in changed
        if int(values[index]) != 0
    ]


class AnalysisPipeline:
    def __init__(self, repository: Repository | None = None):
        self.repository = repository or Repository()

    def analyze(self, run_id: str) -> AnalysisResult:
        run = self.repository.get_run(run_id)
        session = self.repository.get_session(run.session_id)
        artifact_dir = Path(run.artifact_dir)
        streaming_report = V5LReader(run.archive_path).read(load_frames=False).report
        stored_report = IntegrityReport.model_validate_json(
            (artifact_dir / "integrity" / "integrity_report.json").read_text(
                encoding="utf-8"
            )
        )
        if streaming_report.verdict in {Verdict.FAIL, Verdict.REPEAT} and (
            streaming_report.verdict != stored_report.verdict
        ):
            raise ValueError(
                "raw archive integrity no longer matches the import-time report"
            )
        effective_verdict = stored_report.verdict
        columns = _read_parquet(artifact_dir / "derived" / "samples.parquet")
        if not columns:
            raise ValueError("recording has no recoverable frames")
        time_s = _as_float(columns, "time_s")
        availability = np.asarray(columns["trace.availability_bits"], dtype=np.uint32)
        available_union = int(np.bitwise_or.reduce(availability, initial=np.uint32(0)))
        pose_available = bool(available_union & TRACE_POSE_STATE)
        raw_available = bool(available_union & TRACE_RAW_INPUTS)
        validated_available = bool(available_union & TRACE_VALIDATED_STATE)
        pid_available = bool(available_union & TRACE_PID_TERMS)
        actuator_available = bool(available_union & TRACE_ACTUATOR_INTENT)

        left_velocity_radps = np.nanmean(
            np.column_stack(
                [_as_float(columns, f"motor.L{i}.velocity_radps") for i in range(1, 4)]
            ),
            axis=1,
        )
        right_velocity_radps = np.nanmean(
            np.column_stack(
                [_as_float(columns, f"motor.R{i}.velocity_radps") for i in range(1, 4)]
            ),
            axis=1,
        )
        if pose_available:
            primary_velocity = _as_float(columns, "state.body_vx_mps")
            primary_velocity_name = "state.body_vx_mps"
            primary_velocity_unit = "m/s"
        elif raw_available:
            primary_velocity = 0.5 * (left_velocity_radps + right_velocity_radps)
            primary_velocity_name = "derived.mean_drive_motor_velocity_radps"
            primary_velocity_unit = "rad/s (motor shaft; not chassis m/s)"
        else:
            primary_velocity = np.full(len(time_s), np.nan)
            primary_velocity_name = "NOT AVAILABLE"
            primary_velocity_unit = "NOT AVAILABLE"

        acceleration = robust_derivative(time_s, primary_velocity, order=1)
        jerk = robust_derivative(time_s, acceleration, order=1)
        left_voltage = (
            _as_float(columns, "actuator.final_left_V")
            if actuator_available
            else np.full(len(time_s), np.nan)
        )
        right_voltage = (
            _as_float(columns, "actuator.final_right_V")
            if actuator_available
            else np.full(len(time_s), np.nan)
        )
        segments = segment_motion(time_s, left_voltage, right_voltage, primary_velocity)

        timing = {
            "raw_dt_s": finite_stats(_as_float(columns, "timing.raw_dt_s")),
            "exec_s": finite_stats(_as_float(columns, "timing.exec_s")),
            "jitter_s": finite_stats(_as_float(columns, "timing.jitter_s")),
            "sensor_age_us": finite_stats(_as_float(columns, "timing.sensor_age_us")),
            "request_age_us": finite_stats(_as_float(columns, "timing.request_age_us")),
            "actuator_age_us": finite_stats(_as_float(columns, "timing.actuator_age_us")),
            "overrun_frames": int(
                np.count_nonzero(np.asarray(columns["trace.deadline_missed"], dtype=bool))
            ),
            "max_consecutive_overruns": int(
                np.max(columns["timing.consecutive_overruns"], initial=0)
            ),
            "ring_high_watermark": int(
                np.max(columns["trace.ring_high_watermark"], initial=0)
            ),
            "producer_drop_counter_max": int(
                np.max(columns["timing.log_dropped_total"], initial=0)
            ),
        }
        motor_metrics: dict[str, dict] = {}
        for label in [f"L{i}" for i in range(1, 4)] + [f"R{i}" for i in range(1, 4)]:
            motor_metrics[label] = {
                "smart_port": int(np.max(columns[f"motor.{label}.smart_port"], initial=0)),
                "velocity_radps": finite_stats(
                    _as_float(columns, f"motor.{label}.velocity_radps")
                ),
                "current_A": finite_stats(_as_float(columns, f"motor.{label}.current_A")),
                "temperature_C": finite_stats(
                    _as_float(columns, f"motor.{label}.temperature_C")
                ),
                "applied_voltage_V": finite_stats(
                    _as_float(columns, f"motor.{label}.applied_voltage_V")
                ),
                "api_fault_frames": int(
                    np.count_nonzero(columns[f"motor.{label}.api_faults"])
                ),
                "invalid_quality_frames": int(
                    np.count_nonzero(columns[f"motor.{label}.quality"] == 2)
                ),
            }
        left_spread = np.nanmax(
            np.column_stack(
                [_as_float(columns, f"motor.L{i}.velocity_radps") for i in range(1, 4)]
            ),
            axis=1,
        ) - np.nanmin(
            np.column_stack(
                [_as_float(columns, f"motor.L{i}.velocity_radps") for i in range(1, 4)]
            ),
            axis=1,
        )
        right_spread = np.nanmax(
            np.column_stack(
                [_as_float(columns, f"motor.R{i}.velocity_radps") for i in range(1, 4)]
            ),
            axis=1,
        ) - np.nanmin(
            np.column_stack(
                [_as_float(columns, f"motor.R{i}.velocity_radps") for i in range(1, 4)]
            ),
            axis=1,
        )

        events = (
            _event_edges(time_s, columns["recording.event_bits"], "recording")
            + _event_edges(time_s, columns["event.bits"], "system_event")
            + _event_edges(time_s, columns["fault.enter_bits"], "fault_enter")
            + _event_edges(time_s, columns["fault.exit_bits"], "fault_exit")
        )
        events.sort(key=lambda item: item["time_s"])

        anomalies: list[dict] = []
        for issue in stored_report.issues:
            anomalies.append(
                {
                    "id": f"INTEGRITY-{len(anomalies) + 1:03d}",
                    "layer": "logging/integrity",
                    "severity": issue.severity,
                    "summary": issue.message,
                    "evidence_window_s": None,
                    "confidence": "high",
                    "next_test": "repeat the recording if the affected window is required",
                }
            )
        if not pose_available:
            anomalies.append(
                {
                    "id": f"A-{len(anomalies) + 1:03d}",
                    "layer": "state availability",
                    "severity": "info",
                    "summary": "2D pose is NOT AVAILABLE in this firmware/configuration",
                    "evidence_window_s": [float(time_s[0]), float(time_s[-1])],
                    "confidence": "high",
                    "next_test": "commission and log a valid pose estimator before judging trajectory accuracy",
                }
            )
        deadline = np.flatnonzero(np.asarray(columns["trace.deadline_missed"], dtype=bool))
        if deadline.size:
            first = int(deadline[0])
            anomalies.append(
                {
                    "id": f"A-{len(anomalies) + 1:03d}",
                    "layer": "runtime timing",
                    "severity": "warning",
                    "summary": f"{deadline.size} control frame(s) reported a missed deadline",
                    "evidence_window_s": [
                        max(float(time_s[0]), float(time_s[first]) - 0.25),
                        min(float(time_s[-1]), float(time_s[first]) + 0.75),
                    ],
                    "confidence": "high",
                    "next_test": "repeat with logging on/off and inspect exec, jitter, and background load",
                }
            )
        failed_write = np.flatnonzero(
            np.asarray(columns["actuator.write_attempted"], dtype=bool)
            & ~np.asarray(columns["actuator.write_ok"], dtype=bool)
        )
        if failed_write.size:
            first = int(failed_write[0])
            anomalies.append(
                {
                    "id": f"A-{len(anomalies) + 1:03d}",
                    "layer": "output/HAL",
                    "severity": "error",
                    "summary": f"{failed_write.size} attempted actuator write(s) were not successful",
                    "evidence_window_s": [
                        max(float(time_s[0]), float(time_s[first]) - 0.25),
                        min(float(time_s[-1]), float(time_s[first]) + 0.75),
                    ],
                    "confidence": "high",
                    "next_test": "inspect affected ports, API status, and OutputService evidence before tuning",
                }
            )

        frequency = frequency_metrics(time_s, primary_velocity)
        cross_spectral = cross_spectral_metrics(
            time_s, 0.5 * (left_voltage + right_voltage), primary_velocity
        )
        filters = filter_ab(time_s, primary_velocity)
        periodic_requested = any(
            token in session.test_type.lower()
            for token in ("circle", "figure", "8", "periodic", "圆", "字")
        )
        fourier = (
            {
                "x": periodic_fourier_series(time_s, _as_float(columns, "state.pose_x_m")),
                "y": periodic_fourier_series(time_s, _as_float(columns, "state.pose_y_m")),
                "declared_periodic_test": True,
            }
            if pose_available and periodic_requested
            else {
                "available": False,
                "reason": (
                    "Fourier trajectory analysis requires valid pose and an explicitly "
                    "declared periodic/circle/figure-8 test"
                ),
            }
        )
        metrics = {
            "duration_s": float(time_s[-1] - time_s[0]),
            "samples": len(time_s),
            "sampling": {
                "nominal_rate_Hz": 100.0,
                "observed_dt_s": finite_stats(np.diff(time_s)),
                "uses_robot_monotonic_timestamp": True,
            },
            "motion": {
                "velocity_source": primary_velocity_name,
                "velocity_unit": primary_velocity_unit,
                "velocity": finite_stats(primary_velocity),
                "acceleration": finite_stats(acceleration),
                "jerk": finite_stats(jerk),
                "segments": [segment.as_dict() for segment in segments],
            },
            "pose": (
                {
                    "available": True,
                    "x_m": finite_stats(_as_float(columns, "state.pose_x_m")),
                    "y_m": finite_stats(_as_float(columns, "state.pose_y_m")),
                    "theta_rad": finite_stats(_as_float(columns, "state.pose_theta_rad")),
                }
                if pose_available
                else {"available": False, "reason": "TRACE_POSE_STATE availability bit is false"}
            ),
            "pid": (
                {"available": False, "reason": "schema 3.1 has no decoded PID term payload"}
                if not pid_available
                else {"available": False, "reason": "PID availability asserted but schema adapter lacks fields"}
            ),
            "motor": {
                "available": raw_available,
                "per_motor": motor_metrics,
                "left_velocity_spread_radps": finite_stats(left_spread),
                "right_velocity_spread_radps": finite_stats(right_spread),
            },
            "energy": {
                "available": raw_available or actuator_available,
                "battery_V": finite_stats(_as_float(columns, "raw.battery_V")),
                "final_left_V": finite_stats(left_voltage),
                "final_right_V": finite_stats(right_voltage),
                "derate_applied": finite_stats(
                    _as_float(columns, "actuator.derate_applied")
                ),
                "saturation_requires_config_limit": True,
            },
            "timing": timing,
            "frequency": frequency,
            "cross_spectral": cross_spectral,
            "trajectory_fourier": fourier,
            "filter_ab": filters,
            "sysid": {
                "available": False,
                "reason": (
                    "schema 3.1 does not provide calibrated per-side linear velocity plus "
                    "explicit training/validation phase labels"
                ),
                "required_model": "V = kS*sign(v) + kV*v + kA*a",
                "status_if_available": "Draft only",
            },
        }
        restrictions = [
            "No real-machine/HIL conclusion is generated by the PC program.",
            "Automatic recommendations are hypotheses and are never written to robot configuration.",
        ]
        if effective_verdict in {Verdict.REPEAT, Verdict.FAIL}:
            restrictions.append("Integrity hard gate failed; performance and parameter approval are prohibited.")
        if not pose_available:
            restrictions.append("Pose is unavailable; trajectory accuracy and path tracking PASS are prohibited.")
        if not pid_available:
            restrictions.append("PID terms are unavailable; PID overshoot/settling attribution is NOT AVAILABLE.")

        plots = self._generate_plots(
            artifact_dir,
            run_id,
            columns,
            primary_velocity,
            primary_velocity_unit,
            acceleration,
            jerk,
            frequency,
            pose_available,
            raw_available,
            actuator_available,
        )
        methods = {
            "derivative": {
                "algorithm": "per-continuous-window Savitzky-Golay",
                "window_s": 0.11,
                "polyorder": 3,
                "uses_true_timestamps": True,
                "offline_only": True,
            },
            "segmentation": "request/output heuristic v1 with 150 ms hysteresis dilation",
            "frequency": "uniform median-dt resample + Hann Welch PSD",
            "gaps": "segments split at non-increasing time or dt > 3x median",
        }
        result = AnalysisResult(
            run_id=run_id,
            analysis_version=ANALYSIS_VERSION,
            integrity_verdict=effective_verdict,
            capability={
                "raw_inputs": raw_available,
                "validated_state": validated_available,
                "pose": pose_available,
                "pid_terms": pid_available,
                "actuator_intent": actuator_available,
            },
            metrics=metrics,
            anomalies=anomalies,
            events=events,
            signal_sources={
                "primary_velocity": primary_velocity_name,
                "primary_acceleration": f"derived from {primary_velocity_name}",
                "primary_jerk": "derived from filtered acceleration",
            },
            methods=methods,
            plot_paths=plots,
            restrictions=restrictions,
        )
        derived_dir = artifact_dir / "derived"
        derived_dir.mkdir(parents=True, exist_ok=True)
        (derived_dir / "metrics.json").write_text(
            json.dumps(metrics, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        (derived_dir / "events.ndjson").write_text(
            "".join(json.dumps(event, ensure_ascii=False) + "\n" for event in events),
            encoding="utf-8",
        )
        (derived_dir / "analysis_manifest.json").write_text(
            result.model_dump_json(indent=2), encoding="utf-8"
        )
        (artifact_dir / "gui_summary.json").write_text(
            json.dumps(self._gui_summary(result), indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        self._write_evidence_windows(artifact_dir, columns, anomalies)
        self.repository.save_analysis(result)
        session = self.repository.get_session(run.session_id)
        session.status = SessionStatus.ANALYZED
        self.repository.save_session(session)
        self.repository.append_audit(
            artifact_dir,
            "analyze",
            {"analysis_version": ANALYSIS_VERSION, "anomalies": len(anomalies)},
        )
        from statusmonitor.reports.llm import ReportGenerator

        ReportGenerator(self.repository).generate(run_id)
        return result

    @staticmethod
    def _gui_summary(result: AnalysisResult) -> dict:
        timing = result.metrics["timing"]
        energy = result.metrics["energy"]
        frequency = result.metrics["frequency"]
        return {
            "run_id": result.run_id,
            "integrity": result.integrity_verdict.value,
            "pose": "AVAILABLE" if result.capability["pose"] else "NOT AVAILABLE",
            "pid": "AVAILABLE" if result.capability["pid_terms"] else "NOT AVAILABLE",
            "duration_s": result.metrics["duration_s"],
            "samples": result.metrics["samples"],
            "battery_min_V": energy["battery_V"]["min"],
            "exec_p99_ms": (
                timing["exec_s"]["p99"] * 1000 if timing["exec_s"]["p99"] is not None else None
            ),
            "deadline_missed_frames": timing["overrun_frames"],
            "dominant_frequency_Hz": (
                frequency.get("dominant_frequency_Hz") if frequency.get("available") else None
            ),
            "anomaly_count": len(result.anomalies),
            "restrictions": result.restrictions,
        }

    @staticmethod
    def _write_evidence_windows(
        artifact_dir: Path, columns: dict[str, np.ndarray], anomalies: list[dict]
    ) -> None:
        evidence_dir = artifact_dir / "llm" / "evidence"
        evidence_dir.mkdir(parents=True, exist_ok=True)
        time_s = np.asarray(columns["time_s"], dtype=float)
        selected = [
            "time_s",
            "sequence",
            "raw.battery_V",
            "request.forward",
            "request.steering",
            "actuator.final_left_V",
            "actuator.final_right_V",
            "timing.raw_dt_s",
            "timing.exec_s",
            "fault.active_bits",
        ]
        for anomaly in anomalies:
            window = anomaly.get("evidence_window_s")
            if not window:
                continue
            mask = (time_s >= window[0]) & (time_s <= window[1])
            if not np.any(mask):
                continue
            path = evidence_dir / f"{anomaly['id']}_{window[0]:.3f}-{window[1]:.3f}s.csv"
            with path.open("w", encoding="utf-8", newline="") as stream:
                stream.write(",".join(selected) + "\n")
                indices = np.flatnonzero(mask)
                for index in indices:
                    stream.write(
                        ",".join(str(columns[name][index]) for name in selected) + "\n"
                    )

    def _generate_plots(
        self,
        artifact_dir: Path,
        run_id: str,
        columns: dict[str, np.ndarray],
        velocity: np.ndarray,
        velocity_unit: str,
        acceleration: np.ndarray,
        jerk: np.ndarray,
        frequency: dict,
        pose_available: bool,
        raw_available: bool,
        actuator_available: bool,
    ) -> list[str]:
        from statusmonitor.plotting.static import generate_standard_plots

        return generate_standard_plots(
            artifact_dir / "plots",
            run_id,
            columns,
            velocity,
            velocity_unit,
            acceleration,
            jerk,
            frequency,
            pose_available,
            raw_available,
            actuator_available,
        )
