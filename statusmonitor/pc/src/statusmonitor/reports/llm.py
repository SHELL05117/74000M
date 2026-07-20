from __future__ import annotations

import hashlib
import json
import shutil
from pathlib import Path

from jinja2 import Template

from statusmonitor.repository import Repository
from statusmonitor.version import REPORT_SCHEMA


REPORT_TEMPLATE = Template(
    """---
report_schema: {{ report_schema }}
session_id: {{ session.session_id }}
run_id: {{ run.run_id }}
robot_id: {{ run.identity.robot_id }}
software_commit: {{ run.identity.source_commit }}
config_hash: "0x{{ "%08x"|format(run.identity.config_hash) }}"
log_schema: "{{ run.identity.log_schema_major }}.{{ run.identity.log_schema_minor }}"
analysis_version: {{ analysis.analysis_version }}
integrity_verdict: {{ analysis.integrity_verdict.value }}
created_by: OpenAI GPT-5.6 (Codex)
---

# Executive conclusion

Integrity: **{{ analysis.integrity_verdict.value }}**. Pose:
**{{ "AVAILABLE" if analysis.capability.pose else "NOT AVAILABLE" }}**. PID terms:
**{{ "AVAILABLE" if analysis.capability.pid_terms else "NOT AVAILABLE" }}**.
This report is an evidence index; the immutable V5L2 file and Parquet table are
the complete per-frame truth source.

# Scope and test question

- Team: {{ session.team_number }}
- Test type / case: {{ session.test_type }} / {{ session.test_case_id or "unspecified" }}
- Dataset role: {{ session.dataset_role.value }}
- Primary variable: {{ session.primary_variable or "unspecified" }}
- Target: {{ session.target or "unspecified" }}

# Identity and environment

- Robot identity from log: `{{ run.identity.robot_id }}`
- PC team identity: `{{ session.team_number }}` (does not overwrite robot identity)
- Operator / observer / analyst: {{ session.operator }} / {{ session.observer or "N/A" }} / {{ session.analyst or "N/A" }}
- Software: `{{ run.identity.software_version }}` at `{{ run.identity.source_commit }}`
- Dirty build: `{{ run.identity.dirty_build }}`
- Hardware/config/calibration revision: {{ run.identity.hardware_revision }} /
  {{ run.identity.config_schema }} / {{ run.identity.calibration_revision }}
- Surface/location: {{ session.surface or "unspecified" }} / {{ session.location or "unspecified" }}
- Battery/payload/tire state: {{ session.battery_id or "unspecified" }} /
  {{ session.payload or "unspecified" }} / {{ session.tire_state or "unspecified" }}

# Data integrity and usable windows

- Verdict: **{{ integrity.verdict.value }}**
- Complete footer: {{ integrity.complete }}
- Frames / blocks: {{ integrity.recoverable_frames }} / {{ integrity.valid_blocks }}
- Sequence gaps / duplicates / time regressions: {{ integrity.sequence_gaps }} /
  {{ integrity.duplicate_sequences }} / {{ integrity.time_regressions }}
- Producer drops: {{ integrity.producer_drops }}
- Usable windows [s]: `{{ integrity.usable_windows_s }}`
{% for issue in integrity.issues %}
- {{ issue.severity|upper }} `{{ issue.code }}`: {{ issue.message }}
{% endfor %}

# Signal dictionary and units

See [data_dictionary.md](data_dictionary.md). Primary motion signal:
`{{ analysis.signal_sources.primary_velocity }}`. All timing uses the robot's
monotonic `time_us`; nominal 100 Hz is never substituted for actual timestamps.

# Timeline and automatic segmentation

See [timeline.md](timeline.md) and [events.ndjson](events.ndjson).
{% for segment in analysis.metrics.motion.segments %}
- {{ "%.3f"|format(segment.start_s) }}–{{ "%.3f"|format(segment.end_s) }} s:
  {{ segment.label }} ({{ segment.source }})
{% endfor %}

# Control/PID metrics

{% if analysis.metrics.pid.available %}
PID evidence is present in `metrics.json`.
{% else %}
**NOT AVAILABLE:** {{ analysis.metrics.pid.reason }}. No PID tuning conclusion is permitted.
{% endif %}

# Trajectory and motion metrics

- Duration / samples: {{ "%.6f"|format(analysis.metrics.duration_s) }} s /
  {{ analysis.metrics.samples }}
- Velocity source/unit: {{ analysis.metrics.motion.velocity_source }} /
  {{ analysis.metrics.motion.velocity_unit }}
- Velocity RMS: {{ analysis.metrics.motion.velocity.rms }}
- Acceleration p99 / peak absolute: {{ analysis.metrics.motion.acceleration.p99 }} /
  {{ analysis.metrics.motion.acceleration.max_abs }}
- Jerk p99 / peak absolute: {{ analysis.metrics.motion.jerk.p99 }} /
  {{ analysis.metrics.motion.jerk.max_abs }}
- Pose: {{ "available" if analysis.metrics.pose.available else analysis.metrics.pose.reason }}

# Motor, battery, thermal and timing metrics

- Battery minimum: {{ analysis.metrics.energy.battery_V.min }} V
- Control exec p99: {{ analysis.metrics.timing.exec_s.p99 }} s
- Raw dt p99: {{ analysis.metrics.timing.raw_dt_s.p99 }} s
- Deadline-missed frames: {{ analysis.metrics.timing.overrun_frames }}
- Ring high-water mark: {{ analysis.metrics.timing.ring_high_watermark }}
- Producer drop counter max: {{ analysis.metrics.timing.producer_drop_counter_max }}

Full per-motor current, temperature, velocity, voltage and validity statistics
are in [metrics.json](metrics.json).

# Frequency and filter analysis

{% if analysis.metrics.frequency.available %}
- Welch dominant frequency: {{ analysis.metrics.frequency.dominant_frequency_Hz }} Hz
- Sample rate / Nyquist / resolution: {{ analysis.metrics.frequency.sample_rate_Hz }} /
  {{ analysis.metrics.frequency.nyquist_Hz }} /
  {{ analysis.metrics.frequency.resolution_Hz }} Hz
- This is PSD frequency content, **not** an FRF/Bode measurement.
{% else %}
Frequency analysis NOT AVAILABLE: {{ analysis.metrics.frequency.reason }}
{% endif %}

{% if analysis.metrics.cross_spectral.available %}
- Maximum observational coherence: {{ analysis.metrics.cross_spectral.max_coherence }}
  at {{ analysis.metrics.cross_spectral.max_coherence_frequency_Hz }} Hz.
{% else %}
- Cross-spectral/coherence analysis NOT AVAILABLE:
  {{ analysis.metrics.cross_spectral.reason }}
{% endif %}

STFT uses a compact dominant-frequency track in `metrics.json`; the full complex
matrix is not duplicated into Markdown. Periodic trajectory Fourier analysis is
gated by both valid pose and an explicitly declared circle/figure-8/periodic test:
`{{ analysis.metrics.trajectory_fourier }}`.

Filter A/B results are in `metrics.json`. Offline zero-phase/symmetric results
are explicitly not presented as deployable real-time filters.

# Anomalies

{% for anomaly in analysis.anomalies %}
## {{ anomaly.id }} — {{ anomaly.summary }}

- Layer: {{ anomaly.layer }}
- Severity / confidence: {{ anomaly.severity }} / {{ anomaly.confidence }}
- Evidence window [s]: {{ anomaly.evidence_window_s }}
- Recommended next test: {{ anomaly.next_test }}
{% else %}
No automatic anomalies were raised. This does not constitute HIL or field approval.
{% endfor %}

# Parameter candidates

SysId status: **{{ "AVAILABLE" if analysis.metrics.sysid.available else "NOT AVAILABLE" }}**.
{{ analysis.metrics.sysid.reason if not analysis.metrics.sysid.available else
   "Any fit is Draft only and must use separate training/validation data." }}
The application never writes parameters to the robot.

# Restrictions and NOT TESTED items

{% for restriction in analysis.restrictions %}
- {{ restriction }}
{% endfor %}

# Artifact manifest and SHA-256

See [artifact_manifest.json](artifact_manifest.json). The immutable raw source
hash is `{{ run.sha256 }}`. Query the full per-frame data from
`../derived/samples.parquet`; use evidence CSVs only as focused context windows.

# Suggested machine queries

1. Identify the earliest abnormal layer before proposing a PID change.
2. Compare left/right motor validity, current, speed and temperature around each anomaly.
3. Check saturation, derating, battery and timing before attributing oscillation to gains.
4. State which signals are unavailable and do not infer them from zeros.
5. Propose one controlled repeat test and one variable change at a time.
"""
)


def _hash(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(4 * 1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


class ReportGenerator:
    def __init__(self, repository: Repository | None = None):
        self.repository = repository or Repository()

    def generate(self, run_id: str) -> Path:
        run = self.repository.get_run(run_id)
        session = self.repository.get_session(run.session_id)
        analysis = self.repository.get_analysis(run_id)
        artifact_dir = Path(run.artifact_dir)
        integrity_path = artifact_dir / "integrity" / "integrity_report.json"
        from statusmonitor.models import IntegrityReport

        integrity = IntegrityReport.model_validate_json(
            integrity_path.read_text(encoding="utf-8")
        )
        llm_dir = artifact_dir / "llm"
        llm_dir.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(artifact_dir / "derived" / "metrics.json", llm_dir / "metrics.json")
        shutil.copyfile(artifact_dir / "derived" / "events.ndjson", llm_dir / "events.ndjson")
        (llm_dir / "data_dictionary.md").write_text(
            self._data_dictionary(analysis, artifact_dir), encoding="utf-8"
        )
        (llm_dir / "timeline.md").write_text(self._timeline(analysis), encoding="utf-8")
        report_path = artifact_dir / "report_for_llm.md"
        report_path.write_text(
            REPORT_TEMPLATE.render(
                report_schema=REPORT_SCHEMA,
                session=session,
                run=run,
                analysis=analysis,
                integrity=integrity,
            ),
            encoding="utf-8",
        )
        manifest_entries = []
        for path in sorted(artifact_dir.rglob("*")):
            if (
                not path.is_file()
                or path.name == "artifact_manifest.json"
                or path.name == "audit.jsonl"
            ):
                continue
            manifest_entries.append(
                {
                    "path": path.relative_to(artifact_dir).as_posix(),
                    "bytes": path.stat().st_size,
                    "sha256": _hash(path),
                }
            )
        manifest = {
            "run_id": run_id,
            "analysis_version": analysis.analysis_version,
            "raw_sha256": run.sha256,
            "artifacts": manifest_entries,
            "note": (
                "manifest intentionally does not hash itself or the append-only "
                "audit.jsonl, whose entries carry their own timestamps"
            ),
        }
        (llm_dir / "artifact_manifest.json").write_text(
            json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        self.repository.append_audit(
            artifact_dir,
            "report",
            {"report_schema": REPORT_SCHEMA, "artifact_count": len(manifest_entries)},
        )
        return report_path

    @staticmethod
    def _data_dictionary(analysis, artifact_dir: Path) -> str:
        import pyarrow.parquet as pq

        schema = pq.read_schema(artifact_dir / "derived" / "samples.parquet")

        def unit(name: str) -> str:
            lower = name.lower()
            if lower.endswith("_us") or ".time_us" in lower:
                return "µs"
            if lower.endswith("_s") or lower == "time_s":
                return "s"
            if lower.endswith("_radps"):
                return "rad/s"
            if lower.endswith("_rad"):
                return "rad"
            if lower.endswith("_mps"):
                return "m/s"
            if lower.endswith("_m"):
                return "m"
            if lower.endswith("_v"):
                return "V"
            if lower.endswith("_a"):
                return "A"
            if lower.endswith("_c"):
                return "°C"
            if "quality" in lower:
                return "enum: Good=0, Degraded=1, Invalid=2"
            if "bits" in lower or "mask" in lower or "flags" in lower:
                return "schema-bound bitset"
            if lower in {"sequence", "mode_epoch", "run_id_hash"}:
                return "integer identity/order"
            return "dimensionless / enum / count (see field name)"

        rows = "\n".join(
            f"| `{field.name}` | `{field.type}` | {unit(field.name)} |"
            for field in schema
        )
        return f"""# Data dictionary

Analysis version: `{analysis.analysis_version}`.

The table enumerates every Parquet column in this run. `availability_bits` and
quality fields remain authoritative; a stored numeric zero is not automatically
a valid measurement.

| Column | Parquet type | Unit / semantics |
|---|---|---|
{rows}

Primary velocity: `{analysis.signal_sources["primary_velocity"]}`.
Derivative method: `{analysis.methods["derivative"]}`.
"""

    @staticmethod
    def _timeline(analysis) -> str:
        lines = ["# Timeline", ""]
        for segment in analysis.metrics["motion"]["segments"]:
            lines.append(
                f"- {segment['start_s']:.6f}–{segment['end_s']:.6f} s: "
                f"{segment['label']} ({segment['source']})"
            )
        lines.extend(["", "## Events", ""])
        for event in analysis.events:
            lines.append(
                f"- {event['time_s']:.6f} s: {event['kind']} = 0x{event['value']:x}"
            )
        if not analysis.events:
            lines.append("- No non-zero event edges decoded.")
        return "\n".join(lines) + "\n"
