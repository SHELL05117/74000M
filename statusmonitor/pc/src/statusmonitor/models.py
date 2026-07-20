from __future__ import annotations

from datetime import datetime, timezone
from enum import StrEnum
from pathlib import Path
from typing import Any
from uuid import uuid4

from pydantic import BaseModel, ConfigDict, Field, field_validator


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


class Verdict(StrEnum):
    PASS = "PASS"
    CONDITIONAL_PASS = "CONDITIONAL PASS"
    REPEAT = "REPEAT"
    FAIL = "FAIL"
    NOT_TESTED = "NOT TESTED"


class DatasetRole(StrEnum):
    EXPLORATORY = "exploratory"
    TRAINING = "training"
    VALIDATION = "validation"
    ACCEPTANCE = "acceptance"


class SessionStatus(StrEnum):
    DRAFT = "draft"
    READY = "ready"
    IMPORTED = "imported"
    ANALYZED = "analyzed"
    ARCHIVED = "archived"


class SessionMetadata(BaseModel):
    model_config = ConfigDict(extra="forbid")

    session_id: str = Field(default_factory=lambda: f"S-{utc_now():%Y%m%dT%H%M%SZ}-{uuid4().hex[:8]}")
    created_at: datetime = Field(default_factory=utc_now)
    updated_at: datetime = Field(default_factory=utc_now)
    status: SessionStatus = SessionStatus.DRAFT
    team_number: str
    operator: str
    observer: str = ""
    analyst: str = ""
    test_case_id: str = ""
    test_type: str
    direction: str = ""
    repetition_index: int = 1
    target: str = ""
    primary_variable: str = ""
    dataset_role: DatasetRole = DatasetRole.EXPLORATORY
    surface: str = ""
    location: str = ""
    ambient_temperature_C: float | None = None
    payload: str = ""
    battery_id: str = ""
    tire_state: str = ""
    expected_robot_id: str = ""
    notes: str = ""

    @field_validator("team_number", "operator", "test_type")
    @classmethod
    def required_text(cls, value: str) -> str:
        value = value.strip()
        if not value:
            raise ValueError("must not be empty")
        return value

    @field_validator("repetition_index")
    @classmethod
    def positive_repetition(cls, value: int) -> int:
        if value < 1:
            raise ValueError("must be at least 1")
        return value


class RobotIdentity(BaseModel):
    robot_id: str
    robot_id_hash: int
    software_version: str
    source_commit: str
    dirty_build: bool
    config_hash: int
    software_identity_hash: int
    hardware_revision: int
    config_schema: int
    calibration_revision: int
    hardware_verification: int
    log_schema_major: int
    log_schema_minor: int
    frame_size_bytes: int
    boot_id: int
    session_sequence: int
    storage_sequence: int
    start_time_ms: int
    run_id_hash: int


class IntegrityIssue(BaseModel):
    code: str
    severity: str
    message: str
    offset: int | None = None
    frame_sequence: int | None = None


class IntegrityReport(BaseModel):
    path: str
    verdict: Verdict
    complete: bool
    identity: RobotIdentity | None = None
    file_bytes: int = 0
    valid_bytes: int = 0
    valid_blocks: int = 0
    recoverable_frames: int = 0
    first_frame_sequence: int | None = None
    last_frame_sequence: int | None = None
    first_time_us: int | None = None
    last_time_us: int | None = None
    producer_drops: int = 0
    sequence_gaps: int = 0
    duplicate_sequences: int = 0
    time_regressions: int = 0
    invalid_numeric_values: int = 0
    issues: list[IntegrityIssue] = Field(default_factory=list)
    usable_windows_s: list[tuple[float, float]] = Field(default_factory=list)


class RunRecord(BaseModel):
    run_id: str
    session_id: str
    imported_at: datetime = Field(default_factory=utc_now)
    source_path: str
    archive_path: str
    sha256: str
    source_bytes: int
    identity: RobotIdentity
    integrity_verdict: Verdict
    artifact_dir: str


class AnalysisResult(BaseModel):
    run_id: str
    analysis_version: str
    generated_at: datetime = Field(default_factory=utc_now)
    integrity_verdict: Verdict
    capability: dict[str, bool | str]
    metrics: dict[str, Any]
    anomalies: list[dict[str, Any]]
    events: list[dict[str, Any]]
    signal_sources: dict[str, str]
    methods: dict[str, Any]
    plot_paths: list[str]
    restrictions: list[str]


def dump_json(model: BaseModel, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(model.model_dump_json(indent=2), encoding="utf-8")
