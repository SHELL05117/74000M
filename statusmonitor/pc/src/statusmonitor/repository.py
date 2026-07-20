from __future__ import annotations

import json
import sqlite3
from collections.abc import Iterator
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path

from .models import AnalysisResult, RunRecord, SessionMetadata, SessionStatus
from .settings import Settings


class Repository:
    def __init__(self, settings: Settings | None = None):
        self.settings = settings or Settings.load()
        self.settings.database.parent.mkdir(parents=True, exist_ok=True)
        self._initialize()

    @contextmanager
    def connect(self) -> Iterator[sqlite3.Connection]:
        connection = sqlite3.connect(self.settings.database)
        connection.row_factory = sqlite3.Row
        connection.execute("PRAGMA foreign_keys = ON")
        connection.execute("PRAGMA journal_mode = WAL")
        try:
            yield connection
            connection.commit()
        finally:
            connection.close()

    def _initialize(self) -> None:
        with self.connect() as db:
            db.executescript(
                """
                CREATE TABLE IF NOT EXISTS sessions (
                    session_id TEXT PRIMARY KEY,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL,
                    team_number TEXT NOT NULL,
                    operator TEXT NOT NULL,
                    test_type TEXT NOT NULL,
                    status TEXT NOT NULL,
                    metadata_json TEXT NOT NULL
                );
                CREATE INDEX IF NOT EXISTS sessions_updated
                    ON sessions(updated_at DESC);

                CREATE TABLE IF NOT EXISTS runs (
                    run_id TEXT PRIMARY KEY,
                    session_id TEXT NOT NULL REFERENCES sessions(session_id),
                    imported_at TEXT NOT NULL,
                    robot_id TEXT NOT NULL,
                    sha256 TEXT NOT NULL UNIQUE,
                    integrity_verdict TEXT NOT NULL,
                    artifact_dir TEXT NOT NULL,
                    record_json TEXT NOT NULL
                );
                CREATE INDEX IF NOT EXISTS runs_session
                    ON runs(session_id, imported_at DESC);

                CREATE TABLE IF NOT EXISTS analyses (
                    run_id TEXT PRIMARY KEY REFERENCES runs(run_id),
                    generated_at TEXT NOT NULL,
                    analysis_version TEXT NOT NULL,
                    result_json TEXT NOT NULL
                );

                CREATE TABLE IF NOT EXISTS templates (
                    template_name TEXT PRIMARY KEY,
                    metadata_json TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                );
                """
            )

    def create_session(self, session: SessionMetadata) -> SessionMetadata:
        with self.connect() as db:
            db.execute(
                """
                INSERT INTO sessions
                    (session_id, created_at, updated_at, team_number, operator,
                     test_type, status, metadata_json)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    session.session_id,
                    session.created_at.isoformat(),
                    session.updated_at.isoformat(),
                    session.team_number,
                    session.operator,
                    session.test_type,
                    session.status.value,
                    session.model_dump_json(),
                ),
            )
        return session

    def save_session(self, session: SessionMetadata) -> None:
        session.updated_at = datetime.now(timezone.utc)
        with self.connect() as db:
            result = db.execute(
                """
                UPDATE sessions
                SET updated_at=?, team_number=?, operator=?, test_type=?,
                    status=?, metadata_json=?
                WHERE session_id=?
                """,
                (
                    session.updated_at.isoformat(),
                    session.team_number,
                    session.operator,
                    session.test_type,
                    session.status.value,
                    session.model_dump_json(),
                    session.session_id,
                ),
            )
            if result.rowcount != 1:
                raise KeyError(session.session_id)

    def get_session(self, session_id: str) -> SessionMetadata:
        with self.connect() as db:
            row = db.execute(
                "SELECT metadata_json FROM sessions WHERE session_id=?", (session_id,)
            ).fetchone()
        if row is None:
            raise KeyError(session_id)
        return SessionMetadata.model_validate_json(row["metadata_json"])

    def list_sessions(self, limit: int = 200) -> list[SessionMetadata]:
        with self.connect() as db:
            rows = db.execute(
                "SELECT metadata_json FROM sessions ORDER BY updated_at DESC LIMIT ?", (limit,)
            ).fetchall()
        return [SessionMetadata.model_validate_json(row["metadata_json"]) for row in rows]

    def save_template(self, name: str, session: SessionMetadata) -> None:
        template = session.model_copy(
            update={
                "session_id": "TEMPLATE",
                "status": SessionStatus.DRAFT,
            }
        )
        with self.connect() as db:
            db.execute(
                """
                INSERT INTO templates(template_name, metadata_json, updated_at)
                VALUES (?, ?, ?)
                ON CONFLICT(template_name) DO UPDATE SET
                    metadata_json=excluded.metadata_json,
                    updated_at=excluded.updated_at
                """,
                (name, template.model_dump_json(), datetime.now(timezone.utc).isoformat()),
            )

    def list_templates(self) -> dict[str, SessionMetadata]:
        with self.connect() as db:
            rows = db.execute(
                "SELECT template_name, metadata_json FROM templates ORDER BY template_name"
            ).fetchall()
        return {
            row["template_name"]: SessionMetadata.model_validate_json(row["metadata_json"])
            for row in rows
        }

    def add_run(self, run: RunRecord) -> None:
        with self.connect() as db:
            db.execute(
                """
                INSERT INTO runs
                    (run_id, session_id, imported_at, robot_id, sha256,
                     integrity_verdict, artifact_dir, record_json)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    run.run_id,
                    run.session_id,
                    run.imported_at.isoformat(),
                    run.identity.robot_id,
                    run.sha256,
                    run.integrity_verdict.value,
                    run.artifact_dir,
                    run.model_dump_json(),
                ),
            )

    def get_run(self, run_id: str) -> RunRecord:
        with self.connect() as db:
            row = db.execute("SELECT record_json FROM runs WHERE run_id=?", (run_id,)).fetchone()
        if row is None:
            raise KeyError(run_id)
        return RunRecord.model_validate_json(row["record_json"])

    def find_run_by_hash(self, sha256: str) -> RunRecord | None:
        with self.connect() as db:
            row = db.execute("SELECT record_json FROM runs WHERE sha256=?", (sha256,)).fetchone()
        return RunRecord.model_validate_json(row["record_json"]) if row else None

    def list_runs(self, session_id: str | None = None, limit: int = 500) -> list[RunRecord]:
        with self.connect() as db:
            if session_id:
                rows = db.execute(
                    "SELECT record_json FROM runs WHERE session_id=? ORDER BY imported_at DESC LIMIT ?",
                    (session_id, limit),
                ).fetchall()
            else:
                rows = db.execute(
                    "SELECT record_json FROM runs ORDER BY imported_at DESC LIMIT ?", (limit,)
                ).fetchall()
        return [RunRecord.model_validate_json(row["record_json"]) for row in rows]

    def save_analysis(self, result: AnalysisResult) -> None:
        with self.connect() as db:
            db.execute(
                """
                INSERT INTO analyses(run_id, generated_at, analysis_version, result_json)
                VALUES (?, ?, ?, ?)
                ON CONFLICT(run_id) DO UPDATE SET
                    generated_at=excluded.generated_at,
                    analysis_version=excluded.analysis_version,
                    result_json=excluded.result_json
                """,
                (
                    result.run_id,
                    result.generated_at.isoformat(),
                    result.analysis_version,
                    result.model_dump_json(),
                ),
            )

    def get_analysis(self, run_id: str) -> AnalysisResult:
        with self.connect() as db:
            row = db.execute(
                "SELECT result_json FROM analyses WHERE run_id=?", (run_id,)
            ).fetchone()
        if row is None:
            raise KeyError(run_id)
        return AnalysisResult.model_validate_json(row["result_json"])

    @staticmethod
    def append_audit(artifact_dir: Path, action: str, detail: dict) -> None:
        entry = {
            "time": datetime.now(timezone.utc).isoformat(),
            "action": action,
            "detail": detail,
        }
        artifact_dir.mkdir(parents=True, exist_ok=True)
        with (artifact_dir / "audit.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(entry, ensure_ascii=False, sort_keys=True) + "\n")
