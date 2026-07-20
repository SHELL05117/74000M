from __future__ import annotations

import hashlib
import json
import os
import shutil
import stat
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

from statusmonitor.models import IntegrityIssue, RunRecord, SessionStatus, Verdict
from statusmonitor.protocol.v5l import V5LReader, flatten_frames
from statusmonitor.repository import Repository


@dataclass(frozen=True)
class MediaCandidate:
    path: Path
    status: Verdict | str
    robot_id: str
    storage_sequence: int | None
    frames: int
    duration_s: float
    complete: bool


def sha256_file(path: Path, chunk_bytes: int = 4 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(chunk_bytes):
            digest.update(chunk)
    return digest.hexdigest()


def scan_recordings(root: str | os.PathLike[str]) -> list[MediaCandidate]:
    root_path = Path(root)
    paths = (
        [root_path]
        if root_path.is_file()
        else sorted(
            path
            for path in root_path.rglob("*")
            if path.is_file() and path.suffix.upper() in {".V5L", ".TMP"}
        )
    )
    result: list[MediaCandidate] = []
    for path in paths:
        try:
            decoded = V5LReader(path).read(load_frames=False)
            report = decoded.report
            duration_s = (
                (report.last_time_us - report.first_time_us) * 1e-6
                if report.first_time_us is not None and report.last_time_us is not None
                else 0.0
            )
            result.append(
                MediaCandidate(
                    path=path,
                    status=report.verdict,
                    robot_id=report.identity.robot_id if report.identity else "",
                    storage_sequence=(
                        report.identity.storage_sequence if report.identity else None
                    ),
                    frames=report.recoverable_frames,
                    duration_s=duration_s,
                    complete=report.complete,
                )
            )
        except (OSError, ValueError) as error:
            result.append(
                MediaCandidate(
                    path=path,
                    status=f"IO_ERROR: {error}",
                    robot_id="",
                    storage_sequence=None,
                    frames=0,
                    duration_s=0.0,
                    complete=False,
                )
            )
    return result


def _write_parquet_blocks(reader: V5LReader, path: Path) -> None:
    try:
        import pyarrow as pa
        import pyarrow.parquet as pq
    except ImportError as error:
        raise RuntimeError(
            "PyArrow is required for Parquet output. Install the project dependencies."
        ) from error
    path.parent.mkdir(parents=True, exist_ok=True)
    writer = None
    try:
        for block in reader.iter_blocks():
            table = pa.table(flatten_frames(block))
            if writer is None:
                writer = pq.ParquetWriter(
                    path,
                    table.schema,
                    compression="zstd",
                    use_dictionary=True,
                    write_statistics=True,
                )
            writer.write_table(table, row_group_size=100_000)
        if writer is None:
            raise ValueError("recording contains no recoverable frames for Parquet")
    finally:
        if writer is not None:
            writer.close()


class ImportService:
    def __init__(self, repository: Repository | None = None):
        self.repository = repository or Repository()

    def import_recording(self, session_id: str, source: str | os.PathLike[str]) -> RunRecord:
        session = self.repository.get_session(session_id)
        source_path = Path(source).resolve()
        source_hash = sha256_file(source_path)
        duplicate = self.repository.find_run_by_hash(source_hash)
        if duplicate is not None:
            if duplicate.session_id != session_id:
                raise ValueError(
                    f"recording is already archived as {duplicate.run_id} under "
                    f"session {duplicate.session_id}; explicit re-binding is required"
                )
            return duplicate

        decoded = V5LReader(source_path).read(load_frames=False)
        if decoded.report.identity is None:
            raise ValueError("recording has no readable identity header")
        identity = decoded.report.identity
        stamp = datetime.now(timezone.utc)
        short_hash = source_hash[:10]
        run_id = (
            f"R-{identity.boot_id:016x}-{identity.session_sequence:06d}-"
            f"{identity.storage_sequence:06d}-{short_hash}"
        )
        safe_team = "".join(c for c in session.team_number if c.isalnum() or c in "-_") or "UNKNOWN"
        safe_robot = "".join(c for c in identity.robot_id if c.isalnum() or c in "-_") or "UNKNOWN"
        artifact_dir = (
            self.repository.settings.artifacts
            / stamp.strftime("%Y-%m-%d")
            / safe_team
            / safe_robot
            / session.session_id
            / run_id
        )
        raw_dir = artifact_dir / "raw"
        integrity_dir = artifact_dir / "integrity"
        derived_dir = artifact_dir / "derived"
        raw_dir.mkdir(parents=True, exist_ok=False)
        archive_path = raw_dir / f"imported_segment_{identity.storage_sequence:06d}{source_path.suffix.lower()}"
        temp_path = archive_path.with_suffix(archive_path.suffix + ".partial")
        try:
            shutil.copyfile(source_path, temp_path)
            copied_hash = sha256_file(temp_path)
            if copied_hash != source_hash:
                raise IOError("archive hash differs from source after copy")
            os.replace(temp_path, archive_path)
            archive_path.chmod(stat.S_IREAD)

            identity_warning = None
            if session.expected_robot_id and session.expected_robot_id != identity.robot_id:
                identity_warning = (
                    f"session expected robot {session.expected_robot_id}, "
                    f"log reports {identity.robot_id}"
                )
                decoded.report.issues.append(
                    IntegrityIssue(
                        code="IDENTITY_MISMATCH",
                        severity="warning",
                        message=identity_warning,
                    )
                )
                if decoded.report.verdict == Verdict.PASS:
                    decoded.report.verdict = Verdict.CONDITIONAL_PASS

            integrity_dir.mkdir(parents=True, exist_ok=True)
            (integrity_dir / "integrity_report.json").write_text(
                decoded.report.model_dump_json(indent=2), encoding="utf-8"
            )
            (integrity_dir / "usable_windows.json").write_text(
                json.dumps(
                    {"windows_s": decoded.report.usable_windows_s},
                    indent=2,
                    ensure_ascii=False,
                ),
                encoding="utf-8",
            )
            (raw_dir / "original_hashes.json").write_text(
                json.dumps(
                    {
                        "source_path": str(source_path),
                        "archived_path": str(archive_path),
                        "sha256": source_hash,
                        "bytes": source_path.stat().st_size,
                    },
                    indent=2,
                    ensure_ascii=False,
                ),
                encoding="utf-8",
            )
            metadata = {
                "session": session.model_dump(mode="json"),
                "robot_identity": identity.model_dump(mode="json"),
                "imported_at": stamp.isoformat(),
                "source_sha256": source_hash,
                "identity_warning": identity_warning,
            }
            artifact_dir.mkdir(parents=True, exist_ok=True)
            (artifact_dir / "metadata.json").write_text(
                json.dumps(metadata, indent=2, ensure_ascii=False),
                encoding="utf-8",
            )
            _write_parquet_blocks(
                V5LReader(archive_path), derived_dir / "samples.parquet"
            )

            run = RunRecord(
                run_id=run_id,
                session_id=session_id,
                source_path=str(source_path),
                archive_path=str(archive_path),
                sha256=source_hash,
                source_bytes=source_path.stat().st_size,
                identity=identity,
                integrity_verdict=decoded.report.verdict,
                artifact_dir=str(artifact_dir),
            )
            self.repository.add_run(run)
            session.status = SessionStatus.IMPORTED
            self.repository.save_session(session)
            self.repository.append_audit(
                artifact_dir,
                "import",
                {
                    "source": str(source_path),
                    "sha256": source_hash,
                    "verdict": decoded.report.verdict.value,
                    "frames": decoded.report.recoverable_frames,
                },
            )
            return run
        except Exception:
            if temp_path.exists():
                temp_path.unlink()
            raise
