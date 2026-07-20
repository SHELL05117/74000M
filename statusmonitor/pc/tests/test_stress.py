from __future__ import annotations

import os
from pathlib import Path

import pyarrow.parquet as pq
import pytest

from statusmonitor.models import SessionMetadata, Verdict
from statusmonitor.repository import Repository
from statusmonitor.settings import Settings
from statusmonitor.storage.archive import ImportService

from v5l_builder import write_long_v5l


@pytest.mark.stress
@pytest.mark.skipif(
    os.environ.get("STATUSMONITOR_RUN_STRESS") != "1",
    reason="set STATUSMONITOR_RUN_STRESS=1 to write and import a 60-minute 100 Hz recording",
)
def test_streaming_sixty_minute_import(isolated_home: Path, tmp_path: Path):
    frames = 60 * 60 * 100
    source = write_long_v5l(tmp_path / "sixty_minutes.V5L", frames)
    repo = Repository(Settings.load())
    session = repo.create_session(
        SessionMetadata(team_number="74000M", operator="stress", test_type="long-log")
    )
    run = ImportService(repo).import_recording(session.session_id, source)
    assert run.integrity_verdict == Verdict.PASS
    parquet = pq.ParquetFile(Path(run.artifact_dir) / "derived" / "samples.parquet")
    assert parquet.metadata.num_rows == frames
