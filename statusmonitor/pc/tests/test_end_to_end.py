from __future__ import annotations

import json
from pathlib import Path

import pyarrow.parquet as pq

from statusmonitor.analysis.compare import compare_runs
from statusmonitor.analysis.pipeline import AnalysisPipeline
from statusmonitor.models import SessionMetadata, Verdict
from statusmonitor.repository import Repository
from statusmonitor.replay import replay_recorded_evidence
from statusmonitor.settings import Settings
from statusmonitor.storage.archive import ImportService

from v5l_builder import synthetic_frames, write_v5l


def make_session(repo: Repository, team: str = "74000M") -> SessionMetadata:
    return repo.create_session(
        SessionMetadata(team_number=team, operator="operator", test_type="manual")
    )


def test_import_analyze_report_end_to_end(isolated_home: Path, tmp_path: Path):
    settings = Settings.load()
    repo = Repository(settings)
    session = make_session(repo)
    source = write_v5l(tmp_path / "DATA.V5L", synthetic_frames(500))
    source_before = source.read_bytes()
    run = ImportService(repo).import_recording(session.session_id, source)

    assert source.read_bytes() == source_before
    assert Path(run.archive_path).exists()
    assert run.integrity_verdict == Verdict.PASS
    assert pq.read_table(Path(run.artifact_dir) / "derived" / "samples.parquet").num_rows == 500

    result = AnalysisPipeline(repo).analyze(run.run_id)
    assert not result.capability["pose"]
    assert result.metrics["pose"]["available"] is False
    assert len(result.plot_paths) == 11
    assert all(Path(path).exists() for path in result.plot_paths)
    report = Path(run.artifact_dir) / "report_for_llm.md"
    assert report.exists()
    text = report.read_text(encoding="utf-8")
    assert "created_by: OpenAI GPT-5.6 (Codex)" in text
    assert "NOT AVAILABLE" in text
    assert (Path(run.artifact_dir) / "llm" / "artifact_manifest.json").exists()
    replay = replay_recorded_evidence(run.run_id, 1.25, repo)
    assert replay["v5l_parquet_equivalent"]
    assert abs(replay["selected_time_s"] - 1.25) <= 0.011


def test_pose_run_generates_trajectory_and_compare(isolated_home: Path, tmp_path: Path):
    repo = Repository(Settings.load())
    session1 = make_session(repo)
    source1 = write_v5l(tmp_path / "pose1.V5L", synthetic_frames(300, pose=True))
    run1 = ImportService(repo).import_recording(session1.session_id, source1)
    result1 = AnalysisPipeline(repo).analyze(run1.run_id)
    assert result1.capability["pose"]

    # Use a different sample count to create a distinct raw artifact.
    source2 = write_v5l(tmp_path / "pose2.V5L", synthetic_frames(320, pose=True))
    session2 = make_session(repo)
    run2 = ImportService(repo).import_recording(session2.session_id, source2)
    AnalysisPipeline(repo).analyze(run2.run_id)
    compare_path = compare_runs([run1.run_id, run2.run_id], repo)
    compare = json.loads(compare_path.read_text(encoding="utf-8"))
    assert len(compare["runs"]) == 2


def test_duplicate_hash_is_not_silently_rebound(isolated_home: Path, tmp_path: Path):
    repo = Repository(Settings.load())
    source = write_v5l(tmp_path / "same.V5L", synthetic_frames(40))
    first_session = make_session(repo)
    second_session = make_session(repo, team="OTHER")
    service = ImportService(repo)
    service.import_recording(first_session.session_id, source)
    try:
        service.import_recording(second_session.session_id, source)
    except ValueError as error:
        assert "explicit re-binding" in str(error)
    else:
        raise AssertionError("duplicate recording was silently rebound")


def test_expected_robot_mismatch_is_conditional_but_analyzable(
    isolated_home: Path, tmp_path: Path
):
    repo = Repository(Settings.load())
    session = repo.create_session(
        SessionMetadata(
            team_number="74000M",
            operator="operator",
            test_type="identity",
            expected_robot_id="74000M",
        )
    )
    source = write_v5l(tmp_path / "identity.V5L", synthetic_frames(80))
    run = ImportService(repo).import_recording(session.session_id, source)
    assert run.integrity_verdict == Verdict.CONDITIONAL_PASS
    result = AnalysisPipeline(repo).analyze(run.run_id)
    assert result.integrity_verdict == Verdict.CONDITIONAL_PASS
    assert any("reports 1690X" in anomaly["summary"] for anomaly in result.anomalies)
