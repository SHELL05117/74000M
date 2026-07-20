from __future__ import annotations

from pathlib import Path
import subprocess

from statusmonitor.models import Verdict
from statusmonitor.protocol.schema31 import (
    FILE_FOOTER_BYTES,
    FILE_HEADER_BYTES,
    LOG_FRAME_BYTES,
    file_header_dtype,
    log_frame_dtype,
)
from statusmonitor.protocol.v5l import V5LReader, flatten_frames

from v5l_builder import synthetic_frames, write_v5l


def test_schema31_abi_is_locked():
    assert file_header_dtype.itemsize == 160
    assert log_frame_dtype.itemsize == 1536


def test_valid_v5l_round_trip(tmp_path: Path):
    path = write_v5l(tmp_path / "valid.v5l", synthetic_frames(50))
    decoded = V5LReader(path).read()
    assert decoded.report.verdict == Verdict.PASS
    assert decoded.report.recoverable_frames == 50
    assert decoded.report.identity.robot_id == "1690X"
    assert decoded.frames["header"]["sequence"].tolist() == list(range(10, 60))
    columns = flatten_frames(decoded.frames)
    assert "motor.L1.velocity_radps" in columns
    assert len(columns["time_s"]) == 50


def test_payload_corruption_is_rejected(tmp_path: Path):
    path = write_v5l(tmp_path / "corrupt.v5l", synthetic_frames(20))
    data = bytearray(path.read_bytes())
    data[FILE_HEADER_BYTES + 28 + LOG_FRAME_BYTES // 2] ^= 0x5A
    path.write_bytes(data)
    decoded = V5LReader(path).read()
    assert decoded.report.verdict == Verdict.REPEAT
    assert "BAD_PAYLOAD_CRC" in {issue.code for issue in decoded.report.issues}
    assert decoded.report.recoverable_frames == 0


def test_truncated_footer_recovers_full_block_but_requires_repeat(tmp_path: Path):
    path = write_v5l(tmp_path / "truncated.tmp", synthetic_frames(20))
    path.write_bytes(path.read_bytes()[:-FILE_FOOTER_BYTES])
    decoded = V5LReader(path).read()
    assert decoded.report.verdict == Verdict.REPEAT
    assert decoded.report.recoverable_frames == 20
    assert not decoded.report.complete


def test_sequence_gap_is_detected(tmp_path: Path):
    frames = synthetic_frames(20)
    frames["header"]["sequence"][10:] += 2
    path = write_v5l(tmp_path / "gap.v5l", frames)
    decoded = V5LReader(path).read()
    assert decoded.report.verdict == Verdict.REPEAT
    assert decoded.report.sequence_gaps == 2


def test_current_cpp_fixture_when_release_tool_is_available(tmp_path: Path):
    project_root = Path(__file__).resolve().parents[3]
    fixture_tool = project_root / "build" / "Release" / "v5l_fixture.exe"
    if not fixture_tool.exists():
        return
    path = tmp_path / "cpp_fixture.v5l"
    subprocess.run([fixture_tool, path], check=True)
    decoded = V5LReader(path).read()
    assert decoded.report.verdict == Verdict.PASS
    assert decoded.frames[0]["controller"]["left_y"] == 0.25
    assert decoded.frames[1]["left_motor"][0]["position_rad"] == 1.5
