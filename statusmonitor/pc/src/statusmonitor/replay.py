from __future__ import annotations

from pathlib import Path

import numpy as np
import pyarrow.parquet as pq

from .protocol.v5l import V5LReader
from .repository import Repository


REPLAY_COLUMNS = [
    "time_s",
    "time_us",
    "sequence",
    "mode_epoch",
    "raw.battery_V",
    "state.pose_x_m",
    "state.pose_y_m",
    "state.pose_theta_rad",
    "request.forward",
    "request.steering",
    "actuator.final_left_V",
    "actuator.final_right_V",
    "fault.active_bits",
    "trace.availability_bits",
]


def replay_recorded_evidence(
    run_id: str, time_s: float | None = None, repository: Repository | None = None
) -> dict:
    """Verify V5L↔Parquet equivalence and return one causal-chain snapshot."""
    repo = repository or Repository()
    run = repo.get_run(run_id)
    reader = V5LReader(run.archive_path)
    decoded = reader.read(load_frames=False)
    table = pq.read_table(
        Path(run.artifact_dir) / "derived" / "samples.parquet",
        columns=REPLAY_COLUMNS,
    )
    if table.num_rows != decoded.report.recoverable_frames:
        raise ValueError(
            f"Parquet has {table.num_rows} rows but V5L has "
            f"{decoded.report.recoverable_frames} recoverable frames"
        )
    times = table["time_s"].to_numpy()
    if len(times) == 0:
        raise ValueError("run contains no replayable frames")
    requested = float(times[0] if time_s is None else time_s)
    index = int(np.argmin(np.abs(times - requested)))
    snapshot = {
        name: table[name][index].as_py()
        for name in REPLAY_COLUMNS
    }
    v5l_sequence = int(reader.frame_at(index)["header"]["sequence"])
    if snapshot["sequence"] != v5l_sequence:
        raise ValueError("V5L and Parquet sequence mismatch during replay")
    return {
        "run_id": run_id,
        "mode": "recorded causal-chain replay (not estimator recomputation)",
        "integrity": decoded.report.verdict.value,
        "requested_time_s": requested,
        "selected_index": index,
        "selected_time_s": float(times[index]),
        "v5l_parquet_equivalent": True,
        "snapshot": snapshot,
        "restriction": (
            "This replays recorded raw/request/actuator/state evidence. "
            "It does not rerun the C++ SensorValidator/Odometry/SafetyGate."
        ),
    }
