from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from statusmonitor.repository import Repository


def _get_path(data: dict, path: str) -> Any:
    current: Any = data
    for part in path.split("."):
        if not isinstance(current, dict) or part not in current:
            return None
        current = current[part]
    return current


COMPARE_METRICS = [
    "duration_s",
    "motion.velocity.rms",
    "motion.acceleration.p99",
    "motion.jerk.p99",
    "energy.battery_V.min",
    "timing.exec_s.p99",
    "timing.raw_dt_s.p99",
    "timing.overrun_frames",
]


def compare_runs(run_ids: list[str], repository: Repository | None = None) -> Path:
    if len(run_ids) < 2:
        raise ValueError("compare requires at least two runs")
    repo = repository or Repository()
    runs = [repo.get_run(run_id) for run_id in run_ids]
    analyses = [repo.get_analysis(run_id) for run_id in run_ids]
    output_dir = Path(runs[0].artifact_dir) / "compare"
    output_dir.mkdir(parents=True, exist_ok=True)
    baseline = analyses[0]
    rows = []
    for metric in COMPARE_METRICS:
        base = _get_path(baseline.metrics, metric)
        values = []
        for analysis in analyses:
            value = _get_path(analysis.metrics, metric)
            delta = value - base if isinstance(value, (int, float)) and isinstance(base, (int, float)) else None
            values.append({"run_id": analysis.run_id, "value": value, "delta_from_first": delta})
        rows.append({"metric": metric, "values": values})
    result = {
        "alignment": "summary metric comparison; no raw resampling",
        "baseline_run": run_ids[0],
        "runs": [
            {
                "run_id": run.run_id,
                "robot_id": run.identity.robot_id,
                "source_commit": run.identity.source_commit,
                "config_hash": run.identity.config_hash,
                "integrity": analysis.integrity_verdict.value,
            }
            for run, analysis in zip(runs, analyses, strict=True)
        ],
        "metrics": rows,
    }
    json_path = output_dir / ("compare_" + "_".join(run_ids) + ".json")
    json_path.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")

    labels = [run.run_id[-10:] for run in runs]
    selected = [
        ("motion.velocity.rms", "velocity RMS"),
        ("timing.exec_s.p99", "exec p99 [s]"),
        ("energy.battery_V.min", "battery min [V]"),
        ("timing.overrun_frames", "deadline misses"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    for ax, (metric, title) in zip(axes.flat, selected, strict=True):
        values = [_get_path(analysis.metrics, metric) for analysis in analyses]
        numeric = [float(value) if value is not None else 0.0 for value in values]
        ax.bar(labels, numeric)
        ax.set_title(title)
        ax.tick_params(axis="x", rotation=20)
        ax.grid(axis="y", alpha=0.25)
    fig.suptitle("Run comparison (identity/config differences are in JSON)")
    fig.tight_layout()
    fig.savefig(output_dir / "compare_summary.png", dpi=150)
    plt.close(fig)
    return json_path
