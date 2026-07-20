from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Segment:
    label: str
    start_index: int
    end_index: int
    start_s: float
    end_s: float
    source: str

    def as_dict(self) -> dict:
        return {
            "label": self.label,
            "start_index": self.start_index,
            "end_index": self.end_index,
            "start_s": self.start_s,
            "end_s": self.end_s,
            "source": self.source,
        }


def segment_motion(
    time_s: np.ndarray,
    left_voltage: np.ndarray,
    right_voltage: np.ndarray,
    speed: np.ndarray | None = None,
) -> list[Segment]:
    """Deterministic heuristic segmentation; original samples are untouched."""
    count = len(time_s)
    if count == 0:
        return []
    command = np.maximum(np.abs(left_voltage), np.abs(right_voltage))
    finite_command = command[np.isfinite(command)]
    max_command = float(np.max(finite_command)) if finite_command.size else 0.0
    command_threshold = max(0.10, 0.05 * max_command)
    active = np.isfinite(command) & (command >= command_threshold)
    if speed is not None and len(speed) == count:
        finite_speed = np.abs(speed[np.isfinite(speed)])
        if finite_speed.size:
            speed_threshold = max(1e-6, 0.05 * float(np.max(finite_speed)))
            active |= np.isfinite(speed) & (np.abs(speed) >= speed_threshold)

    dt = np.diff(time_s)
    median_dt = float(np.median(dt[(dt > 0) & np.isfinite(dt)])) if np.any(dt > 0) else 0.01
    hold = max(1, int(round(0.15 / median_dt)))
    kernel = np.ones(hold, dtype=np.int32)
    active = np.convolve(active.astype(np.int32), kernel, mode="same") > 0

    boundaries = np.flatnonzero(np.r_[True, active[1:] != active[:-1], True])
    segments: list[Segment] = []
    for start, stop in zip(boundaries[:-1], boundaries[1:], strict=True):
        end = max(start, stop - 1)
        label = "Active" if active[start] else ("PreIdle" if not segments else "PostIdle")
        segments.append(
            Segment(
                label=label,
                start_index=int(start),
                end_index=int(end),
                start_s=float(time_s[start]),
                end_s=float(time_s[end]),
                source="request/output heuristic v1",
            )
        )
    return segments
