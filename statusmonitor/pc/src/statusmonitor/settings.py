from __future__ import annotations

import os
import sys
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Settings:
    home: Path
    artifacts: Path
    database: Path

    @classmethod
    def load(cls) -> "Settings":
        default_home = (
            Path.home() / "Documents" / "VEXFlightStatusMonitor"
            if getattr(sys, "frozen", False)
            else Path(__file__).resolve().parents[3]
        )
        home = Path(os.environ.get("STATUSMONITOR_HOME", default_home)).resolve()
        artifacts = Path(os.environ.get("STATUSMONITOR_ARTIFACTS", home / "artifacts")).resolve()
        database = Path(os.environ.get("STATUSMONITOR_DB", artifacts / "statusmonitor.sqlite3")).resolve()
        artifacts.mkdir(parents=True, exist_ok=True)
        return cls(home=home, artifacts=artifacts, database=database)
