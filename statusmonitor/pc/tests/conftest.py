from __future__ import annotations

from pathlib import Path

import pytest


@pytest.fixture
def isolated_home(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    home = tmp_path / "statusmonitor"
    monkeypatch.setenv("STATUSMONITOR_HOME", str(home))
    monkeypatch.setenv("STATUSMONITOR_ARTIFACTS", str(home / "artifacts"))
    monkeypatch.setenv("STATUSMONITOR_DB", str(home / "artifacts" / "index.sqlite3"))
    return home
