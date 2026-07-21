from __future__ import annotations

from pathlib import Path

import pytest


PC_ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize(
    "filename",
    ["run_statusmonitor_macos.command", "build_macos_app.command"],
)
def test_macos_launcher_is_portable_and_uses_lf(filename: str):
    payload = (PC_ROOT / filename).read_bytes()
    text = payload.decode("utf-8")

    assert payload.startswith(b"#!/usr/bin/env bash\n")
    assert b"\r\n" not in payload
    assert ".venv-macos" in text
    assert "python3.11" in text
    assert "C:\\Users\\" not in text
    assert "/Users/" not in text


def test_macos_build_creates_native_app_bundle():
    text = (PC_ROOT / "build_macos_app.command").read_text(encoding="utf-8")

    assert "-m PyInstaller" in text
    assert "--windowed" in text
    assert "VEXFlightStatusMonitor.app" in text
