from __future__ import annotations

import argparse
import importlib.metadata
import json
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path

from pydantic import ValidationError

from .analysis.compare import compare_runs
from .analysis.pipeline import AnalysisPipeline
from .models import DatasetRole, SessionMetadata, Verdict
from .protocol.schema31 import assert_abi
from .protocol.v5l import V5LReader
from .reports.llm import ReportGenerator
from .replay import replay_recorded_evidence
from .repository import Repository
from .settings import Settings
from .storage.archive import ImportService, scan_recordings


def _json(value) -> None:
    if hasattr(value, "model_dump"):
        value = value.model_dump(mode="json")
    print(json.dumps(value, indent=2, ensure_ascii=False, default=str))


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="statusmonitor",
        description="VEX V5 flight-log session, import, analysis, GUI, and report tool.",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    doctor = sub.add_parser("doctor", help="check runtime, ABI, storage, and optional dependencies")
    doctor.add_argument("--json", action="store_true")

    new = sub.add_parser("new-session", help="create a session identity record")
    new.add_argument("--from-template")
    new.add_argument("--team")
    new.add_argument("--operator")
    new.add_argument("--test-type")
    new.add_argument("--observer")
    new.add_argument("--analyst")
    new.add_argument("--test-case")
    new.add_argument("--direction")
    new.add_argument("--repetition", type=int)
    new.add_argument("--target")
    new.add_argument("--primary-variable")
    new.add_argument("--dataset-role", choices=[role.value for role in DatasetRole])
    new.add_argument("--surface")
    new.add_argument("--location")
    new.add_argument("--ambient-C", type=float)
    new.add_argument("--payload")
    new.add_argument("--battery")
    new.add_argument("--tire-state")
    new.add_argument("--expected-robot")
    new.add_argument("--notes")
    new.add_argument("--save-template")

    sub.add_parser("list-sessions", help="list sessions")
    sub.add_parser("list-templates", help="list reusable session templates")
    list_runs = sub.add_parser("list-runs", help="list imported runs")
    list_runs.add_argument("--session")

    window_start = sub.add_parser("start-window", help="record a PC-side observation window marker")
    window_start.add_argument("session")
    window_stop = sub.add_parser("stop-window", help="close a PC-side observation window marker")
    window_stop.add_argument("session")

    scan = sub.add_parser("scan-media", help="scan a file/directory for V5L/TMP recordings")
    scan.add_argument("path")

    check = sub.add_parser("check", help="verify V5L2 integrity")
    check.add_argument("path")

    imp = sub.add_parser("import", help="archive and decode recordings into a session")
    imp.add_argument("session")
    imp.add_argument("path")

    analyze = sub.add_parser("analyze", help="run deterministic analysis and generate reports")
    analyze.add_argument("run")

    compare = sub.add_parser("compare", help="compare two or more analyzed runs")
    compare.add_argument("runs", nargs="+")

    report = sub.add_parser("report", help="regenerate the LLM evidence report")
    report.add_argument("run")
    replay = sub.add_parser("replay", help="verify and inspect a recorded causal-chain snapshot")
    replay.add_argument("run")
    replay.add_argument("--time-s", type=float)

    sub.add_parser("gui", help="launch the PySide6 desktop application")
    return parser


def _window_path(settings: Settings, session_id: str) -> Path:
    path = settings.artifacts / "pc_windows" / session_id / "windows.jsonl"
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def _window_event(repo: Repository, session_id: str, action: str) -> dict:
    repo.get_session(session_id)
    path = _window_path(repo.settings, session_id)
    entry = {
        "session_id": session_id,
        "action": action,
        "pc_time": datetime.now(timezone.utc).isoformat(),
        "note": "PC observation marker only; it does not command the Brain recorder",
    }
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(entry, ensure_ascii=False) + "\n")
    return entry


def _doctor(settings: Settings) -> dict:
    assert_abi()
    packages = {}
    for name in [
        "numpy",
        "scipy",
        "matplotlib",
        "pydantic",
        "Jinja2",
        "pyarrow",
        "polars",
        "PySide6",
        "pyqtgraph",
    ]:
        try:
            packages[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            packages[name] = None
    usage = shutil.disk_usage(settings.artifacts)
    return {
        "status": "PASS" if all(packages.values()) else "CONDITIONAL",
        "python": sys.version,
        "home": str(settings.home),
        "artifacts": str(settings.artifacts),
        "database": str(settings.database),
        "v5l_abi": "V5L2 / LogFrame 3.1 / 1536 bytes",
        "packages": packages,
        "disk_free_bytes": usage.free,
        "gui_ready": bool(packages["PySide6"] and packages["pyqtgraph"]),
        "parquet_ready": bool(packages["pyarrow"]),
    }


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    settings = Settings.load()
    repo = Repository(settings)
    try:
        if args.command == "doctor":
            result = _doctor(settings)
            _json(result)
            return 0 if result["status"] == "PASS" else 2
        if args.command == "new-session":
            values = {}
            if args.from_template:
                templates = repo.list_templates()
                if args.from_template not in templates:
                    raise KeyError(f"template {args.from_template!r}")
                values = templates[args.from_template].model_dump(
                    exclude={"session_id", "created_at", "updated_at", "status"}
                )
            overrides = {
                "team_number": args.team,
                "operator": args.operator,
                "observer": args.observer,
                "analyst": args.analyst,
                "test_case_id": args.test_case,
                "test_type": args.test_type,
                "direction": args.direction,
                "repetition_index": args.repetition,
                "target": args.target,
                "primary_variable": args.primary_variable,
                "dataset_role": DatasetRole(args.dataset_role) if args.dataset_role else None,
                "surface": args.surface,
                "location": args.location,
                "ambient_temperature_C": args.ambient_C,
                "payload": args.payload,
                "battery_id": args.battery,
                "tire_state": args.tire_state,
                "expected_robot_id": args.expected_robot,
                "notes": args.notes,
            }
            values.update({key: value for key, value in overrides.items() if value is not None})
            session = SessionMetadata(**values)
            repo.create_session(session)
            if args.save_template:
                repo.save_template(args.save_template, session)
            _json(session)
            return 0
        if args.command == "list-sessions":
            _json([session.model_dump(mode="json") for session in repo.list_sessions()])
            return 0
        if args.command == "list-templates":
            _json(
                {
                    name: session.model_dump(mode="json")
                    for name, session in repo.list_templates().items()
                }
            )
            return 0
        if args.command == "list-runs":
            _json([run.model_dump(mode="json") for run in repo.list_runs(args.session)])
            return 0
        if args.command == "start-window":
            _json(_window_event(repo, args.session, "START"))
            return 0
        if args.command == "stop-window":
            _json(_window_event(repo, args.session, "STOP"))
            return 0
        if args.command == "scan-media":
            _json(
                [
                    {
                        **candidate.__dict__,
                        "path": str(candidate.path),
                        "status": (
                            candidate.status.value
                            if isinstance(candidate.status, Verdict)
                            else candidate.status
                        ),
                    }
                    for candidate in scan_recordings(args.path)
                ]
            )
            return 0
        if args.command == "check":
            report = V5LReader(args.path).read().report
            _json(report)
            return 0 if report.verdict == Verdict.PASS else 2
        if args.command == "import":
            service = ImportService(repo)
            source = Path(args.path)
            paths = (
                [source]
                if source.is_file()
                else [candidate.path for candidate in scan_recordings(source)]
            )
            if not paths:
                raise ValueError("no V5L/TMP recordings found")
            records = [service.import_recording(args.session, path) for path in paths]
            _json([record.model_dump(mode="json") for record in records])
            return 0
        if args.command == "analyze":
            result = AnalysisPipeline(repo).analyze(args.run)
            _json(result)
            return 0
        if args.command == "compare":
            path = compare_runs(args.runs, repo)
            _json({"comparison": str(path)})
            return 0
        if args.command == "report":
            path = ReportGenerator(repo).generate(args.run)
            _json({"report": str(path)})
            return 0
        if args.command == "replay":
            _json(replay_recorded_evidence(args.run, args.time_s, repo))
            return 0
        if args.command == "gui":
            from .app import main as gui_main

            return gui_main()
        return 1
    except (OSError, KeyError, ValueError, RuntimeError, ValidationError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
