from __future__ import annotations

import json
import os
import sys
import traceback
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable

try:
    import pyqtgraph as pg
    from PySide6 import QtCore, QtGui, QtWidgets
except ImportError as import_error:  # pragma: no cover - exercised by doctor/packaging
    pg = None
    QtCore = QtGui = QtWidgets = None
    _GUI_IMPORT_ERROR = import_error
else:
    _GUI_IMPORT_ERROR = None

from .analysis.compare import compare_runs
from .analysis.pipeline import AnalysisPipeline
from .models import DatasetRole, SessionMetadata
from .reports.llm import ReportGenerator
from .repository import Repository
from .storage.archive import ImportService, scan_recordings


if QtCore is not None:

    class WorkerSignals(QtCore.QObject):
        result = QtCore.Signal(object)
        error = QtCore.Signal(str)
        finished = QtCore.Signal()


    class Worker(QtCore.QRunnable):
        def __init__(self, function: Callable, *args, **kwargs):
            super().__init__()
            self.function = function
            self.args = args
            self.kwargs = kwargs
            self.signals = WorkerSignals()

        @QtCore.Slot()
        def run(self):
            try:
                result = self.function(*self.args, **self.kwargs)
            except Exception:
                self.signals.error.emit(traceback.format_exc())
            else:
                self.signals.result.emit(result)
            finally:
                self.signals.finished.emit()


    def _error(parent, message: str) -> None:
        QtWidgets.QMessageBox.critical(parent, "Status Monitor", message)


    def _info(parent, message: str) -> None:
        QtWidgets.QMessageBox.information(parent, "Status Monitor", message)


    class RefreshablePage(QtWidgets.QWidget):
        def __init__(self, repo: Repository, on_change: Callable[[], None]):
            super().__init__()
            self.repo = repo
            self.on_change = on_change

        def refresh(self) -> None:
            pass


    class SessionPage(RefreshablePage):
        def __init__(self, repo, on_change):
            super().__init__(repo, on_change)
            layout = QtWidgets.QVBoxLayout(self)
            title = QtWidgets.QLabel("Home / Session")
            title.setObjectName("pageTitle")
            layout.addWidget(title)
            hint = QtWidgets.QLabel(
                "Create the human/test identity here. Robot identity is read from V5L2 and is never overwritten."
            )
            hint.setWordWrap(True)
            layout.addWidget(hint)
            template_row = QtWidgets.QHBoxLayout()
            self.template = QtWidgets.QComboBox()
            apply_template = QtWidgets.QPushButton("Apply template")
            apply_template.clicked.connect(self.apply_template)
            self.save_template_name = QtWidgets.QLineEdit()
            self.save_template_name.setPlaceholderText("Optional template name to save")
            template_row.addWidget(QtWidgets.QLabel("Template"))
            template_row.addWidget(self.template, 1)
            template_row.addWidget(apply_template)
            template_row.addWidget(self.save_template_name, 1)
            layout.addLayout(template_row)
            form = QtWidgets.QFormLayout()
            self.team = QtWidgets.QLineEdit("74000M")
            self.operator = QtWidgets.QLineEdit()
            self.observer = QtWidgets.QLineEdit()
            self.test_type = QtWidgets.QComboBox()
            self.test_type.setEditable(True)
            self.test_type.addItems(["manual", "straight", "turn", "PID", "SysId", "thermal", "fault"])
            self.test_case = QtWidgets.QLineEdit()
            self.dataset_role = QtWidgets.QComboBox()
            self.dataset_role.addItems([role.value for role in DatasetRole])
            self.surface = QtWidgets.QLineEdit()
            self.battery = QtWidgets.QLineEdit()
            self.expected_robot = QtWidgets.QLineEdit()
            self.notes = QtWidgets.QPlainTextEdit()
            self.notes.setMaximumHeight(70)
            for label, widget in [
                ("Team number *", self.team),
                ("Operator *", self.operator),
                ("Observer", self.observer),
                ("Test type *", self.test_type),
                ("Test case ID", self.test_case),
                ("Dataset role", self.dataset_role),
                ("Surface", self.surface),
                ("Battery ID", self.battery),
                ("Expected robot ID", self.expected_robot),
                ("Notes", self.notes),
            ]:
                form.addRow(label, widget)
            layout.addLayout(form)
            buttons = QtWidgets.QHBoxLayout()
            create = QtWidgets.QPushButton("Create session")
            create.clicked.connect(self.create_session)
            buttons.addWidget(create)
            buttons.addStretch()
            layout.addLayout(buttons)
            self.table = QtWidgets.QTableWidget(0, 6)
            self.table.setHorizontalHeaderLabels(
                ["Session", "Team", "Operator", "Test", "Role", "Status"]
            )
            self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Stretch)
            self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
            layout.addWidget(self.table)
            self.refresh()

        def create_session(self):
            try:
                session = SessionMetadata(
                    team_number=self.team.text(),
                    operator=self.operator.text(),
                    observer=self.observer.text(),
                    test_type=self.test_type.currentText(),
                    test_case_id=self.test_case.text(),
                    dataset_role=DatasetRole(self.dataset_role.currentText()),
                    surface=self.surface.text(),
                    battery_id=self.battery.text(),
                    expected_robot_id=self.expected_robot.text(),
                    notes=self.notes.toPlainText(),
                )
                self.repo.create_session(session)
                if self.save_template_name.text().strip():
                    self.repo.save_template(self.save_template_name.text().strip(), session)
            except Exception as error:
                _error(self, str(error))
                return
            self.refresh()
            self.on_change()
            _info(self, f"Created {session.session_id}")

        def apply_template(self):
            name = self.template.currentData()
            if not name:
                return
            session = self.repo.list_templates()[name]
            self.team.setText(session.team_number)
            self.operator.setText(session.operator)
            self.observer.setText(session.observer)
            self.test_type.setCurrentText(session.test_type)
            self.test_case.setText(session.test_case_id)
            self.dataset_role.setCurrentText(session.dataset_role.value)
            self.surface.setText(session.surface)
            self.battery.setText(session.battery_id)
            self.expected_robot.setText(session.expected_robot_id)
            self.notes.setPlainText(session.notes)

        def refresh(self):
            sessions = self.repo.list_sessions()
            current_template = self.template.currentData()
            self.template.clear()
            self.template.addItem("No template", None)
            for name in self.repo.list_templates():
                self.template.addItem(name, name)
            if current_template:
                index = self.template.findData(current_template)
                if index >= 0:
                    self.template.setCurrentIndex(index)
            self.table.setRowCount(len(sessions))
            for row, session in enumerate(sessions):
                values = [
                    session.session_id,
                    session.team_number,
                    session.operator,
                    session.test_type,
                    session.dataset_role.value,
                    session.status.value,
                ]
                for column, value in enumerate(values):
                    self.table.setItem(row, column, QtWidgets.QTableWidgetItem(str(value)))


    class RecordWindowPage(RefreshablePage):
        def __init__(self, repo, on_change):
            super().__init__(repo, on_change)
            layout = QtWidgets.QVBoxLayout(self)
            title = QtWidgets.QLabel("Record Window")
            title.setObjectName("pageTitle")
            layout.addWidget(title)
            text = QtWidgets.QLabel(
                "These buttons record PC observation timestamps only. They do not command the Brain. "
                "On the Controller: hold Y ≥3 s and release to start; hold Y ≥1 s and release to stop. "
                "Complete data remains on the microSD/TF card."
            )
            text.setWordWrap(True)
            layout.addWidget(text)
            self.session = QtWidgets.QComboBox()
            layout.addWidget(self.session)
            buttons = QtWidgets.QHBoxLayout()
            self.start = QtWidgets.QPushButton("Start PC window")
            self.stop = QtWidgets.QPushButton("Stop PC window")
            self.start.clicked.connect(lambda: self.marker("START"))
            self.stop.clicked.connect(lambda: self.marker("STOP"))
            buttons.addWidget(self.start)
            buttons.addWidget(self.stop)
            layout.addLayout(buttons)
            self.status = QtWidgets.QLabel("No PC window marker in this app session.")
            self.status.setObjectName("statusCard")
            layout.addWidget(self.status)
            layout.addStretch()
            self.refresh()

        def refresh(self):
            current = self.session.currentData()
            self.session.clear()
            for session in self.repo.list_sessions():
                self.session.addItem(
                    f"{session.session_id} · {session.team_number} · {session.test_type}",
                    session.session_id,
                )
            if current:
                index = self.session.findData(current)
                if index >= 0:
                    self.session.setCurrentIndex(index)

        def marker(self, action: str):
            session_id = self.session.currentData()
            if not session_id:
                return
            path = self.repo.settings.artifacts / "pc_windows" / session_id / "windows.jsonl"
            path.parent.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now(timezone.utc).isoformat()
            with path.open("a", encoding="utf-8") as stream:
                stream.write(
                    json.dumps(
                        {
                            "session_id": session_id,
                            "action": action,
                            "pc_time": stamp,
                            "control_scope": "marker only; no Brain downlink",
                        },
                        ensure_ascii=False,
                    )
                    + "\n"
                )
            self.status.setText(f"{action} marker · {stamp}")


    class ImportPage(RefreshablePage):
        def __init__(self, repo, on_change):
            super().__init__(repo, on_change)
            self.pool = QtCore.QThreadPool.globalInstance()
            self.candidates = []
            layout = QtWidgets.QVBoxLayout(self)
            title = QtWidgets.QLabel("Import microSD / TF recordings")
            title.setObjectName("pageTitle")
            layout.addWidget(title)
            top = QtWidgets.QHBoxLayout()
            self.path = QtWidgets.QLineEdit()
            browse = QtWidgets.QPushButton("Browse…")
            browse.clicked.connect(self.browse)
            scan = QtWidgets.QPushButton("Scan")
            scan.clicked.connect(self.scan)
            top.addWidget(self.path, 1)
            top.addWidget(browse)
            top.addWidget(scan)
            layout.addLayout(top)
            self.session = QtWidgets.QComboBox()
            layout.addWidget(self.session)
            self.table = QtWidgets.QTableWidget(0, 7)
            self.table.setHorizontalHeaderLabels(
                ["Import", "File", "Status", "Robot", "Storage #", "Frames", "Duration [s]"]
            )
            self.table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeMode.Stretch)
            layout.addWidget(self.table)
            bottom = QtWidgets.QHBoxLayout()
            self.progress = QtWidgets.QProgressBar()
            self.progress.setRange(0, 1)
            self.progress.setValue(0)
            import_selected = QtWidgets.QPushButton("Import checked")
            import_selected.clicked.connect(self.import_checked)
            bottom.addWidget(self.progress, 1)
            bottom.addWidget(import_selected)
            layout.addLayout(bottom)
            self.refresh()

        def refresh(self):
            current = self.session.currentData()
            self.session.clear()
            for session in self.repo.list_sessions():
                self.session.addItem(
                    f"{session.session_id} · {session.team_number} · {session.test_type}",
                    session.session_id,
                )
            if current:
                index = self.session.findData(current)
                if index >= 0:
                    self.session.setCurrentIndex(index)

        def browse(self):
            path = QtWidgets.QFileDialog.getExistingDirectory(self, "Select microSD/TF root")
            if path:
                self.path.setText(path)

        def _busy(self, busy: bool):
            self.progress.setRange(0, 0 if busy else 1)
            if not busy:
                self.progress.setValue(1)

        def scan(self):
            if not self.path.text():
                return
            self._busy(True)
            worker = Worker(scan_recordings, self.path.text())
            worker.signals.result.connect(self.show_candidates)
            worker.signals.error.connect(lambda error: _error(self, error))
            worker.signals.finished.connect(lambda: self._busy(False))
            self.pool.start(worker)

        def show_candidates(self, candidates):
            self.candidates = candidates
            self.table.setRowCount(len(candidates))
            for row, candidate in enumerate(candidates):
                check = QtWidgets.QTableWidgetItem()
                check.setFlags(check.flags() | QtCore.Qt.ItemFlag.ItemIsUserCheckable)
                check.setCheckState(QtCore.Qt.CheckState.Checked)
                self.table.setItem(row, 0, check)
                status = candidate.status.value if hasattr(candidate.status, "value") else candidate.status
                values = [
                    str(candidate.path),
                    status,
                    candidate.robot_id,
                    candidate.storage_sequence,
                    candidate.frames,
                    f"{candidate.duration_s:.3f}",
                ]
                for column, value in enumerate(values, start=1):
                    self.table.setItem(row, column, QtWidgets.QTableWidgetItem(str(value)))

        def import_checked(self):
            session_id = self.session.currentData()
            selected = [
                candidate.path
                for row, candidate in enumerate(self.candidates)
                if self.table.item(row, 0).checkState() == QtCore.Qt.CheckState.Checked
            ]
            if not session_id or not selected:
                return
            self._busy(True)

            def execute():
                service = ImportService(self.repo)
                return [service.import_recording(session_id, path) for path in selected]

            worker = Worker(execute)
            worker.signals.result.connect(
                lambda runs: (_info(self, f"Imported {len(runs)} recording(s)."), self.on_change())
            )
            worker.signals.error.connect(lambda error: _error(self, error))
            worker.signals.finished.connect(lambda: self._busy(False))
            self.pool.start(worker)


    class IntegrityPage(RefreshablePage):
        def __init__(self, repo, on_change):
            super().__init__(repo, on_change)
            layout = QtWidgets.QVBoxLayout(self)
            title = QtWidgets.QLabel("Integrity hard gate")
            title.setObjectName("pageTitle")
            layout.addWidget(title)
            self.run = QtWidgets.QComboBox()
            self.run.currentIndexChanged.connect(self.load)
            layout.addWidget(self.run)
            self.verdict = QtWidgets.QLabel("No run")
            self.verdict.setObjectName("verdict")
            layout.addWidget(self.verdict)
            self.text = QtWidgets.QPlainTextEdit()
            self.text.setReadOnly(True)
            layout.addWidget(self.text)
            self.refresh()

        def refresh(self):
            current = self.run.currentData()
            self.run.blockSignals(True)
            self.run.clear()
            for run in self.repo.list_runs():
                self.run.addItem(f"{run.run_id} · {run.identity.robot_id}", run.run_id)
            self.run.blockSignals(False)
            if current:
                index = self.run.findData(current)
                if index >= 0:
                    self.run.setCurrentIndex(index)
            self.load()

        def load(self):
            run_id = self.run.currentData()
            if not run_id:
                return
            run = self.repo.get_run(run_id)
            path = Path(run.artifact_dir) / "integrity" / "integrity_report.json"
            data = json.loads(path.read_text(encoding="utf-8"))
            self.verdict.setText(data["verdict"])
            self.verdict.setProperty("verdictValue", data["verdict"])
            self.style().unpolish(self.verdict)
            self.style().polish(self.verdict)
            self.text.setPlainText(json.dumps(data, indent=2, ensure_ascii=False))


    class OverviewPage(RefreshablePage):
        def __init__(self, repo, on_change):
            super().__init__(repo, on_change)
            self.pool = QtCore.QThreadPool.globalInstance()
            layout = QtWidgets.QVBoxLayout(self)
            title = QtWidgets.QLabel("Overview")
            title.setObjectName("pageTitle")
            layout.addWidget(title)
            top = QtWidgets.QHBoxLayout()
            self.run = QtWidgets.QComboBox()
            self.run.currentIndexChanged.connect(self.load)
            analyze = QtWidgets.QPushButton("Analyze / regenerate")
            analyze.clicked.connect(self.analyze)
            top.addWidget(self.run, 1)
            top.addWidget(analyze)
            layout.addLayout(top)
            self.cards = QtWidgets.QGridLayout()
            layout.addLayout(self.cards)
            self.anomalies = QtWidgets.QTableWidget(0, 5)
            self.anomalies.setHorizontalHeaderLabels(
                ["ID", "Layer", "Severity", "Summary", "Evidence window"]
            )
            self.anomalies.horizontalHeader().setSectionResizeMode(3, QtWidgets.QHeaderView.ResizeMode.Stretch)
            layout.addWidget(self.anomalies)
            self.refresh()

        def refresh(self):
            current = self.run.currentData()
            self.run.blockSignals(True)
            self.run.clear()
            for run in self.repo.list_runs():
                self.run.addItem(f"{run.run_id} · {run.identity.robot_id}", run.run_id)
            self.run.blockSignals(False)
            if current:
                index = self.run.findData(current)
                if index >= 0:
                    self.run.setCurrentIndex(index)
            self.load()

        def analyze(self):
            run_id = self.run.currentData()
            if not run_id:
                return
            worker = Worker(AnalysisPipeline(self.repo).analyze, run_id)
            worker.signals.result.connect(lambda _: (self.load(), self.on_change()))
            worker.signals.error.connect(lambda error: _error(self, error))
            self.pool.start(worker)

        def load(self):
            while self.cards.count():
                item = self.cards.takeAt(0)
                if item.widget():
                    item.widget().deleteLater()
            self.anomalies.setRowCount(0)
            run_id = self.run.currentData()
            if not run_id:
                return
            try:
                analysis = self.repo.get_analysis(run_id)
            except KeyError:
                card = QtWidgets.QLabel("Not analyzed yet")
                card.setObjectName("statusCard")
                self.cards.addWidget(card, 0, 0)
                return
            metrics = analysis.metrics
            values = [
                ("Integrity", analysis.integrity_verdict.value),
                ("Pose", "AVAILABLE" if analysis.capability["pose"] else "NOT AVAILABLE"),
                ("PID", "AVAILABLE" if analysis.capability["pid_terms"] else "NOT AVAILABLE"),
                ("Duration", f"{metrics['duration_s']:.3f} s"),
                ("Samples", str(metrics["samples"])),
                ("Exec p99", f"{(metrics['timing']['exec_s']['p99'] or 0)*1000:.3f} ms"),
                ("Deadline misses", str(metrics["timing"]["overrun_frames"])),
                ("Anomalies", str(len(analysis.anomalies))),
            ]
            for index, (label, value) in enumerate(values):
                card = QtWidgets.QLabel(f"<small>{label}</small><br><b>{value}</b>")
                card.setObjectName("statusCard")
                card.setTextFormat(QtCore.Qt.TextFormat.RichText)
                self.cards.addWidget(card, index // 4, index % 4)
            self.anomalies.setRowCount(len(analysis.anomalies))
            for row, anomaly in enumerate(analysis.anomalies):
                for column, value in enumerate(
                    [
                        anomaly["id"],
                        anomaly["layer"],
                        anomaly["severity"],
                        anomaly["summary"],
                        anomaly.get("evidence_window_s"),
                    ]
                ):
                    self.anomalies.setItem(row, column, QtWidgets.QTableWidgetItem(str(value)))


    class PlotsPage(RefreshablePage):
        SIGNALS = [
            "raw.battery_V",
            "actuator.final_left_V",
            "actuator.final_right_V",
            "timing.raw_dt_s",
            "timing.exec_s",
            "trace.mapped_throttle",
            "trace.mapped_turn",
            "motor.L1.velocity_radps",
            "motor.L2.velocity_radps",
            "motor.L3.velocity_radps",
            "motor.R1.velocity_radps",
            "motor.R2.velocity_radps",
            "motor.R3.velocity_radps",
        ]

        def __init__(self, repo, on_change):
            super().__init__(repo, on_change)
            layout = QtWidgets.QVBoxLayout(self)
            title = QtWidgets.QLabel("Plots")
            title.setObjectName("pageTitle")
            layout.addWidget(title)
            top = QtWidgets.QHBoxLayout()
            self.run = QtWidgets.QComboBox()
            self.run.currentIndexChanged.connect(self.load)
            self.signal = QtWidgets.QComboBox()
            self.signal.addItems(self.SIGNALS)
            self.signal.currentIndexChanged.connect(self.load)
            open_dir = QtWidgets.QPushButton("Open static plot folder")
            open_dir.clicked.connect(self.open_folder)
            top.addWidget(self.run, 1)
            top.addWidget(self.signal)
            top.addWidget(open_dir)
            layout.addLayout(top)
            self.plot = pg.PlotWidget()
            self.plot.showGrid(x=True, y=True, alpha=0.25)
            self.plot.setLabel("bottom", "robot monotonic time", units="s")
            layout.addWidget(self.plot)
            self.note = QtWidgets.QLabel(
                "Interactive view uses full Parquet samples. Static plots include trajectory, "
                "motion, motor, energy, timing, events and Welch PSD."
            )
            self.note.setWordWrap(True)
            layout.addWidget(self.note)
            self.refresh()

        def refresh(self):
            current = self.run.currentData()
            self.run.blockSignals(True)
            self.run.clear()
            for run in self.repo.list_runs():
                self.run.addItem(f"{run.run_id} · {run.identity.robot_id}", run.run_id)
            self.run.blockSignals(False)
            if current:
                index = self.run.findData(current)
                if index >= 0:
                    self.run.setCurrentIndex(index)
            self.load()

        def load(self):
            self.plot.clear()
            run_id = self.run.currentData()
            signal_name = self.signal.currentText()
            if not run_id or not signal_name:
                return
            try:
                import pyarrow.parquet as pq

                run = self.repo.get_run(run_id)
                table = pq.read_table(
                    Path(run.artifact_dir) / "derived" / "samples.parquet",
                    columns=["time_s", signal_name],
                )
                time_s = table["time_s"].to_numpy()
                values = table[signal_name].to_numpy()
                self.plot.plot(time_s, values, pen=pg.mkPen("#2f9e44", width=1.4))
                self.plot.setLabel("left", signal_name)
                self.plot.setTitle(f"{run_id} · {signal_name}")
            except Exception as error:
                self.note.setText(str(error))

        def open_folder(self):
            run_id = self.run.currentData()
            if not run_id:
                return
            path = Path(self.repo.get_run(run_id).artifact_dir) / "plots"
            QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(path)))


    class ComparePage(RefreshablePage):
        def __init__(self, repo, on_change):
            super().__init__(repo, on_change)
            layout = QtWidgets.QVBoxLayout(self)
            title = QtWidgets.QLabel("Compare Runs")
            title.setObjectName("pageTitle")
            layout.addWidget(title)
            self.runs = QtWidgets.QListWidget()
            self.runs.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.MultiSelection)
            layout.addWidget(self.runs)
            button = QtWidgets.QPushButton("Compare selected analyzed runs")
            button.clicked.connect(self.compare)
            layout.addWidget(button)
            self.output = QtWidgets.QPlainTextEdit()
            self.output.setReadOnly(True)
            layout.addWidget(self.output)
            self.refresh()

        def refresh(self):
            self.runs.clear()
            for run in self.repo.list_runs():
                item = QtWidgets.QListWidgetItem(f"{run.run_id} · {run.identity.robot_id}")
                item.setData(QtCore.Qt.ItemDataRole.UserRole, run.run_id)
                self.runs.addItem(item)

        def compare(self):
            run_ids = [
                item.data(QtCore.Qt.ItemDataRole.UserRole)
                for item in self.runs.selectedItems()
            ]
            try:
                path = compare_runs(run_ids, self.repo)
                self.output.setPlainText(path.read_text(encoding="utf-8"))
            except Exception as error:
                _error(self, str(error))


    class ReportPage(RefreshablePage):
        def __init__(self, repo, on_change):
            super().__init__(repo, on_change)
            layout = QtWidgets.QVBoxLayout(self)
            title = QtWidgets.QLabel("Report")
            title.setObjectName("pageTitle")
            layout.addWidget(title)
            top = QtWidgets.QHBoxLayout()
            self.run = QtWidgets.QComboBox()
            self.run.currentIndexChanged.connect(self.load)
            regenerate = QtWidgets.QPushButton("Regenerate")
            regenerate.clicked.connect(self.regenerate)
            open_file = QtWidgets.QPushButton("Open Markdown")
            open_file.clicked.connect(self.open_file)
            top.addWidget(self.run, 1)
            top.addWidget(regenerate)
            top.addWidget(open_file)
            layout.addLayout(top)
            self.text = QtWidgets.QPlainTextEdit()
            self.text.setReadOnly(True)
            layout.addWidget(self.text)
            self.refresh()

        def refresh(self):
            current = self.run.currentData()
            self.run.blockSignals(True)
            self.run.clear()
            for run in self.repo.list_runs():
                self.run.addItem(f"{run.run_id} · {run.identity.robot_id}", run.run_id)
            self.run.blockSignals(False)
            if current:
                index = self.run.findData(current)
                if index >= 0:
                    self.run.setCurrentIndex(index)
            self.load()

        def report_path(self) -> Path | None:
            run_id = self.run.currentData()
            return (
                Path(self.repo.get_run(run_id).artifact_dir) / "report_for_llm.md"
                if run_id
                else None
            )

        def load(self):
            path = self.report_path()
            self.text.setPlainText(
                path.read_text(encoding="utf-8")
                if path and path.exists()
                else "Analyze this run to generate the evidence report."
            )

        def regenerate(self):
            run_id = self.run.currentData()
            if not run_id:
                return
            try:
                ReportGenerator(self.repo).generate(run_id)
                self.load()
            except Exception as error:
                _error(self, str(error))

        def open_file(self):
            path = self.report_path()
            if path and path.exists():
                QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(path)))


    class MainWindow(QtWidgets.QMainWindow):
        def __init__(self):
            super().__init__()
            self.repo = Repository()
            self.setWindowTitle("74000M · VEX Flight Log Status Monitor")
            self.resize(1440, 900)
            self.tabs = QtWidgets.QTabWidget()
            self.setCentralWidget(self.tabs)
            self.pages: list[RefreshablePage] = [
                SessionPage(self.repo, self.refresh_all),
                RecordWindowPage(self.repo, self.refresh_all),
                ImportPage(self.repo, self.refresh_all),
                IntegrityPage(self.repo, self.refresh_all),
                OverviewPage(self.repo, self.refresh_all),
                PlotsPage(self.repo, self.refresh_all),
                ComparePage(self.repo, self.refresh_all),
                ReportPage(self.repo, self.refresh_all),
            ]
            labels = [
                "Home / Session",
                "Record Window",
                "Import",
                "Integrity",
                "Overview",
                "Plots",
                "Compare Runs",
                "Report",
            ]
            for label, page in zip(labels, self.pages, strict=True):
                self.tabs.addTab(page, label)
            self.statusBar().showMessage(
                f"Artifacts: {self.repo.settings.artifacts} · observation only · GPT-5.6"
            )

        def refresh_all(self):
            for page in self.pages:
                page.refresh()


    STYLE = """
    QMainWindow, QWidget { background: #f6f7f9; color: #20242a; font-size: 13px; }
    QTabWidget::pane { border: 1px solid #d9dde3; background: white; }
    QTabBar::tab { padding: 10px 16px; background: #e9ecef; margin-right: 2px; }
    QTabBar::tab:selected { background: white; color: #146c43; font-weight: 600; }
    QLabel#pageTitle { font-size: 24px; font-weight: 700; color: #133f2d; padding: 8px 0; }
    QLabel#statusCard { background: white; border: 1px solid #dfe3e8; border-radius: 8px;
                        padding: 16px; min-height: 42px; }
    QLabel#verdict { font-size: 24px; font-weight: 700; padding: 12px; border-radius: 8px; }
    QLabel#verdict[verdictValue="PASS"] { background: #d3f9d8; color: #2b8a3e; }
    QLabel#verdict[verdictValue="CONDITIONAL PASS"] { background: #fff3bf; color: #a05a00; }
    QLabel#verdict[verdictValue="REPEAT"], QLabel#verdict[verdictValue="FAIL"] {
        background: #ffe3e3; color: #c92a2a;
    }
    QPushButton { background: #146c43; color: white; border: none; border-radius: 5px;
                  padding: 8px 14px; font-weight: 600; }
    QPushButton:hover { background: #0f5132; }
    QLineEdit, QComboBox, QPlainTextEdit, QTableWidget, QListWidget {
        background: white; border: 1px solid #ccd2d9; border-radius: 4px; padding: 5px;
    }
    QHeaderView::section { background: #edf1f4; padding: 7px; border: none; font-weight: 600; }
    """


def main() -> int:
    if _GUI_IMPORT_ERROR is not None:
        print(
            "PySide6 and pyqtgraph are required for the GUI. "
            "Install the project dependencies with `pip install -e .`.\n"
            f"Import error: {_GUI_IMPORT_ERROR}",
            file=sys.stderr,
        )
        return 2
    application = QtWidgets.QApplication(sys.argv)
    application.setApplicationName("VEX Flight Log Status Monitor")
    application.setOrganizationName("74000M")
    application.setStyle("Fusion")
    application.setStyleSheet(STYLE)
    window = MainWindow()
    window.show()
    if os.environ.get("STATUSMONITOR_SMOKE_TEST") == "1":
        QtCore.QTimer.singleShot(250, application.quit)
    return application.exec()


if __name__ == "__main__":
    raise SystemExit(main())
