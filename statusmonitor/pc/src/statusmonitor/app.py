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
from .ui_style import MUTED, PAPER, RULE, SWISS_GRID_STYLE, VEX_RED
from .version import __version__


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


    TEST_TYPE_ITEMS = [
        ("手动驾驶", "manual"),
        ("直线测试", "straight"),
        ("转向测试", "turn"),
        ("PID 测试", "PID"),
        ("系统辨识", "SysId"),
        ("热负载测试", "thermal"),
        ("故障注入", "fault"),
    ]
    TEST_TYPE_LABELS = {value: label for label, value in TEST_TYPE_ITEMS}
    DATASET_ROLE_LABELS = {
        DatasetRole.EXPLORATORY: "探索数据",
        DatasetRole.TRAINING: "训练数据",
        DatasetRole.VALIDATION: "验证数据",
        DatasetRole.ACCEPTANCE: "验收数据",
    }
    SESSION_STATUS_LABELS = {
        "draft": "草稿",
        "ready": "就绪",
        "imported": "已导入",
        "analyzed": "已分析",
        "archived": "已归档",
    }
    VERDICT_LABELS = {
        "PASS": "通过 · PASS",
        "CONDITIONAL PASS": "有条件通过",
        "REPEAT": "需要复测",
        "FAIL": "失败",
        "NOT TESTED": "未测试",
    }


    def _error(parent, message: str) -> None:
        QtWidgets.QMessageBox.critical(parent, "飞行记录与诊断系统", message)


    def _info(parent, message: str) -> None:
        QtWidgets.QMessageBox.information(parent, "飞行记录与诊断系统", message)


    def _primary(button: QtWidgets.QPushButton) -> QtWidgets.QPushButton:
        button.setProperty("role", "primary")
        return button


    def _page_layout(
        page: QtWidgets.QWidget, title_text: str, description: str
    ) -> QtWidgets.QVBoxLayout:
        page.setObjectName("page")
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)
        title = QtWidgets.QLabel(title_text)
        title.setObjectName("pageTitle")
        layout.addWidget(title)
        if description:
            hint = QtWidgets.QLabel(description)
            hint.setObjectName("pageDescription")
            hint.setWordWrap(True)
            layout.addWidget(hint)
        rule = QtWidgets.QFrame()
        rule.setObjectName("sectionRule")
        rule.setFrameShape(QtWidgets.QFrame.Shape.HLine)
        layout.addWidget(rule)
        return layout


    def _configure_table(table: QtWidgets.QTableWidget) -> None:
        table.setAlternatingRowColors(True)
        table.setShowGrid(False)
        table.verticalHeader().setVisible(False)
        table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)


    def _ensure_ui_font() -> None:
        application = QtWidgets.QApplication.instance()
        if application is None:
            return
        preferred = ["Noto Sans SC", "Microsoft YaHei UI", "Microsoft YaHei", "PingFang SC"]
        families = set(QtGui.QFontDatabase.families())
        family = next((name for name in preferred if name in families), "")
        if not family:
            candidates = [
                Path(os.environ.get("WINDIR", "C:/Windows"))
                / "Fonts"
                / "NotoSansSC-VF.ttf",
                Path(os.environ.get("WINDIR", "C:/Windows")) / "Fonts" / "msyh.ttc",
                Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
            ]
            for candidate in candidates:
                if not candidate.exists():
                    continue
                font_id = QtGui.QFontDatabase.addApplicationFont(str(candidate))
                loaded = QtGui.QFontDatabase.applicationFontFamilies(font_id)
                if loaded:
                    family = loaded[0]
                    break
        if family:
            application.setFont(QtGui.QFont(family, 10))


    def _metric_widget(label_text: str, value_text: str) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        widget.setObjectName("metricPanel")
        layout = QtWidgets.QVBoxLayout(widget)
        layout.setContentsMargins(14, 11, 14, 11)
        layout.setSpacing(3)
        label = QtWidgets.QLabel(label_text)
        label.setObjectName("metricLabel")
        value = QtWidgets.QLabel(value_text)
        value.setObjectName("metricValue")
        value.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(label)
        layout.addWidget(value)
        return widget


    class RefreshablePage(QtWidgets.QWidget):
        def __init__(self, repo: Repository, on_change: Callable[[], None]):
            super().__init__()
            self.setObjectName("page")
            self.repo = repo
            self.on_change = on_change

        def refresh(self) -> None:
            pass


    class SessionPage(RefreshablePage):
        def __init__(self, repo, on_change):
            super().__init__(repo, on_change)
            layout = _page_layout(
                self,
                "首页与会话",
                "在这里建立操作者与测试身份。机器人身份始终从 V5L2 原始记录读取，"
                "不会被人工信息覆盖。",
            )
            template_row = QtWidgets.QHBoxLayout()
            self.template = QtWidgets.QComboBox()
            apply_template = QtWidgets.QPushButton("应用模板")
            apply_template.clicked.connect(self.apply_template)
            self.save_template_name = QtWidgets.QLineEdit()
            self.save_template_name.setPlaceholderText("可选：将本次信息另存为模板")
            template_row.addWidget(QtWidgets.QLabel("会话模板"))
            template_row.addWidget(self.template, 1)
            template_row.addWidget(apply_template)
            template_row.addWidget(self.save_template_name, 1)
            layout.addLayout(template_row)
            form = QtWidgets.QGridLayout()
            form.setHorizontalSpacing(14)
            form.setVerticalSpacing(8)
            self.team = QtWidgets.QLineEdit("74000M")
            self.operator = QtWidgets.QLineEdit()
            self.observer = QtWidgets.QLineEdit()
            self.test_type = QtWidgets.QComboBox()
            self.test_type.setEditable(True)
            for label, value in TEST_TYPE_ITEMS:
                self.test_type.addItem(label, value)
            self.test_case = QtWidgets.QLineEdit()
            self.dataset_role = QtWidgets.QComboBox()
            for role in DatasetRole:
                self.dataset_role.addItem(DATASET_ROLE_LABELS[role], role)
            self.surface = QtWidgets.QLineEdit()
            self.battery = QtWidgets.QLineEdit()
            self.expected_robot = QtWidgets.QLineEdit()
            self.notes = QtWidgets.QPlainTextEdit()
            self.notes.setMaximumHeight(76)
            fields = [
                ("队号 *", self.team, "观察员", self.observer),
                ("操作员 *", self.operator, "场地表面", self.surface),
                ("测试类型 *", self.test_type, "电池编号", self.battery),
                ("测试用例 ID", self.test_case, "预期机器人 ID", self.expected_robot),
                ("数据集用途", self.dataset_role, "备注", self.notes),
            ]
            for row, (left_label, left_widget, right_label, right_widget) in enumerate(fields):
                form.addWidget(QtWidgets.QLabel(left_label), row, 0)
                form.addWidget(left_widget, row, 1)
                form.addWidget(QtWidgets.QLabel(right_label), row, 2)
                form.addWidget(right_widget, row, 3)
            form.setColumnStretch(1, 1)
            form.setColumnStretch(3, 1)
            layout.addLayout(form)
            buttons = QtWidgets.QHBoxLayout()
            create = _primary(QtWidgets.QPushButton("创建会话"))
            create.clicked.connect(self.create_session)
            buttons.addWidget(create)
            buttons.addStretch()
            layout.addLayout(buttons)
            self.table = QtWidgets.QTableWidget(0, 6)
            self.table.setHorizontalHeaderLabels(
                ["会话 ID", "队号", "操作员", "测试类型", "数据用途", "状态"]
            )
            _configure_table(self.table)
            self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Stretch)
            layout.addWidget(self.table)
            self.refresh()

        def create_session(self):
            try:
                session = SessionMetadata(
                    team_number=self.team.text(),
                    operator=self.operator.text(),
                    observer=self.observer.text(),
                    test_type=self.test_type.currentData() or self.test_type.currentText(),
                    test_case_id=self.test_case.text(),
                    dataset_role=self.dataset_role.currentData(),
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
            _info(self, f"已创建会话：{session.session_id}")

        def apply_template(self):
            name = self.template.currentData()
            if not name:
                return
            session = self.repo.list_templates()[name]
            self.team.setText(session.team_number)
            self.operator.setText(session.operator)
            self.observer.setText(session.observer)
            test_index = self.test_type.findData(session.test_type)
            if test_index >= 0:
                self.test_type.setCurrentIndex(test_index)
            else:
                self.test_type.setEditText(session.test_type)
            self.test_case.setText(session.test_case_id)
            role_index = self.dataset_role.findData(session.dataset_role)
            if role_index >= 0:
                self.dataset_role.setCurrentIndex(role_index)
            self.surface.setText(session.surface)
            self.battery.setText(session.battery_id)
            self.expected_robot.setText(session.expected_robot_id)
            self.notes.setPlainText(session.notes)

        def refresh(self):
            sessions = self.repo.list_sessions()
            current_template = self.template.currentData()
            self.template.clear()
            self.template.addItem("不使用模板", None)
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
                    TEST_TYPE_LABELS.get(session.test_type, session.test_type),
                    DATASET_ROLE_LABELS[session.dataset_role],
                    SESSION_STATUS_LABELS.get(session.status.value, session.status.value),
                ]
                for column, value in enumerate(values):
                    self.table.setItem(row, column, QtWidgets.QTableWidgetItem(str(value)))


    class RecordWindowPage(RefreshablePage):
        def __init__(self, repo, on_change):
            super().__init__(repo, on_change)
            layout = _page_layout(
                self,
                "录制窗口",
                "本页按钮只记录 PC 观察时间，不会向 Brain 下发命令。Controller 长按 Y "
                "至少 3 秒并松开开始录制；长按 Y 至少 1 秒并松开停止。完整数据保存在 TF 卡。",
            )
            session_label = QtWidgets.QLabel("关联会话")
            session_label.setObjectName("sectionTitle")
            layout.addWidget(session_label)
            self.session = QtWidgets.QComboBox()
            layout.addWidget(self.session)
            buttons = QtWidgets.QHBoxLayout()
            self.start = _primary(QtWidgets.QPushButton("记录 PC 开始时间"))
            self.stop = QtWidgets.QPushButton("记录 PC 结束时间")
            self.start.clicked.connect(lambda: self.marker("START"))
            self.stop.clicked.connect(lambda: self.marker("STOP"))
            buttons.addWidget(self.start)
            buttons.addWidget(self.stop)
            layout.addLayout(buttons)
            self.status = QtWidgets.QLabel("当前应用会话中尚未记录 PC 时间标记。")
            self.status.setObjectName("statusCard")
            layout.addWidget(self.status)
            layout.addStretch()
            self.refresh()

        def refresh(self):
            current = self.session.currentData()
            self.session.clear()
            for session in self.repo.list_sessions():
                self.session.addItem(
                    f"{session.session_id} · {session.team_number} · "
                    f"{TEST_TYPE_LABELS.get(session.test_type, session.test_type)}",
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
            action_label = "开始" if action == "START" else "结束"
            self.status.setText(f"PC {action_label}时间标记 · {stamp}")


    class ImportPage(RefreshablePage):
        def __init__(self, repo, on_change):
            super().__init__(repo, on_change)
            self.pool = QtCore.QThreadPool.globalInstance()
            self.candidates = []
            layout = _page_layout(
                self,
                "导入记录",
                "扫描 TF 卡中的 V5L/TMP 文件，先检查身份与完整性，再复制为只读原始档案。",
            )
            top = QtWidgets.QHBoxLayout()
            self.path = QtWidgets.QLineEdit()
            self.path.setPlaceholderText("选择 TF 卡盘符或包含 FLIGHT 目录的文件夹")
            browse = QtWidgets.QPushButton("浏览…")
            browse.clicked.connect(self.browse)
            scan = QtWidgets.QPushButton("扫描")
            scan.clicked.connect(self.scan)
            top.addWidget(self.path, 1)
            top.addWidget(browse)
            top.addWidget(scan)
            layout.addLayout(top)
            session_label = QtWidgets.QLabel("导入到会话")
            session_label.setObjectName("sectionTitle")
            layout.addWidget(session_label)
            self.session = QtWidgets.QComboBox()
            layout.addWidget(self.session)
            self.table = QtWidgets.QTableWidget(0, 7)
            self.table.setHorizontalHeaderLabels(
                ["导入", "文件", "完整性", "机器人", "存储序号", "帧数", "时长 [s]"]
            )
            _configure_table(self.table)
            self.table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeMode.Stretch)
            layout.addWidget(self.table)
            bottom = QtWidgets.QHBoxLayout()
            self.progress = QtWidgets.QProgressBar()
            self.progress.setRange(0, 1)
            self.progress.setValue(0)
            import_selected = _primary(QtWidgets.QPushButton("导入已勾选记录"))
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
                    f"{session.session_id} · {session.team_number} · "
                    f"{TEST_TYPE_LABELS.get(session.test_type, session.test_type)}",
                    session.session_id,
                )
            if current:
                index = self.session.findData(current)
                if index >= 0:
                    self.session.setCurrentIndex(index)

        def browse(self):
            path = QtWidgets.QFileDialog.getExistingDirectory(self, "选择 TF 卡或记录目录")
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
                status_value = (
                    candidate.status.value if hasattr(candidate.status, "value") else candidate.status
                )
                values = [
                    str(candidate.path),
                    VERDICT_LABELS.get(status_value, status_value),
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
                lambda runs: (_info(self, f"已导入 {len(runs)} 段记录。"), self.on_change())
            )
            worker.signals.error.connect(lambda error: _error(self, error))
            worker.signals.finished.connect(lambda: self._busy(False))
            self.pool.start(worker)


    class IntegrityPage(RefreshablePage):
        def __init__(self, repo, on_change):
            super().__init__(repo, on_change)
            layout = _page_layout(
                self,
                "完整性校验",
                "性能结论必须建立在可追溯的数据上。缺帧、截断或身份不一致会明确标为复测或失败。",
            )
            self.run = QtWidgets.QComboBox()
            self.run.currentIndexChanged.connect(self.load)
            layout.addWidget(self.run)
            self.verdict = QtWidgets.QLabel("尚未导入记录")
            self.verdict.setObjectName("verdict")
            layout.addWidget(self.verdict)
            raw_title = QtWidgets.QLabel("原始完整性报告 · JSON")
            raw_title.setObjectName("sectionTitle")
            layout.addWidget(raw_title)
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
                self.verdict.setText("尚未导入记录")
                self.text.clear()
                return
            run = self.repo.get_run(run_id)
            path = Path(run.artifact_dir) / "integrity" / "integrity_report.json"
            data = json.loads(path.read_text(encoding="utf-8"))
            self.verdict.setText(VERDICT_LABELS.get(data["verdict"], data["verdict"]))
            self.verdict.setProperty("verdictValue", data["verdict"])
            self.style().unpolish(self.verdict)
            self.style().polish(self.verdict)
            self.text.setPlainText(json.dumps(data, indent=2, ensure_ascii=False))


    class OverviewPage(RefreshablePage):
        def __init__(self, repo, on_change):
            super().__init__(repo, on_change)
            self.pool = QtCore.QThreadPool.globalInstance()
            layout = _page_layout(
                self,
                "运行总览",
                "生成确定性指标、异常证据和静态图。缺少位姿或 PID 分项时会显示“不可用”。",
            )
            top = QtWidgets.QHBoxLayout()
            self.run = QtWidgets.QComboBox()
            self.run.currentIndexChanged.connect(self.load)
            analyze = _primary(QtWidgets.QPushButton("开始分析 / 重新生成"))
            analyze.clicked.connect(self.analyze)
            top.addWidget(self.run, 1)
            top.addWidget(analyze)
            layout.addLayout(top)
            self.cards = QtWidgets.QGridLayout()
            layout.addLayout(self.cards)
            self.anomalies = QtWidgets.QTableWidget(0, 5)
            self.anomalies.setHorizontalHeaderLabels(
                ["ID", "层级", "严重性", "摘要", "证据时间窗"]
            )
            _configure_table(self.anomalies)
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
                card = QtWidgets.QLabel("尚未分析。选择运行后点击“开始分析”。")
                card.setObjectName("statusCard")
                self.cards.addWidget(card, 0, 0)
                return
            metrics = analysis.metrics
            exec_p99 = metrics["timing"]["exec_s"]["p99"]
            values = [
                (
                    "完整性",
                    VERDICT_LABELS.get(
                        analysis.integrity_verdict.value, analysis.integrity_verdict.value
                    ),
                ),
                ("位姿", "可用" if analysis.capability["pose"] else "不可用"),
                ("PID 分项", "可用" if analysis.capability["pid_terms"] else "不可用"),
                ("录制时长", f"{metrics['duration_s']:.3f} s"),
                ("样本数", str(metrics["samples"])),
                (
                    "执行时间 p99",
                    f"{exec_p99 * 1000:.3f} ms" if exec_p99 is not None else "不可用",
                ),
                ("超期帧", str(metrics["timing"]["overrun_frames"])),
                ("异常数", str(len(analysis.anomalies))),
            ]
            for index, (label, value) in enumerate(values):
                self.cards.addWidget(_metric_widget(label, value), index // 4, index % 4)
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
        NOTE = (
            "交互图直接读取完整 Parquet 样本。静态图包含轨迹、运动、电机、"
            "能源、时序、事件和 Welch 功率谱。"
        )
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
            layout = _page_layout(
                self,
                "图表分析",
                "按完整样本交互查看信号；时间缺口保持断开，不进行静默插值。",
            )
            top = QtWidgets.QHBoxLayout()
            self.run = QtWidgets.QComboBox()
            self.run.currentIndexChanged.connect(self.load)
            self.signal = QtWidgets.QComboBox()
            self.signal.addItems(self.SIGNALS)
            self.signal.currentIndexChanged.connect(self.load)
            open_dir = QtWidgets.QPushButton("打开静态图目录")
            open_dir.clicked.connect(self.open_folder)
            top.addWidget(self.run, 1)
            top.addWidget(self.signal)
            top.addWidget(open_dir)
            layout.addLayout(top)
            self.plot = pg.PlotWidget()
            self.plot.setBackground(PAPER)
            self.plot.showGrid(x=True, y=True, alpha=0.12)
            self.plot.setLabel("bottom", "机器人单调时间", units="s")
            for axis_name in ("left", "bottom"):
                axis = self.plot.getAxis(axis_name)
                axis.setPen(pg.mkPen(RULE))
                axis.setTextPen(pg.mkPen(MUTED))
            layout.addWidget(self.plot)
            self.note = QtWidgets.QLabel(self.NOTE)
            self.note.setObjectName("pageDescription")
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
            self.note.setText(self.NOTE)
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
                self.plot.plot(time_s, values, pen=pg.mkPen(VEX_RED, width=1.6))
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
            layout = _page_layout(
                self,
                "运行对比",
                "选择至少两段已分析记录。系统会保留软件、配置、机器人身份和工况差异。",
            )
            self.runs = QtWidgets.QListWidget()
            self.runs.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.MultiSelection)
            layout.addWidget(self.runs)
            button = _primary(QtWidgets.QPushButton("对比所选运行"))
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
            layout = _page_layout(
                self,
                "LLM 报告",
                "查看面向 LLM 的可追溯证据报告。报告保留技术字段和英文数据键，便于机器读取。",
            )
            top = QtWidgets.QHBoxLayout()
            self.run = QtWidgets.QComboBox()
            self.run.currentIndexChanged.connect(self.load)
            regenerate = _primary(QtWidgets.QPushButton("重新生成报告"))
            regenerate.clicked.connect(self.regenerate)
            open_file = QtWidgets.QPushButton("打开 Markdown")
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
                else "请先分析该运行，以生成可追溯的 LLM 证据报告。"
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
            _ensure_ui_font()
            self.repo = Repository()
            self.setWindowTitle("VEX V5 飞行记录与诊断系统")
            self.resize(1480, 920)
            self.setMinimumSize(1120, 720)

            shell = QtWidgets.QWidget()
            shell.setObjectName("appShell")
            shell_layout = QtWidgets.QHBoxLayout(shell)
            shell_layout.setContentsMargins(0, 0, 0, 0)
            shell_layout.setSpacing(0)

            sidebar = QtWidgets.QWidget()
            sidebar.setObjectName("sidebar")
            sidebar.setFixedWidth(228)
            sidebar_layout = QtWidgets.QVBoxLayout(sidebar)
            sidebar_layout.setContentsMargins(0, 0, 0, 0)
            sidebar_layout.setSpacing(0)

            brand_box = QtWidgets.QWidget()
            brand_layout = QtWidgets.QVBoxLayout(brand_box)
            brand_layout.setContentsMargins(28, 28, 24, 24)
            brand_layout.setSpacing(2)
            brand = QtWidgets.QLabel("VEX\nV5")
            brand.setObjectName("brand")
            brand_caption = QtWidgets.QLabel("飞行数据实验室")
            brand_caption.setObjectName("brandCaption")
            brand_layout.addWidget(brand)
            brand_layout.addWidget(brand_caption)
            sidebar_layout.addWidget(brand_box)

            self.navigation = QtWidgets.QListWidget()
            self.navigation.setObjectName("navigation")
            self.navigation.setHorizontalScrollBarPolicy(
                QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff
            )
            self.navigation.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
            sidebar_layout.addWidget(self.navigation, 1)

            sidebar_status = QtWidgets.QLabel(
                f"●  离线分析\n只读观察与证据处理\n版本 {__version__}"
            )
            sidebar_status.setObjectName("sidebarStatus")
            sidebar_layout.addWidget(sidebar_status)
            shell_layout.addWidget(sidebar)

            content = QtWidgets.QWidget()
            content.setObjectName("contentArea")
            content_layout = QtWidgets.QVBoxLayout(content)
            content_layout.setContentsMargins(38, 24, 38, 18)
            content_layout.setSpacing(12)

            kicker = QtWidgets.QLabel("VEX V5 / 飞行数据")
            kicker.setObjectName("appKicker")
            content_layout.addWidget(kicker)
            app_title = QtWidgets.QLabel("飞行记录与诊断系统")
            app_title.setObjectName("appTitle")
            content_layout.addWidget(app_title)
            identity_row = QtWidgets.QHBoxLayout()
            self.identity = QtWidgets.QLabel("74000M · 等待导入")
            self.identity.setObjectName("identity")
            read_only = QtWidgets.QLabel("●  离线分析 · 不控制机器人")
            read_only.setObjectName("readOnlyChip")
            identity_row.addWidget(self.identity)
            identity_row.addStretch()
            identity_row.addWidget(read_only)
            content_layout.addLayout(identity_row)
            header_rule = QtWidgets.QFrame()
            header_rule.setObjectName("headerRule")
            header_rule.setFrameShape(QtWidgets.QFrame.Shape.HLine)
            content_layout.addWidget(header_rule)

            self.tabs = QtWidgets.QStackedWidget()
            content_layout.addWidget(self.tabs, 1)
            shell_layout.addWidget(content, 1)
            self.setCentralWidget(shell)

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
                "首页与会话",
                "录制窗口",
                "导入记录",
                "完整性校验",
                "运行总览",
                "图表分析",
                "运行对比",
                "LLM 报告",
            ]
            for index, (label, page) in enumerate(zip(labels, self.pages, strict=True), start=1):
                item = QtWidgets.QListWidgetItem(f"{index:02d}    {label}")
                item.setSizeHint(QtCore.QSize(220, 50))
                item.setToolTip(f"Ctrl+{index} · {label}")
                self.navigation.addItem(item)
                self.tabs.addWidget(page)
                shortcut = QtGui.QShortcut(QtGui.QKeySequence(f"Ctrl+{index}"), self)
                shortcut.activated.connect(
                    lambda page_index=index - 1: self.navigation.setCurrentRow(page_index)
                )
            self.navigation.currentRowChanged.connect(self._select_page)
            self.navigation.setCurrentRow(0)
            self.refresh_chrome()
            self.statusBar().showMessage(
                f"产物目录：{self.repo.settings.artifacts}  ·  只读分析  ·  GPT-5.6"
            )

        def _select_page(self, index: int) -> None:
            if 0 <= index < len(self.pages):
                self.tabs.setCurrentIndex(index)
                self.pages[index].refresh()

        def refresh_chrome(self) -> None:
            sessions = self.repo.list_sessions()
            runs = self.repo.list_runs()
            if runs:
                run = runs[0]
                session = self.repo.get_session(run.session_id)
                self.identity.setText(f"{session.team_number} · {run.identity.robot_id}")
            elif sessions:
                self.identity.setText(f"{sessions[0].team_number} · 等待导入")
            else:
                self.identity.setText("74000M · 等待导入")

        def refresh_all(self):
            for page in self.pages:
                page.refresh()
            self.refresh_chrome()


def main() -> int:
    if _GUI_IMPORT_ERROR is not None:
        print(
            "图形界面需要 PySide6 和 pyqtgraph。"
            "请使用 `pip install -e .` 安装项目依赖。\n"
            f"导入错误：{_GUI_IMPORT_ERROR}",
            file=sys.stderr,
        )
        return 2
    application = QtWidgets.QApplication(sys.argv)
    application.setApplicationName("VEX V5 飞行记录与诊断系统")
    application.setOrganizationName("74000M")
    application.setStyle("Fusion")
    _ensure_ui_font()
    application.setStyleSheet(SWISS_GRID_STYLE)
    window = MainWindow()
    window.show()
    if os.environ.get("STATUSMONITOR_SMOKE_TEST") == "1":
        QtCore.QTimer.singleShot(250, application.quit)
    return application.exec()


if __name__ == "__main__":
    raise SystemExit(main())
