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
from .ui_style import (
    ACCENT,
    BAD,
    IDLE,
    LINE,
    LINE_SOFT,
    OK,
    PLOT_BACKGROUND,
    PLOT_GRID_ALPHA,
    TEXT_DIM,
    TEXT_FAINT,
    WARN,
    application_stylesheet,
)
from .version import ANALYSIS_VERSION, __version__


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
    # 语义色映射：CONDITIONAL PASS 绝不渲染成 OK；FAIL/REPEAT/NOT TESTED 不隐藏。
    VERDICT_COLORS = {
        "PASS": OK,
        "CONDITIONAL PASS": WARN,
        "REPEAT": BAD,
        "FAIL": BAD,
        "NOT TESTED": IDLE,
    }
    VERDICT_STATES = {
        "PASS": "ok",
        "CONDITIONAL PASS": "warn",
        "REPEAT": "bad",
        "FAIL": "bad",
    }
    SEVERITY_COLORS = {
        "info": IDLE,
        "warning": WARN,
        "error": BAD,
    }
    METRIC_STATE_COLORS = {
        "ok": OK,
        "warn": WARN,
        "bad": BAD,
        "idle": IDLE,
    }


    def _error(parent, message: str) -> None:
        QtWidgets.QMessageBox.critical(parent, "飞行记录与诊断系统", message)


    def _info(parent, message: str) -> None:
        QtWidgets.QMessageBox.information(parent, "飞行记录与诊断系统", message)


    def _primary(button: QtWidgets.QPushButton) -> QtWidgets.QPushButton:
        button.setProperty("role", "primary")
        return button


    def _quiet(button: QtWidgets.QPushButton) -> QtWidgets.QPushButton:
        button.setProperty("role", "quiet")
        return button


    def _semantic_color(status_value: str) -> str:
        return VERDICT_COLORS.get(status_value, IDLE)


    def _verdict_state(status_value: str) -> str:
        return VERDICT_STATES.get(status_value, "idle")


    def _severity_color(severity: str) -> str:
        return SEVERITY_COLORS.get(severity, IDLE)


    def _kicker_label(text: str, object_name: str = "kicker") -> QtWidgets.QLabel:
        label = QtWidgets.QLabel(text)
        label.setObjectName(object_name)
        font = label.font()
        font.setLetterSpacing(QtGui.QFont.SpacingType.AbsoluteSpacing, 2.0)
        label.setFont(font)
        return label


    def _form_label(text: str) -> QtWidgets.QWidget:
        """表单标签容器：必填星号用 ACCENT 色（双 QLabel，确定性渲染）。"""
        container = QtWidgets.QWidget()
        row = QtWidgets.QHBoxLayout(container)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(2)
        required = text.endswith("*")
        base_text = text[:-1].rstrip() if required else text
        base = QtWidgets.QLabel(base_text)
        base.setObjectName("formLabel")
        row.addWidget(base)
        if required:
            star = QtWidgets.QLabel("*")
            star.setObjectName("formLabel")
            star.setStyleSheet(f"color: {ACCENT};")
            row.addWidget(star)
        row.addStretch()
        return container


    def _faint_label(text: str = "") -> QtWidgets.QLabel:
        label = QtWidgets.QLabel(text)
        label.setStyleSheet(f"color: {TEXT_FAINT}; font-size: 11px;")
        return label


    def _page_layout(
        page: QtWidgets.QWidget,
        title_text: str,
        description: str,
        embedded: bool = False,
    ) -> QtWidgets.QVBoxLayout:
        """页面骨架；embedded=True 时省略页内大标题与分隔线（用于结果中心 tab）。"""
        page.setObjectName("page")
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)
        if not embedded:
            title = QtWidgets.QLabel(title_text)
            title.setObjectName("pageTitle")
            layout.addWidget(title)
        if description:
            hint = QtWidgets.QLabel(description)
            hint.setObjectName("pageDescription")
            hint.setWordWrap(True)
            layout.addWidget(hint)
        if not embedded:
            rule = QtWidgets.QFrame()
            rule.setObjectName("sectionRule")
            rule.setFrameShape(QtWidgets.QFrame.Shape.HLine)
            layout.addWidget(rule)
        return layout


    def _configure_table(table: QtWidgets.QTableWidget) -> None:
        table.setAlternatingRowColors(True)
        table.setShowGrid(False)
        table.verticalHeader().setVisible(False)
        table.verticalHeader().setDefaultSectionSize(34)
        table.horizontalHeader().setDefaultAlignment(
            QtCore.Qt.AlignmentFlag.AlignLeft | QtCore.Qt.AlignmentFlag.AlignVCenter
        )
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


    def _metric_widget(
        label_text: str, value_text: str, state: str | None = None
    ) -> QtWidgets.QWidget:
        """仪表读数卡：缺失数据一律显示“不可用”（IDLE 色），不显示 0。"""
        widget = QtWidgets.QWidget()
        widget.setObjectName("metricPanel")
        layout = QtWidgets.QVBoxLayout(widget)
        layout.setContentsMargins(16, 14, 16, 14)
        layout.setSpacing(4)
        label = QtWidgets.QLabel(label_text)
        label.setObjectName("metricLabel")
        value = QtWidgets.QLabel(value_text)
        value.setObjectName("metricValue")
        if state:
            value.setProperty("state", state)
            # 内联颜色，不依赖 QSS 属性选择器单一路径：
            # "不可用" 必为 IDLE，"可用" 为 OK，判定按 verdict 语义着色。
            value.setStyleSheet(f"color: {METRIC_STATE_COLORS.get(state, IDLE)};")
        value.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(label)
        layout.addWidget(value)
        return widget


    def _select_combo_data(combo: QtWidgets.QComboBox, value: object) -> bool:
        index = combo.findData(value)
        if index < 0:
            return False
        combo.setCurrentIndex(index)
        return True


    class ActionCard(QtWidgets.QWidget):
        """首页纵向操作卡：2px 竖条 + 序号 + 标题/描述 + 箭头。"""

        clicked = QtCore.Signal()

        def __init__(
            self,
            number: str,
            title: str,
            description: str,
            callback: Callable[[], None],
            featured: bool = False,
        ):
            super().__init__()
            self.setObjectName("actionCard")
            # 自定义 QWidget 子类默认不绘制 QSS 背景/边框，必须显式打开，
            # 否则 PANEL 底 + 1px LINE 边 + hover RAISED 全部不可见。
            self.setAttribute(QtCore.Qt.WidgetAttribute.WA_StyledBackground, True)
            self.setProperty("featured", bool(featured))
            self.setProperty("hovered", False)
            self.setCursor(QtCore.Qt.CursorShape.PointingHandCursor)
            self.setMinimumHeight(104)
            self.title_text = title
            self.description_text = description

            layout = QtWidgets.QHBoxLayout(self)
            layout.setContentsMargins(0, 0, 20, 0)
            layout.setSpacing(0)
            bar = QtWidgets.QFrame()
            bar.setObjectName("actionCardBar")
            layout.addWidget(bar)
            layout.addSpacing(22)
            number_label = QtWidgets.QLabel(number)
            number_label.setObjectName("actionCardNumber")
            layout.addWidget(number_label)
            layout.addSpacing(22)
            text_column = QtWidgets.QVBoxLayout()
            text_column.setContentsMargins(0, 0, 0, 0)
            text_column.setSpacing(4)
            title_label = QtWidgets.QLabel(title)
            title_label.setObjectName("actionCardTitle")
            desc_label = QtWidgets.QLabel(description)
            desc_label.setObjectName("actionCardDesc")
            text_column.addWidget(title_label)
            text_column.addWidget(desc_label)
            layout.addLayout(text_column, 1)
            arrow = QtWidgets.QLabel("→")
            arrow.setObjectName("actionCardArrow")
            layout.addWidget(arrow)
            self.clicked.connect(callback)

        def click(self) -> None:
            """与 QPushButton.click() 等价，供测试与键盘路径使用。"""
            self.clicked.emit()

        def _set_hovered(self, hovered: bool) -> None:
            """悬停状态走动态属性；QSS 的 :hover 伪态在离屏环境下不可靠。"""
            if self.property("hovered") == hovered:
                return
            self.setProperty("hovered", hovered)
            self.style().unpolish(self)
            self.style().polish(self)

        def enterEvent(self, event: QtCore.QEnterEvent) -> None:
            self._set_hovered(True)
            super().enterEvent(event)

        def leaveEvent(self, event: QtCore.QEvent) -> None:
            self._set_hovered(False)
            super().leaveEvent(event)

        def mouseReleaseEvent(self, event: QtGui.QMouseEvent) -> None:
            if event.button() == QtCore.Qt.MouseButton.LeftButton:
                self.clicked.emit()
                event.accept()
                return
            super().mouseReleaseEvent(event)

        def keyPressEvent(self, event: QtGui.QKeyEvent) -> None:
            if event.key() in (
                QtCore.Qt.Key.Key_Return,
                QtCore.Qt.Key.Key_Enter,
                QtCore.Qt.Key.Key_Space,
            ):
                self.clicked.emit()
                event.accept()
                return
            super().keyPressEvent(event)


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
                embedded=True,
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
            buttons.addStretch()
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
                    item = QtWidgets.QTableWidgetItem(str(value))
                    if column == 2:
                        item.setForeground(QtGui.QColor(_semantic_color(status_value)))
                    self.table.setItem(row, column, item)

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
                embedded=True,
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

        def _set_verdict(self, text: str, verdict_value: str) -> None:
            self.verdict.setText(text)
            self.verdict.setProperty("verdictValue", verdict_value)
            self.style().unpolish(self.verdict)
            self.style().polish(self.verdict)

        def load(self):
            run_id = self.run.currentData()
            if not run_id:
                self._set_verdict("尚未导入记录", "")
                self.text.clear()
                return
            try:
                run = self.repo.get_run(run_id)
                path = Path(run.artifact_dir) / "integrity" / "integrity_report.json"
                data = json.loads(path.read_text(encoding="utf-8"))
            except Exception as error:
                # 报告缺失/损坏：如实显示不可用与错误内容，不伪造判定。
                self._set_verdict("完整性报告不可用", "")
                self.text.setPlainText(str(error))
                return
            self._set_verdict(
                VERDICT_LABELS.get(data["verdict"], data["verdict"]), data["verdict"]
            )
            self.text.setPlainText(json.dumps(data, indent=2, ensure_ascii=False))


    class OverviewPage(RefreshablePage):
        def __init__(self, repo, on_change):
            super().__init__(repo, on_change)
            self.pool = QtCore.QThreadPool.globalInstance()
            layout = _page_layout(
                self,
                "运行总览",
                "生成确定性指标、异常证据和静态图。缺少位姿或 PID 分项时会显示“不可用”。",
                embedded=True,
            )
            top = QtWidgets.QHBoxLayout()
            self.run = QtWidgets.QComboBox()
            self.run.currentIndexChanged.connect(self.load)
            self.analyze_button = _primary(
                QtWidgets.QPushButton("开始分析 / 重新生成")
            )
            self.analyze_button.setMaximumWidth(220)
            self.analyze_button.clicked.connect(self.analyze)
            top.addWidget(self.run, 1)
            top.addStretch()
            top.addWidget(self.analyze_button)
            layout.addLayout(top)
            self.cards = QtWidgets.QGridLayout()
            self.cards.setHorizontalSpacing(12)
            self.cards.setVerticalSpacing(12)
            layout.addLayout(self.cards)
            self.anomalies_empty = _faint_label("未检测到异常证据")
            self.anomalies_empty.setContentsMargins(4, 8, 4, 8)
            layout.addWidget(self.anomalies_empty)
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
                    item.widget().hide()
                    item.widget().deleteLater()
            self.anomalies.setRowCount(0)
            run_id = self.run.currentData()
            if not run_id:
                self._update_anomaly_visibility()
                return
            try:
                analysis = self.repo.get_analysis(run_id)
            except KeyError:
                card = QtWidgets.QLabel("尚未分析。选择运行后点击“开始分析”。")
                card.setObjectName("statusCard")
                self.cards.addWidget(card, 0, 0)
                self._update_anomaly_visibility()
                return
            metrics = analysis.metrics
            exec_p99 = metrics["timing"]["exec_s"]["p99"]
            verdict_value = analysis.integrity_verdict.value
            values = [
                (
                    "完整性",
                    VERDICT_LABELS.get(verdict_value, verdict_value),
                    _verdict_state(verdict_value),
                ),
                ("位姿", "可用" if analysis.capability["pose"] else "不可用",
                 "ok" if analysis.capability["pose"] else "idle"),
                ("PID 分项", "可用" if analysis.capability["pid_terms"] else "不可用",
                 "ok" if analysis.capability["pid_terms"] else "idle"),
                ("录制时长", f"{metrics['duration_s']:.3f} s", None),
                ("样本数", str(metrics["samples"]), None),
                (
                    "执行时间 p99",
                    f"{exec_p99 * 1000:.3f} ms" if exec_p99 is not None else "不可用",
                    None if exec_p99 is not None else "idle",
                ),
                ("超期帧", str(metrics["timing"]["overrun_frames"]), None),
                ("异常数", str(len(analysis.anomalies)), None),
            ]
            for index, (label, value, state) in enumerate(values):
                self.cards.addWidget(_metric_widget(label, value, state), index // 4, index % 4)
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
                    item = QtWidgets.QTableWidgetItem(str(value))
                    if column == 2:
                        item.setForeground(QtGui.QColor(_severity_color(anomaly["severity"])))
                    self.anomalies.setItem(row, column, item)
            self._update_anomaly_visibility()

        def _update_anomaly_visibility(self) -> None:
            has_rows = self.anomalies.rowCount() > 0
            self.anomalies.setVisible(has_rows)
            self.anomalies_empty.setVisible(not has_rows)


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
                embedded=True,
            )
            top = QtWidgets.QHBoxLayout()
            self.run = QtWidgets.QComboBox()
            self.run.currentIndexChanged.connect(self.load)
            self.signal = QtWidgets.QComboBox()
            self.signal.setMinimumWidth(240)
            self.signal.addItems(self.SIGNALS)
            self.signal.currentIndexChanged.connect(self.load)
            open_dir = QtWidgets.QPushButton("打开静态图目录")
            open_dir.clicked.connect(self.open_folder)
            top.addWidget(self.run, 1)
            top.addWidget(self.signal)
            top.addWidget(open_dir)
            top.addStretch()
            layout.addLayout(top)
            self.plot = pg.PlotWidget()
            self.plot.setBackground(PLOT_BACKGROUND)
            self.plot.showGrid(x=True, y=True, alpha=PLOT_GRID_ALPHA)
            self.plot.setLabel("bottom", "机器人单调时间", units="s")
            for axis_name in ("left", "bottom"):
                axis = self.plot.getAxis(axis_name)
                axis.setPen(pg.mkPen(LINE))
                axis.setTextPen(pg.mkPen(TEXT_FAINT))
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
                self.plot.plot(time_s, values, pen=pg.mkPen(ACCENT, width=1.8))
                self.plot.setLabel("left", signal_name)
                self.plot.setTitle(
                    f"{run_id} · {signal_name}", color=TEXT_DIM, size="12pt"
                )
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
                embedded=True,
            )
            self.runs = QtWidgets.QListWidget()
            layout.addWidget(self.runs)
            button_row = QtWidgets.QHBoxLayout()
            button = _primary(QtWidgets.QPushButton("对比所选运行"))
            button.setMaximumWidth(220)
            button.clicked.connect(self.compare)
            button_row.addWidget(button)
            button_row.addStretch()
            layout.addLayout(button_row)
            self.output = QtWidgets.QPlainTextEdit()
            self.output.setReadOnly(True)
            self.output.setPlaceholderText("勾选至少两段已分析记录后点击对比。")
            output_palette = self.output.palette()
            output_palette.setColor(
                QtGui.QPalette.ColorRole.PlaceholderText, QtGui.QColor(TEXT_FAINT)
            )
            self.output.setPalette(output_palette)
            layout.addWidget(self.output)
            self.refresh()

        def refresh(self):
            self.runs.clear()
            for run in self.repo.list_runs():
                item = QtWidgets.QListWidgetItem(f"{run.run_id} · {run.identity.robot_id}")
                item.setData(QtCore.Qt.ItemDataRole.UserRole, run.run_id)
                item.setFlags(item.flags() | QtCore.Qt.ItemFlag.ItemIsUserCheckable)
                item.setCheckState(QtCore.Qt.CheckState.Unchecked)
                self.runs.addItem(item)

        def compare(self):
            run_ids = [
                self.runs.item(index).data(QtCore.Qt.ItemDataRole.UserRole)
                for index in range(self.runs.count())
                if self.runs.item(index).checkState() == QtCore.Qt.CheckState.Checked
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
                embedded=True,
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
            top.addStretch()
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


    class HomePage(RefreshablePage):
        def __init__(self, repo, on_new, on_continue, on_history):
            super().__init__(repo, lambda: None)
            layout = QtWidgets.QHBoxLayout(self)
            layout.setContentsMargins(40, 48, 40, 48)
            layout.setSpacing(48)

            left_column = QtWidgets.QVBoxLayout()
            left_column.setContentsMargins(8, 0, 0, 0)
            left_column.setSpacing(10)
            left_column.addSpacing(40)
            left_column.addWidget(_kicker_label("VEX V5 · TF FLIGHT RECORD"))
            title = QtWidgets.QLabel("飞行记录与诊断系统")
            title.setObjectName("homeTitle")
            left_column.addWidget(title)
            subtitle = QtWidgets.QLabel(
                "面向机器人研发与工程测试的离线记录导入、完整性校验与数据诊断。"
            )
            subtitle.setObjectName("homeSubtitle")
            subtitle.setWordWrap(True)
            left_column.addWidget(subtitle)
            left_column.addSpacing(18)
            rule = QtWidgets.QFrame()
            rule.setObjectName("sectionRule")
            rule.setFrameShape(QtWidgets.QFrame.Shape.HLine)
            left_column.addWidget(rule)
            left_column.addSpacing(12)
            meta_rows = [
                ("版本", f"v{__version__}"),
                ("分析算法版本", ANALYSIS_VERSION),
            ]
            meta = QtWidgets.QGridLayout()
            meta.setHorizontalSpacing(18)
            meta.setVerticalSpacing(6)
            for row, (key, value) in enumerate(meta_rows):
                if not value:
                    continue
                key_label = QtWidgets.QLabel(key)
                key_label.setObjectName("homeMetaKey")
                value_label = QtWidgets.QLabel(value)
                value_label.setObjectName("homeMetaValue")
                meta.addWidget(key_label, row, 0)
                meta.addWidget(value_label, row, 1)
            left_column.addLayout(meta)
            left_column.addStretch()
            left_widget = QtWidgets.QWidget()
            left_widget.setLayout(left_column)
            left_widget.setFixedWidth(400)
            layout.addWidget(left_widget)

            right_column = QtWidgets.QVBoxLayout()
            right_column.setContentsMargins(0, 0, 0, 0)
            right_column.setSpacing(14)
            right_column.addStretch()
            definitions = [
                (
                    "01",
                    "新建 TF 记录",
                    "填写本次实验信息，选择 TF 卡记录并自动分析",
                    on_new,
                    True,
                ),
                (
                    "02",
                    "继续会话",
                    "选择已有会话，继续导入下一批 TF 记录",
                    on_continue,
                    False,
                ),
                (
                    "03",
                    "历史记录",
                    "查看历史会话的图表、指标和 LLM 信息包",
                    on_history,
                    False,
                ),
            ]
            self.action_buttons = []
            for number, label, description, callback, featured in definitions:
                card = ActionCard(number, label, description, callback, featured)
                right_column.addWidget(card)
                self.action_buttons.append(card)
            right_column.addStretch()
            layout.addLayout(right_column, 1)


    class LoadingPage(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            self.setObjectName("page")
            self._base_text = "正在处理"
            self._dots = 0
            self.timer = QtCore.QTimer(self)
            self.timer.setInterval(360)
            self.timer.timeout.connect(self._animate)
            outer = QtWidgets.QVBoxLayout(self)
            outer.setContentsMargins(40, 40, 40, 40)
            outer.addStretch()
            center_row = QtWidgets.QHBoxLayout()
            center_row.addStretch()
            column = QtWidgets.QVBoxLayout()
            column.setSpacing(10)
            self.code = _kicker_label("OFFLINE PIPELINE", "loadingCode")
            self.title = QtWidgets.QLabel(self._base_text)
            self.title.setObjectName("loadingTitle")
            self.detail = QtWidgets.QLabel(
                "系统正在校验原始记录、生成分析结果和 LLM 信息包，请勿移除数据源。"
            )
            self.detail.setObjectName("pageDescription")
            self.detail.setWordWrap(True)
            self.progress = QtWidgets.QProgressBar()
            self.progress.setRange(0, 0)
            progress_rule_top = self._hair_rule()
            progress_rule_bottom = self._hair_rule()
            self.steps_line = QtWidgets.QLabel(
                "完整性校验 → 只读归档 → 指标与图表分析 → LLM 信息包"
            )
            self.steps_line.setObjectName("loadingSteps")
            column.addWidget(self.code)
            column.addWidget(self.title)
            column.addWidget(self.detail)
            column.addSpacing(10)
            column.addWidget(progress_rule_top)
            column.addSpacing(8)
            column.addWidget(self.progress)
            column.addSpacing(8)
            column.addWidget(progress_rule_bottom)
            column.addSpacing(4)
            column.addWidget(self.steps_line)
            center_row.addLayout(column)
            center_row.addStretch()
            outer.addLayout(center_row)
            outer.addStretch()

        @staticmethod
        def _hair_rule() -> QtWidgets.QFrame:
            rule = QtWidgets.QFrame()
            # QSS max-height 只限制上限，空 QFrame sizeHint 高 0 会被布局压没；
            # 必须在代码里给定 1px 固定高度。
            rule.setFixedHeight(1)
            rule.setStyleSheet(f"background: {LINE_SOFT}; border: none;")
            return rule

        def start(self, message: str, detail: str = "") -> None:
            self._base_text = message
            self._dots = 0
            self.title.setText(message)
            if detail:
                self.detail.setText(detail)
            self.timer.start()

        def stop(self) -> None:
            self.timer.stop()

        def _animate(self) -> None:
            self._dots = (self._dots + 1) % 4
            self.title.setText(self._base_text + "." * self._dots)


    class SessionWizardPage(RefreshablePage):
        STEP_NAMES = ["会话信息", "实验条件", "选择 TF 记录"]

        def __init__(self, repo, on_home, on_busy, on_complete, on_failure):
            super().__init__(repo, lambda: None)
            self.on_home = on_home
            self.on_busy = on_busy
            self.on_complete = on_complete
            self.on_failure = on_failure
            self.pool = QtCore.QThreadPool.globalInstance()
            self.session_id: str | None = None
            self.candidates = []
            self.step_index = 0

            layout = QtWidgets.QVBoxLayout(self)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(0)
            top = QtWidgets.QHBoxLayout()
            top.setContentsMargins(32, 20, 32, 12)
            back_home = _quiet(QtWidgets.QPushButton("← 返回首页"))
            back_home.clicked.connect(on_home)
            self.flow_title = QtWidgets.QLabel("新建 TF 记录")
            self.flow_title.setObjectName("pageTitle")
            top.addWidget(back_home)
            top.addSpacing(12)
            top.addWidget(self.flow_title)
            top.addStretch()
            layout.addLayout(top)

            body = QtWidgets.QHBoxLayout()
            body.setContentsMargins(0, 0, 0, 0)
            body.setSpacing(0)

            rail = QtWidgets.QWidget()
            rail.setObjectName("wizardRail")
            rail.setFixedWidth(230)
            rail_layout = QtWidgets.QVBoxLayout(rail)
            rail_layout.setContentsMargins(0, 16, 0, 20)
            rail_layout.setSpacing(6)
            self.step_labels = []
            self.step_blocks = []
            for index, name in enumerate(self.STEP_NAMES, start=1):
                block = QtWidgets.QWidget()
                block.setObjectName("wizardStepBlock")
                block_layout = QtWidgets.QHBoxLayout(block)
                block_layout.setContentsMargins(20, 8, 12, 8)
                block_layout.setSpacing(10)
                number = QtWidgets.QLabel(f"{index:02d}")
                number.setObjectName("wizardStepNumber")
                label = QtWidgets.QLabel(name)
                label.setObjectName("wizardStep")
                block_layout.addWidget(number)
                block_layout.addWidget(label)
                block_layout.addStretch()
                rail_layout.addWidget(block)
                self.step_blocks.append(block)
                self.step_labels.append(label)
            rail_layout.addStretch()
            rail_hint = QtWidgets.QLabel(
                "带 * 为必填；机器人身份以 TF 原始记录为准"
            )
            rail_hint.setObjectName("wizardRailHint")
            rail_hint.setWordWrap(True)
            rail_hint.setContentsMargins(20, 0, 16, 0)
            rail_layout.addWidget(rail_hint)
            body.addWidget(rail)

            content = QtWidgets.QWidget()
            content_layout = QtWidgets.QVBoxLayout(content)
            content_layout.setContentsMargins(28, 16, 28, 12)
            content_layout.setSpacing(0)
            self.steps = QtWidgets.QStackedWidget()
            self.steps.addWidget(self._build_identity_step())
            self.steps.addWidget(self._build_conditions_step())
            self.steps.addWidget(self._build_import_step())
            content_layout.addWidget(self.steps, 1)
            body.addWidget(content, 1)
            layout.addLayout(body, 1)

            control_rule = QtWidgets.QFrame()
            control_rule.setObjectName("wizardControlRule")
            control_rule.setFrameShape(QtWidgets.QFrame.Shape.HLine)
            layout.addWidget(control_rule)
            controls = QtWidgets.QHBoxLayout()
            controls.setContentsMargins(32, 12, 32, 16)
            self.back_button = QtWidgets.QPushButton("上一步")
            self.back_button.clicked.connect(self.previous_step)
            self.next_button = _primary(QtWidgets.QPushButton("下一步"))
            self.next_button.clicked.connect(self.next_step)
            controls.addWidget(self.back_button)
            controls.addStretch()
            controls.addWidget(self.next_button)
            layout.addLayout(controls)
            self._set_step(0)

        def _form_page(self, title: str, description: str) -> tuple[QtWidgets.QWidget, QtWidgets.QFormLayout]:
            page = QtWidgets.QWidget()
            page.setObjectName("wizardPanel")
            outer = QtWidgets.QVBoxLayout(page)
            outer.setContentsMargins(28, 24, 28, 24)
            outer.setSpacing(14)
            heading = QtWidgets.QLabel(title)
            heading.setObjectName("sectionHeading")
            hint = QtWidgets.QLabel(description)
            hint.setObjectName("pageDescription")
            hint.setWordWrap(True)
            outer.addWidget(heading)
            outer.addWidget(hint)
            form = QtWidgets.QFormLayout()
            form.setHorizontalSpacing(22)
            form.setVerticalSpacing(14)
            form.setLabelAlignment(
                QtCore.Qt.AlignmentFlag.AlignRight | QtCore.Qt.AlignmentFlag.AlignVCenter
            )
            form_host = QtWidgets.QWidget()
            form_host.setMaximumWidth(760)  # 表单不拉满面板，右侧留白
            form_host.setLayout(form)
            outer.addWidget(form_host)
            outer.addStretch()
            return page, form

        def _build_identity_step(self) -> QtWidgets.QWidget:
            page, form = self._form_page(
                "这次记录属于谁？",
                "带 * 的信息用于建立会话身份；机器人身份仍以 TF 原始记录为准。",
            )
            self.team = QtWidgets.QLineEdit("74000M")
            self.operator = QtWidgets.QLineEdit()
            self.test_type = QtWidgets.QComboBox()
            self.test_type.setEditable(True)
            for label, value in TEST_TYPE_ITEMS:
                self.test_type.addItem(label, value)
            self.test_case = QtWidgets.QLineEdit()
            self.test_case.setPlaceholderText("例如：DRIVE-STRAIGHT-01")
            self.dataset_role = QtWidgets.QComboBox()
            for role in DatasetRole:
                self.dataset_role.addItem(DATASET_ROLE_LABELS[role], role)
            form.addRow(_form_label("队号 *"), self.team)
            form.addRow(_form_label("操作员 *"), self.operator)
            form.addRow(_form_label("测试类型 *"), self.test_type)
            form.addRow(_form_label("测试用例 ID"), self.test_case)
            form.addRow(_form_label("数据集用途"), self.dataset_role)
            return page

        def _build_conditions_step(self) -> QtWidgets.QWidget:
            page, form = self._form_page(
                "补充实验条件",
                "这些信息会进入报告，帮助区分场地、电池和操作者造成的差异。",
            )
            self.observer = QtWidgets.QLineEdit()
            self.surface = QtWidgets.QLineEdit()
            self.battery = QtWidgets.QLineEdit()
            self.expected_robot = QtWidgets.QLineEdit()
            self.notes = QtWidgets.QPlainTextEdit()
            self.notes.setMaximumHeight(100)
            form.addRow(_form_label("观察员"), self.observer)
            form.addRow(_form_label("场地表面"), self.surface)
            form.addRow(_form_label("电池编号"), self.battery)
            form.addRow(_form_label("预期机器人 ID"), self.expected_robot)
            form.addRow(_form_label("备注"), self.notes)
            return page

        def _build_import_step(self) -> QtWidgets.QWidget:
            page = QtWidgets.QWidget()
            page.setObjectName("wizardPanel")
            layout = QtWidgets.QVBoxLayout(page)
            layout.setContentsMargins(28, 24, 28, 24)
            layout.setSpacing(14)
            heading = QtWidgets.QLabel("选择要导入的 TF 记录")
            heading.setObjectName("sectionHeading")
            hint = QtWidgets.QLabel(
                "选择 TF 卡盘符或包含 FLIGHT 文件夹的目录。扫描后可取消不需要的记录。"
            )
            hint.setObjectName("pageDescription")
            hint.setWordWrap(True)
            layout.addWidget(heading)
            layout.addWidget(hint)
            path_row = QtWidgets.QHBoxLayout()
            self.path = QtWidgets.QLineEdit()
            self.path.setPlaceholderText("选择 TF 卡或记录目录")
            browse = QtWidgets.QPushButton("选择目录…")
            browse.clicked.connect(self.browse)
            self.scan_button = _primary(QtWidgets.QPushButton("扫描记录"))
            self.scan_button.clicked.connect(self.scan)
            path_row.addWidget(self.path, 1)
            path_row.addWidget(browse)
            path_row.addWidget(self.scan_button)
            layout.addLayout(path_row)
            self.scan_progress = QtWidgets.QProgressBar()
            self.scan_progress.setRange(0, 1)
            self.scan_progress.setValue(0)
            layout.addWidget(self.scan_progress)
            self.table = QtWidgets.QTableWidget(0, 7)
            self.table.setHorizontalHeaderLabels(
                ["导入", "文件", "完整性", "机器人", "序号", "帧数", "时长 [s]"]
            )
            _configure_table(self.table)
            # 列宽策略：完整性为语义关键列，固定宽且禁止截断；
            # 文件列 Stretch + ElideMiddle 保留盘符与文件名，完整路径进 ToolTip。
            self.table.setTextElideMode(QtCore.Qt.TextElideMode.ElideMiddle)
            header = self.table.horizontalHeader()
            header.setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Interactive)
            header.setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeMode.Stretch)
            header.setStretchLastSection(False)
            self.table.setColumnWidth(0, 44)
            self.table.setColumnWidth(2, 160)
            self.table.setColumnWidth(3, 90)
            self.table.setColumnWidth(4, 70)
            self.table.setColumnWidth(5, 80)
            self.table.setColumnWidth(6, 90)
            layout.addWidget(self.table, 1)
            return page

        def start_new(self) -> None:
            self.session_id = None
            self.flow_title.setText("新建 TF 记录")
            self.team.setText("74000M")
            self.operator.clear()
            self.observer.clear()
            self.test_type.setCurrentIndex(0)
            self.test_case.clear()
            self.dataset_role.setCurrentIndex(0)
            self.surface.clear()
            self.battery.clear()
            self.expected_robot.clear()
            self.notes.clear()
            self.path.clear()
            self.candidates = []
            self.table.setRowCount(0)
            self.scan_progress.setRange(0, 1)
            self.scan_progress.setValue(0)
            self._set_step(0)

        def continue_session(self, session_id: str) -> None:
            session = self.repo.get_session(session_id)
            self.session_id = session_id
            self.flow_title.setText("继续会话")
            self.team.setText(session.team_number)
            self.operator.setText(session.operator)
            self.observer.setText(session.observer)
            if not _select_combo_data(self.test_type, session.test_type):
                self.test_type.setEditText(session.test_type)
            self.test_case.setText(session.test_case_id)
            _select_combo_data(self.dataset_role, session.dataset_role)
            self.surface.setText(session.surface)
            self.battery.setText(session.battery_id)
            self.expected_robot.setText(session.expected_robot_id)
            self.notes.setPlainText(session.notes)
            self.path.clear()
            self.candidates = []
            self.table.setRowCount(0)
            self._set_step(2)

        def _set_step(self, index: int) -> None:
            self.step_index = max(0, min(index, len(self.STEP_NAMES) - 1))
            self.steps.setCurrentIndex(self.step_index)
            for position, (block, label) in enumerate(
                zip(self.step_blocks, self.step_labels)
            ):
                if position < self.step_index:
                    label_state = "done"
                elif position == self.step_index:
                    label_state = "active"
                else:
                    label_state = "idle"
                label.setProperty("state", label_state)
                block.setProperty(
                    "state", "active" if position == self.step_index else "idle"
                )
                for widget in (block, label):
                    widget.style().unpolish(widget)
                    widget.style().polish(widget)
            self.back_button.setEnabled(self.step_index > 0)
            self.next_button.setText(
                "导入、分析并查看结果" if self.step_index == 2 else "下一步"
            )

        def previous_step(self) -> None:
            self._set_step(self.step_index - 1)

        def next_step(self) -> None:
            if self.step_index == 0:
                if not self.team.text().strip() or not self.operator.text().strip():
                    _error(self, "请填写队号和操作员。")
                    return
                if not (self.test_type.currentData() or self.test_type.currentText().strip()):
                    _error(self, "请选择或填写测试类型。")
                    return
                self._set_step(1)
                return
            if self.step_index == 1:
                self._set_step(2)
                return
            self.import_and_analyze()

        def browse(self) -> None:
            path = QtWidgets.QFileDialog.getExistingDirectory(self, "选择 TF 卡或记录目录")
            if path:
                self.path.setText(path)

        def scan(self) -> None:
            source = self.path.text().strip()
            if not source:
                _error(self, "请先选择 TF 卡或记录目录。")
                return
            self.scan_button.setEnabled(False)
            self.scan_progress.setRange(0, 0)
            worker = Worker(scan_recordings, source)
            worker.signals.result.connect(self.show_candidates)
            worker.signals.error.connect(lambda error: _error(self, error))
            worker.signals.finished.connect(self._scan_finished)
            self.pool.start(worker)

        def _scan_finished(self) -> None:
            self.scan_button.setEnabled(True)
            self.scan_progress.setRange(0, 1)
            self.scan_progress.setValue(1)

        def show_candidates(self, candidates) -> None:
            self.candidates = candidates
            self.table.setRowCount(len(candidates))
            for row, candidate in enumerate(candidates):
                check = QtWidgets.QTableWidgetItem()
                check.setFlags(check.flags() | QtCore.Qt.ItemFlag.ItemIsUserCheckable)
                check.setCheckState(QtCore.Qt.CheckState.Checked)
                self.table.setItem(row, 0, check)
                status_value = (
                    candidate.status.value
                    if hasattr(candidate.status, "value")
                    else candidate.status
                )
                values = [
                    # 文件列只显示文件名，杜绝长路径把语义列挤出可视区；
                    # 完整路径放 ToolTip 与 StatusTip，可随时核对来源。
                    candidate.path.name,
                    VERDICT_LABELS.get(status_value, status_value),
                    candidate.robot_id,
                    candidate.storage_sequence,
                    candidate.frames,
                    f"{candidate.duration_s:.3f}",
                ]
                for column, value in enumerate(values, start=1):
                    item = QtWidgets.QTableWidgetItem(str(value))
                    if column == 1:
                        item.setToolTip(str(candidate.path))
                        item.setStatusTip(str(candidate.path))
                    if column == 2:
                        item.setForeground(QtGui.QColor(_semantic_color(status_value)))
                    self.table.setItem(row, column, item)
            if not candidates:
                _info(self, "没有找到可识别的 V5L/TMP 记录。")

        def _metadata_fields(self) -> dict:
            return {
                "team_number": self.team.text(),
                "operator": self.operator.text(),
                "observer": self.observer.text(),
                "test_type": self.test_type.currentData() or self.test_type.currentText(),
                "test_case_id": self.test_case.text(),
                "dataset_role": self.dataset_role.currentData(),
                "surface": self.surface.text(),
                "battery_id": self.battery.text(),
                "expected_robot_id": self.expected_robot.text(),
                "notes": self.notes.toPlainText(),
            }

        def import_and_analyze(self) -> None:
            selected = [
                candidate.path
                for row, candidate in enumerate(self.candidates)
                if self.table.item(row, 0) is not None
                and self.table.item(row, 0).checkState() == QtCore.Qt.CheckState.Checked
            ]
            if not selected:
                _error(self, "请先扫描并至少选择一段记录。")
                return
            fields = self._metadata_fields()
            try:
                if self.session_id:
                    metadata = self.repo.get_session(self.session_id).model_copy(update=fields)
                else:
                    metadata = SessionMetadata(**fields)
            except Exception as error:
                _error(self, str(error))
                return
            existing_session_id = self.session_id
            self.on_busy(
                "正在生成本次记录",
                f"共 {len(selected)} 段文件：校验 → 归档 → 分析 → 生成 LLM 报告",
            )

            def execute():
                if existing_session_id:
                    self.repo.save_session(metadata)
                else:
                    self.repo.create_session(metadata)
                service = ImportService(self.repo)
                pipeline = AnalysisPipeline(self.repo)
                runs = []
                for source in selected:
                    run = service.import_recording(metadata.session_id, source)
                    pipeline.analyze(run.run_id)
                    runs.append(run)
                return metadata.session_id, [run.run_id for run in runs]

            worker = Worker(execute)
            worker.signals.result.connect(
                lambda result: self.on_complete(result[0], result[1])
            )
            worker.signals.error.connect(self.on_failure)
            self.pool.start(worker)


    class SessionPickerPage(RefreshablePage):
        def __init__(self, repo, on_home, on_continue, on_history):
            super().__init__(repo, lambda: None)
            self.on_home = on_home
            self.on_continue = on_continue
            self.on_history = on_history
            self.mode = "continue"
            layout = QtWidgets.QVBoxLayout(self)
            layout.setContentsMargins(32, 24, 32, 20)
            layout.setSpacing(12)
            top = QtWidgets.QHBoxLayout()
            home = _quiet(QtWidgets.QPushButton("← 返回首页"))
            home.clicked.connect(on_home)
            self.title = QtWidgets.QLabel("继续会话")
            self.title.setObjectName("pageTitle")
            top.addWidget(home)
            top.addSpacing(12)
            top.addWidget(self.title)
            top.addStretch()
            layout.addLayout(top)
            self.description = QtWidgets.QLabel()
            self.description.setObjectName("pageDescription")
            layout.addWidget(self.description)
            self.table = QtWidgets.QTableWidget(0, 7)
            self.table.setHorizontalHeaderLabels(
                ["会话", "更新时间", "队号", "操作员", "测试类型", "记录数", "状态"]
            )
            _configure_table(self.table)
            self.table.verticalHeader().setDefaultSectionSize(36)
            # 会话列 Stretch，其余列内容宽（ResizeToContents）：
            # 1120 宽下更新时间等任何一列都不会被截断。
            self.table.setTextElideMode(QtCore.Qt.TextElideMode.ElideRight)
            header = self.table.horizontalHeader()
            header.setSectionResizeMode(
                QtWidgets.QHeaderView.ResizeMode.ResizeToContents
            )
            header.setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeMode.Stretch)
            header.setStretchLastSection(False)
            self.table.doubleClicked.connect(lambda _: self.open_selected())
            layout.addWidget(self.table, 1)
            bottom = QtWidgets.QHBoxLayout()
            self.empty = QtWidgets.QLabel()
            self.empty.setObjectName("pageDescription")
            self.open_button = _primary(QtWidgets.QPushButton())
            self.open_button.clicked.connect(self.open_selected)
            bottom.addWidget(self.empty)
            bottom.addStretch()
            bottom.addWidget(self.open_button)
            layout.addLayout(bottom)

        def set_mode(self, mode: str) -> None:
            self.mode = mode
            if mode == "history":
                self.title.setText("历史记录")
                self.description.setText("选择一个历史会话，一键查看图表、指标与信息包。")
                self.open_button.setText("查看图表与信息")
            else:
                self.title.setText("继续会话")
                self.description.setText("选择一个已有会话，继续导入下一批 TF 记录。")
                self.open_button.setText("继续导入 TF 记录")
            self.refresh()

        def refresh(self) -> None:
            sessions = self.repo.list_sessions()
            self.table.setRowCount(len(sessions))
            for row, session in enumerate(sessions):
                run_count = len(self.repo.list_runs(session.session_id))
                local_time = session.updated_at.astimezone()
                values = [
                    session.session_id,
                    # 缩短时间格式保证窄窗口不截断；完整时间放 ToolTip。
                    local_time.strftime("%m-%d %H:%M"),
                    session.team_number,
                    session.operator,
                    TEST_TYPE_LABELS.get(session.test_type, session.test_type),
                    run_count,
                    SESSION_STATUS_LABELS.get(session.status.value, session.status.value),
                ]
                for column, value in enumerate(values):
                    item = QtWidgets.QTableWidgetItem(str(value))
                    if column == 0:
                        item.setData(QtCore.Qt.ItemDataRole.UserRole, session.session_id)
                        item.setToolTip(session.session_id)
                    if column == 1:
                        item.setToolTip(local_time.strftime("%Y-%m-%d %H:%M"))
                    self.table.setItem(row, column, item)
            if sessions:
                self.table.selectRow(0)
                self.empty.clear()
                self.open_button.setEnabled(True)
            else:
                self.empty.setText("还没有会话，请先新建 TF 记录。")
                self.open_button.setEnabled(False)

        def open_selected(self) -> None:
            row = self.table.currentRow()
            item = self.table.item(row, 0) if row >= 0 else None
            session_id = item.data(QtCore.Qt.ItemDataRole.UserRole) if item else None
            if not session_id:
                return
            if self.mode == "history":
                self.on_history(session_id)
            else:
                self.on_continue(session_id)


    class ResultsPage(RefreshablePage):
        def __init__(self, repo, on_home, on_continue, on_change):
            super().__init__(repo, on_change)
            self.session_id: str | None = None
            layout = QtWidgets.QVBoxLayout(self)
            layout.setContentsMargins(28, 20, 28, 26)
            layout.setSpacing(12)
            top = QtWidgets.QHBoxLayout()
            home = _quiet(QtWidgets.QPushButton("← 首页"))
            home.clicked.connect(on_home)
            self.title = QtWidgets.QLabel("会话结果")
            self.title.setObjectName("pageTitle")
            self.run = QtWidgets.QComboBox()
            self.run.setMinimumWidth(280)
            self.run.currentIndexChanged.connect(self._sync_run)
            continue_button = QtWidgets.QPushButton("继续导入 TF 记录")
            continue_button.clicked.connect(
                lambda: on_continue(self.session_id) if self.session_id else None
            )
            top.addWidget(home)
            top.addSpacing(10)
            top.addWidget(self.title)
            top.addStretch()
            top.addWidget(_faint_label("当前记录"))
            top.addWidget(self.run)
            top.addWidget(continue_button)
            layout.addLayout(top)
            self.summary = QtWidgets.QLabel()
            self.summary.setObjectName("pageDescription")
            layout.addWidget(self.summary)

            self.views = QtWidgets.QTabWidget()
            self.views.setObjectName("resultTabs")
            self.overview = OverviewPage(repo, on_change)
            self.plots = PlotsPage(repo, on_change)
            self.integrity = IntegrityPage(repo, on_change)
            self.report = ReportPage(repo, on_change)
            self.compare = ComparePage(repo, on_change)
            self.record_window = RecordWindowPage(repo, on_change)
            for page in [self.overview, self.plots, self.integrity, self.report]:
                page.run.setVisible(False)
            self.views.addTab(self.overview, "运行总览")
            self.views.addTab(self.plots, "图表分析")
            self.views.addTab(self.integrity, "完整性")
            self.views.addTab(self.report, "LLM 信息包")
            self.views.addTab(self.compare, "运行对比")
            self.views.addTab(self.record_window, "会话工具")
            layout.addWidget(self.views, 1)

        def show_session(self, session_id: str, run_id: str | None = None) -> None:
            self.session_id = session_id
            session = self.repo.get_session(session_id)
            runs = self.repo.list_runs(session_id)
            self.title.setText(f"{session.team_number} · {TEST_TYPE_LABELS.get(session.test_type, session.test_type)}")
            self.summary.setText(
                f"会话 {session.session_id}　·　操作员 {session.operator}　·　"
                f"{len(runs)} 段记录　·　{SESSION_STATUS_LABELS.get(session.status.value, session.status.value)}"
            )
            self.run.blockSignals(True)
            self.run.clear()
            for run in runs:
                self.run.addItem(
                    f"{run.identity.robot_id} · 记录 {run.identity.storage_sequence:06d} · {run.run_id}",
                    run.run_id,
                )
            self.run.blockSignals(False)
            if run_id:
                _select_combo_data(self.run, run_id)
            self.refresh()

        def refresh(self) -> None:
            if not self.session_id:
                return
            selected = self.run.currentData()
            for page in [
                self.overview,
                self.plots,
                self.integrity,
                self.report,
                self.compare,
                self.record_window,
            ]:
                page.refresh()
            _select_combo_data(self.record_window.session, self.session_id)
            if selected:
                self._select_embedded_run(selected)

        def _sync_run(self) -> None:
            run_id = self.run.currentData()
            if run_id:
                self._select_embedded_run(run_id)

        def _select_embedded_run(self, run_id: str) -> None:
            for page in [self.overview, self.plots, self.integrity, self.report]:
                if _select_combo_data(page.run, run_id):
                    page.load()


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
            shell_layout = QtWidgets.QVBoxLayout(shell)
            shell_layout.setContentsMargins(0, 0, 0, 0)
            shell_layout.setSpacing(0)
            header = QtWidgets.QWidget()
            header.setObjectName("topBar")
            header.setFixedHeight(52)
            header_layout = QtWidgets.QHBoxLayout(header)
            header_layout.setContentsMargins(24, 0, 24, 0)
            header_layout.setSpacing(0)
            brand_mark = QtWidgets.QLabel()
            brand_mark.setObjectName("brandMark")
            brand_mark.setFixedSize(8, 8)
            brand = QtWidgets.QPushButton("74000M · 飞行记录与诊断")
            brand.setObjectName("topBrand")
            brand.clicked.connect(self.show_home)
            offline = QtWidgets.QLabel("●  离线分析 · 不控制机器人")
            offline.setObjectName("offlineChip")
            version = QtWidgets.QLabel(f"v{__version__} · 离线版")
            version.setObjectName("topMeta")
            header_layout.addWidget(brand_mark)
            header_layout.addSpacing(10)
            header_layout.addWidget(brand)
            header_layout.addStretch()
            header_layout.addWidget(offline)
            header_layout.addSpacing(12)
            header_layout.addWidget(version)
            shell_layout.addWidget(header)

            self.tabs = QtWidgets.QStackedWidget()
            shell_layout.addWidget(self.tabs, 1)

            strip = QtWidgets.QWidget()
            strip.setObjectName("statusStrip")
            strip.setFixedHeight(26)
            strip_layout = QtWidgets.QHBoxLayout(strip)
            strip_layout.setContentsMargins(24, 0, 24, 0)
            strip_layout.setSpacing(0)
            workspace_label = QtWidgets.QLabel("本地离线工作区 · 自动保存")
            workspace_label.setObjectName("statusWorkspace")
            note_label = QtWidgets.QLabel("PC 时间标记仅记录本地时间 · 不下发机器人")
            note_label.setObjectName("statusNote")
            strip_layout.addWidget(workspace_label)
            strip_layout.addStretch()
            strip_layout.addWidget(note_label)
            shell_layout.addWidget(strip)
            self.setCentralWidget(shell)

            self.home_page = HomePage(
                self.repo,
                self.start_new,
                lambda: self.show_picker("continue"),
                lambda: self.show_picker("history"),
            )
            self.wizard_page = SessionWizardPage(
                self.repo,
                self.show_home,
                self.show_loading,
                self._import_complete,
                self._import_failed,
            )
            self.picker_page = SessionPickerPage(
                self.repo,
                self.show_home,
                self.continue_session,
                self.open_results,
            )
            self.results_page = ResultsPage(
                self.repo,
                self.show_home,
                self.continue_session,
                self.refresh_all,
            )
            self.loading_page = LoadingPage()
            self.pages = [
                self.home_page,
                self.wizard_page,
                self.picker_page,
                self.results_page,
                self.loading_page,
            ]
            for page in self.pages:
                self.tabs.addWidget(page)
            self.show_home()

        def show_home(self) -> None:
            self.loading_page.stop()
            self.tabs.setCurrentWidget(self.home_page)

        def start_new(self) -> None:
            self.wizard_page.start_new()
            self.tabs.setCurrentWidget(self.wizard_page)

        def show_picker(self, mode: str) -> None:
            self.picker_page.set_mode(mode)
            self.tabs.setCurrentWidget(self.picker_page)

        def continue_session(self, session_id: str | None) -> None:
            if not session_id:
                return
            self.wizard_page.continue_session(session_id)
            self.tabs.setCurrentWidget(self.wizard_page)

        def show_loading(self, message: str, detail: str = "") -> None:
            self.loading_page.start(message, detail)
            self.tabs.setCurrentWidget(self.loading_page)

        def _import_complete(self, session_id: str, run_ids: list[str]) -> None:
            self.loading_page.stop()
            self.results_page.show_session(
                session_id, run_ids[-1] if run_ids else None
            )
            self.tabs.setCurrentWidget(self.results_page)

        def _import_failed(self, error: str) -> None:
            self.loading_page.stop()
            self.tabs.setCurrentWidget(self.wizard_page)
            _error(self, error)

        def open_results(self, session_id: str) -> None:
            runs = self.repo.list_runs(session_id)
            if not runs:
                _info(self, "这个会话还没有导入记录，可从“继续会话”添加 TF 数据。")
                return
            latest = runs[0]
            try:
                self.repo.get_analysis(latest.run_id)
            except KeyError:
                self.show_loading("正在生成历史会话信息", "首次打开该记录，需要完成离线分析。")
                worker = Worker(AnalysisPipeline(self.repo).analyze, latest.run_id)
                worker.signals.result.connect(
                    lambda _: self._import_complete(session_id, [latest.run_id])
                )
                worker.signals.error.connect(self._history_failed)
                QtCore.QThreadPool.globalInstance().start(worker)
                return
            self.results_page.show_session(session_id, latest.run_id)
            self.tabs.setCurrentWidget(self.results_page)

        def _history_failed(self, error: str) -> None:
            self.loading_page.stop()
            self.tabs.setCurrentWidget(self.picker_page)
            _error(self, error)

        def refresh_all(self):
            self.picker_page.refresh()
            self.results_page.refresh()


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
    application.setStyleSheet(application_stylesheet())
    window = MainWindow()
    window.show()
    if os.environ.get("STATUSMONITOR_SMOKE_TEST") == "1":
        QtCore.QTimer.singleShot(250, application.quit)
    return application.exec()


if __name__ == "__main__":
    raise SystemExit(main())
