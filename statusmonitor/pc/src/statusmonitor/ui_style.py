"""深色工程仪器视觉系统。

本文件是 PC 前端视觉的唯一实现来源，令牌与样式逐项对应
项目根目录 ``design_frontend.md``（深色工程仪器控制台）。
只包含表现层定义，不含任何业务/数据逻辑。
"""

# --- 表面层级（暖调石墨，哑光分层） ----------------------------------------
SHELL = "#14161A"  # 窗口最底层 / 图表底
CANVAS = "#191C21"  # 页面背景
PANEL = "#1F232A"  # 卡片 / 面板 / 输入底
RAISED = "#262B33"  # 悬停、浮起元素
OVERLAY = "#2D333D"  # 按下 / 激活
LINE = "#32383F"  # 结构发丝线（1px）
LINE_SOFT = "#282D34"  # 次级分隔线（表格行线）

# --- 文字 -------------------------------------------------------------------
TEXT = "#E8E9EC"  # 主文字
TEXT_DIM = "#A3A9B2"  # 次要文字 / 说明
TEXT_FAINT = "#6B7280"  # 元信息 / 占位 / 禁用

# --- 强调色（信号琥珀，全局唯一强调色） --------------------------------------
ACCENT = "#E8A33D"
ACCENT_HOVER = "#F2B45A"
ACCENT_PRESS = "#C9882C"
ACCENT_TEXT = "#171A1E"  # 琥珀底上的文字
ACCENT_SOFT = "#2E2A20"  # rgba(232,163,61,0.14) 的实心近似，选中底色

# --- 语义色（判定 / 状态；绝不美化 FAIL/REPEAT/NOT TESTED） -------------------
OK = "#57B98A"  # PASS / 可用
WARN = "#D9A13B"  # CONDITIONAL PASS
BAD = "#DE6E62"  # FAIL / REPEAT / 错误
IDLE = "#6B7280"  # NOT TESTED / 不可用 / 草稿

OK_BG = "#223129"  # 语义 chip 底色（12% 透明度实心近似）
WARN_BG = "#332D1D"
BAD_BG = "#3A2624"
IDLE_BG = "#262A30"

# --- 图表系列色（数据用色，与 UI 铬件分离） -----------------------------------
SERIES_1 = "#E8A33D"  # 主系列琥珀
SERIES_2 = "#7EA8C9"  # 钢蓝
SERIES_3 = "#57B98A"
SERIES_4 = "#DE6E62"
SERIES_5 = "#A3A9B2"
PLOT_BACKGROUND = SHELL
PLOT_GRID_ALPHA = 0.06

UI_FONT_STACK = '"Noto Sans SC", "Microsoft YaHei UI", "Microsoft YaHei", "Segoe UI", sans-serif'
MONO_FONT_STACK = '"Cascadia Mono", "Consolas", monospace'

INSTRUMENT_STYLE = f"""
* {{
    font-family: {UI_FONT_STACK};
    font-size: 13px;
    color: {TEXT};
}}
QMainWindow, QWidget#appShell, QWidget#contentArea, QWidget#page {{
    background: {CANVAS};
}}

/* ---------- 顶栏（高 52，PANEL 底，下 1px LINE） ---------- */
QWidget#topBar {{
    background: {PANEL};
    border-bottom: 1px solid {LINE};
}}
QLabel#brandMark {{
    background: {ACCENT};
    min-width: 8px;
    max-width: 8px;
    min-height: 8px;
    max-height: 8px;
}}
QPushButton#topBrand {{
    color: {TEXT};
    background: transparent;
    border: none;
    min-height: 24px;
    padding: 0;
    font-size: 13px;
    font-weight: 600;
    text-align: left;
}}
QPushButton#topBrand:hover {{
    color: {ACCENT};
    background: transparent;
}}
QLabel#offlineChip {{
    color: {OK};
    background: transparent;
    border: 1px solid #2F4A3A;
    border-radius: 3px;
    padding: 3px 10px;
    font-size: 11px;
}}
QLabel#topMeta {{
    color: {TEXT_FAINT};
    font-size: 11px;
}}

/* ---------- 底栏状态条（高 26，SHELL 底，上 1px LINE_SOFT） ---------- */
QWidget#statusStrip {{
    background: {SHELL};
    border-top: 1px solid {LINE_SOFT};
}}
QLabel#statusPath, QLabel#statusNote {{
    color: {TEXT_FAINT};
    font-size: 11px;
}}
QLabel#statusPath {{
    font-family: {MONO_FONT_STACK};
}}

QToolTip {{
    background: {RAISED};
    color: {TEXT};
    border: 1px solid {LINE};
    padding: 6px 8px;
}}

/* ---------- 通用文字层级 ---------- */
QLabel#kicker {{
    color: {ACCENT};
    font-size: 11px;
    font-weight: 600;
}}
QLabel#pageTitle {{
    color: {TEXT};
    font-size: 22px;
    font-weight: 500;
    padding: 0;
}}
QLabel#pageDescription {{
    color: {TEXT_DIM};
    font-size: 12px;
    padding: 0 0 6px 0;
}}
QLabel#sectionTitle {{
    color: {TEXT};
    font-size: 14px;
    font-weight: 600;
    padding: 7px 0 3px 0;
}}
QLabel#sectionHeading {{
    color: {TEXT};
    font-size: 17px;
    font-weight: 600;
}}
QFrame#headerRule, QFrame#sectionRule {{
    color: {LINE};
    background: {LINE};
    border: none;
    max-height: 1px;
}}
QLabel#formLabel {{
    color: {TEXT_DIM};
    font-size: 13px;
}}
QLabel#monoText {{
    font-family: {MONO_FONT_STACK};
}}

/* ---------- 首页 ---------- */
QLabel#homeTitle {{
    color: {TEXT};
    font-size: 30px;
    font-weight: 500;
}}
QLabel#homeSubtitle {{
    color: {TEXT_DIM};
    font-size: 12px;
}}
QLabel#homeMetaKey {{
    color: {TEXT_FAINT};
    font-size: 11px;
}}
QLabel#homeMetaValue {{
    color: {TEXT_DIM};
    font-size: 11px;
    font-family: {MONO_FONT_STACK};
}}
QWidget#actionCard {{
    background: {PANEL};
    border: 1px solid {LINE};
}}
QWidget#actionCard[hovered="true"] {{
    background: {RAISED};
    border-color: #49505A;
}}
QFrame#actionCardBar {{
    background: {LINE};
    border: none;
    max-width: 2px;
    min-width: 2px;
}}
QWidget#actionCard[hovered="true"] QFrame#actionCardBar {{
    background: {ACCENT};
}}
QWidget#actionCard[featured="true"] QFrame#actionCardBar {{
    background: {ACCENT};
}}
QLabel#actionCardNumber {{
    color: {TEXT_FAINT};
    font-family: {MONO_FONT_STACK};
    font-size: 20px;
}}
QLabel#actionCardTitle {{
    color: {TEXT};
    font-size: 16px;
    font-weight: 600;
}}
QLabel#actionCardDesc {{
    color: {TEXT_DIM};
    font-size: 12px;
}}
QLabel#actionCardArrow {{
    color: {TEXT_FAINT};
    font-size: 16px;
}}
QWidget#actionCard[hovered="true"] QLabel#actionCardArrow {{
    color: {ACCENT};
}}

/* ---------- 向导：左侧步骤轨 ---------- */
QWidget#wizardRail {{
    background: {PANEL};
    border-right: 1px solid {LINE};
}}
QWidget#wizardStepBlock {{
    background: transparent;
    border-left: 2px solid transparent;
}}
QWidget#wizardStepBlock[state="active"] {{
    border-left: 2px solid {ACCENT};
}}
QLabel#wizardStepNumber {{
    color: {TEXT_FAINT};
    font-family: {MONO_FONT_STACK};
    font-size: 12px;
}}
QLabel#wizardStep {{
    color: {TEXT_FAINT};
    font-size: 13px;
}}
QLabel#wizardStep[state="active"] {{
    color: {TEXT};
}}
QLabel#wizardStep[state="done"] {{
    color: {TEXT_DIM};
}}
QLabel#wizardRailHint {{
    color: {TEXT_FAINT};
    font-size: 12px;
}}
QWidget#wizardPanel {{
    background: {PANEL};
    border: 1px solid {LINE};
}}
QFrame#wizardControlRule {{
    color: {LINE};
    background: {LINE};
    border: none;
    max-height: 1px;
}}

/* ---------- 加载页 ---------- */
QLabel#loadingCode {{
    color: {ACCENT};
    font-size: 11px;
    font-weight: 600;
}}
QLabel#loadingTitle {{
    color: {TEXT};
    font-size: 22px;
    font-weight: 500;
}}
QLabel#loadingSteps {{
    color: {TEXT_FAINT};
    font-family: {MONO_FONT_STACK};
    font-size: 12px;
}}

/* ---------- 状态卡 / 指标卡 ---------- */
QLabel#statusCard {{
    background: {PANEL};
    border: 1px solid {LINE};
    padding: 13px 16px;
    color: {TEXT_DIM};
    font-size: 12px;
}}
QWidget#metricPanel {{
    background: {PANEL};
    border: 1px solid {LINE};
}}
QLabel#metricLabel {{
    color: {TEXT_FAINT};
    font-size: 11px;
}}
QLabel#metricValue {{
    color: {TEXT};
    font-family: {MONO_FONT_STACK};
    font-size: 22px;
    font-weight: 500;
}}
QLabel#metricValue[state="ok"] {{ color: {OK}; }}
QLabel#metricValue[state="warn"] {{ color: {WARN}; }}
QLabel#metricValue[state="bad"] {{ color: {BAD}; }}
QLabel#metricValue[state="idle"] {{ color: {IDLE}; }}

/* ---------- 判定横幅（左 3px 语义条） ---------- */
QLabel#verdict {{
    color: {TEXT_FAINT};
    background: {PANEL};
    border: 1px solid {LINE};
    border-left: 3px solid {LINE};
    padding: 12px 16px;
    font-size: 20px;
    font-weight: 600;
}}
QLabel#verdict[verdictValue="PASS"] {{
    color: {OK};
    border-left-color: {OK};
}}
QLabel#verdict[verdictValue="CONDITIONAL PASS"] {{
    color: {WARN};
    border-left-color: {WARN};
}}
QLabel#verdict[verdictValue="REPEAT"],
QLabel#verdict[verdictValue="FAIL"] {{
    color: {BAD};
    border-left-color: {BAD};
}}
QLabel#verdict[verdictValue="NOT TESTED"] {{
    color: {IDLE};
    border-left-color: {IDLE};
}}

/* ---------- 按钮 ---------- */
QPushButton {{
    background: transparent;
    color: {TEXT};
    border: 1px solid {LINE};
    border-radius: 3px;
    min-height: 32px;
    padding: 6px 18px;
    font-size: 13px;
}}
QPushButton:hover {{
    background: {RAISED};
    border-color: #49505A;
}}
QPushButton:pressed {{
    background: {OVERLAY};
}}
QPushButton:disabled {{
    color: {TEXT_FAINT};
    background: #3A3F47;
    border-color: {LINE};
}}
QPushButton[role="primary"] {{
    background: {ACCENT};
    color: {ACCENT_TEXT};
    border: none;
    font-weight: 600;
}}
QPushButton[role="primary"]:hover {{
    background: {ACCENT_HOVER};
}}
QPushButton[role="primary"]:pressed {{
    background: {ACCENT_PRESS};
}}
QPushButton[role="primary"]:disabled {{
    background: #3A3F47;
    color: {TEXT_FAINT};
}}
QPushButton[role="quiet"] {{
    background: transparent;
    border: none;
    color: {TEXT_DIM};
}}
QPushButton[role="quiet"]:hover {{
    color: {TEXT};
    background: {RAISED};
}}
QPushButton[role="quiet"]:pressed {{
    background: {OVERLAY};
}}

/* ---------- 输入控件 ---------- */
QLineEdit, QComboBox, QPlainTextEdit {{
    background: {PANEL};
    color: {TEXT};
    border: 1px solid {LINE};
    border-radius: 3px;
    padding: 6px 10px;
    selection-background-color: {ACCENT_SOFT};
    selection-color: {TEXT};
}}
QLineEdit, QComboBox {{
    min-height: 32px;
}}
QLineEdit:focus, QComboBox:focus, QPlainTextEdit:focus {{
    border-color: {ACCENT};
}}
QLineEdit::placeholder, QPlainTextEdit::placeholder {{
    color: {TEXT_FAINT};
}}
QLineEdit:disabled, QComboBox:disabled, QPlainTextEdit:disabled {{
    color: {TEXT_FAINT};
}}
QComboBox::drop-down {{
    border: none;
    width: 28px;
}}
QComboBox::drop-down:hover {{
    background: {RAISED};
}}
QComboBox QAbstractItemView {{
    background: {PANEL};
    color: {TEXT};
    border: 1px solid {LINE};
    selection-background-color: {ACCENT_SOFT};
    selection-color: {TEXT};
    outline: none;
}}
QPlainTextEdit {{
    font-family: {MONO_FONT_STACK};
    font-size: 12px;
}}

/* ---------- 列表 ---------- */
QListWidget {{
    background: {PANEL};
    color: {TEXT};
    border: 1px solid {LINE};
    border-radius: 3px;
    padding: 4px;
    outline: none;
}}
QListWidget::item {{
    padding: 8px;
    border-radius: 3px;
}}
QListWidget::item:hover {{
    background: {RAISED};
}}
QListWidget::item:selected {{
    background: {ACCENT_SOFT};
    color: {TEXT};
}}

/* ---------- 表格 ---------- */
QTableWidget {{
    background: {CANVAS};
    color: {TEXT};
    border: 1px solid {LINE};
    gridline-color: {LINE_SOFT};
    alternate-background-color: #1C2026;
    selection-background-color: {ACCENT_SOFT};
    selection-color: {TEXT};
    outline: none;
}}
QTableWidget::item {{
    padding: 6px 10px;
    border: none;
}}
QTableWidget::item:hover {{
    background: {RAISED};
}}
QTableWidget::item:selected {{
    background: {ACCENT_SOFT};
    color: {TEXT};
}}
QHeaderView::section {{
    background: {CANVAS};
    color: {TEXT_FAINT};
    border: none;
    border-bottom: 1px solid {LINE};
    padding: 8px 10px;
    font-size: 11px;
    font-weight: 400;
    text-align: left;
}}

/* ---------- 复选指示器（表格导入列 / 对比多选列表） ---------- */
/* 深色主题下原生未勾选框几乎不可见：显式给出描边方框， */
/* 勾选态为 ACCENT 底 + 深色勾号（图像在 application_stylesheet 注入）。 */
QTableWidget::indicator, QListWidget::indicator {{
    width: 14px;
    height: 14px;
    border: 1px solid #49505A;
    border-radius: 2px;
    background: {PANEL};
}}
QTableWidget::indicator:hover, QListWidget::indicator:hover {{
    border-color: {TEXT_DIM};
}}
QTableWidget::indicator:checked, QListWidget::indicator:checked {{
    background: {ACCENT};
    border-color: {ACCENT};
}}
QTableWidget::indicator:disabled, QListWidget::indicator:disabled {{
    border-color: {LINE};
    background: {CANVAS};
}}

/* ---------- 标签页（下划线式） ---------- */
QTabWidget#resultTabs::pane {{
    border: none;
    border-top: 1px solid {LINE};
    background: {CANVAS};
}}
QTabWidget#resultTabs QTabBar::tab {{
    background: transparent;
    color: {TEXT_DIM};
    border: none;
    border-bottom: 2px solid transparent;
    padding: 10px 14px;
    font-size: 13px;
}}
QTabWidget#resultTabs QTabBar::tab:hover {{
    color: {TEXT};
}}
QTabWidget#resultTabs QTabBar::tab:selected {{
    color: {TEXT};
    border-bottom: 2px solid {ACCENT};
}}
QTabBar::scroller {{
    width: 24px;
}}

/* ---------- 进度条（高 4px，无文字） ---------- */
QProgressBar {{
    min-height: 4px;
    max-height: 4px;
    background: {LINE_SOFT};
    border: none;
    text-align: center;
    color: transparent;
}}
QProgressBar::chunk {{
    background: {ACCENT};
}}

/* ---------- 滚动条 ---------- */
QScrollBar:vertical {{
    background: transparent;
    width: 10px;
    margin: 0;
}}
QScrollBar::handle:vertical {{
    background: #3A414A;
    min-height: 28px;
    border-radius: 5px;
}}
QScrollBar::handle:vertical:hover {{
    background: #4A525E;
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0;
}}
QScrollBar:horizontal {{
    background: transparent;
    height: 10px;
    margin: 0;
}}
QScrollBar::handle:horizontal {{
    background: #3A414A;
    min-width: 28px;
    border-radius: 5px;
}}
QScrollBar::handle:horizontal:hover {{
    background: #4A525E;
}}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{
    width: 0;
}}
QScrollBar::add-page, QScrollBar::sub-page {{
    background: transparent;
}}

QMessageBox {{
    background: {PANEL};
}}
QFileDialog {{
    background: {CANVAS};
}}
"""


def _ui_asset_dir():
    """运行时自建资产目录：QStandardPaths.CacheLocation/ui_assets。

    纯运行时代码生成，不入 git；PyInstaller 打包后同样生成与加载，
    无需额外 datas。
    """
    import tempfile
    from pathlib import Path

    from PySide6 import QtCore

    base = QtCore.QStandardPaths.writableLocation(
        QtCore.QStandardPaths.StandardLocation.CacheLocation
    )
    root = Path(base) if base else Path(tempfile.gettempdir()) / "statusmonitor-cache"
    directory = root / "ui_assets"
    directory.mkdir(parents=True, exist_ok=True)
    return directory


def _combo_arrow_path():
    """运行时用 QPainter 自绘下拉小三角 PNG（自建资产，无版权问题）。"""
    from PySide6 import QtCore, QtGui

    path = _ui_asset_dir() / "combo_down_arrow.png"
    if not path.exists():
        image = QtGui.QImage(10, 6, QtGui.QImage.Format.Format_ARGB32)
        image.fill(QtCore.Qt.GlobalColor.transparent)
        painter = QtGui.QPainter(image)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.setBrush(QtGui.QColor(TEXT_DIM))
        painter.drawPolygon(
            QtGui.QPolygon(
                [QtCore.QPoint(0, 0), QtCore.QPoint(9, 0), QtCore.QPoint(4, 5)]
            )
        )
        painter.end()
        image.save(str(path))
    return path


def _indicator_check_path():
    """运行时用 QPainter 自绘复选框勾号 PNG（深色勾，铺在 ACCENT 底上）。"""
    from PySide6 import QtCore, QtGui

    path = _ui_asset_dir() / "indicator_check.png"
    if not path.exists():
        image = QtGui.QImage(10, 9, QtGui.QImage.Format.Format_ARGB32)
        image.fill(QtCore.Qt.GlobalColor.transparent)
        painter = QtGui.QPainter(image)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        pen = QtGui.QPen(QtGui.QColor(ACCENT_TEXT))
        pen.setWidth(2)
        pen.setCapStyle(QtCore.Qt.PenCapStyle.RoundCap)
        pen.setJoinStyle(QtCore.Qt.PenJoinStyle.RoundJoin)
        painter.setPen(pen)
        painter.drawPolyline(
            QtGui.QPolygon(
                [QtCore.QPoint(1, 5), QtCore.QPoint(4, 8), QtCore.QPoint(9, 1)]
            )
        )
        painter.end()
        image.save(str(path))
    return path


def application_stylesheet() -> str:
    """组装完整 QSS：基础样式 + 运行时生成的图像资产引用。

    QSS url 路径必须使用正斜杠，因此使用 Path.as_posix()。
    需要在 QApplication 实例存在后、setStyleSheet 之前调用。
    """
    arrow = _combo_arrow_path()
    check = _indicator_check_path()
    return INSTRUMENT_STYLE + (
        "\nQComboBox::down-arrow {\n"
        f'    image: url("{arrow.as_posix()}");\n'
        "    width: 10px;\n"
        "    height: 6px;\n"
        "}\n"
        "\nQTableWidget::indicator:checked, QListWidget::indicator:checked {\n"
        f'    image: url("{check.as_posix()}");\n'
        "}\n"
    )
