"""Swiss-grid visual system for the status-monitor desktop application."""

VEX_RED = "#e30613"
INK = "#101114"
MUTED = "#666a70"
CANVAS = "#f7f6f2"
PAPER = "#fffefa"
RULE = "#d9d7d0"
SUCCESS = "#16803a"


SWISS_GRID_STYLE = f"""
* {{
    font-family: "Noto Sans SC", "Microsoft YaHei UI", "Segoe UI", sans-serif;
    font-size: 13px;
    color: {INK};
}}
QMainWindow, QWidget#appShell, QWidget#contentArea, QWidget#page {{
    background: {CANVAS};
}}
QToolTip {{
    background: {INK};
    color: white;
    border: none;
    padding: 6px 8px;
}}
QWidget#sidebar {{
    background: {PAPER};
    border-right: 1px solid {RULE};
}}
QLabel#brand {{
    color: {VEX_RED};
    font-size: 34px;
    font-weight: 900;
    letter-spacing: -1px;
}}
QLabel#brandCaption {{
    color: {MUTED};
    font-size: 11px;
    font-weight: 600;
}}
QLabel#sidebarStatus {{
    color: {MUTED};
    font-size: 12px;
    padding: 14px 18px;
    border-top: 1px solid {RULE};
}}
QListWidget#navigation {{
    background: transparent;
    border: none;
    outline: none;
    padding: 0;
}}
QListWidget#navigation::item {{
    background: transparent;
    color: #2a2c30;
    padding: 0 18px;
    border-left: 3px solid transparent;
}}
QListWidget#navigation::item:hover {{
    background: #f4f2ed;
}}
QListWidget#navigation::item:selected {{
    background: {PAPER};
    color: {VEX_RED};
    border-left: 3px solid {VEX_RED};
    font-weight: 700;
}}
QLabel#appKicker {{
    color: {VEX_RED};
    font-size: 11px;
    font-weight: 800;
    letter-spacing: 1px;
}}
QLabel#appTitle {{
    color: {INK};
    font-size: 34px;
    font-weight: 800;
    letter-spacing: -1px;
}}
QLabel#identity {{
    color: {INK};
    font-size: 17px;
    font-weight: 650;
}}
QLabel#readOnlyChip {{
    color: {SUCCESS};
    font-size: 12px;
    font-weight: 700;
    padding: 5px 9px;
    border: 1px solid #b9d8c2;
    background: #f2faf4;
}}
QFrame#headerRule, QFrame#sectionRule {{
    color: {RULE};
    background: {RULE};
    border: none;
    max-height: 1px;
}}
QLabel#pageTitle {{
    color: {INK};
    font-size: 26px;
    font-weight: 750;
    padding: 0;
}}
QLabel#pageDescription {{
    color: {MUTED};
    font-size: 12px;
    padding: 0 0 6px 0;
}}
QLabel#sectionTitle {{
    color: {INK};
    font-size: 14px;
    font-weight: 700;
    padding: 7px 0 3px 0;
}}
QLabel#statusCard {{
    background: {PAPER};
    border: none;
    border-top: 1px solid {RULE};
    border-bottom: 1px solid {RULE};
    padding: 15px 16px;
    min-height: 50px;
}}
QLabel#metricLabel {{
    color: {MUTED};
    font-size: 11px;
}}
QWidget#metricPanel {{
    background: {PAPER};
    border: none;
    border-right: 1px solid {RULE};
    border-bottom: 1px solid {RULE};
}}
QLabel#metricValue {{
    color: {INK};
    font-size: 22px;
    font-weight: 650;
}}
QLabel#verdict {{
    color: {INK};
    background: {PAPER};
    border: none;
    border-left: 4px solid {RULE};
    padding: 13px 16px;
    font-size: 23px;
    font-weight: 750;
}}
QLabel#verdict[verdictValue="PASS"] {{
    color: {SUCCESS};
    border-left-color: {SUCCESS};
}}
QLabel#verdict[verdictValue="CONDITIONAL PASS"] {{
    color: #946200;
    border-left-color: #d59500;
}}
QLabel#verdict[verdictValue="REPEAT"],
QLabel#verdict[verdictValue="FAIL"] {{
    color: #b4232c;
    border-left-color: {VEX_RED};
}}
QPushButton {{
    background: {PAPER};
    color: {INK};
    border: 1px solid {INK};
    border-radius: 0;
    min-height: 34px;
    padding: 6px 15px;
    font-weight: 650;
}}
QPushButton:hover {{
    background: #f0eee8;
}}
QPushButton:pressed {{
    background: #e7e4dc;
}}
QPushButton:disabled {{
    color: #a4a5a7;
    border-color: #c8c7c2;
    background: #f2f1ed;
}}
QPushButton[role="primary"] {{
    color: white;
    background: {VEX_RED};
    border-color: {VEX_RED};
}}
QPushButton[role="primary"]:hover {{
    background: #c9000e;
    border-color: #c9000e;
}}
QLineEdit, QComboBox, QPlainTextEdit, QListWidget, QTableWidget {{
    background: {PAPER};
    color: {INK};
    border: 1px solid #c9c7c0;
    border-radius: 0;
    padding: 5px 7px;
    selection-background-color: {VEX_RED};
    selection-color: white;
}}
QLineEdit:focus, QComboBox:focus, QPlainTextEdit:focus,
QListWidget:focus, QTableWidget:focus {{
    border-color: {INK};
}}
QLineEdit, QComboBox {{
    min-height: 30px;
}}
QComboBox::drop-down {{
    border: none;
    width: 24px;
}}
QComboBox QAbstractItemView {{
    background: {PAPER};
    border: 1px solid {INK};
    selection-background-color: {INK};
    selection-color: white;
}}
QPlainTextEdit {{
    font-family: "Cascadia Mono", "Consolas", monospace;
    font-size: 12px;
}}
QTableWidget {{
    gridline-color: #e2e0da;
    alternate-background-color: #faf9f5;
    padding: 0;
}}
QTableWidget::item {{
    padding: 6px;
    border: none;
}}
QTableWidget::item:selected {{
    background: #fce8e9;
    color: {INK};
}}
QHeaderView::section {{
    background: {PAPER};
    color: #4c4f54;
    border: none;
    border-bottom: 1px solid {INK};
    padding: 8px 7px;
    font-size: 11px;
    font-weight: 700;
}}
QListWidget::item {{
    padding: 8px;
}}
QProgressBar {{
    min-height: 8px;
    max-height: 8px;
    background: #e4e2dc;
    border: none;
    text-align: center;
    color: transparent;
}}
QProgressBar::chunk {{
    background: {VEX_RED};
}}
QStatusBar {{
    background: {PAPER};
    color: {MUTED};
    border-top: 1px solid {RULE};
}}
QStatusBar::item {{
    border: none;
}}
QScrollBar:vertical {{
    background: transparent;
    width: 10px;
    margin: 0;
}}
QScrollBar::handle:vertical {{
    background: #c9c7c0;
    min-height: 28px;
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0;
}}
QMessageBox {{
    background: {PAPER};
}}
"""
