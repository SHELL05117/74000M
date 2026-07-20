from __future__ import annotations

import pytest


def test_gui_constructs_eight_pages(isolated_home, monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    pytest.importorskip("PySide6")
    pytest.importorskip("pyqtgraph")
    from PySide6 import QtWidgets

    from statusmonitor.app import MainWindow

    _application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = MainWindow()
    assert window.tabs.count() == 8
    assert window.navigation.count() == 8
    assert window.windowTitle() == "VEX V5 飞行记录与诊断系统"
    assert [window.navigation.item(index).text() for index in range(8)] == [
        "01    首页与会话",
        "02    录制窗口",
        "03    导入记录",
        "04    完整性校验",
        "05    运行总览",
        "06    图表分析",
        "07    运行对比",
        "08    LLM 报告",
    ]
    window.navigation.setCurrentRow(5)
    assert window.tabs.currentIndex() == 5
    window.close()
