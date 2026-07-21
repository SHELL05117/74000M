from __future__ import annotations

import pytest


def test_gui_constructs_guided_tf_workflow(isolated_home, monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    pytest.importorskip("PySide6")
    pytest.importorskip("pyqtgraph")
    from PySide6 import QtWidgets

    from statusmonitor.app import MainWindow

    _application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = MainWindow()
    assert window.tabs.count() == 5
    assert window.windowTitle() == "VEX V5 飞行记录与诊断系统"
    assert window.tabs.currentWidget() is window.home_page

    # 首页三张纵向操作卡：标题通过卡片属性与子控件双重稳定途径断言。
    assert len(window.home_page.action_buttons) == 3
    titles = []
    for card in window.home_page.action_buttons:
        title_label = card.findChild(QtWidgets.QLabel, "actionCardTitle")
        assert title_label is not None
        assert title_label.text() == card.title_text
        titles.append(card.title_text)
    assert titles == [
        "新建 TF 记录",
        "继续会话",
        "历史记录",
    ]

    window.home_page.action_buttons[0].click()
    assert window.tabs.currentWidget() is window.wizard_page
    assert window.wizard_page.steps.count() == 3
    assert window.wizard_page.steps.currentIndex() == 0
    assert window.wizard_page.next_button.text() == "下一步"
    assert [label.text() for label in window.wizard_page.step_labels] == [
        "会话信息",
        "实验条件",
        "选择 TF 记录",
    ]
    window.wizard_page._set_step(1)
    assert window.wizard_page.next_button.text() == "下一步"
    window.wizard_page._set_step(2)
    assert window.wizard_page.next_button.text() == "导入、分析并查看结果"

    window.show_picker("continue")
    assert window.tabs.currentWidget() is window.picker_page
    assert window.picker_page.title.text() == "继续会话"
    assert window.picker_page.open_button.text() == "继续导入 TF 记录"

    window.show_picker("history")
    assert window.tabs.currentWidget() is window.picker_page
    assert window.picker_page.title.text() == "历史记录"
    assert window.picker_page.open_button.text() == "查看图表与信息"

    window.show_loading("正在测试")
    assert window.tabs.currentWidget() is window.loading_page
    assert window.loading_page.timer.isActive()
    window.show_home()
    assert not window.loading_page.timer.isActive()
    assert window.results_page.views.count() == 6
    window.close()


def test_gui_wizard_lists_scanned_tf_recording(isolated_home, monkeypatch, tmp_path):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    pytest.importorskip("PySide6")
    pytest.importorskip("pyqtgraph")
    from PySide6 import QtCore, QtWidgets

    from statusmonitor.app import MainWindow
    from statusmonitor.storage.archive import scan_recordings
    from v5l_builder import synthetic_frames, write_v5l

    source = write_v5l(tmp_path / "DATA.V5L", synthetic_frames(40))
    candidates = scan_recordings(tmp_path)
    assert [candidate.path for candidate in candidates] == [source]

    _application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = MainWindow()
    window.start_new()
    window.wizard_page._set_step(2)
    window.wizard_page.show_candidates(candidates)
    assert window.wizard_page.table.rowCount() == 1
    assert window.wizard_page.table.columnCount() == 7
    assert (
        window.wizard_page.table.item(0, 0).checkState()
        == QtCore.Qt.CheckState.Checked
    )
    assert window.wizard_page.table.item(0, 2).text() == "通过 · PASS"
    assert window.wizard_page.next_button.text() == "导入、分析并查看结果"
    window.close()
