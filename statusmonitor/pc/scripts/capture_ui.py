"""GUI 离屏截图工具（QA 专用）。

用项目 venv 的 PySide6 以 ``QT_QPA_PLATFORM=offscreen`` 运行，
通过 ``QWidget.grab()`` 输出 PNG。

两种模式加一个变体开关：

- ``empty``（模式 A，空库）：临时目录作为独立 STATUSMONITOR_HOME，
  直接实例化 MainWindow，截取首页、向导第 1/2/3 步、加载页、picker 两种模式。
- ``synthetic``（模式 B，默认，截图更真实）：在临时 home 中用
  ``tests/v5l_builder.py`` 生成 1 个 V5L，直接调用后端 API
  （Repository、ImportService、AnalysisPipeline）建会话 → 导入 → 分析，
  然后实例化 MainWindow 截取首页、picker 历史模式、结果中心 6 个 tab、
  向导第 3 步（show_candidates 填充真实扫描结果）。
- ``--verdicts``（判定色变体）：同模式 B 合成数据后，把该 run 的
  ``integrity/integrity_report.json`` 复制改写为 CONDITIONAL PASS / FAIL 两份，
  依次让 IntegrityPage 加载并截取判定横幅，同时断言横幅渲染色
  （CONDITIONAL PASS=WARN 琥珀，FAIL=BAD 红）。

合成数据只存在于 ``tempfile.TemporaryDirectory`` 中，绝不写入真实用户库。

用法（在项目根目录）::

    statusmonitor\\pc\\.venv\\Scripts\\python.exe statusmonitor\\pc\\scripts\\capture_ui.py \
        --size 1480x920 --out statusmonitor\\pc\\.qa_shots\\1480
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile
import time
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MPLBACKEND", "Agg")

PC_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PC_ROOT / "src"
TESTS_DIR = PC_ROOT / "tests"
for path in (str(SRC_DIR), str(TESTS_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)


def parse_size(text: str) -> tuple[int, int]:
    width_text, _, height_text = text.lower().partition("x")
    width, height = int(width_text), int(height_text)
    if width < 1120 or height < 720:
        raise ValueError(f"窗口尺寸不能小于最小可用窗口 1120x720：{text}")
    return width, height


def pump(application, milliseconds: int = 200) -> None:
    """推进事件循环，确保布局与绘制完成。"""
    deadline = time.monotonic() + milliseconds / 1000.0
    while time.monotonic() < deadline:
        application.processEvents()
        time.sleep(0.01)


def use_temp_home(home: Path) -> None:
    os.environ["STATUSMONITOR_HOME"] = str(home)
    os.environ["STATUSMONITOR_ARTIFACTS"] = str(home / "artifacts")
    os.environ["STATUSMONITOR_DB"] = str(home / "artifacts" / "index.sqlite3")


def make_window(application, width: int, height: int):
    from statusmonitor.app import MainWindow
    from statusmonitor.ui_style import application_stylesheet

    application.setApplicationName("VEX V5 飞行记录与诊断系统")
    application.setOrganizationName("74000M")
    application.setStyle("Fusion")
    application.setStyleSheet(application_stylesheet())
    window = MainWindow()
    window.resize(width, height)
    window.show()
    pump(application, 300)
    return window


def capture(window, page, path: Path, application, settle_ms: int = 250) -> Path:
    window.tabs.setCurrentWidget(page)
    pump(application, settle_ms)
    image = window.grab()
    if not image.save(str(path)):
        raise RuntimeError(f"截图保存失败：{path}")
    print(f"written: {path}")
    return path


def shot_current(window, path: Path, application, settle_ms: int = 250) -> Path:
    pump(application, settle_ms)
    image = window.grab()
    if not image.save(str(path)):
        raise RuntimeError(f"截图保存失败：{path}")
    print(f"written: {path}")
    return path


def capture_empty(out_dir: Path, size: tuple[int, int]) -> None:
    """模式 A：空库截图。"""
    from PySide6 import QtWidgets

    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    with tempfile.TemporaryDirectory(prefix="statusmonitor_qa_empty_") as temp:
        use_temp_home(Path(temp))
        window = make_window(application, *size)

        capture(window, window.home_page, out_dir / "home.png", application)

        window.start_new()
        capture(window, window.wizard_page, out_dir / "wizard_step1.png", application)
        window.wizard_page._set_step(1)
        capture(window, window.wizard_page, out_dir / "wizard_step2.png", application)
        window.wizard_page._set_step(2)
        capture(window, window.wizard_page, out_dir / "wizard_step3.png", application)

        window.show_loading(
            "正在生成本次记录", "共 1 段文件：校验 → 归档 → 分析 → 生成 LLM 报告"
        )
        pump(application, 500)  # 让打点动画至少推进一拍
        shot_current(window, out_dir / "loading.png", application)
        window.loading_page.stop()

        window.show_picker("continue")
        capture(window, window.picker_page, out_dir / "picker_continue.png", application)
        window.show_picker("history")
        capture(window, window.picker_page, out_dir / "picker_history.png", application)

        window.close()
        pump(application, 100)


def build_synthetic_session(home: Path):
    """在临时 home 中用后端 API 建会话 → 导入 → 分析（不走 GUI Worker）。"""
    from statusmonitor.analysis.pipeline import AnalysisPipeline
    from statusmonitor.models import SessionMetadata
    from statusmonitor.repository import Repository
    from statusmonitor.storage.archive import ImportService
    from v5l_builder import synthetic_frames, write_v5l

    source_dir = home / "media" / "FLIGHT" / "1690X" / "R000017_T0000009000"
    source_dir.mkdir(parents=True, exist_ok=True)
    source = write_v5l(source_dir / "DATA.V5L", synthetic_frames(600, pose=True))

    repo = Repository()
    session = repo.create_session(
        SessionMetadata(
            team_number="74000M",
            operator="QA Screenshot",
            observer="QA",
            test_type="manual",
            test_case_id="DRIVE-MANUAL-QA",
            surface="field tiles",
            battery_id="BAT-QA-01",
        )
    )
    run = ImportService(repo).import_recording(session.session_id, source)
    AnalysisPipeline(repo).analyze(run.run_id)
    return session.session_id, run.run_id, home / "media"


def capture_synthetic(out_dir: Path, size: tuple[int, int]) -> None:
    """模式 B：合成数据截图（默认）。"""
    from PySide6 import QtWidgets

    from statusmonitor.storage.archive import scan_recordings

    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    with tempfile.TemporaryDirectory(prefix="statusmonitor_qa_synth_") as temp:
        home = Path(temp)
        use_temp_home(home)
        session_id, run_id, media_dir = build_synthetic_session(home)

        window = make_window(application, *size)

        capture(window, window.home_page, out_dir / "home.png", application)

        window.show_picker("history")
        capture(window, window.picker_page, out_dir / "picker_history.png", application)

        # 向导第 3 步：show_candidates 填充真实扫描结果。
        window.continue_session(session_id)
        candidates = scan_recordings(media_dir)
        if candidates:
            window.wizard_page.show_candidates(candidates)
        capture(window, window.wizard_page, out_dir / "wizard_step3.png", application)

        # 结果中心：6 个 tab 每 tab 一张。
        window.results_page.show_session(session_id, run_id)
        window.tabs.setCurrentWidget(window.results_page)
        pump(application, 400)
        tab_names = [
            "results_overview.png",
            "results_plots.png",
            "results_integrity.png",
            "results_report.png",
            "results_compare.png",
            "results_tools.png",
        ]
        views = window.results_page.views
        for index, name in enumerate(tab_names):
            views.setCurrentIndex(index)
            shot_current(window, out_dir / name, application, settle_ms=350)

        window.close()
        pump(application, 100)


def _assert_banner_color(page, verdict: str) -> None:
    """断言判定横幅真实渲染色：CONDITIONAL PASS 必为 WARN 琥珀，FAIL 必为 BAD 红。

    通过抓取横幅 QLabel 的左 3px 语义条像素验证样式表 wiring，
    防止 CONDITIONAL PASS 被渲染成 OK 绿之类的回归。
    """
    from PySide6 import QtGui

    from statusmonitor.ui_style import BAD, WARN

    expected_hex = {"CONDITIONAL PASS": WARN, "FAIL": BAD}[verdict]
    image = page.verdict.grab().toImage()
    rendered = image.pixelColor(1, image.height() // 2)
    expected = QtGui.QColor(expected_hex)
    delta = max(
        abs(rendered.red() - expected.red()),
        abs(rendered.green() - expected.green()),
        abs(rendered.blue() - expected.blue()),
    )
    print(
        f"banner check: verdict={verdict!r} rendered={rendered.name()} "
        f"expected={expected.name()}"
    )
    if delta > 8:
        raise RuntimeError(
            f"判定横幅颜色错误：verdict={verdict!r} 渲染为 {rendered.name()}，"
            f"应为 {expected.name()}"
        )


def capture_verdicts(out_dir: Path, size: tuple[int, int]) -> None:
    """verdicts 模式：同一合成 run，改写判定横幅两种语义色截图。

    照常合成 V5L → 建会话 → 导入 → 分析；把该 run 归档目录下的
    ``integrity/integrity_report.json`` 复制改写两份（CONDITIONAL PASS、FAIL），
    依次让 IntegrityPage 加载并截图。改写只发生在临时 home 内，
    截图后恢复原报告，绝不触碰真实用户库。
    """
    from PySide6 import QtWidgets

    from statusmonitor.repository import Repository

    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    with tempfile.TemporaryDirectory(prefix="statusmonitor_qa_verdicts_") as temp:
        home = Path(temp)
        use_temp_home(home)
        session_id, run_id, _media_dir = build_synthetic_session(home)

        window = make_window(application, *size)
        window.results_page.show_session(session_id, run_id)
        window.tabs.setCurrentWidget(window.results_page)
        window.results_page.views.setCurrentIndex(2)  # “完整性” tab
        pump(application, 400)

        run = Repository().get_run(run_id)
        report_path = Path(run.artifact_dir) / "integrity" / "integrity_report.json"
        original_text = report_path.read_text(encoding="utf-8")
        base_report = json.loads(original_text)
        page = window.results_page.integrity

        variants = {
            "verdict_conditional.png": "CONDITIONAL PASS",
            "verdict_fail.png": "FAIL",
        }
        try:
            for filename, verdict in variants.items():
                variant = dict(base_report)
                variant["verdict"] = verdict
                variant_path = report_path.with_name(
                    f"integrity_report_{verdict.lower().replace(' ', '_')}.json"
                )
                variant_path.write_text(
                    json.dumps(variant, indent=2, ensure_ascii=False),
                    encoding="utf-8",
                )
                # IntegrityPage 固定读 integrity_report.json，临时覆盖为变体内容。
                report_path.write_text(
                    variant_path.read_text(encoding="utf-8"), encoding="utf-8"
                )
                page.load()
                pump(application, 300)
                actual = page.verdict.property("verdictValue")
                if actual != verdict:
                    raise RuntimeError(
                        f"判定横幅 wiring 错误：期望 {verdict!r}，实际 {actual!r}"
                    )
                _assert_banner_color(page, verdict)
                shot_current(window, out_dir / filename, application, settle_ms=300)
        finally:
            report_path.write_text(original_text, encoding="utf-8")

        window.close()
        pump(application, 100)


def main() -> int:
    parser = argparse.ArgumentParser(description="状态监视器 GUI 离屏截图工具")
    parser.add_argument(
        "--size",
        default="1480x920",
        help="窗口尺寸，例如 1480x920 或 1120x720（不得小于 1120x720）",
    )
    parser.add_argument(
        "--out",
        default=None,
        help="输出目录；默认 statusmonitor/pc/.qa_shots/<width>x<height>",
    )
    parser.add_argument(
        "--mode",
        choices=["empty", "synthetic"],
        default="synthetic",
        help="empty=模式 A 空库；synthetic=模式 B 合成数据（默认）",
    )
    parser.add_argument(
        "--verdicts",
        action="store_true",
        help="判定色变体模式：合成数据后改写完整性报告为 CONDITIONAL PASS / FAIL，"
        "截取判定横幅 verdict_conditional.png 与 verdict_fail.png",
    )
    args = parser.parse_args()
    size = parse_size(args.size)
    if args.out:
        out_dir = Path(args.out)
    elif args.verdicts:
        out_dir = PC_ROOT / ".qa_shots" / f"verdicts_{size[0]}x{size[1]}"
    else:
        out_dir = PC_ROOT / ".qa_shots" / f"{size[0]}x{size[1]}"
    out_dir.mkdir(parents=True, exist_ok=True)
    if args.verdicts:
        capture_verdicts(out_dir, size)
        print(f"done: mode=verdicts size={size[0]}x{size[1]} out={out_dir}")
        return 0
    if args.mode == "empty":
        capture_empty(out_dir, size)
    else:
        capture_synthetic(out_dir, size)
    print(f"done: mode={args.mode} size={size[0]}x{size[1]} out={out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
