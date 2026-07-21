#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd -- "$(dirname -- "$0")" && pwd)"
VENV="${STATUSMONITOR_VENV:-$ROOT/.venv-macos}"
VENV_PYTHON="$VENV/bin/python"

pause_on_error() {
    local code=$?
    trap - ERR
    set +e
    echo
    echo "macOS 应用构建失败（错误码 $code）。"
    echo "请确认已安装 Python 3.11 或更高版本，并检查上方错误信息。"
    if [[ -t 0 ]]; then
        read -r -p "按 Return 键关闭此窗口。" _
    fi
    exit "$code"
}
trap pause_on_error ERR

find_python() {
    if [[ -n "${STATUSMONITOR_PYTHON:-}" ]]; then
        printf '%s\n' "$STATUSMONITOR_PYTHON"
        return 0
    fi

    local candidate
    for candidate in python3.13 python3.12 python3.11 python3; do
        if command -v "$candidate" >/dev/null 2>&1 \
            && "$candidate" -c 'import sys; raise SystemExit(sys.version_info < (3, 11))'; then
            command -v "$candidate"
            return 0
        fi
    done
    return 1
}

if [[ ! -x "$VENV_PYTHON" ]]; then
    BASE_PYTHON="$(find_python)" || {
        echo "未找到 Python 3.11 或更高版本。请先从 python.org 或 Homebrew 安装。"
        false
    }
    echo "正在创建 macOS 专用构建环境……"
    "$BASE_PYTHON" -m venv "$VENV"
fi

cd "$ROOT"
echo "正在安装应用打包依赖……"
"$VENV_PYTHON" -m pip install --upgrade pip
"$VENV_PYTHON" -m pip install -e "$ROOT[build]"

echo "正在生成当前 Mac 架构适用的 VEXFlightStatusMonitor.app……"
"$VENV_PYTHON" -m PyInstaller \
    --noconfirm \
    --clean \
    --name VEXFlightStatusMonitor \
    --windowed \
    --osx-bundle-identifier org.vex74000m.flightstatusmonitor \
    --distpath "$ROOT/dist" \
    --workpath "$ROOT/build" \
    --specpath "$ROOT" \
    --collect-all pyqtgraph \
    --hidden-import pyarrow \
    --hidden-import polars \
    "$ROOT/packaging_entry.py"

APP="$ROOT/dist/VEXFlightStatusMonitor.app"
echo
echo "构建完成：$APP"
open "$APP"
