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
    echo "VEX 飞行记录与诊断系统启动失败（错误码 $code）。"
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
    echo "正在创建 macOS 专用运行环境……"
    "$BASE_PYTHON" -m venv "$VENV"
fi

if ! "$VENV_PYTHON" -c \
    'import statusmonitor, PySide6, pyqtgraph, pyarrow, polars' >/dev/null 2>&1; then
    echo "首次启动：正在安装应用与分析依赖，请保持网络连接……"
    "$VENV_PYTHON" -m pip install --upgrade pip
    "$VENV_PYTHON" -m pip install -e "$ROOT"
fi

cd "$ROOT"
"$VENV_PYTHON" -m statusmonitor gui
