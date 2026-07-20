$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$Python = Join-Path $Root ".venv\Scripts\python.exe"

if (-not (Test-Path -LiteralPath $Python)) {
    throw "Virtual environment not found. Run .\setup.ps1 first."
}

& $Python -m pip install -e "$Root[build]"
& $Python -m PyInstaller `
    --noconfirm `
    --clean `
    --name VEXFlightStatusMonitor `
    --windowed `
    --distpath (Join-Path $Root "dist") `
    --workpath (Join-Path $Root "build") `
    --specpath $Root `
    --collect-all pyqtgraph `
    --hidden-import pyarrow `
    --hidden-import polars `
    (Join-Path $Root "packaging_entry.py")

Write-Host "Built application under $Root\dist\VEXFlightStatusMonitor"
