[CmdletBinding()]
param(
    [Parameter(Mandatory = $true, Position = 0)]
    [ValidateSet("492X", "492Z")]
    [string]$Robot,

    [switch]$SelectOnly
)

$ErrorActionPreference = "Stop"
$root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$selectionPath = Join-Path $root ".robot-profile.local.mk"

function Find-ProsMake {
    foreach ($editor in @("Windsurf", "Devin", "Code")) {
        $toolchain = Join-Path $env:APPDATA `
            "$editor\User\globalStorage\sigbots.pros\install\pros-toolchain-windows\usr\bin"
        $candidate = Join-Path $toolchain "make.exe"
        if (Test-Path -LiteralPath $candidate) {
            $env:Path = "$toolchain;$env:Path"
            return $candidate
        }
    }

    $command = Get-Command make -ErrorAction SilentlyContinue
    if ($null -ne $command) {
        return $command.Source
    }

    throw "PROS make toolchain was not found. Install or enable the PROS extension."
}

[System.IO.File]::WriteAllText(
    $selectionPath,
    "ROBOT_PROFILE:=$Robot`n",
    [System.Text.UTF8Encoding]::new($false))

$make = Find-ProsMake
Push-Location $root
try {
    Write-Output "Selected robot: $Robot"
    Write-Output "Cleaning objects from the previous robot profile..."
    & $make clean
    if ($LASTEXITCODE -ne 0) {
        throw "PROS clean failed with exit code $LASTEXITCODE"
    }

    if ($SelectOnly) {
        Write-Output "Selection saved. The next normal PROS build/download will use $Robot."
        return
    }

    Write-Output "Building firmware for $Robot..."
    & $make quick
    if ($LASTEXITCODE -ne 0) {
        throw "PROS build failed with exit code $LASTEXITCODE"
    }

    Write-Output "Firmware ready: bin/hot.package.bin"
    Write-Output "Before driving, confirm the Brain displays $Robot."
}
finally {
    Pop-Location
}
