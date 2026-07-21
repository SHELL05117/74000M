param()

$ErrorActionPreference = "Stop"
$root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$failures = [System.Collections.Generic.List[string]]::new()

function Search-Lines {
    param([string]$Pattern, [string[]]$Paths)
    $arguments = @("--no-heading", "-n", $Pattern) + $Paths
    $lines = & rg @arguments 2>$null
    if ($LASTEXITCODE -gt 1) {
        throw "rg failed while checking: $Pattern"
    }
    return @($lines)
}

Push-Location $root
try {
    $motorWriters = Search-Lines "motor_move_voltage|move_voltage" @("src")
    foreach ($line in $motorWriters) {
        if ($line -notmatch '^src[\\/]platform[\\/]pros_adapters\.cpp:') {
            $failures.Add("motor writer outside PROS adapter: $line")
        }
    }
    if ($motorWriters.Count -eq 0) {
        $failures.Add("no platform motor writer found")
    }

    $businessWrites = Search-Lines "\.writeVoltage\s*\(" @("include/robot", "src")
    foreach ($line in $businessWrites) {
        if ($line -notmatch 'output_service\.hpp:' -and
            $line -notmatch 'pros_adapters\.cpp:') {
            $failures.Add("DriveIO write bypasses OutputService: $line")
        }
    }

    $prosLeaks = Search-Lines '#include\s*[<"](pros/|api\.h|main\.h)' @("include/robot", "src/robot.cpp")
    foreach ($line in $prosLeaks) {
        $failures.Add("platform header leaked into robot core: $line")
    }

    $batteryAmplification = Search-Lines '12(\.0+)?\s*/\s*[^;\r\n]*(battery|battery_V)|12(\.0+)?\s*\*\s*[^;\r\n]*/\s*(battery|battery_V)' @("include/robot", "src")
    foreach ($line in $batteryAmplification) {
        $failures.Add("forbidden physical-voltage amplification: $line")
    }

    if (Test-Path "include/robot/autonomy") {
        $blocking = Search-Lines 'while\s*\(|for\s*\(\s*;\s*;' @("include/robot/autonomy")
        foreach ($line in $blocking) {
            $failures.Add("blocking autonomous loop: $line")
        }
    }

    foreach ($profilePath in @("config/robots/492X.yaml",
                               "config/robots/492Z.yaml")) {
        $profile = Get-Content -Raw -Encoding utf8 $profilePath
        foreach ($capability in @("hardware_output", "driver_control", "pose_good",
                                  "autonomous_chassis_velocity", "autonomous_motion",
                                  "competition_routes")) {
            if ($profile -notmatch "(?m)^\s*$capability\s*:\s*false\s*$") {
                $failures.Add("offline capability is not locked in ${profilePath}: $capability")
            }
        }
        if ($profile -notmatch '(?m)^\s*selected_route\s*:\s*"DoNothing"\s*$') {
            $failures.Add("offline selected route is not DoNothing in $profilePath")
        }
        if ($profile -notmatch '(?m)^\s*hardware\s*:\s*"Unverified"\s*$') {
            $failures.Add("hardware verification changed without HIL evidence in $profilePath")
        }
    }
    if ((Get-Content -Raw "CMakeLists.txt") -notmatch 'CMAKE_CXX_STANDARD 17' -or
        (Get-Content -Raw "Makefile") -notmatch 'CXX_STANDARD:=gnu\+\+17') {
        $failures.Add("PC and PROS builds are not both locked to C++17")
    }
}
finally {
    Pop-Location
}

if ($failures.Count -gt 0) {
    Write-Output "OFFLINE FREEZE AUDIT: FAIL"
    $failures | ForEach-Object { Write-Output "- $_" }
    exit 1
}

Write-Output "OFFLINE FREEZE AUDIT: PASS"
Write-Output "motor writer: src/platform/pros_adapters.cpp only"
Write-Output "business output path: OutputService only"
Write-Output "core platform isolation: PASS"
Write-Output "battery amplification guard: PASS"
Write-Output "492X/492Z capabilities and DoNothing locks: PASS"
Write-Output "language standard: PC/PROS C++17"
exit 0
