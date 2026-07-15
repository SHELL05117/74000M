param()

$ErrorActionPreference = "Stop"
$root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$build = Join-Path $root "build"

function Require-Success {
    param([string]$Name, [scriptblock]$Action)
    $output = & $Action
    if ($LASTEXITCODE -ne 0) {
        $output | Write-Output
        throw "$Name failed with exit code $LASTEXITCODE"
    }
    return @($output)
}

Push-Location $root
try {
    $configure = Require-Success "CMake configure" {
        & cmake -S . -B build -DBUILD_TESTING=ON
    }
    $pcBuild = Require-Success "PC build" {
        & cmake --build build --config Debug
    }
    $ctest = Require-Success "CTest" {
        & ctest --test-dir build -C Debug --output-on-failure
    }
    $testOutput = Require-Success "robot_tests" {
        & .\build\Debug\robot_tests.exe
    }
    $testSummary = ($testOutput | Select-String 'tests passed$' |
                    Select-Object -Last 1).Line
    if ([string]::IsNullOrWhiteSpace($testSummary)) {
        throw "robot_tests did not emit a summary"
    }
    $audit = Require-Success "offline freeze audit" {
        & .\tools\audit_offline_freeze.ps1
    }

    $toolchain = Join-Path $env:APPDATA `
        "Devin\User\globalStorage\sigbots.pros\install\pros-toolchain-windows\usr\bin"
    $make = Join-Path $toolchain "make.exe"
    if (-not (Test-Path $make)) {
        $makeCommand = Get-Command make -ErrorAction SilentlyContinue
        if ($null -eq $makeCommand) {
            throw "PROS make toolchain was not found"
        }
        $make = $makeCommand.Source
        $toolchain = Split-Path $make
    }
    $env:Path = "$toolchain;$env:Path"
    $prosBuild = Require-Success "PROS build" {
        & $make quick
    }

    $commit = (git rev-parse HEAD).Trim()
    $configHash = (Get-FileHash -Algorithm SHA256 `
        config\hardware_profile.yaml).Hash.ToLowerInvariant()
    $dirty = @(git status --porcelain | Where-Object {
        $_ -notmatch 'GitHub address\.txt'
    })
    $evidence = [ordered]@{
        schema_version = 1
        generated_utc = [DateTime]::UtcNow.ToString("o")
        source_commit = $commit
        hardware_profile_sha256 = $configHash
        robot_id = "UNVERIFIED_74000M"
        calibration_revision = 0
        pc_tests = $testSummary
        ctest = "PASS"
        static_audit = "PASS"
        pros_cpp17_build = "PASS"
        fake_closed_loop = "PASS"
        scoped_worktree_clean = ($dirty.Count -eq 0)
        old_robot_logs = "NOT TESTED - no historical hardware logs supplied"
        hil = "NOT TESTED - robot unavailable"
        field = "NOT TESTED - robot unavailable"
        competition_approval = "NOT APPROVED"
        rollback_commit = "a9b97ef"
        capabilities_locked = $true
    }
    $evidencePath = Join-Path $build "offline_release_evidence.json"
    $evidence | ConvertTo-Json -Depth 4 |
        Set-Content -Encoding utf8 $evidencePath
    Write-Output "OFFLINE RELEASE VALIDATION: PASS"
    Write-Output $testSummary
    Write-Output "source commit: $commit"
    Write-Output "hardware profile sha256: $configHash"
    Write-Output "evidence: $evidencePath"
}
finally {
    Pop-Location
}
exit 0
