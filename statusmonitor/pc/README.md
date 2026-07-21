# VEX Flight Log Status Monitor

This is the PC half of the 74000M VEX V5 flight-recorder system. It imports
V5L2 recordings from a microSD/TF card, verifies integrity before analysis,
archives the original bytes by SHA-256, creates derived columnar data, computes
diagnostic metrics and plots, and emits both a desktop GUI summary and an LLM
evidence pack.

The application never writes robot parameters or commands a robot.

## Desktop interface

Version 0.4 uses a guided Chinese engineering-console workflow.
The home screen contains only three choices: create a TF recording session,
continue an existing session, or open history. Creation and import use a
three-step form, then integrity checking, analysis, plots, and the LLM evidence
pack are generated in one background operation. The detailed tools remain
available inside the selected session instead of competing for attention on
the home screen.

The interface intentionally does not display absolute project, account, or
artifact paths. Runtime storage remains configurable through
`STATUSMONITOR_HOME`, `STATUSMONITOR_ARTIFACTS`, and `STATUSMONITOR_DB`.

## Windows install and run

```powershell
cd statusmonitor\pc
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e ".[dev]"
.\.venv\Scripts\statusmonitor.exe doctor
.\.venv\Scripts\statusmonitor-gui.exe
```

The repository also includes `setup.ps1`, `run_statusmonitor.ps1`, and
`build_windows.ps1` for setup, source launch, and a native Windows package.

## macOS one-click launch

Install Python 3.11 or newer, then double-click
`run_statusmonitor_macos.command` in Finder. The first launch creates the
isolated `.venv-macos` environment and installs the application dependencies;
later launches open the GUI directly. If macOS blocks the first launch,
Control-click the file and choose **Open**.

To create a normal native application bundle, double-click
`build_macos_app.command` on the target Mac. It builds and opens:

```text
dist/VEXFlightStatusMonitor.app
```

PyInstaller applications are platform- and architecture-specific, so the
macOS bundle must be built on a Mac. The generated `.app`, build directory,
virtual environment, and local analysis artifacts remain untracked.

For a headless workflow:

```powershell
statusmonitor new-session --team 74000M --operator Alex --test-type manual
statusmonitor scan-media E:\
statusmonitor import <session-id> E:\FLIGHT
statusmonitor analyze <run-id>
statusmonitor report <run-id>
```

Source runs default to the repository's `statusmonitor/artifacts` directory;
packaged desktop builds use the current user's
`Documents/VEXFlightStatusMonitor/artifacts` directory. Raw recordings are
copied once, hashed, and made read-only. Derived files are replaceable because
their algorithm version and source hash are recorded.

## Capability boundary

The current robot schema is V5L2 / LogFrame 3.1 (1536 bytes). Signals whose
availability bits are false are reported as `NOT AVAILABLE`; zeros are never
invented as measurements. A run with an invalid or unavailable pose does not
receive a trajectory-performance pass.
