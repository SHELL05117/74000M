# 492X / 492Z robot profiles

The two robots share all control, safety, geometry, gearing, direction and Lift
travel code. Only identity and Smart Port assignments differ.

| Position | 492X | 492Z | Shared configuration |
|---|---:|---:|---|
| Left front | 2 | 3 | reversed, 600 rpm (6:1) |
| Left middle | 3 | 2 | reversed, 200 rpm (18:1) |
| Left rear | 1 | 1 | forward, 600 rpm (6:1) |
| Right front | 14 | 13 | forward, 600 rpm (6:1) |
| Right middle | 13 | 12 | forward, 200 rpm (18:1) |
| Right rear | 11 | 11 | reversed, 600 rpm (6:1) |
| Lift left | 19 | 14 | reversed, 200 rpm (18:1) |
| Lift right | 18 | 5 | forward, 200 rpm (18:1) |

## Selecting a robot

PROS builds default to `492Z` in the root `Makefile`. Always clean when
switching profiles:

```powershell
make clean
make ROBOT_PROFILE=492X
```

or:

```powershell
make clean
make ROBOT_PROFILE=492Z
```

For PC builds, pass `-DROBOT_PROFILE=492X` or `-DROBOT_PROFILE=492Z` to CMake.
The selection changes `RobotIdentity`, all motor ports, LCD identity, controller
team display and recording metadata as one unit.

## Files

- `include/robot/config/robot_profiles.hpp`: executable C++ configuration and
  compile-time selection.
- `config/robots/492X.yaml`: 492X hardware manifest.
- `config/robots/492Z.yaml`: 492Z hardware manifest.
- `config/hardware_profile.yaml`: profile registry and shared-fact declaration.

## Required HIL checks

Both profiles remain unverified. Before normal floor testing, perform the
airborne per-port direction test separately on each robot, verify that the
Brain displays the correct identity, confirm the Lift starts at its physical
lower stop, and confirm that the downloaded profile matches the connected
robot. Do not promote either robot based on evidence collected from the other.
