# 492X / 492Z first-drive procedure

This procedure applies to the shared 12 V commissioning program. It does not
promote `hardware_output` or `driver_control`; each robot needs its own HIL
evidence.

## Before downloading

1. Select the exact robot profile and clean the prior build:

   ```powershell
   make clean
   make ROBOT_PROFILE=492X
   ```

   Replace `492X` with `492Z` only when working on 492Z.
2. Confirm the Brain download target and the intended robot identity aloud.
3. Compare the physical wiring to `docs/ROBOT_PROFILES.md`.
4. Put the Lift at its physical lower stop before power-up.

## Airborne test

1. Raise the drivetrain so no wheel touches the floor. Keep one observer at the
   Brain stop/disable control.
2. Boot the program and confirm the Brain and Controller show the selected
   identity. A mismatch means the build is wrong; do not move a joystick.
3. Apply about 10% forward input and confirm all six wheels produce a forward
   chassis tendency without fighting each other.
4. Check reverse, left/right automatic Quick Turn, neutral Coast and the B-button
   Coast override.
5. At low Axis2 input, confirm both Lift motors raise together. Stop immediately
   for opposite motion, encoder disagreement, noise or binding.
6. Disconnect the Controller and disable/re-enable to confirm stale commands do
   not resume.

## Floor test

Only continue after the airborne test passes. Start with low input in an open
area, then test straight forward/reverse, Curvature turns, automatic Quick Turn,
neutral Coast distance and short full-voltage acceleration runs. Record motor
velocity, current, voltage, temperature and any persistent yaw separately for
492X and 492Z.

## Stop conditions

Immediately disable for a wrong identity, wrong wheel or Lift direction,
mechanical opposition, abnormal gear noise, cable interference, sustained high
current, control loss or failure to Coast/Hold as specified. Evidence from one
robot must never unlock the other profile.
