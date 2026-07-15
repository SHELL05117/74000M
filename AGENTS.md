# VEX0713 mandatory agent instructions

These instructions apply to the entire project tree.

1. Before planning, generating, reviewing, debugging, or modifying project code or configuration, read `74000pros.md` completely and follow it as the default project skill.
2. Treat `C:\Users\alexh\Documents\VEX0713` as the read-only external documentation library. Identify the affected plan block(s), resolve each `docs/...` or `docs2/...` path against that library, and read every listed file completely before completing or writing code for the block.
3. If a required external document is missing, unreadable, empty, truncated, or otherwise unavailable, do not generate or modify code for that block. Stop and report the exact absolute path and failure.
4. A summary, previous conversation, heading list, or remembered content does not replace reading the current document.
5. Respect software interface dependencies, but an unmet hardware/HIL/field exit gate does not block later offline implementation when required contracts are frozen or faithfully faked. Keep unknown parameters unverified, affected capabilities disabled, and reserve exit gates for capability unlock, hardware tuning, release, and competition approval.
6. `docs2` controls platform facts, safety semantics, ownership, units, scheduling, and current architecture when it conflicts with `docs`.
7. Never introduce a second drivetrain motor writer, direct motor access from commands, blocking autonomous loops, `12 / battery_V` voltage amplification, or code that bypasses request arbitration and the safety gate.

All development and generated project files belong under this code project, not under the external documentation library. The discoverable project skill entry is `.agents/skills/74000pros/SKILL.md`.
For hardware bring-up, calibration, HIL, field validation, and capability unlock, also follow `docs/HARDWARE_COMMISSIONING.md`.
