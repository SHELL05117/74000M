---
name: 74000pros
description: Enforce the 74000M PROS project workflow for architecture, C++ implementation, offline parallel development, simulation, testing, calibration, HMI, drivetrain, odometry, safety, command scheduling, and autonomous control. Use whenever an agent reads, plans, creates, reviews, debugs, or modifies code or configuration in C:\Users\alexh\Documents\override\74000M\74000M; required design documents come from the external C:\Users\alexh\Documents\VEX0713 library, while unverified hardware capabilities remain locked.
---

# 74000 PROS project skill

Treat `../../../74000pros.md` as the canonical and complete project execution specification.

## Mandatory procedure

1. Use `C:\Users\alexh\Documents\override\74000M\74000M` as the writable code project root.
2. Use `C:\Users\alexh\Documents\VEX0713` as the read-only external documentation root.
3. Read the project-root `74000pros.md` completely before planning or changing project code.
4. Identify every plan block affected by the request.
5. Resolve each block's `docs/...`, `docs2/...`, and `docs2/sim/...` paths against the external documentation root and read every listed document completely before generating or modifying code for that block.
6. If any required document is missing, unreadable, empty, truncated, or otherwise unavailable, stop that block. Do not generate, modify, or propose code for it. Report the exact absolute path.
7. Distinguish implementation dependencies from release gates. Do not implement against undefined upstream contracts, but allow downstream offline implementation when the needed interfaces are frozen or faithfully faked, even if upstream hardware/HIL/field exit evidence is pending.
8. For work that advances past an unmet hardware exit gate, keep unknown values explicitly unverified, use Fake IO/simulation/replay, leave affected capabilities disabled, and never claim HILValidated, FieldValidated, or CompetitionApproved without evidence.
9. Treat exit gates as requirements for capability unlock, hardware tuning, release, and competition approval rather than universal blockers on later offline coding.
10. Read `../../../docs/HARDWARE_COMMISSIONING.md` whenever planning or executing hardware bring-up, calibration, HIL, field validation, capability unlock, or competition approval.
11. Write all development files, configuration, build output, plans, and project rules only inside the writable code project unless the user explicitly requests otherwise. Do not modify the external documentation library.

Do not treat a prior summary, memory, heading scan, or excerpt as having read a required document. Read the current file from disk during the task.

Do not edit `74000pros.md`, this skill, or `AGENTS.md` unless the user explicitly asks to change the project protocol.
