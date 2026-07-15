---
name: 74000pros
description: Enforce the 74000M PROS project workflow for mandatory project orientation, architecture, C++ implementation, offline parallel development, simulation, testing, calibration, HMI, drivetrain, odometry, safety, command scheduling, autonomous control, scoped Git commits, and the English project change log. Use whenever an agent reads, plans, creates, reviews, debugs, documents, tests, or modifies anything in C:\Users\alexh\Documents\override\74000M\74000M; required design documents come from the external C:\Users\alexh\Documents\VEX0713 library, while unverified hardware capabilities remain locked.
---

# 74000 PROS project skill

Treat `../../../74000pros.md` as the canonical and complete project execution specification.

## Mandatory procedure

1. Use `C:\Users\alexh\Documents\override\74000M\74000M` as the writable code project root.
2. Use `C:\Users\alexh\Documents\VEX0713` as the read-only external documentation root.
3. Read the project-root `74000pros.md` completely before planning or changing project code.
4. Read `../understand-74000m/SKILL.md` completely before planning, inspecting, reviewing, documenting, testing, or changing any project artifact. Use it as the mandatory architecture, directory, API, coding, and test orientation.
5. Identify every plan block affected by the request.
6. Resolve each block's `docs/...`, `docs2/...`, and `docs2/sim/...` paths against the external documentation root and read every listed document completely before generating or modifying code for that block.
7. If any required document is missing, unreadable, empty, truncated, or otherwise unavailable, stop that block. Do not generate, modify, or propose code for it. Report the exact absolute path.
8. Distinguish implementation dependencies from release gates. Do not implement against undefined upstream contracts, but allow downstream offline implementation when the needed interfaces are frozen or faithfully faked, even if upstream hardware/HIL/field exit evidence is pending.
9. For work that advances past an unmet hardware exit gate, keep unknown values explicitly unverified, use Fake IO/simulation/replay, leave affected capabilities disabled, and never claim HILValidated, FieldValidated, or CompetitionApproved without evidence.
10. Treat exit gates as requirements for capability unlock, hardware tuning, release, and competition approval rather than universal blockers on later offline coding.
11. Read `../../../docs/HARDWARE_COMMISSIONING.md` whenever planning or executing hardware bring-up, calibration, HIL, field validation, capability unlock, or competition approval.
12. Write all development files, configuration, build output, plans, and project rules only inside the writable code project unless the user explicitly requests otherwise. Do not modify the external documentation library.

## Mandatory commit and change-log gate

After every completed project modification:

1. Preserve unrelated working-tree changes and stage only the task's files.
2. Run the required validation, then create a scoped implementation commit and capture its full 40-character Git hash.
3. Append exactly one simple English entry to `../../../CHANGELOG.md` using `- <full commit hash> — <plain description of what changed>.` Group entries by date, newest date first.
4. Commit only the log update with `chore(log): record <short implementation hash>`.
5. Do not create a log entry for the dedicated log-only commit; this exception prevents infinite recursive log commits.
6. Never invent a commit hash. If a commit cannot be created, report the blocker and do not claim the modification is fully complete. A later explicit user instruction can override committing, but not permit a false hash.

Do not treat a prior summary, memory, heading scan, or excerpt as having read a required document. Read the current file from disk during the task.

Do not edit `74000pros.md`, this skill, or `AGENTS.md` unless the user explicitly asks to change the project protocol.
