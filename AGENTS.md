# Robot_Treasure Repository Rules

## Git 与阶段规则

- Before making changes, inspect `git status`.
- Do not run `git commit` or `git push` automatically.
- Do not delete existing user files unless the task explicitly requires it.
- Do not overwrite a verified working test project.
- Do not casually modify historical-stage projects or rewrite historical facts. Each
  new experiment should preferentially be a separately copied project, so that a
  verified stage remains reproducible.
- Do not run `git add`, `git commit`, or `git push` unless the user explicitly asks.
- When real hardware verification is complete and a stage is being prepared for
  submission: update `Docs/阶段总结.md`, inspect `Docs/问题记录.md`, add a problem
  record only for a real problem that occurred and was investigated, then check the
  build, `git diff`, and `git status`. Commit and push still require explicit user
  authorization.
- Current-baseline documentation may be updated to reflect current facts. Do not
  retroactively rewrite a historical stage record as though it were the current
  state.
- Record only facts confirmed by hardware testing or by the actual code. When
  hardware testing has not been completed, state exactly: `仅编译通过，尚未完成硬件验证`.

## 工作原则

- Read the relevant existing code and documentation before changing anything; do not
  replace a working design from assumptions.
- Reuse modules that have already been verified where they fit the current task.
- Do not create empty drivers or simulated hardware modules for hardware that is not
  actually present.
- Do not perform broad refactors unrelated to the current task. Keep each change
  small, traceable, reviewable, and easy to roll back.

## Hardware and review safety
- After changes, list every modified and added file and provide a `git diff` summary.
- Stop and ask when the hardware model, pins, voltage, polarity, or wiring order is uncertain.
- Never guess power polarity.
- Do not start, drive, or test real motors automatically.
- Do not modify files unrelated to the current task.
- Keep build products and Keil-generated temporary files covered by `.gitignore`.
- Keep every change easy to review and roll back.
