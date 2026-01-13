# AGENTS.md
Audience: automated coding agents (Codex, etc.) working in this repository.

## Prime directive
1) **Do not break working systems.** Keep changes minimal, focused, and testable.
2) **GitHub is the source of truth.** Modify repo files, open a PR, and describe what changed.
3) **Preserve human intent.** This repo likely runs on real hardware; avoid sweeping refactors.

---

## Coding style & preferences (very important)
### Comments
- **Do NOT remove existing Python comments** unless they are wrong or dangerously misleading.
- If a comment must change: prefer **fixing** it (or expanding it) rather than deleting it.
- Adding comments to improve human readability is welcome. Keep them practical.

### Python
- Prefer **clear, explicit code** over cleverness.
- Add **type hints** for new/changed public functions where reasonable.
- Prefer docstrings for modules/classes with non-trivial behavior.
- Avoid adding heavy dependencies unless explicitly requested.

### Arduino / ESP32 (C++ / sketches)
- Keep changes small and hardware-safe.
- Avoid timing-sensitive behavior changes unless explicitly requested.
- Prefer constants, enums, and clear pin naming.
- If touching IO pins, power, ADC scaling, or interrupts: **explain assumptions** in the PR.

### Testing of code
- Testing for code is considered done if the code compiles without errors.

---

## Repo hygiene rules
- **Do not rename/move files** unless explicitly requested.
- Avoid formatting-only PRs.
- Keep PRs small (ideally one concern per PR).
- Never commit secrets, tokens, WiFi credentials, or private keys.

---

## What “done” looks like
A PR is “done” when it includes:
- A clear summary + rationale
- Tests/build steps run (or why not)
- Any new/changed docs needed (README/docs)

---

## How to run / verify (use what exists; don’t invent tooling)
Before adding new tooling, check what the repo already uses:
- Look for: `pyproject.toml`, `requirements.txt`, `setup.cfg`, `tox.ini`, `.pre-commit-config.yaml`,
  `platformio.ini`, `arduino-cli.yaml`, `Makefile`, GitHub Actions workflows.

### Python: preferred order
1) If `pyproject.toml` + Poetry:
   - Install: `poetry install`
   - Run: `poetry run pytest -q` (or existing commands)
2) If `requirements.txt`:
   - Install: `python -m venv .venv && . .venv/bin/activate && pip install -r requirements.txt`
   - Run: `pytest -q` (or existing commands)

### Python quality gates (only if present in repo)
- Formatting: `ruff format .` or `black .`
- Lint: `ruff check .` or `flake8`
- Types: `mypy .`

**Rule:** If the repo already uses a tool, use it. If not, don’t add one unless asked.

---

## Firmware build / check
### PlatformIO (if `platformio.ini` exists)
- Build: `pio run`
- If tests exist: `pio test`

### Arduino CLI (if used in repo)
- Compile (example): `arduino-cli compile --fqbn <FQBN> <sketch_dir>`
- Do not guess FQBNs. Look for existing CI scripts/docs.

**Rule:** If you can’t build firmware due to missing toolchain/board config, state that clearly in the PR and keep changes conservative.

---

## Change strategy
When asked to implement something:
1) Identify the smallest set of files to change.
2) Implement with guardrails (validation, bounds checks, sane defaults).
3) Update docs/comments where it prevents future mistakes.
4) Run the best available checks/tests.
5) Open PR with a high-signal description.

---

## PR description template (use this structure)
**What**
- (1–3 bullets)

**Why**
- (short rationale)

**How**
- Key implementation notes (include hardware assumptions if any)

**Verification**
- Commands run + results
- If not run: explain why + risk

**Notes / Follow-ups**
- Any TODOs, edge cases, or recommended next steps

---

## Hardware-safety flags (call these out explicitly)
If changes touch any of the following, add a dedicated “Hardware impact” section in the PR:
- Pin mappings / GPIO modes
- PWM frequency/duty behavior
- ADC scaling, voltage dividers, calibration constants
- Power control, relays, motors, H-bridges
- Interrupts, watchdogs, real-time loops
- Serial/I2C/SPI protocol timing

---

## When uncertain
- Prefer asking a clarifying question in the PR description or as a comment rather than guessing.
- If you must assume: **state assumptions explicitly** and keep the change minimal.
