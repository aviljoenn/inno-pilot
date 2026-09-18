# CLAUDE.md
Audience: Claude Code (and other AI coding agents) working in this repository.

## Prime directive
1) **Do not break working systems.** Keep changes minimal, focused, and testable.
2) **GitHub is the source of truth.** Modify repo files, open a PR, and describe what changed.
3) **Preserve human intent.** This repo runs on real hardware; avoid sweeping refactors.

---

## Coding style & preferences (very important)
### Comments
- **Do NOT remove existing Python comments** unless they are wrong or dangerously misleading.
- If a comment must change: prefer **fixing** it (or expanding it) rather than deleting it.
- Adding comments to improve human readability is mandatory. Keep them practical.

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
- Code must be compiled, started and executed at the least.
- Attempt must be made to automatically simulate user inputs where possible.
- If automatically simulating user inputs is not possible, the user must be guided and prompted on what to do to complete testing.

---

## Repo hygiene rules
- **Do not rename/move files** unless explicitly requested.
- Avoid formatting-only PRs.
- Keep PRs small (ideally one concern per PR).
- Never commit secrets, tokens, WiFi credentials, or private keys.

---

## What "done" looks like
A PR is "done" when it includes:
- A clear summary + rationale
- Tests/build steps run (or why not)
- Any new/changed docs needed (README/docs)
- Release notes updated

---

## How to run / verify (use what exists; don't invent tooling)
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

**Rule:** If the repo already uses a tool, use it. If not, don't add one unless asked.

---

## Firmware build / check
### PlatformIO (if `platformio.ini` exists)
- Build: `pio run`
- If tests exist: `pio test`

### Arduino CLI (if used in repo)
- Compile (example): `arduino-cli compile --fqbn <FQBN> <sketch_dir>`
- Do not guess FQBNs. Look for existing CI scripts/docs.

**Rule:** If you can't build firmware due to missing toolchain/board config, state that clearly in the PR and keep changes conservative.

> **Facts about the Inno-Pilot Raspberry Pi** 
> Every Inno-Pilot constructed may use different hardware types, E.g. Pi5, Pi Zero, Arduino nano, Pi Pico etc.
> Every Inno-Pilot constructed may be on different IP subnets with different IP addresses
> `arduino-cli` is installed at `/usr/local/bin/arduino-cli`
> with the `arduino:avr` core. The Nano is on `/dev/ttyUSB0`.
> Compile: `arduino-cli compile --clean --fqbn arduino:avr:nano --build-property "build.extra_flags=-DSERIAL_RX_BUFFER_SIZE=128" .`
> Upload:  `arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano .`
> The `SERIAL_RX_BUFFER_SIZE=128` flag is **required** — the default 64-byte buffer overflows
> during bridge telemetry bursts while the OLED I2C draw blocks `loop()`.
> `--clean` is **equally required**: without it `arduino-cli` reuses cached core/library
> objects that may have been built WITHOUT the flag, silently linking a 64-byte-buffer
> `HardwareSerial` against a sketch that believes it has 128. Under `-flto` that is a
> real ODR violation, and the only visible hint is an easily-missed note reading
> `HardwareSerial.h:93: array types have different bounds`.
> Demonstrated on 2026-09-17 — identical source, same command, cache state the only
> difference: incremental build = 1137 bytes RAM (64-byte buffer, warning present),
> clean build = 1201 bytes RAM (128-byte buffer, no warning). The 64-byte delta is the
> buffer. Verify a build landed correctly by checking the `Serial` object size in the
> ELF (`avr-nm --print-size --radix=d <elf> | grep ' Serial'`): **221 bytes = 128-byte
> RX buffer**; ~157 bytes means the flag did not reach the linked core.
> Stop `inno-pilot-bridge`, `inno-pilot-socat`, `pypilot` services before flashing; restart after.

---

## Change strategy
When asked to implement something:
1) Identify the smallest set of files to change.
2) Implement with guardrails (validation, bounds checks, sane defaults).
3) Update docs/comments where it prevents future mistakes.
4) Run the best available checks/tests.
5) Open PR with a high-signal description.
6) Check all components of inno-remote that might be versioned, like the nano sketch, the bridge, updated OTA binary that also lands at /var/lib/inno-pilot/ota/ on the Pi, inno-remote and inno-web-remote. Keep the version numbers of all those components in sync and push/flash the same version number to all components.

---

## Deployment: ALWAYS via `inno_deploy.sh` — never invoke pypilot's `setup.py` (or any other component's install steps) by hand

**`inno_deploy.sh` (run on the Pi) is the only sanctioned way to deploy changes
to a live Inno-Pilot instance.** This applies to every component it covers —
pypilot, the bridge, the web remote, systemd units, the Nano sketch — not just
pypilot.

Do **not** manually replicate its steps (e.g. running `setup.py install`
yourself, hand-copying files into `dist-packages`, manually restarting
individual services in some other order) even when you know exactly what a
step does and why — e.g. the two-pass `setup.py install` gotcha documented
below. `inno_deploy.sh` encodes the full ordering, guards, and safety checks
(stopping/restarting the right services in the right sequence, freeing
`/dev/ttyUSB0` before any Nano flash, the pyproject.toml/deps two-pass
workaround, etc.) — reproducing part of it ad hoc is how those guards get
silently skipped.

**How to apply:** if a fix needs deploying/testing on real hardware, say so
and either run `inno_deploy.sh` itself (after checking what it does — it can
flash the Nano and restart every service, so confirm with the user first
given the blast radius) or ask the user to run it. Never invoke a component's
own build/install tooling (`setup.py`, `pio run --target upload`,
`arduino-cli upload`, etc.) directly against a live instance as a shortcut —
those are implementation details `inno_deploy.sh` owns, not entry points for
an agent to call.

---

## PR description template (use this structure)
**What**
- (1–3 bullets)

**Why**
- (short rationale)

**How**
- Key implementation notes (include hardware assumptions if any)

**Notes / Follow-ups**
- Any TODOs, edge cases, or recommended next steps

---

## Hardware-safety flags (call these out explicitly)
If changes touch any of the following, add a dedicated "Hardware impact" section in the PR:
- Pin mappings / GPIO modes
- PWM frequency/duty behavior
- ADC scaling, voltage dividers, calibration constants
- Power control, relays, motors, H-bridges
- Interrupts, watchdogs, real-time loops
- Serial/I2C/SPI protocol timing

---

## pypilot fork strategy: progressively strip unwanted code

`compute_module/pypilot/` is a **fork of upstream pypilot**, vendored into this
repo, not a pristine third-party dependency. The long-term direction is for
Inno-Pilot to run its own autopilot-core code; until that replacement exists,
this fork is adapted incrementally to fit Inno-Pilot's actual needs rather than
kept upstream-compatible for its own sake.

**Principle:** whenever a piece of pypilot's behavior is found to conflict with,
duplicate, or not serve Inno-Pilot's standalone operation, strip it out —
don't just patch around it. Inno-Pilot instances must run rock-solid
standalone (no dependency on network services, discovery protocols, or
companion apps that aren't part of this project). Candidates for stripping
include anything pypilot does for generic/standalone-pypilot use cases that
Inno-Pilot doesn't need: third-party client auto-discovery, signalk/web
integrations we don't use, optional hardware backends we don't have, etc.

**How to apply:**
- Prefer deleting/commenting the offending code (with a clear comment
  explaining what it did and why it was removed — see Comments rules above)
  over adding a workaround elsewhere that tolerates the bad behavior.
- Keep changes scoped to what was actually diagnosed as a problem; this is
  incremental stripping as issues surface, not a license for a sweeping
  rewrite of the fork in one PR (see Prime directive #1 and #3).
- Because this is a vendored source fork (not `pip install pypilot`), edits
  live directly in `compute_module/pypilot/pypilot/*.py`. Deploying them
  always goes through `inno_deploy.sh` (see "Deployment: ALWAYS via
  inno_deploy.sh" above) — never hand-run the two-pass `setup.py install`
  described in the TWO-PASS gotcha below; that gotcha explains *why*
  `inno_deploy.sh` needs two passes, it is not deploy instructions to follow
  by hand.
- Note removed/disabled behavior in the PR description so it's clear this was
  a deliberate strip, not a regression, if upstream pypilot is ever diffed
  against again.

**First instance (2026-09-17):** `server.py`'s self-announcing zeroconf
(`zeroconf_service.py`, `_pypilot._tcp.local.`) was disabled — it ran a second
IPv4 mDNS responder on the same host as `avahi-daemon`, and the two competed
for UDP 5353, causing intermittent `<hostname>.local` resolution failures on
the LAN. Diagnosed on Dyason; avahi itself logged
`WARNING: Detected another IPv4 mDNS stack running on this host`. Nothing in
Inno-Pilot's bridge/web stack used pypilot's self-announcement (they connect
over a known local port), so it was pure unwanted surface area.

**Second instance (2026-09-17):** `signalk.py`'s `ZeroConfProcess` (also
python-zeroconf, scanning for `_http._tcp.local.` to auto-discover a SignalK
server) was disabled too — same UDP 5353 conflict with `avahi-daemon`,
persisting even after the first fix. Unlike the self-announcement, SignalK
integration itself is a real, wanted feature (Inno-Pilot's architecture is one
central SignalK Pi that every instance subscribes to — see
`servo_motor_control/docs/ARCHITECTURE.md`) — it's the *live LAN discovery*
mechanism that was unwanted, not SignalK support. `signalk.py`'s data
translation and HTTP polling logic are untouched; `ZeroConfProcess.process()`
now idles instead of scanning, so `signalk_host_port` never auto-populates.
**Follow-up:** add a static `signalk_host` config option so a known central
SignalK Pi's address can be configured directly, without depending on mDNS at
all — not yet implemented. **See `TODO.md` for the full two-scenario design
task this is parked under** (standalone-no-SignalK vs. central-SignalK) — this
was a narrow mDNS-conflict fix, not the real SignalK integration design.

**Third source, not a fork strip (2026-09-17):** even after both fixes above,
avahi still logged the conflict warning. Root cause was in **our own**
`compute_module/glue/inno_pilot_bridge.py`, not the pypilot fork: its
`pypilot_worker()` thread called `pypilotClient()` with no host argument.
pypilot's `client.py` sets `can_probe = not host` — an empty host makes every
(re)connect import python-zeroconf and scan for `_pypilot._tcp.local.`,
exactly the client-side counterpart of the two server-side sources above.
Bridge and pypilot always run co-resident on the same Pi at a fixed port, so
no discovery was ever needed here. Fixed by pinning the host explicitly:
`pypilotClient('127.0.0.1')`, which also sets `can_probe = False`. Lesson for
future pypilot integration code in this repo: **always pass an explicit host
to `pypilotClient()`** — an implicit/omitted host silently opens this same
mDNS surface area again.

---

## Known hardware gotcha: Nano reset via HUPCL

**The Arduino Nano resets whenever `/dev/ttyUSB0` is closed by any process.**

The Nano's RESET pin is wired to DTR through a 100 nF RC differentiator (standard
Arduino Uno/Nano design). Linux serial ports have the HUPCL flag set by default,
which drops DTR whenever the file descriptor is closed — even if `dtr=False` was
set while the port was open.

**Symptoms:** Nano shows high CRC error counts, bridge receives zero bytes from Nano
after any process closes the port (including a previous bridge instance, a flash
tool, or `stty`). The Nano is stuck in its `setup()` splash delay (~3 s) and can't
communicate.

**Fix already in place:** `open_serial_no_reset()` in `inno_pilot_bridge.py` clears
HUPCL via `termios.tcsetattr` immediately after opening the port.

**If you ever add a new tool that opens `/dev/ttyUSB0`** (diagnostic scripts,
sniffers, etc.), either:
1. Run `stty -F /dev/ttyUSB0 -hupcl` before closing, **or**
2. Open with `pyserial` and apply the same `termios` fix, **or**
3. Accept that the Nano will reset and allow ≥ 5 s before expecting frames.

---

---

## Known install issue: pypilot_client.conf stale IP after fresh install

After installing pypilot on a Pi that was previously connected to another pypilot
instance (or moved to a different network), `~/.pypilot/pypilot_client.conf` may
contain a stale IP address.  pypilot will then try to connect to the wrong host
instead of its own local instance.

**Symptom:** pypilot log shows connection attempts to an IP that is not this Pi.
OTA server unreachable because the bridge also picked up the wrong host from a
similar config.

**Fix:**
```bash
echo '{"host":"127.0.0.1","port":23322}' > ~/.pypilot/pypilot_client.conf
sudo systemctl restart pypilot
```

This file is auto-generated by pypilot's zeroconf discovery and will be overwritten
again if zeroconf finds a remote pypilot.  After a fresh install, always check and
reset it before the first reboot.

The OTA server host in the bridge was separately fixed (auto-detected via
`_local_ip()`) and no longer depends on this file.

---

## Known outstanding issue: pypilot EBUSY on /dev/ttyINNOPILOT (PTY)

pypilot's servo subprocess cannot open `/dev/ttyINNOPILOT` (errno 16, EBUSY) after
the bridge or pypilot restarts.

**Root cause (diagnosed, not yet fixed):** socat holds the master side of the PTY
pair open continuously.  When any process sets `TIOCEXCL` (exclusive mode) on the
slave PTY and then closes it, the exclusive flag persists on the device as long as
socat holds the master open.  pypilot's servo subprocess retries the open in a loop
and never succeeds until the service is restarted.

**Impact:** The autopilot servo is not connected to pypilot's control loop after a
bridge restart without a full service restart.

**Workaround:** `sudo systemctl restart pypilot inno-pilot-bridge inno-pilot-socat`
followed by ~5 s wait.  A full system reboot also clears the condition.

**Status:** Unresolved.  The fix likely requires either (a) not setting TIOCEXCL on
the slave PTY, or (b) having socat recreate the PTY pair on restart.

---

## Known issue: pypilot_web Flask 2.3+ incompatibility (fixed in source)

`flask.Markup` was removed in Flask 2.3 and moved to `markupsafe`.  On Raspberry Pi
OS Bookworm (Flask 3.x), the original `web/web.py` crashes with:
```
ImportError: cannot import name 'Markup' from 'flask'
```
**Fix is in the source** (`compute_module/pypilot/web/web.py`) with a try/except
import guard.  When deploying to a new Pi, run `setup.py install` from the repo so
the fixed file is used.  If you install pypilot from upstream (not the inno-pilot
fork), you will need to patch `web/web.py` manually.

---

## Known gotcha: editing pypilot Python source needs a TWO-PASS `setup.py install`

**A single `setup.py install` does NOT deploy edits to pypilot's pure-Python
source** (e.g. `web/web.py`, `autopilot.py`, anything under `compute_module/pypilot/`).
Only the C extensions and `pypilot_data` get copied; the Python modules are silently
skipped, so the running service keeps using the **old installed copy**.

**Why:** `dependencies.py` (invoked inside `setup.py install`) clones `pypilot_data`,
which drops a `pyproject.toml` into `compute_module/pypilot/`.  On setuptools 78 /
Python 3.13 (Bookworm/Trixie) that `pyproject.toml` is treated as authoritative and
overrides `setup(packages=…)`, so `build_py` skips the pypilot package source.

**Symptom:** you edit `web/web.py` (or any pypilot module), deploy, the service
restarts fine and reports `active` — but the change isn't there (diff the installed
copy under `/usr/local/lib/python3.*/dist-packages/pypilot/` against the repo and it
still shows the old value).  This was discovered while iterating on `web/web.py`:
a single-pass redeploy kept silently running the previously-installed copy.

**Fix (already in place):** both `install.sh` (Phase 3) and `inno_deploy.sh`
(Step 4b) do a **two-pass** install — pass 1 fetches deps + builds C extensions,
then pass 2 bypasses `dependencies.py` (`rm -f pyproject.toml; touch deps`) so
`build_py` uses `setup(packages=…)` and actually installs the Python modules.
Verify pass 2 ran by looking for `copying web/web.py -> build/...` in the log.

**If you add another deploy/install path that touches pypilot**, replicate the
two-pass pattern or your pure-Python edits will not land.

---

## When uncertain
- Prefer asking a clarifying question in the PR description or as a comment rather than guessing.
- If you must assume: **state assumptions explicitly** and keep the change minimal.
