# TODO / Backlog

## [HIGH PRIORITY] Rudder limits are not enforced while AP or REMOTE is steering (safety)

**Status:** found 2026-10-02 during the settings review, not started.

`rudder_limit_port_pct` / `rudder_limit_stbd_pct` (Settings) are only honoured by the
bridge's NUDGE and the rudder sweep. In AP and REMOTE the Nano stops only at
±`rudder_range` (`pilot_dira_lim_deg10` / `pilot_dirb_lim_deg10` come from pypilot's
`rudder.range`), and the Nano never reads the limit bytes the bridge writes into its
EEPROM. So e.g. Dyason's 80/20 limits protect nothing when pypilot drives.
Fix direction: enforce the soft limits on the Nano in every mode (it already has the
limit/hysteresis machinery for the pilot range), and send them in degrees.

## [MEDIUM] Settings & commissioning redesign

**Status:** discussed with the user 2026-10-02, not started. Two purposes for every
test: (1) find the right setting values for this installation, (2) prove hardware +
settings work reliably and safely. Every test must name the setting or check it feeds.

Findings (verified in code):
- `pgain` and `off_course_alarm_deg` are stored and written to the Nano EEPROM but
  **never read anywhere** — dead settings. The real heading gains are pypilot's
  (`ap.pilot.*`), not exposed in Inno-Pilot's UI.
- `deadband_pct` does three jobs in different units: Nano command threshold (% of
  command range), REMOTE/sweep position tolerance (% of full span — ±6° on Dyason at
  10%), and the bridge stall check. One value cannot suit all three.
- Rudder limits not enforced in AP/REMOTE (see HIGH item above).
- Rudder calibration (offset/scale/nonlinearity/range) lives only in pypilot's web UI
  (port 8000); `rudder_range` is stored twice (Inno-Pilot settings + pypilot) and synced.

Proposed commissioning order (each step depends on the ones before):
0. Wiring declarations (sensors present, invert_motor, invert_clutch, limit switches) —
   check: jog moves the way the button says; clutch engages/releases.
1. Rudder sensor calibration — check: 0° at centre, correct sign both ways.
2. End stops + soft limits a few degrees inside them — check: limit holds in EVERY mode.
3. Motor protection: max current from the sweep's measured peak (needs current sensor).
4. Position tolerance tuned with the rudder sweep (end error, overshoot, no hunting).
5. Rudder speed / hard-over at full power (needs a full-power drive mode — the sweep's
   position mode slows near target and can only flag it).
6. Compass calibration; re-check with the motor running.
7. Heading gains at sea (pypilot); then the off-course alarm.
Then reliability/safety verification: STOP from every source, limits in AP, loss of
remote / pypilot (open gap, see memory/bridge failsafe) / Nano, sea trial.

Proposed settings changes: soft limits in degrees and enforced everywhere; split
`deadband_pct` into a position tolerance (°) and a command threshold; replace `pgain`
with one or two real pypilot gains in plain words; implement or remove the off-course
alarm; add max motor current; move comms thresholds / ping interval to Advanced; bring
rudder calibration into Inno-Pilot; add a "commissioned" state so AP will not engage
until steps 0–4 have passed. The rudder sweep should then also refuse to run before
steps 1–2 are done. Its deadband lookup is isolated in `ram_nano_deadband_deg()`.

## [HIGH PRIORITY] Proper SignalK integration design — two operating scenarios

**Status:** parked, not started (2026-09-17).

Live SignalK auto-discovery (pypilot's `signalk.py` `ZeroConfProcess`) was disabled
in favor of a static config approach (see `CLAUDE.md` → "pypilot fork strategy" →
"Second instance"), because it conflicted with the host's own mDNS. That was a
narrow fix for the mDNS conflict, not a real design for how Inno-Pilot should use
SignalK. SignalK is core to the system and always wanted — this needs proper
engineering, not just "discovery off."

Two scenarios need explicit, designed-for behavior:

1. **No central SignalK instance present.** Inno-Pilot must stand alone and
   operate reliably on its own sensors (no wind/GPS/water-speed/etc. from
   SignalK). Needs: clear detection of "no SignalK configured/reachable" vs.
   "SignalK configured but temporarily down," and defined fallback behavior for
   every value pypilot normally sources from SignalK (see `signalk_table` in
   `compute_module/pypilot/pypilot/signalk.py`) — what does the autopilot do
   with each one when SignalK isn't there?

2. **Central SignalK instance present.** Target architecture per
   `servo_motor_control/docs/ARCHITECTURE.md`: one central, more powerful Pi
   runs a single SignalK instance; every other system (including each
   Inno-Pilot unit) connects to it as a client — not ad hoc LAN discovery of
   whichever SignalK happens to answer. Needs: a static `signalk_host`
   (host:port) config surface so an installer points an Inno-Pilot unit at its
   known central SignalK box directly, plus a sane reconnect/retry strategy if
   that connection drops (distinct from scenario 1's "never configured" case).

**Follow-up work, not yet started:**
- Design the config surface (where the SignalK host address lives per-instance
  — `/etc/inno-pilot/`, a pypilot config value, etc.).
- Implement it in the vendored pypilot fork (`compute_module/pypilot/pypilot/signalk.py`).
- Decide and implement standalone-mode fallback behavior for each SignalK-sourced value.
- Update `servo_motor_control/docs/ARCHITECTURE.md` to describe both scenarios explicitly.

---

## [HIGH PRIORITY] Nano watchdog — the other half of the hang protection

**Status:** not started (2026-09-19). Blocked on a bootloader check.

`Wire.setWireTimeout()` (B10) stops a stuck I2C bus hanging the Nano forever, but it
only covers I2C. Any other infinite loop or lockup still leaves the Nano wedged **with
the H-bridge pins held in whatever state they were driving** — a hard-over risk, since
nothing on the Nano side can recover it. The Pi-side failsafe (`pi_alive`, 5 s) cannot
help: that disengages the AP on the Pi, it cannot un-wedge a Nano holding `D9` high.

Needs: `wdt_enable(WDTO_2S)` plus `wdt_reset()` in `loop()`.

**Blocker to resolve first:** enabling the AVR watchdog requires a bootloader that
clears `WDRF` on boot. Older bootloaders bootloop into a brick-until-reflashed state.
Confirm which bootloader the Nano carries before turning this on.

## [MEDIUM] Surface `Wire.getWireTimeoutFlag()` as telemetry

**Status:** not started (2026-09-19).

B10 added the I2C timeout, which means a bus fault is now **silently recovered** and
leaves no trace. A recurring fault would be invisible. Surface
`Wire.getWireTimeoutFlag()` / `clearWireTimeoutFlag()` as a profiling metric (next free
code is `0xDD`) so it can be seen and alerted on. This bears directly on the outstanding
I2C ecosystem health-check idea.

## [MEDIUM] Reclaim ~2 KB flash and stack from printf/float formatting

**Status:** not started (2026-09-19). Measured, not yet acted on.

Flash is at **92%** (28278/30720) and RAM at **69%** (1427/2048, 621 bytes free). Both
are now real constraints on any further work, and AVR needs the SRAM headroom for stack.

Measured from the linked ELF (`avr-nm --print-size --size-sort`):

| symbol | bytes | pulled in by |
|---|---|---|
| `vfprintf` | 948 | 8 × `snprintf`, all using only `%u`, `%03u`, `%d`, `%s` |
| `dtoa_prf` | 702 | 3 × `dtostrf` |
| `__ftoa_engine` | 432 | same |

Replacing those with small integer-to-string helpers (formatting fixed-point as
`%u.%u`) recovers roughly **2 KB of flash and a chunk of stack**. The format strings
involved are trivial, so this is mechanical rather than risky.

Lower down: `OneWire::search` (256 B) + `DallasTemperature::isConnected` (228 B) could
shrink by storing the sensor's ROM address instead of index lookup. Soft-float is
~750 B+, but converting the ADC/calibration maths to fixed-point risks the
hand-calibrated constants — rank it last.

## [MEDIUM] Bridge flags a false RUDDER STALL on entering REMOTE with the rudder off-centre

`MODE MANUAL` sets `bstate.manual_rud_target = 500` as a placeholder (the Nano seeds
its real target from its own ADC and holds still). Until the first `RUD` arrives,
the bridge's stall check compares that placeholder with the actual rudder position,
sees a "commanded" move that never happens, and after ~1 s raises RUDDER_STALL
("RUDDER: NOT MOVING" on the remote). Reproduced in the v3.0.3 simulator
(`MODE MANUAL` with rudder at −9.6°). Pre-existing; found while testing the rudder
sweep. Fix direction: seed `manual_rud_target` from the current rudder_pct on entry,
or skip stall checks in MANUAL until the first `RUD`.

## [LOW] Hardware tests 2–9: bench-firmware integration (greyed out since v3.0.3)

They need `pwm_test.ino` plus a run-time protocol it does not have: a `TEST <id>`
dispatcher and `TEST_LINE`/`TEST_DONE` output, instead of compile-time `#define`
mode selection. Flip `NANO_TEST_PROTOCOL_SUPPORTED` in the bridge and set
`available: true` in the web remote's `TESTS` list once it exists. Before reviving
any of them, each must name the setting or metric it feeds (or be dropped) —
several overlap (overshoot, burst, fine-pulse all characterise Nano braking).
Also add a `test_mode` timeout so a missing `TEST_DONE` cannot latch the relay.

## [MEDIUM] Web remote: rudder display still lags ~220 ms via pypilot (option 2a)

**Status:** parked (2026-10-01). v3.0.2 fixed the SSE flood and stair-stepped rudder bar;
this is the remaining display lag. Ask the user how v3.0.2 feels on their device first.

The page's rudder position goes Nano `0xA7` (5 Hz) → pypilot `rudder.angle` (~220 ms later,
measured on Dyason 2026-09-30) → bridge 200 ms telemetry tick → web remote. During a nudge
the bar keeps moving ~0.5 s after release, which makes it hard to stop at a chosen angle.
Raw logs: `~/dyason-webremote-2026-09-30/` on the dev laptop.

Proposed fix (2a): the bridge, which already decodes every `0xA7` frame it relays, sends the
Nano's raw rudder straight to the web remote. Guards — all required:
- Convert raw → degrees with **pypilot's own** `rudder.scale` / `offset` / `nonlinearity`
  (watch them like `rudder.range`); never a second, hard-coded calibration — a divergent
  copy would show a different angle from the one pypilot steers with (cf. the Malu
  rudder-sign runaway history).
- Send it **only to the local (loopback) web remote**, or only on change. Remote sockets are
  non-blocking (`inno_pilot_bridge.py` ~2119): a full send buffer makes `sendall` fail and
  drops the client, so extra traffic would disconnect a physical ESP32 on weak WiFi.
- Keep rudder-stall detection and the nudge limit check on pypilot's value.

Rejected for now (2b): raising the Nano's `RUDDER_PERIOD_MS`. Needs a reflash + version bump,
adds frames to pypilot's servo loop on a Pi that already logs "running too _slowly_", and
`oled_draw()` blocking (26–56 ms) caps a steady rate anyway. Does not remove pypilot's lag.

## [MEDIUM] Investigate: steering motor disturbs the compass

**Status:** not started (2026-10-01). Observation only, one sample.

In the 2026-09-30 Debug capture (AP IDLE, boat not turning) `imu.heading` swung
240.2° → 244.5° during a ~2 s full-power nudge and returned after the motor stopped.
Suggests the motor or its supply current is a magnetic disturbance near the IMU — relevant
to Dyason's permanently latched "compass distortions" warning and wrong inclination
(+26° vs ~−65° expected for South Africa). Repeat with longer motor runs in both directions
and compare heading drift against motor current; check IMU placement vs motor/cabling.

## [LOW] `temp_service()` is now the second-largest loop blocker

**Status:** not started (2026-09-19).

After the `oled_draw()` work (184 ms → ~26 ms), `temp_service()` at **~27 ms** is now
roughly half the remaining per-second blackout. It is OneWire, not I2C, so none of the
I2C work touched it.

The DS18B20 *conversion* is already async (`setWaitForConversion(false)` at 10-bit). The
blocking part is the `getTempCByIndex()` scratchpad read. Making that async the same way
would roughly halve what remains.

Related, cheaper: `oled_draw()` still calls `read_voltage_v()` and `read_current_a()`
itself (~3 ms each — 3 throwaway reads × 300 µs settle + 16 conversions), duplicating
work the 200 ms block in `loop()` already does. ~6 ms recoverable by reusing those values.

## [LOW] Nano `oled_draw()` is slower while the motor runs

**Status:** not started (2026-10-01).

Dyason profiling during nudges (2026-09-30): `oled_draw` mean 33–44 ms, max ~56 ms, `loop`
max ~58 ms — versus the ~26 / ~36 ms idle baseline in CLAUDE.md. Likely more rows change
per redraw while the rudder moves (fewer row-shadow hits). Not a fault (RX high-water stayed
≤ 36 bytes of 128), but it is the loop blackout while steering. See the `temp_service()`
item above for the other blocker.

## [LOW] Bridge debug log labels motor direction backwards

**Status:** not started (2026-10-01). Cosmetic, debug log only.

`inno_pilot_bridge.py` ~1967 decodes `Nano motor pins` as `[PORT]` when D3 (LPWM) is high,
but on Dyason a NUDGE STBD drove D3 and the rudder moved to starboard (pypilot angle went
negative). Confirm against the Nano sketch's actual Dir-A/Dir-B mapping before changing —
it may be unit-wiring specific — then fix the label so debug captures aren't misleading.
