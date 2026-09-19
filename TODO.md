# TODO / Backlog

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
