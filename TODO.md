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
