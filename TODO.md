# TODO / Backlog

> Unit naming: "Dyason" was renamed **Inno-Pilot-Rosa** on 2026-10-15 (destined for the yacht Rosa).

## [HIGH PRIORITY] Rudder limits are not enforced while AP or REMOTE is steering (safety)

**Status:** found 2026-10-02 during the settings review, not started.

`rudder_limit_port_pct` / `rudder_limit_stbd_pct` (Settings) are only honoured by the
bridge's NUDGE and the rudder sweep. In AP and REMOTE the Nano stops only at
±`rudder_range` (`pilot_dira_lim_deg10` / `pilot_dirb_lim_deg10` come from pypilot's
`rudder.range`), and the Nano never reads the limit bytes the bridge writes into its
EEPROM. So e.g. Inno-Pilot-Rosa's 80/20 limits protect nothing when pypilot drives.
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
  command range), REMOTE/sweep position tolerance (% of full span — ±6° on Inno-Pilot-Rosa at
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

### Setup wizards (agreed with the user 2026-10-15)

Settings panel is now split into **SETUP WIZARDS** (Startup, Dock-Side, Sea Trial —
placeholders, "COMING SOON") and **ADVANCED SETTINGS** (every individual setting,
collapsed by default). Wizards are not built yet.

**Hard rule: no automated rudder movement of any kind until rudder limits are set.**
That includes jog. Limits are therefore found by hand: clutch out, the skipper turns
the wheel/tiller to each stop while the wizard reads the rudder sensor. The rule must
be enforced in the bridge (refuse sweep/jog/PWM tests/AP/REMOTE), not only in the
wizard UI. Note: limits are not yet enforced in AP/REMOTE (HIGH item above), so
"limits set" must not be presented as "limits protect you" until that is fixed.

1. **Startup Wizard** — boat name; drive type (hydraulic pump / linear ram / wheel or
   tiller drive); hardware fitted (sensors, buttons, OLED type); rudder range; GPS
   source; notifications (Telegram); on-board Wi-Fi SSID + PSK (save last: the phone
   loses the connection). **Wi-Fi/network settings are currently dead** (see below) —
   this step needs a real apply mechanism on the Pi first.
2. **Dock-Side Wizard** — in order: rudder sensor read-out by hand → rudder limits by
   hand → *only then* jog: motor direction (invert motor) and clutch engage/release
   (invert clutch) → PWM capability test: start the motor, reduce PWM until it stops,
   measure rudder speed at each step, ask the skipper about squealing/whistling →
   result becomes the absolute minimum PWM (or "on/off drive only") → coast distance
   after cut-off at min / mid / max PWM → stop-to-stop speed → max motor current →
   position tolerance / deadband via the sweep → IMU mounting + level → gyro still-bias
   check → compass interference (motor, engine/alternator, bilge pump, radio on/off) →
   compass offset against the berth bearing → remote range test (dock conditions) →
   STOP from every source, clutch release, voltage/temperature against meters.
   All motion attended, dead-man hold.
3. **Sea Trial Wizard** — compass calibration (circles) → 8-direction alignment as an
   acceptance check (pypilot has one heading offset, no deviation table) → gains →
   current/temperature under load → hard-over under way → remote range test (sea
   conditions) → tack check (sail).
   **Gains:** P, I, D, DD, PR, FF (pypilot `basic` pilot) are too many for a skipper.
   The user expects Inno-Pilot to find them programmatically or with an AI agent.
   pypilot's own `pilots/autotune.py` is an unfinished experiment (`disabled = True`,
   searches P and D only) — a starting point, not a solution. Design task, open.

**Off-course alarm:** stays an Advanced setting, default 20° (already the default in
bridge and web remote; common marine default). Dead today — needs implementing.

### Dead settings register (verified 2026-10-15)

Stored in `settings.json` and written to the Nano EEPROM blob, but nothing on the Pi or
the Nano acts on them. The web remote shows them greyed out and locked, "(NOT USED)"
(`dead:true` in the `SF` table). Decide later: implement or remove.

| ID | Setting(s) | Notes | Status |
|---|---|---|---|
| DS-01 | `network.ip_mode`, `ip`, `mask`, `gateway`, `dns1`, `dns2` | Never applied to the Pi's network | open |
| DS-02 | `wifi.ssid`, `wifi.key` | Never applied to the Pi; needed by the Startup Wizard | open |
| DS-03 | `vessel.type` (sail/power) | Not read; could select tack check / pilot defaults | open |
| DS-04 | `autopilot.pgain` | Not read; real gains are pypilot's | open |
| DS-05 | `autopilot.off_course_alarm_deg` | Not read; keep, default 20°, implement | open |
| DS-06 | `safety.auto_disengage_on_fault` | Not read — shows ON but does nothing; misleading safety setting | open |
| DS-07 | `safety.comms_warn_threshold_pct`, `comms_crit_threshold_pct` | Not read | open |

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

**Interim link on Inno-Pilot-Rosa 2026-10-13: on 07:54, OFF 10:33:53 (GPS-24), back ON 11:03:30 UTC after GPS-24 was fixed; RETIRED 2026-10-15 09:29 UTC (Inno-Pilot-Rosa moving to another boat)** (scenario 2,
no code): pypilot's own persistent `nmea.client` setting pointed at the central SignalK Pi's
NMEA 0183 port `192.168.6.10:10110`. GPS + wind arrived as source `tcp`; a silent but open
socket dropped the sources after 8 s, data resuming restored them, a closed socket
reconnected at once. **But it was not one-way** (GPS-24): pypilot writes its own NMEA to
every socket, and SignalK's `nmea0183_feed` provider ingests and re-sends whatever arrives on
10110 — now disabled. See the protocol-migration item and register.

---

## [HIGH PRIORITY] Migrate the pypilot ↔ SignalK link from NMEA 0183 to SignalK's own protocol

**Status:** requested by the user 2026-10-13, not started. Part of the SignalK design item above.

NMEA 0183 over TCP (port 10110) was chosen as the fastest standard interim: no code, nothing
to change on the SignalK side. Its weaknesses all showed up on day one: loose text
conventions (magvar sign bug, local time instead of UTC — GPS-01/05), no units or source
metadata, duplicate sentences (GPS-04), ~100 sentences/s costing Inno-Pilot-Rosa ~1% IMU loop rate,
and only one direction without risking an echo loop (GPS-11).

Target: pypilot's built-in SignalK client (`compute_module/pypilot/pypilot/signalk.py`,
WebSocket + JSON deltas) — the standard OpenPlotter/pypilot pairing. SI units, source and
timestamp on every value, per-path subscriptions at chosen rates (e.g. position/COG/SOG 1 Hz,
wind 2 Hz), and two-way (pypilot heading/attitude/autopilot state back into SignalK/InfluxDB).
Work:
- Static SignalK host:port config (we stripped zeroconf discovery — see the item above).
- One-time device access request approved in SignalK admin; token stored in
  `~/.pypilot/signalk-token`.
- Check `signalk_table` for loops: it also subscribes to `navigation.headingMagnetic` /
  attitude (`imu`); pypilot must never consume its own published heading back.
- SignalK ranks lowest in pypilot (`source_priority` signalk = 4) — fine as sole source,
  but a local GPS (gpsd = 1) must keep pre-empting it.
- Then retire the `nmea.client` interim; code change → version bump of all components.
- NMEA 2000 is the other industry standard, but Malu has no N2K backbone; revisit only if
  N2K instruments are fitted.

---

## [HIGH PRIORITY] GPS / compass / SignalK findings register (opened 2026-10-13)

**Status:** open register, stable IDs — close items in place, never delete. Found while
linking Inno-Pilot-Rosa to the central SignalK Pi ("Malu Brain", lysmarine, 192.168.6.10). Reference
values for Malu's berth (WMM2025, 33.919 S 18.444 E, Oct 2026): declination −26.8° (W),
dip −64.6°, total field 25.0 µT, horizontal only 10.7 µT (small biases → big heading errors).
Berth bearing from OpenStreetMap: pier finger 132.4° true = 159.2° magnetic.
**Context (user, 2026-10-13):** Malu Brain is on the test bench **without the wind vane and anemometer
connected** — today's wind values (AWA ~44°, AWS 0, "true wind" 180°) are bench artefacts, not real wind.

| ID | Where | Finding | Fix (standard convention) | Status |
|---|---|---|---|---|
| GPS-01 | Brain: gpsd 3.22 pseudo-NMEA | `$GPRMC` magvar `-25.4,W` (double negative); pypilot stores `gps.declination` +25.4. gpsd's own JSON says `magvar: -25.4` (correct sign) — the bug is only in gpsd's NMEA formatter; 25.4 is gpsd's coarse table (WMM2025: 26.8 W) | Single magvar source from WMM2025; newer gpsd or don't relay gpsd's RMC magvar | open — **2026-10-15:** since the PVT-only change (GPS-04) the RMC magvar is `-4.8,W` (was `-25.4,W`; truth 26.8 W): now wrong in size as well as sign. Anything taking variation from RMC (OpenCPN, AvNav) is ~22° off. Likely gpsd now reads a value from NAV-PVT — investigate |
| GPS-02 | Brain derived-data; pypilot `gps_filter.py` | Variation from expired WMM2020 (−26.16°); gpsd table 25.4 W; WMM2025 −26.81° | Use WMM2025 everywhere | open |
| GPS-03 | Brain: u-blox 7 (USB, 1546:01a7) | Was 5 Hz (`ublox7-apply.sh`): `txbuf alloc` ~7×/20 s, ~7% of fixes lost | **Set to 1 Hz** (`REQ_RATE=1000`, sea model kept; script backed up `.bak-2026-10-13`). Verified over 60 s: 60 fixes, 0 `txbuf alloc`, 0 ANTSTATUS, 0 gpsd restarts | **fixed 2026-10-13 11:22 UTC** |
| GPS-04 | Brain: gpsd → port 10110 | Receiver sent 6 UBX messages per fix (NAV-PVT, -SOL, -VELNED, -TIMEGPS, -DOP, -SVINFO); gpsd 3.22 emitted 2–3 TPV per fix, **the first with the new time but the previous second's position/speed**; → 2 RMC per fix, plus a 3rd GGA from the SignalK plugin | `ublox7-apply.sh` now enables NAV-PVT and **disables NAV-SOL/-VELNED/-TIMEGPS**, then SAVEs (backup `.bak-2026-10-13b`); plugin GGA off. Verified 30 s: receiver sends PVT+DOP+SVINFO only; **1 RMC and 1 GGA per fix, 0 conflicting**; GPS 11 sentences/s. Remaining identical GSA/GBS repeats (~1.5/fix) are harmless | **fixed 2026-10-13 14:27 UTC** |
| GPS-05 | Brain: signalk-to-nmea0183 | `$GPGLL` stamped with local time (SAST) not UTC | Plugin's GLL (and duplicate GGA) turned off — gpsd supplies the GPS sentences (config backup `sk-to-nmea0183.json.bak-2026-10-13`) | **closed 2026-10-13** |
| GPS-06 | Brain → Inno-Pilot-Rosa | SignalK true wind (no water-speed sensor) outranks pypilot's own (`tcp` 3 vs `gps+wind` 6) | No true-wind mode until validated under way, or stop sending MWV,T | open (rule) — dock values were bench artefacts (sensors disconnected) |
| GPS-07 | Brain: u-blox | `ANTSTATUS` DONTKNOW→INIT→OK mid-stream | Explained by GPS-19 (receiver reconfigured every ~43 s) | explained |
| GPS-08 | Brain → Inno-Pilot-Rosa | A Freeboard-SK destination would put APB on 10110 → nav mode offered on Inno-Pilot-Rosa | Keep APB out of 10110 until nav mode is commissioned | open (rule) |
| GPS-09 | Inno-Pilot-Rosa port 20220 | Any LAN device can inject GPS/wind/route NMEA, unauthenticated | Standard pypilot design; Wi-Fi PSK is the boundary | awareness |
| GPS-10 | Network | Router LAN renumbered to 192.168.6.0/24 (notes said 192.168.252.x); DHCP pool .100–.254 | Fixed leases outside the pool on the MikroTik: **Inno-Pilot-Rosa 192.168.6.20, Venus OS Pi 192.168.6.11** (Brain .10 is static on the host). Config backup taken first | **fixed 2026-10-13** (Inno-Pilot-Rosa moved 11:02 UTC) |
| GPS-11 | Echo loop | Inno-Pilot-Rosa's `$APRSA` came back to it via SignalK (live 07:54–10:33 UTC through GPS-24); ignored only because the servo rudder source outranks `tcp` | Any SignalK connection carrying Inno-Pilot-Rosa's sentences must have `suppress0183event: true` (`pypilot_nmea` has it) | **resolved with GPS-24** |
| GPS-12 | Inno-Pilot-Rosa compass | Heading did not match the berth reference | After GPS-14/15/16: **158.9° vs berth 159.2° M / steering compass 162.5–165°**. Still needs an on-water compass swing | improved, open |
| GPS-13 | Inno-Pilot UI | Steering mode (compass/gps/wind) only selectable in pypilot's own web page | See calibration-tab item; mode selector later | open |
| GPS-14 | Inno-Pilot-Rosa `imu.heading_offset` | Was +45° although the IMU is mounted x→bow, y→stbd, z→keel | Set to 0 (user-approved, after the mirror test passed) | **fixed 2026-10-13 10:57 UTC** |
| GPS-15 | Inno-Pilot-Rosa IMU | Handled 09:21 UTC for inspection; tilt changed ~5° | pypilot level (`imu.alignmentCounter=100`): pitch/roll now ≈0.0° | **fixed 2026-10-13 10:57 UTC** |
| GPS-16 | Inno-Pilot-Rosa stored calibration | Old fit 55.9 µT / −76.3° (z poorly constrained, locked) | Replaced by pypilot's own fit from the hand tumble: bias (−1.58, 7.45, −7.00), 17.05 µT, −59°; **re-locked 10:58 UTC**. Acceptance criterion corrected: the fit radius equals the *local* field at the IMU (17–18 µT here, see GPS-17), not Earth's 25 µT — accept by heading agreement on several headings | fixed; swing open |
| GPS-17 | Inno-Pilot-Rosa magnetometer / mount | Hand test: **no mirrored axis** (gyro +105.7° vs magnetometer +103.9°); sensor's own bias (−0.84, 6.09, −7.29) µT; no gain anisotropy. Local field at the mount 18.0 µT (Earth 25.0): **~8 µT local disturbance, mostly vertical**, steep gradient → a close source | Locate it: phone magnetometer sweep / list steel, magnets, motor, speakers, DC cables within ~0.5 m; move the IMU if needed | open — prime suspect: the hydraulic pump motor ~0.5 m from the IMU (Brain is 3 m away, not a suspect); user plans to relocate the IMU and re-test |
| GPS-18 | Malu steering compass | 151° was a rough handheld-compass reading (user); steering compass 162.5° / 165° consistent, ~3–6° from berth bearing | Deviation card after a compass swing | closed |
| GPS-19 | Brain: `ublox7-apply.path` | `PathExists=` re-triggers the oneshot `ublox7-apply.service` forever: 252 reconfigs and 29 gpsd stop/starts in 3 h (flaky readback → "mismatch" → stops gpsd); each run burns ~10 s CPU, i.e. ~¼ core continuously | Drop-in `10-run-once-per-plug.conf`: `RemainAfterExit=yes` + `BindsTo=/After=dev-ttyLYS_gps_0.device` (re-plug re-applies) | **fixed 2026-10-13 10:31 UTC**, verified: last run 10:33:09, then stays active (exited) |
| GPS-20 | Brain: `gps-health.service` | Failed every run (`HOME: unbound variable`); log files only, wrong UBX parsing | Hourly timer **retired** (`systemctl disable --now gps-health.timer`); script kept for manual use | **closed 2026-10-13** |
| GPS-21 | Brain: SignalK `pypilot_nmea` | Stock LysMarine TCP client to `127.0.0.1:20220` (no local pypilot), refused every 5 s | **Parked:** host → `192.168.6.20:20220` (Inno-Pilot-Rosa), `enabled: false`, `suppress0183event: true`. Enable once the compass swing is done | parked 2026-10-13 |
| GPS-22 | Brain: signalk-venus-plugin | MQTT host was `192.168.6.114` (stale) | `venus.json` → **192.168.6.11**; the Venus OS Pi moved there 11:16:48 UTC, plugin connected (78 electrical values), errors stopped | **fixed 2026-10-13** |
| GPS-23 | Inno-Pilot-Rosa interim link | Re-enabled 11:03:30 UTC after GPS-24 was fixed. Open: survives a pypilot restart? Wi-Fi roaming behaviour? | Verify | open — **retired 2026-10-15 09:29 UTC:** `nmea.client` cleared on user instruction (Inno-Pilot-Rosa is moving to another boat; the Brain stays on Malu) |
| GPS-24 | Brain: SignalK `nmea0183_feed` | `tcpserver` input on 10110 (`suppress0183event: false`) ingested Inno-Pilot-Rosa's sentences and re-sent them | **Disabled** (user confirmed nothing else sends into 10110); settings backed up as `*.bak-2026-10-13`. Verified after re-linking: SignalK holds no Inno-Pilot-Rosa heading/rudder, 0 `$AP` sentences on 10110 | **fixed 2026-10-13 11:01 UTC** |
| GPS-25 | Inno-Pilot-Rosa bridge `ota_host()` | Caches its own LAN IP on first success, forever: after an IP change or roam the OTA URL offered to a remote is stale until the bridge restarts | Re-resolve per offer (or on HELLO) | open (latent; remotes offline) |
| GPS-26 | Brain: wind → 10110 | Wind dominates the stream at 1 Hz GPS: ~18 `$IIMWV`/s (R + T) and ~9.5 `$IIVWR`/s (the `Pyth_*` scripts update ~9×/s; the NMEA plugin converts every update) — most of what Inno-Pilot-Rosa now parses | Throttle wind output (or script rate) to ~2 Hz | open — rate measured with the vane disconnected (ADC noise); re-measure with sensors connected |
| GPS-27 | Brain: `Wind_Angle_and_Battery_Voltage_to_SigK.py` (Malu_Brain repo) | Computes "true wind" from apparent wind + GPS **SOG** but publishes it as `environment.wind.angleTrueWater`/`speedTrue` (SignalK reserves those for speed-through-water); assumes motion along the bow (no COG/leeway); uses apparent speed polled from SignalK once a second; with zero apparent wind it always yields 180° (explains GPS-06's dock reading). Publishes ≥5 Hz plus early sends on tiny angle/voltage changes (12-bit ADC jitter) → ~9 Hz, battery voltage riding along | Publish as `angleTrueGround`/`speedOverGround` (or compute water-referenced only when STW exists); publish at 2 Hz at source if wanted | open |
| GPS-28 | Malu_Brain repo (github.com/aviljoenn/Malu_Brain) | Today's live changes on Brain are not in the repo: `ublox7-apply.sh` (1 Hz, PVT-only), the `ublox7-apply.service.d` drop-ins (incl. the pre-existing `override.conf`) and `.path` unit, `gps-health.timer` retired, SignalK settings edits. A reinstall from the repo would revert them | Sync via a Malu_Brain branch/PR (only when the user says) | open |
| GPS-29 | Inno-Pilot-Rosa backup GPS (if Malu Brain dies) | Inno-Pilot-Rosa sits deep in the stern under the solar panels — a module there cannot see the sky | **Stop-gap (2026-10-13):** spare Android phone running Share GPS as a TCP *client* to `192.168.6.20:20220` (same `tcp` rank as the SignalK feed; pypilot keeps the active device and switches ~8 s after it goes quiet). **Permanent:** u-blox M8/M10 on Inno-Pilot-Rosa's UART (GPIO14/15, pins 8/10) as `/dev/gps0` via gpsd (rank 1 → primary, SignalK becomes the backup), with an **external active antenna** routed to open sky; optional flag-gated step in `install.sh`/`inno_deploy.sh` | **stop-gap working 2026-10-13 19:05 UTC**: Galaxy Tab S3 (SM-T825, Android 9) + Share GPS 1.0.0 (Brilliant Mantis), Connect/Client → 192.168.6.20:20220. Failover test: SignalK feed removed → Inno-Pilot-Rosa switched to the tablet (`GNsocket8`) in 1.7 s; magvar from the tablet has the correct sign (−24.9). With SignalK restored, Inno-Pilot-Rosa stays on the tablet (same-rank device lock) until the tablet stops. Limits: auto-connect is Premium; needs Wi-Fi + router — **2026-10-15:** Inno-Pilot-Rosa's only GPS is now the S10 (unofficial Android 14) running Share GPS → `inno-pilot.local:20220`; it took over in 0.3 s when the SignalK feed was cut. Inno-Pilot-Rosa keeps a stale `gps.declination` of +4.8 from the Brain (only used by align-to-GPS) — **confirmed 09:34 UTC:** tablet connection deleted; Inno-Pilot-Rosa kept 1 fix/s from `GPsocket27` = the S10 via `inno-pilot.local` |
| GPS-33 | Inno-Pilot-Rosa hostname | Inno-Pilot-Rosa still uses the generic default hostname `Inno-Pilot` (mDNS `inno-pilot.local`). A second Inno-Pilot on the same LAN would make avahi rename one (`Inno-Pilot-2.local`), silently pointing name-based clients (e.g. the S10's Share GPS) at the wrong unit or nowhere | Give each unit a unique hostname (e.g. `dyason`) as an install-time setting | open |
| GPS-34 | Inno-Pilot-Rosa Wi-Fi on the other boat | Only one saved Wi-Fi (`preconfigured` = Malu's RENDRE) → unreachable on any other boat | **Fallback hotspot `Innopilot-Rosa`** (user choice 2026-10-15): `/usr/local/sbin/innopilot-hotspot-fallback.sh` + `innopilot-hotspot-fallback.service` (enabled; at boot waits 90 s for a known network, else `nmcli con up Innopilot-Rosa`, 10.42.0.1/24, ch 6, WPA2); self-reverting test `/usr/local/sbin/innopilot-hotspot-test.sh [secs]`. Hotspot profile created 2026-10-15 12:0x UTC from vault key `Inno-Pilot/Innopilot-Rosa/psk` (root-only keyfile, autoconnect=no); **Test 2026-10-15 10:10–10:16 UTC:** hotspot up on ch 6, both phones joined and reached Inno-Pilot-Rosa by `inno-pilot.local` (web pages); Inno-Pilot-Rosa returned to RENDRE at 10:16:27. **But Share GPS went Idle when the network changed and did not reconnect** → no GPS during the window (no new pypilot socket until a manual reconnect: `GPsocket28` at 10:18). Fix proposed: reverse direction (Share GPS Listen/Server on the S10, pypilot `nmea.client` → S10 at a reserved hotspot address; pypilot retries every 30 s). Later: fold into `install.sh` as an option | test passed for Wi-Fi/names; GPS reconnect open — **10:31 UTC: UDP route live:** `innopilot-nmea-udp.service` on Inno-Pilot-Rosa (UDP 10110 → pypilot 20220, checksum-validated, `$PYPBS` dropped); S10 Share GPS = TCP/IP, Connect/Client, `inno-pilot.local:10110`, "Use UDP" on → Inno-Pilot-Rosa GPS via the receiver (`GPsocket29`) on Malu. Next: hotspot test with a mid-window hotspot bounce (no phone taps allowed) — **Bounce test 10:35–10:45 UTC — FAILED for Share GPS (corrected after the user's report):** receiver log: 1080 sentences/min from the S10 on RENDRE (192.168.6.245) before the test; **0 from 10:36 to 10:43** although the S10 joined the hotspot at 10:35:55 (10.42.0.144); flow resumed only at ~10:43:40 when the user tapped Connect, and stopped again after the window. Share GPS, in UDP mode too, stops sending on every network change. The hotspot, DHCP, receiver and `.local` all worked. Next: gpsdRelay (F-Droid, UDP broadcast, network monitoring, autostart). **Finding:** when the hotspot went down, NM at once rejoined RENDRE (autoconnect); on Rosa (no saved network) a hotspot drop would leave Inno-Pilot-Rosa off Wi-Fi until reboot — make the fallback a periodic watchdog |
| GPS-35 | Phone GPS app | Share GPS stops sending on every network change (TCP and UDP) — needs a tap | **Parked by the user 2026-10-15 — GPS is a FUTURE ENHANCEMENT, not a safety/reliability priority:** evaluate **gpsdRelay** (F-Droid, Apache-2.0, v2.2 Oct 2025): UDP broadcast to port 10110 (Inno-Pilot-Rosa's receiver already listens), network-change monitoring, start at boot. Test with the hotspot bounce test, no taps allowed | parked |
| GPS-30 | Brain: u-blox 7 / `ublox7-apply.sh` / gpsd 3.22 | **(a) SOG wandered 0–2.2 kn at rest** (mean 0.98 kn, position scatter 8–15 m) while the dongle sat indoors on the bench: 4–5 satellites, ≤23 dB-Hz, HDOP 3.0–3.7, GBS σ 45/33/25 m. **(b) After the dongle was moved to open deck and re-plugged, the receiver sent NAV-SOL/TIMEGPS/DOP/SVINFO and no NAV-PVT** (the saved message set did not survive the power loss). gpsd 3.22 then reported lat/lon 0,0, speed 0.000, `mode` 3 and RMC status `A`, so SignalK's position became 0,0 and OpenCPN/AvNav/Freeboard were fed a valid-looking null-island fix. `ublox7-apply.sh` verified only rate + dynModel, so it logged OK. Inno-Pilot-Rosa was unaffected (on the tablet feed, same-rank lock) | (a) Antenna with sky view: on deck, 9 sats, HDOP 0.9, SOG mean 0.15 kn (max 0.40), scatter 0.7 m. (b) Live fix 11:41 UTC: `ubxtool -P 14 127.0.0.1:2947 -p CFG-MSG,1,7,1 -p CFG-MSG,1,6,0 -p CFG-MSG,1,18,0 -p CFG-MSG,1,32,0`. Durable: `ublox7-apply.sh` now watches the wire (`msgs_ok`), requires NAV-PVT and no SOL/VELNED/TIMEGPS, re-applies up to 3×, exits 1 + WARN summary on failure (installed 13:50 SAST 2026-10-14, backup `.bak-2026-10-14`; tested incl. the failure path) | (a)+(b) **fixed 2026-10-14**. **gpsd-restart gap closed 2026-10-14 12:00 UTC:** a gpsd restart used to put the wrong message set back (SOL/TIMEGPS, no PVT) because the apply script was not re-run; drop-in `/etc/systemd/system/gpsd.service.d/20-reapply-ublox.conf` (`ExecStartPost=-` queues `systemctl --no-block restart ublox7-apply.service`, skipped while it is `activating`) now re-triggers it. Tested live: gpsd restart 12:00:45 → apply restarted at 12:00:49 → wrong set on the wire for ~45 s (30 s `ExecStartPre` sleep) → corrected 12:01:35, summary `MESSAGES … ok=yes`, exit 0, one gpsd start only. Drop-in source: `~/gps-sog-check-2026-10-14/20-reapply-ublox.conf` (to be added to Malu_Brain with GPS-28). Re-plug (Test A, 13:00 UTC) and full power cycle (Test B, 13:09 UTC) both passed: the apply script restored dyn=5 and PVT-only automatically, its retry loop fired in both, no 0,0 fix, valid fix 91 s / 195 s after the event (logs in `~/gps-sog-check-2026-10-14/testA_*.log`, `testB_laptop.log`). Still open: with SOL-only gpsd 3.22 gave valid lat/lon after a clean restart, so the 0,0 fix after the re-plug is not fully explained by the message set alone; COG is held at rest (consumers should ignore COG/SOG below ~1 kn); no consumer-side quality gate (sats < 6 or HDOP > 2.5) in pypilot — would be a code change + version bump; script not yet in Malu_Brain repo (GPS-28) |
| GPS-31 | Brain: gpsd RMC magvar | gpsd 3.22 computes magvar from the first position it sees and caches it for the session: after a cold start/0,0 episode it sends `-4.8,W` (correct ≈ −26.8, WMM2025) until restarted. Inno-Pilot-Rosa is unaffected only while it takes GPS from the tablet (`gps.declination` comes from the active source's RMC) | A manual gpsd restart cleared it (12:00 UTC 2026-10-14, back to gpsd's coarse `-25.4,W`) **but it recurred after the full power cycle (Test B): `-4.8,W` at 13:14 UTC with a valid fix**, because gpsd started before the receiver had a fix. So it recurs after every cold start/re-plug. Options: restart gpsd once after the first valid fix (e.g. from the apply script), or stop relaying gpsd's magvar (GPS-01/02) | **open — recurs on every cold start** (reopened 2026-10-14 13:15 UTC) |
| GPS-32 | Brain: SignalK / gpsd after a dongle re-plug or cold start | Found in the Test A re-plug, 2026-10-14 13:00–13:02 UTC. After the replug the receiver cold-started and, for **53 s (13:01:30–13:02:23)**, sent RMC with status **V** (no fix) and a position drifting around **−18.0, −25.35** (Atlantic, thousands of km from Malu); **SignalK took it as the vessel position** (`navigation.position` followed it, and showed 0,0 earlier at 13:01:58). The anchoralarm plugin is installed on the Brain, so an armed alarm would show a false drag; OpenCPN/AvNav/Freeboard show the boat jumping. REST also returned nothing for 66 s while the data was stale. Inno-Pilot-Rosa is immune (`nmea.py` hard-rejects RMC status V) but would not be immune to the 0,0 *A* fix of GPS-30. Also confirmed: **an unplug wipes ALL receiver config** (dynModel back to 0, message set back to default); the apply script restored dyn=5 and PVT-only 49 s after the replug (first attempt found the wrong set, retry loop fixed it) and a valid fix returned 91 s after the replug | Find which SignalK provider carries gpsd's data and make it ignore fixes with status V / `mode` < 2 (provider setting, or a position-validity guard); until then, treat Brain GPS as untrustworthy for ~2 min after any replug/boot and do not arm the anchor alarm in that window | open (found 2026-10-14) **Test B (full power cycle, 2026-10-14 13:09–13:14 UTC) repeated it and was worse:** clean poweroff 13:09:24, power restored ≈13:10:40; stream back 13:10:57; then **155 s of status-V positions (13:11:20–13:13:55), SignalK followed 151 of those seconds** (positions jumping around central Africa, ≈ −7, 21); first valid fix 13:13:55 = **3 min 15 s after power on**. `ublox7-apply` at boot: 30 s delay, first check found the wrong message set (SOL/TIMEGPS), retry fixed it, OK at 13:11:39 (dyn back to 0 after the power loss, restored to 5); no 0,0 (A) fix this time |

### [HIGH] Malu Brain stack upgrade: SignalK + plugins + Debian + BBN OS (research 2026-10-14, GPS-32 option D)

**Status:** user chose the full upgrade (2026-10-14); research done, nothing changed on the Brain. Inventory (read-only): BBN OS build 2025-05-18 (= the newest BBN release), Debian 12.x bookworm, Pi 5, kernel 6.12.47, Node 22.20, SignalK 2.13.5 + parser 3.15.3 (npm global in /usr/lib/node_modules, runs as `signalk`, config /home/signalk/.signalk), gpsd 3.22, OpenCPN 5.10.2, AvNav 20250822, InfluxDB 1.12.2, Kapacitor 1.8.1, Chronograf 1.10.8, Grafana 12.0.0, Telegraf 1.36.1, Mopidy 3.4.2, motion 4.7.0, Tailscale 1.102.2, RustDesk 1.4.6; 312 apt upgrades pending.
- **Why upgrade:** 2.13.5 is affected by ~12 published advisories, incl. 2 critical (privilege escalation via /enableSecurity, fixed 2.24.0-beta.4; unauthenticated JWT-secret disclosure via /skServer/ziplogs, fixed 2.33.0 — mitigation without upgrading: set *Data Logging Directory* in server settings), 2 high (ws-login brute force, ReDoS on subscription paths; fixed 2.25.0). Brain listens on `*:3000`/`*:10110`, reachable over LAN and Tailscale. Also fixes GPS-32 (parser honours RMC status V, parser >= 3.22).
- **SignalK target:** 2.33.0 (needs Node >= 22: OK). Avoid 2.24-2.26 (REST slowdown on Pi 5, fixed 2.27, issue #2624). 2.24 also: baconjs 3 (plugin crash class, #2521 fixed), `allow_readonly` now defaults false when security is on (check our read-only REST users: monitors, laptop scripts), plugin crashes isolated, React-19 admin UI default. 2.14.0 needed Node 20/22.
- **No BBN bundle exists to upgrade to:** newest BBN release is v2025-05-18 (Bookworm); `trixie` branch is 80 commits ahead with green CI builds to 2026-07-01 but **no release**. BBN docs: apt upgrade for OS packages; SignalK is *not* a Debian package; "SignalK upgrade via web UI is not really supported by BBN OS for stability reasons"; modules by CLI via pnpm; "you might end up with a configuration which wasn't previously tested". BBN issue #515 (open): no supported in-place upgrade path, only re-flash. A major Debian in-place upgrade was declared unsupported/broken by BBN for buster->bullseye; Raspberry Pi also advises clean install for bookworm->trixie.
- **Trixie facts:** 3rd-party repos publish trixie (Mopidy, free-x/AvNav, Tailscale, RPi, Mosquitto; Influx `stable`). Trixie has gpsd 3.25 (our GPS-30/31 workarounds were built on 3.22 — retest), OpenCPN 5.10.2 (5.14.1 in trixie-backports), Python 3.13 (externally-managed pip; venv rebuilds for pypilot/motionEye/Python scripts), labwc/Wayland default on RPi OS (BBN uses Budgie/X11).
- **Known smaller issues:** BBN #480 (SignalK 100% CPU at boot with an ESP32 sensor hub on direct USB; powered hub fixes), BBN #449 (OpenCPN 5.10 lockups with Dutch ENC), SignalK #2877 (NMEA2000 loss; Brain has no N2K), pypilot probes serial ports aggressively (SignalK #2684).
- **Custom Brain work that a re-flash would erase (GPS-28!):** `ublox7-apply.sh` + units/drop-ins, `gpsd.service.d/20-reapply-ublox.conf`, `ttyLYS_gps_0` udev rule, SignalK settings (providers, nmea0183_feed off, sk-to-nmea0183 GGA/GLL off, venus.json, pypilot_nmea parked), `Pyth_*` wind/battery scripts, `/home/user/Malu_Brain`, InfluxDB/Grafana data, Tailscale/RustDesk identity.
- **Not yet checked (needs sudo on the Brain):** installed SignalK plugin list + versions and per-plugin compatibility with 2.33 / baconjs 3; whether Node-RED, pypilot (BBN's own) and AvNav plugins are in use; Influx/Grafana dashboards to export.
- **Plugin inventory done 2026-10-14** (sudo, read-only, secrets redacted; raw report `~/gps-sog-check-2026-10-14/signalk_inventory.txt`): ~80 plugin packages installed by BBN (pnpm layout, `pnpm-lock.yaml`, `SIGNALK_DISABLE_SERVER_UPDATES=true`, config dir 1.3 GB) but only **10 configured/enabled**: anchoralarm 1.17.3, autopilot 1.5.0, charts 3.0.0, derived-data 1.36.0, freeboard-sk (bundled), path-filter 1.0.1, simple-notifications 1.3.0, sk-to-nmea0183 (bundled), venus 1.43.1, xdr-parser 1.0.5. KIP 3.2.0 webapp installed. Node-RED not running. Providers enabled: gpsd, OpenCPN, plus dead serial gps1/gps2/ais0 and gps1_gpsd/gps2_gpsd (ports 29471/2 not listening) and the two `Pyth_*` scripts.
  - **Node 22 vs plugins:** newest `signalk-derived-data` 1.46.0 and `@signalk/charts-plugin` 3.8.0 need **Node >= 24**; derived-data 1.45.0 (>=20.10) and charts 3.7.0 (no engines) do not. SignalK 2.33 itself needs >= 22 (Docker default is 24). Choose: stay on Node 22 and pin those two, or move Node 22 -> 24 (NodeSource `node_24.x` publishes for Debian 12).
  - **Major jumps, all appear to be TypeScript ports + new features, no config-format break found in the notes:** venus 1.43.1 -> 2.4.1 (2.0.1 fixed dbus option errors; PUT for VRM; DC-DC mappings), anchoralarm 1.17.3 -> 2.0.1 (rode-counter automation; **safety-relevant: re-test alarm radius/state after upgrade**), autopilot 1.5.0 -> 2.6.0 (V2 API optional; configured but Brain has no N2K autopilot), KIP 3.2 -> 4.8.5 (4.0.x had upgrade bugs: back up applicationData first). xdr-parser, path-filter, simple-notifications unchanged since 2022 (simple-notifications bundles its own old baconjs; fine).
  - **Settings that matter for the upgrade:** `security.json` has `allow_readonly: true` (persisted, so the 2.24 default change should not apply — verify after), `allowNewUserRegistration: true` and `expiration: NEVER` (review with the advisories), 2 users, 2 device tokens. interfaces include `ws`, `tcp`, `nmea-tcp` open on all interfaces. mDNS on (BBN default).
- **Proposed order (each step reversible, one at a time, boat attended):** 0) tarball of /home/signalk/.signalk + Influx/Grafana export + sync Malu_Brain repo (GPS-28) + full NVMe image; 1) mitigate GHSA-4w32 now (Data Logging Directory); 2) SignalK 2.33.0 + plugin updates on bookworm (`npm i -g signalk-server@2.33.0`, config backed up, rollback = reinstall 2.13.5 + restore dir); re-test GPS-32 with a re-plug; 3) `apt upgrade` within bookworm; 4) only then Trixie: wait for a BBN trixie release, or build the `trixie` branch image yourself, onto a spare NVMe, config migrated by script — never in place.

Existing items that still apply: never use pypilot's "align compass to GPS" (crash-bugged in
`sensors.py`: `last`/`total` undefined); the bridge has no pypilot-death failsafe.

---

## [MEDIUM] Web remote: Compass & Gyro Calibration tab

**Status:** requested by the user 2026-10-13, not started. Today compass/IMU calibration is
only reachable in pypilot's own web page (port 8000) — see GPS-12/13/14/15.

Scope (plain skipper wording on screen; numbers/µT only in Debug — remote UI rules):
- Compass calibration: status (good / needs calibration), lock/unlock, guided "turn slow
  circles" flow with progress; accept only when field strength and dip match the local
  reference (WMM for the boat's GPS position) within tolerance.
- Level (pypilot `imu.alignmentCounter`), heading offset (`imu.heading_offset`) with a
  "match the ship's compass" helper, gyro bias status, accelerometer calibration status.
- Refuse while the autopilot is engaged; log every change to the event log.
- Slots into "Settings & commissioning redesign" step 6.

---

## [MEDIUM] Reconcile our pypilot fork with upstream pypilot

**Status:** requested by the user 2026-10-13, not started.

`compute_module/pypilot/` is vendored at `strversion = '0.56'` and has drifted. Goal: a
documented delta, not a blind merge (the fork-strip strategy in `CLAUDE.md` still applies).
- Record the upstream base commit/tag the fork came from; diff against current upstream.
- Classify every difference: (a) upstream fixes we should take, (b) our fixes worth
  offering upstream (Flask `Markup` import guard in `web/web.py`, `gps_filter.py`
  `import datetime`, possibly the align-to-GPS crash and WMM2020→WMM2025), (c) deliberate
  Inno-Pilot strips/patches to keep (zeroconf ×2, `signalk.py` discovery idle,
  `serialprobe.py` skipping `.real` links).
- Note the two-pass `setup.py install` gotcha when testing (deploy only via `inno_deploy.sh`).

**Progress 2026-10-14:** upstream (github.com/pypilot/pypilot) is at **0.71** (HEAD `fdff88f`,
2026-10-07); there are no release tags after v0.9, the version only moves in
`pypilot/version.py`. 0.56 = `f74400a` (2024-07-10), 0.60 = `1c84266`, 0.70 = `6560fdd`,
0.71 = `fe1acd2`; 191 upstream commits since 0.56. The fork's `rudder.py` is byte-identical to
upstream 0.56 and `server.py` differs only by our zeroconf strip. Not yet diffed: the rest.
Cherry-pick candidates, each to be checked against the fork before applying (one concern per PR):
- `ab9339a` (atomic `pypilot.conf` write — **being applied 2026-10-14**; the SignalK token/timeout,
  `TrueWind` re-synthesis and NMEA lock/APB parts of the same commit are separate candidates).
- `61a81d4` non-owner writes to persistent values (a cleared `nmea.client` reappears after restart).
- `fad5f3b` RSA parse failure no longer yields angle 0; `06467f3` MWV direction range 0..359.
- `d7b2e0c` SignalK host override config (also answers the parked static `signalk_host` follow-up).
- Serial probing: `ca897e4`, `b577624`, `4b702b0`, `bb645ec` (baud in `~/.pypilot/serial_ports`).
- Pilot behaviour (`7481254`, `7a3b809`, `82cf670`, `04488fa`): changes steering — attended
  boat test only.
- **Checked and rejected:** `37d0e07` (rudder `lastrange`). Our `if self.lastrange and ...`
  already skips the recalculation when `lastrange` is 0, and applying the patch as written would
  stop the first `poll()` computing `minmax`, giving wrong nonlinear angles after a restart.
- Do **not** take wholesale: the `pyproject.toml` migration, lgpio and dependency overhaul collide
  with `inno_deploy.sh`'s two-pass install and the zeroconf/SignalK strips. Several upstream
  fixes repair bugs introduced after 0.56 (e.g. `3ab6c76`, `abfdece`) and do not apply to us.

---

## [MEDIUM] Settings are duplicated across pypilot, the bridge and the Nano (opened 2026-10-14)

**Status:** open register, stable IDs — close items in place, never delete. Found while
checking the atomic-config-write fix; read-only comparison of Inno-Pilot-Rosa's live files
(`~/.pypilot/pypilot.conf`, `/var/lib/inno-pilot/settings.json`,
`/etc/inno-pilot/rct_settings.json`) with `inno_pilot_bridge.py` and `motor_simple.ino`.

Three stores: pypilot (`pypilot.conf`), the bridge (`settings.json`, `rct_settings.json`) and the
Nano's EEPROM, which holds **two** regions — the bridge's settings block (addresses 0–175,
`_serialize_settings()`) and the 32-byte arduino_servo mirror (base 180). The Nano reads back
only three fields from the bridge block (feature flags, deadband, feature flags 2).
With `servo.use_eeprom=true` (Inno-Pilot-Rosa) pypilot **adopts the Nano's arduino_servo copy on every
connect** (`servo.py:698`), overwriting `rudder.*` and `servo.*` from `pypilot.conf` — so
`pypilot.conf` is not the last word for those keys.

| ID | Value | Copies | Finding | Suggested fix | Status |
|---|---|---|---|---|---|
| DUP-01 | WiFi SSID + key | `settings.json` (mode 0644, group `dialout`, i.e. world-readable) and Nano EEPROM | Key stored in plaintext in both; the Nano never reads it back. (The key also appeared in a Claude tool transcript on 2026-10-14 — rotate if that transcript is shared.) | Stop writing wifi/network to the Nano block; tighten `settings.json` to 0600; decide whether the key belongs in NetworkManager only | open |
| DUP-02 | Rudder range (30° on Inno-Pilot-Rosa) | `rudder.range`; `vessel.rudder_range_deg`; arduino_servo mirror; bridge block byte 12 | Three-way: bridge syncs both ways when they differ ≥1° (`inno_pilot_bridge.py` "Fix #5c"), and pypilot also takes the Nano's value on connect. No misbehaviour seen, but three writers | One owner (the Nano's EEPROM already wins on connect); the others become read-through | open |
| DUP-03 | Rudder calibration (`rudder.scale/offset/nonlinearity` = 167.75 / 0.25 / −46.25 on Inno-Pilot-Rosa) and `servo.*` limits/factors | `pypilot.conf`; Nano arduino_servo mirror (int16, quantised: offset×64, scale×8, nonlinearity×8); Nano RAM local calibration (RCAL, re-sent every 10 s) | Three copies; Nano EEPROM wins on connect; the Inno-Pilot-Rosa values look like multiples of 1/8, consistent with having round-tripped through the mirror (EEPROM not read to confirm). Nonlinearity is **non-zero** on Inno-Pilot-Rosa | Document the precedence in `docs/`; decide the single owner together with DUP-02 | open |
| DUP-04 | Feature flags (limit switches, temp sensor, current sensor, SH1106, clutch invert, …) | `settings.json` `features.*`; Nano EEPROM bytes 3 and 23; live `FEATURES_CODE` frames | Same data by two routes: EEPROM gives the Nano its config at boot before the Pi is up; the live frame follows. Intentional, but both must be pushed on every change | Keep; note it in the protocol docs | open (documented intent) |
| DUP-05 | Deadband (1.5 % on Inno-Pilot-Rosa) | `settings.json` `autopilot.deadband_pct` (bridge reads it directly); Nano EEPROM bytes 13–14 (`g_deadband`, loaded at boot) | Nano only picks up a change after the push + reboot | Keep; note it | open (documented intent) |
| DUP-06 | Mirror-only fields in the Nano EEPROM block | RCT ×7 (also `rct_settings.json`), vessel type/name/range, `pgain`, `off_course_alarm_deg`, rudder limit pcts, safety flags/thresholds, network, SSID/key | Written to the Nano, **never read back** by `motor_simple.ino` | Do not remove yet (changes the EEPROM layout + needs a Nano reflash for little gain); stop adding new fields to the block | open (low) |
| DUP-07 | `pgain` | `settings.json`; Nano EEPROM bytes 15–16; web remote UI | No consumer found: the bridge only holds the default and serialises it. Other mirror-only fields (`off_course_alarm_deg`, comms thresholds, `auto_disengage_on_fault`) were not individually traced | Trace each; remove the dead settings or wire them up | open |
| DUP-08 | Bridge settings files | `/var/lib/inno-pilot/settings.json`, `/etc/inno-pilot/rct_settings.json` | Written with plain `open(path, "w")` (truncate-then-write) — same power-loss exposure the pypilot atomic-write fix removes from `pypilot.conf` | Same tempfile + fsync + `os.replace` pattern in `_persist_pilot_settings()` / `_save_rct_settings()` — separate PR | open |

---

## [HIGH PRIORITY] Nano local STOP is immediate but not latched (safety)

**Status:** found 2026-10-15 (code review for the user's "immediate and unconditional disengage" priority).
**Implemented 2026-10-15 on branch `fix/safety-stop-latch-deadman-failsafe` (not merged, needs bench test):**
STOP latch + single-press STOP + Nano AP command watchdog. The wired helm STOP for Inno-Pilot-Rosa is still open (hardware).

`motor_simple.ino`: the Nano's STOP button acts locally at once (clears `ap_enabled_remote`, `remote_manual_active`,
command → neutral; clutch drops because `clutch_should` needs an active mode) and then sends `BTN_EVT_STOP` to the
bridge. But `COMMAND_CODE` handling sets `ap_enabled_remote = true` on ANY command frame, so if the STOP event is
lost or pypilot keeps commanding, the next COMMAND (~10 Hz) re-engages motor and clutch. Fix direction: a STOP
latch on the Nano — after a local STOP ignore COMMAND until a deliberate re-engage (e.g. pypilot reported
`ap.enabled` false and a fresh engage arrives). Also: on Inno-Pilot-Rosa the Nano box sits deep in the stern, so its
STOP is not reachable from the helm — needs a wired helm STOP. Good news verified at the same time: the Nano
disengages by itself after 5 s without Pi frames (`PI_OFFLINE_TIMEOUT_MS`) and after 3 s of critical Pi↔Nano
comms errors.

---

## [HIGH PRIORITY] REMOTE steering has no phone dead-man (safety)

**Status:** found 2026-10-15 while reviewing Wi-Fi failure modes for Rosa.
**Implemented 2026-10-15 on branch `fix/safety-stop-latch-deadman-failsafe` (not merged):** `ALIVE`
heartbeat (0.4 s / 1.5 s, armed by `MODE MANUAL ALIVE`, only from the page that selected REMOTE), plus the
NUDGE dead-man and the bridge pypilot-loss failsafe. Simulator-tested; needs a real-phone test.

The bridge drops REMOTE (`MODE MANUAL`) to IDLE + STEER LOSS only when its TCP client disconnects. With the
web remote, that client is `inno_web_remote.py` on the Pi, which stays connected when the phone vanishes
(screen off, battery, overboard, Wi-Fi lost). The web remote server keeps no browser-presence state, so a phone
lost mid-REMOTE leaves the rudder at its last target. Only the rudder sweep has a browser heartbeat
(`RAM HOLD`, 1 s). Fix direction: browser heartbeat while in REMOTE (like `RAM HOLD`); on loss → stop motion,
exit to IDLE with STEER LOSS, never auto-centre (rudder-sweep invariants in `CLAUDE.md`). AP mode must NOT
disengage on phone loss (keep course + alarm).

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
measured on Inno-Pilot-Rosa 2026-09-30) → bridge 200 ms telemetry tick → web remote. During a nudge
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
to Inno-Pilot-Rosa's permanently latched "compass distortions" warning and wrong inclination
(+26° vs ~−65° expected for South Africa). Repeat with longer motor runs in both directions
and compare heading drift against motor current; check IMU placement vs motor/cabling.

**Update 2026-10-13:** the IMU is ~0.5 m from the hydraulic pump motor (permanent-magnet motor =
static field; its supply current = field while running). Hand tumble measured a ~8 µT mostly-vertical
local disturbance at the mount (GPS-17). A magnet's stray field falls roughly with distance³ (2× further
≈ 8× weaker); a current-carrying cable's field falls with distance (30 A at 0.5 m ≈ 12 µT unless the
supply pair is run tightly together/twisted). The user plans to relocate the IMU, then repeat: hand
tumble, level, heading vs steering compass, attended motor-nudge test, on-water swing.

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

Inno-Pilot-Rosa profiling during nudges (2026-09-30): `oled_draw` mean 33–44 ms, max ~56 ms, `loop`
max ~58 ms — versus the ~26 / ~36 ms idle baseline in CLAUDE.md. Likely more rows change
per redraw while the rudder moves (fewer row-shadow hits). Not a fault (RX high-water stayed
≤ 36 bytes of 128), but it is the loop blackout while steering. See the `temp_service()`
item above for the other blocker.

## [LOW] Bridge debug log labels motor direction backwards

**Status:** not started (2026-10-01). Cosmetic, debug log only.

`inno_pilot_bridge.py` ~1967 decodes `Nano motor pins` as `[PORT]` when D3 (LPWM) is high,
but on Inno-Pilot-Rosa a NUDGE STBD drove D3 and the rudder moved to starboard (pypilot angle went
negative). Confirm against the Nano sketch's actual Dir-A/Dir-B mapping before changing —
it may be unit-wiring specific — then fix the label so debug captures aren't misleading.
