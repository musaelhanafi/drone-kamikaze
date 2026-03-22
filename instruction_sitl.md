# SITL ArduPlane + X-Plane — Laptop Setup

Run ArduPlane entirely in software (SITL) on your laptop with X-Plane 11
providing the flight model. No hardware required.

`sitl_mavlink_xplane.py` bridges the two: it converts X-Plane sensor data
into the ArduPilot JSON external-physics format and forwards ArduPlane servo
outputs back to X-Plane as DREF commands.

---

## Architecture

```
X-Plane 11                 sitl_mavlink_xplane.py        ArduPlane SITL
──────────                 ──────────────────────        ──────────────
DATA@ rows  ──UDP:49005──▶  parse sensor data  ──UDP:9003──▶  JSON state
                                                              (--model JSON)

DREFs       ◀─UDP:49000──  xp_dref()          ◀─UDP:9002──  servo PWM

                            RC_CHANNELS_OVERRIDE ──TCP:5760──▶ arduplane (joystick)

QGroundControl  ◀──────────────────────────────────TCP:5760──▶  arduplane
```

All traffic stays on `127.0.0.1` (loopback) — no network required.

---

## Prerequisites

### Python packages

```bash
pip3 install pymavlink mavproxy
pip3 install pygame   # only needed for joystick support
```

### ArduPilot SITL binary

Build from source if you have not already:

```bash
cd /path/to/ardupilot
./Tools/environment_install/install-prereqs-mac.sh   # macOS
# or: ./Tools/environment_install/install-prereqs-ubuntu.sh
./waf configure --board sitl
./waf plane
```

Binary lands at `build/sitl/bin/arduplane`.

### X-Plane 11

Running, with a fixed-wing aircraft loaded, sim **unpaused**.

---

## Step 1 — Configure X-Plane networking (once)

1. Go to **Settings → Net Connections → Data**.
2. Under **Send network data output**: IP `127.0.0.1`, port `49005`.
3. Under **Accept network data input**: port `49000` (default).
4. Open **Settings → Data Output** and tick **"Send data over the net"**
   for these rows:

   | Row # | Name | Used for |
   |---|---|---|
   | 1 | Frame rate | sim elapsed time |
   | 3 | Speeds | IAS → airspeed sensor |
   | 4 | G-load | body-frame accelerations |
   | 16 | Angular velocities | gyro rates |
   | 17 | Pitch, roll, heading | attitude + GPS yaw |
   | 20 | Lat, lon, altitude | GPS position |
   | 21 | Loc, vel, dist | NED velocity |

X-Plane saves these settings — only needed once.

---

## Step 2 — Start ArduPlane SITL

```bash
cd /path/to/ardupilot

python3 Tools/autotest/sim_vehicle.py \
  --vehicle Plane \
  --model JSON \
  --no-rebuild \
  --location WICC \
  --no-mavproxy
```

- `--model JSON` — use external JSON physics backend (SIM_JSON); SITL receives
  state from `sitl_mavlink_xplane.py` on UDP 9003 and sends servo PWM on UDP 9002.
- `--no-rebuild` — skip recompile (remove if you just changed code).
- `--no-mavproxy` — skip MAVProxy; connect QGroundControl and
  `sitl_mavlink_xplane.py` directly to arduplane on TCP 5760.

> **Note:** `--map` and `--console` require extra MAVProxy GUI packages
> (`pip3 install MAVProxy[gui]`). Omit them if not needed — use QGroundControl instead.

### WICC location variants

Defined in `Tools/autotest/locations.txt` (elevation 738 m MSL):

| `--location` | Description | Heading |
|---|---|---|
| `WICC` | Airport reference point | 108° |
| `WICC_RWY13` | RWY 13 threshold — take off toward SE | 130° |
| `WICC_RWY31` | RWY 31 threshold — take off toward NW | 310° |

Set the same location in X-Plane: **Location → Airports → WICC** then position
the aircraft on the matching runway end before unpausing.

Wait for the MAVProxy prompt:

```
MAV>
```

---

## Step 3 — Set SITL parameters

At the `MAV>` prompt (or in the MAVProxy console window):

```
param set ARSPD_TYPE 100
param set EK3_SRC1_YAW 2
param set COMPASS_USE 0
param set ARMING_REQUIRE 0
param set SIM_SPEEDUP 1
reboot
```

Wait for SITL to restart and the `MAV>` prompt to return.

> These are saved in `/tmp/ArduPlane.stg` — only needed once per clean SITL instance.

> **Note:** `GPS1_TYPE` is not required — with `--model JSON` the SITL physics
> backend provides GPS directly; no `GPS_INPUT` MAVLink injection is used.

---

## Step 4 — Run sitl_mavlink_xplane.py

Open a **second terminal**:

```bash
cd /path/to/ardupilot

python3 sitl_mavlink_xplane.py
```

Add `--debug` to print attitude, sensor, and DREF values every 5 s.

With joystick:

```bash
python3 sitl_mavlink_xplane.py \
  --joystick \
  --joy-index 0 \
  --joy-roll-axis 0 \
  --joy-pitch-axis 1 \
  --joy-thr-axis 2 \
  --joy-yaw-axis 3 \
  --joy-fltmode-axis 4\
  --debug
```

All ports use defaults (`--bind-port 49005`, `--xplane-port 49000`,
`--json-port 9003`, `--servo-port 9002`, `--sitl udp:127.0.0.1:14560`).

### Expected startup output

```
[MAV] Connecting to udp:127.0.0.1:14560 …
[MAV] Heartbeat  sysid=1  compid=0
[XP]  Listening on :49005  →  ('127.0.0.1', 49000)
[XP]  Parking brake SET
[XP]  DSEL rows [1, 3, 4, 16, 17, 20, 21]

Running — Ctrl+C to stop

[XP]  X-Plane 11 (build 115501)
[XP]  First DREF → ('127.0.0.1', 49000)
[NED] Origin set  lat=-6.900300  lon=107.575200  alt=738.0 m
[ATT] First attitude  hdg=108.0°
[SRV] (none)  (0 msg/s)
```

Once X-Plane is unpaused, `[JSON]` lines update each cycle and the
aircraft attitude in QGroundControl tracks the X-Plane aircraft.

---

## Step 5 — Open QGroundControl (optional)

Auto-connects to `UDP :14550`. Use it for arming, mode changes, and
parameter tuning.

---

## Startup order

| Order | What | Where |
|---|---|---|
| 1 | X-Plane 11 loaded + unpaused | X-Plane |
| 2 | `sim_vehicle.py` | Terminal 1 |
| 3 | `sitl_mavlink_xplane.py` | Terminal 2 |
| 4 | QGroundControl | App (optional) |

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `link 1 down` in MAVProxy | Normal with `--model JSON` — SITL waits for JSON data before sending heartbeats; start `sitl_mavlink_xplane.py` and the link comes back up |
| `[wait] No X-Plane DATA@` | X-Plane not sending data | Check Net Connections; unpause sim |
| `[MAV] Connecting …` hangs | SITL not running | Start `sim_vehicle.py` first |
| Controls don't move in X-Plane | DREFs lost after reload | Wait up to 5 s for auto-refresh; reload aircraft |
| EKF not initialising | No JSON state received | Check `sitl_mavlink_xplane.py` is running; check X-Plane is unpaused |
| Attitude in QGC wrong/drifting | Compass still enabled | Set `COMPASS_USE=0` and reboot SITL |
| X-Plane aircraft doesn't move | Parking brake on | Disengage brake in X-Plane (`B` key) |
| Joystick not detected | `pygame` not installed | `pip3 install pygame` |

---

## Stopping

1. `Ctrl+C` in the `sitl_mavlink_xplane.py` terminal.
2. `Ctrl+C` in the `sim_vehicle.py` terminal (or type `quit` at `MAV>`).
3. Close QGroundControl.
