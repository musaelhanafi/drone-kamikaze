# SITL ArduPlane + X-Plane — HIL Sensor Injection

Run ArduPlane entirely in software (SITL) on your laptop with X-Plane 11
providing **all sensor and GPS data** via MAVLink HIL messages.  No hardware
and no `--model JSON` required.

`mavlink_xplane.py` (the same script used for HITL hardware) connects directly
to the SITL binary and injects `HIL_SENSOR`, `HIL_STATE_QUATERNION`, and
`GPS_INPUT`.  SITL's own physics engine is overridden by the injected state.

---

## Architecture

```
X-Plane 11                 mavlink_xplane.py              ArduPlane SITL
──────────                 ─────────────────              ──────────────
DATA@ rows  ──UDP:49005──▶  parse sensor data  ──TCP:5760──▶  HIL_SENSOR
                                                             HIL_STATE_QUATERNION
                                                             GPS_INPUT

DREFs       ◀─UDP:49000──  xp_dref()          ◀─TCP:5760──  SERVO_OUTPUT_RAW

                            RC_CHANNELS_OVERRIDE ──TCP:5760──▶  joystick
```

All traffic stays on `127.0.0.1` (loopback).

---

## Comparison with `--model JSON`

| | This approach (HIL injection) | JSON model (`sitl_mavlink_xplane.py`) |
|---|---|---|
| Protocol | MAVLink HIL_SENSOR + GPS_INPUT | UDP JSON state |
| Script | `mavlink_xplane.py` | `sitl_mavlink_xplane.py` |
| SITL model flag | none (any model) | `--model JSON` |
| Servo feedback | `SERVO_OUTPUT_RAW` MAVLink | binary servo packet UDP:9002 |
| Same script as HITL? | **Yes** | No |

---

## Prerequisites

Same as the JSON-model setup — see [instruction_sitl.md](instruction_sitl.md#prerequisites).

---

## Step 1 — Configure X-Plane networking (once)

Same as [instruction_sitl.md Step 1](instruction_sitl.md#step-1--configure-x-plane-networking-once).

---

## Step 2 — Start ArduPlane SITL

```bash
cd /path/to/ardupilot

python3 Tools/autotest/sim_vehicle.py \
  --vehicle Plane \
  --no-rebuild \
  --location WICC \
  --no-mavproxy
```

- No `--model` flag — the internal physics model does not matter; HIL injection
  overrides it.
- `--no-mavproxy` — connect `mavlink_xplane.py` directly to arduplane on
  TCP:5760; no MAVProxy link-down noise.

---

## Step 3 — Set SITL parameters

Connect to arduplane (QGroundControl → TCP:5760, or `mavproxy.py --master
tcp:127.0.0.1:5760`) and set:

```
param set GPS1_TYPE      14
param set ARSPD_TYPE    100
param set EK3_SRC1_YAW   2
param set COMPASS_USE    0
param set ARMING_REQUIRE 0
param set SIM_SPEEDUP    1
reboot
```

| Parameter | Value | Why |
|---|---|---|
| `GPS1_TYPE` | 14 | Accept `GPS_INPUT` MAVLink injection |
| `ARSPD_TYPE` | 100 | X-Plane differential pressure via `HIL_SENSOR` |
| `EK3_SRC1_YAW` | 2 | GPS yaw from `HIL_STATE_QUATERNION` |
| `COMPASS_USE` | 0 | Compass driven by `HIL_SENSOR` mag; disable fuse |
| `ARMING_REQUIRE` | 0 | Skip pre-arm checks in HIL |
| `SIM_SPEEDUP` | 1 | Real-time 1:1 |

> These are saved in `/tmp/ArduPlane.stg` — only needed once per clean SITL instance.

---

## Step 4 — Run mavlink_xplane.py

Open a **second terminal**:

```bash
cd /path/to/ardupilot

python3 mavlink_xplane.py --pixhawk tcp:127.0.0.1:5760
```

With joystick:

```bash
python3 mavlink_xplane.py \
  --pixhawk tcp:127.0.0.1:5760 \
  --joystick \
  --joy-index 0 \
  --joy-roll-axis 0 \
  --joy-pitch-axis 1 \
  --joy-thr-axis 2 \
  --joy-yaw-axis 3 \
  --debug
```

### Expected startup output

```
[MAV] Connecting to tcp:127.0.0.1:5760 …
[MAV] Heartbeat  sysid=1  compid=0
[MAV] HIL mode enabled
[XP]  Listening on :49005  →  ('127.0.0.1', 49000)
[XP]  Parking brake SET
[XP]  DSEL rows [1, 3, 4, 16, 17, 20, 21]

Running — Ctrl+C to stop

[XP]  X-Plane 11 (build 115501)
[ATT] First attitude  hdg=108.0°
[GPS] lat=-6.900300  lon=107.575800  alt=738.0 m  hdg=108.0°
[SRV] (none)  (0 msg/s)
```

---

## Step 5 — Open QGroundControl (optional)

Connect manually: **TCP**, host `127.0.0.1`, port `5760`.

---

## Startup order

| Order | What | Where |
|---|---|---|
| 1 | X-Plane 11 loaded + unpaused | X-Plane |
| 2 | `sim_vehicle.py` | Terminal 1 |
| 3 | `mavlink_xplane.py --pixhawk tcp:127.0.0.1:5760` | Terminal 2 |
| 4 | QGroundControl (TCP:5760) | App (optional) |

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `[wait] No X-Plane DATA@` | X-Plane not sending data | Check Net Connections; unpause sim |
| `[MAV] Connecting …` hangs | SITL not running | Start `sim_vehicle.py` first |
| EKF not initialising | GPS1_TYPE not set to 14 | `param set GPS1_TYPE 14` and reboot |
| Controls don't move in X-Plane | DREFs lost | Wait 5 s for auto-refresh; reload aircraft |
| Attitude wrong/drifting | Compass still fused | `param set COMPASS_USE 0` and reboot |
| X-Plane aircraft doesn't move | Parking brake on | `B` key in X-Plane |

---

## Stopping

1. `Ctrl+C` in the `mavlink_xplane.py` terminal.
2. `Ctrl+C` in the `sim_vehicle.py` terminal.
3. Close QGroundControl.
