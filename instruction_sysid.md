# Flight Dynamics System Identification — SITL + X-Plane

Match ArduPlane SITL flight dynamics to X-Plane's physics model by logging
control inputs and aircraft responses from both, then fitting transfer-function
models per axis.

---

## Overview

```
ArduPlane SITL (AUTO)
    │  SERVO_OUTPUT_RAW (50 Hz)
    ▼
sysid_logger.py ──DREF──▶  X-Plane 11  ──DATA@ port 49005──▶  sysid_logger.py
    │                        (physics)
    │  SIMSTATE / VFR_HUD (50 Hz)
    ◀───────────────────────────────────────────────────────────────────────────
    │
    ▼
sysid_log.csv
    │  time | ail | elev | rud | thr | xp_roll/pitch/p/q/r | mav_roll/pitch/p/q/r
    ▼
sysid_analyze.py
    │  detect steps → fit gain + τ per axis → compare XP vs SITL → suggest params
    ▼
Suggested ArduPlane parameters:
    RLL_RATE_P / I / D,  RLL_2SRV_RMAX
    PTCH_RATE_P / I / D,  PTCH_2SRV_RMAX_UP / DN,  TECS_SPDWEIGHT
```

---

## Prerequisites

### Software
```bash
pip3 install pymavlink
pip3 install matplotlib      # optional — for --plot
```

### X-Plane 11 setup

1. **Settings → Net Connections → Data** tab
2. Under **"Send network data output"**:
   - IP: `127.0.0.1`  Port: `49005`
   - Check **Send** for these rows:

   | Row | Name |
   |-----|------|
   | 3   | Speeds |
   | 4   | Mach, VVI, G-load |
   | 16  | Angular velocities |
   | 17  | Pitch, roll, headings |
   | 20  | Lat, lon, altitude |

3. Under **"Accept network data input"**:
   - Port: `49000` (receives DREF commands from sysid_logger)

---

## Step 1 — Start ArduPlane SITL

```bash
cd /path/to/ardupilot
python3 Tools/autotest/sim_vehicle.py \
    --vehicle Plane \
    --location WICC \
    --map --console
```

Wait for the SITL prompt. Open QGroundControl or MAVProxy to upload a mission.

---

## Step 2 — Prepare a Sysid Mission

Create an AUTO mission that produces **sustained, varied control inputs**:

- Straight level cruise segment (30+ s) — captures throttle↔airspeed
- Banked turns left and right — captures roll axis
- Pitch up / pitch down — captures pitch axis
- Coordinated turns with rudder inputs — captures yaw axis

Upload the mission via QGroundControl or MAVProxy:
```
wp load sysid_mission.waypoints
mode AUTO
arm throttle
```

---

## Step 3 — Run the Logger

```bash
python3 sysid_logger.py \
    --sitl udp:127.0.0.1:14560 \
    --xplane-port 49000 \
    --xplane-data 49005 \
    --out flight01.csv \
    --rate 50
```

**Options:**

| Flag | Default | Description |
|------|---------|-------------|
| `--sitl` | `udp:127.0.0.1:14560` | MAVLink connection string |
| `--xplane-port` | `49000` | X-Plane DREF receive port |
| `--xplane-data` | `49005` | X-Plane DATA@ output port |
| `--out` | `sysid_log.csv` | Output CSV file |
| `--rate` | `50` | MAVLink message rate (Hz) |
| `--debug` | off | Print per-row debug output |

The logger will print status every 5 s:
```
[LOG]   45.0 s  2250 rows  ail=+0.32  thr=0.72  xp_ok
```

Press **Ctrl+C** to stop. The CSV is saved automatically.

---

## Step 4 — Analyse the Log

```bash
python3 sysid_analyze.py flight01.csv
```

With plot (requires matplotlib):
```bash
python3 sysid_analyze.py flight01.csv --plot
```

**Options:**

| Flag | Default | Description |
|------|---------|-------------|
| `--min-step` | `0.15` | Minimum control deflection to detect as a step |
| `--cruise-ias` | `60` | Cruise indicated airspeed (kts) for TECS mapping |
| `--plot` | off | Show time-series comparison plot |

### Sample output

```
========================================================================
 SYSTEM IDENTIFICATION RESULTS
========================================================================

  [ROLL]  3 step(s) found
    X-Plane  gain=42.3 °/s/unit   tau=0.18 s
    SITL/MAV gain=31.1 °/s/unit   tau=0.22 s
    Gap ratio (SITL/XP): 0.74  ⚠ needs tuning

  [PITCH]  2 step(s) found
    X-Plane  gain=28.7 °/s/unit   tau=0.21 s
    SITL/MAV gain=27.9 °/s/unit   tau=0.20 s
    Gap ratio (SITL/XP): 0.97  ✓ matched

  [YAW]  1 step(s) found
    X-Plane  gain=14.1 °/s/unit   tau=0.31 s
    (SITL data not available for this axis)

  [THROTTLE→AIRSPEED]  2 step(s)
    X-Plane  gain=38.4 kts/unit   tau=n/a

------------------------------------------------------------------------
  SUGGESTED ARDUPLANE PARAMETERS
------------------------------------------------------------------------
    RLL_RATE_P           = 0.132
    RLL_RATE_I           = 0.013
    RLL_RATE_D           = 0.045
    RLL_2SRV_RMAX        = 42.3
    PTCH_RATE_P          = 0.166
    PTCH_RATE_I          = 0.017
    PTCH_RATE_D          = 0.053
    PTCH_2SRV_RMAX_UP    = 28.7
    PTCH_2SRV_RMAX_DN    = 28.7
    TECS_SPDWEIGHT       = 1.56

  NOTE: These are starting points from first-order fits.
  Validate with AutoTune or in-flight testing before operations.
========================================================================
```

---

## Step 5 — Apply Parameters

Enter parameters via MAVProxy or QGroundControl:
```
param set RLL_RATE_P        0.132
param set RLL_RATE_I        0.013
param set RLL_RATE_D        0.045
param set RLL_2SRV_RMAX     42.3
param set PTCH_RATE_P       0.166
param set PTCH_RATE_I       0.017
param set PTCH_RATE_D       0.053
param set PTCH_2SRV_RMAX_UP 28.7
param set PTCH_2SRV_RMAX_DN 28.7
param set TECS_SPDWEIGHT    1.56
```

Re-run the logger and analyser to verify the gap ratio converges toward 1.0.

---

## Gap Ratio Interpretation

| Gap ratio | Meaning | Action |
|-----------|---------|--------|
| 0.8 – 1.2 | ✓ Matched | No change needed |
| < 0.8 | SITL under-responds vs X-Plane | Increase P gain / lower τ |
| > 1.2 | SITL over-responds vs X-Plane | Decrease P gain / raise τ |

---

## CSV Column Reference

| Column | Source | Description |
|--------|--------|-------------|
| `time_s` | logger | Elapsed seconds |
| `ail_r` | MAVLink servo1 | Aileron `[-1, 1]` |
| `elev_r` | MAVLink servo2 | Elevator `[-1, 1]` |
| `thr` | MAVLink servo3 | Throttle `[0, 1]` |
| `rud_r` | MAVLink servo4 | Rudder `[-1, 1]` |
| `xp_roll_deg` | X-Plane row 17 | Roll angle (°) |
| `xp_pitch_deg` | X-Plane row 17 | Pitch angle (°) |
| `xp_hdg_deg` | X-Plane row 17 | True heading (°) |
| `xp_p_dps` | X-Plane row 16 | Roll rate (°/s) |
| `xp_q_dps` | X-Plane row 16 | Pitch rate (°/s) |
| `xp_r_dps` | X-Plane row 16 | Yaw rate (°/s) |
| `xp_ias_kts` | X-Plane row 3 | Indicated airspeed (kts) |
| `xp_alt_ft` | X-Plane row 20 | Altitude MSL (ft) |
| `xp_vvi_fpm` | X-Plane row 4 | Vertical speed (ft/min) |
| `mav_roll_deg` | SIMSTATE | Roll angle (°) |
| `mav_pitch_deg` | SIMSTATE | Pitch angle (°) |
| `mav_hdg_deg` | SIMSTATE | Heading (°) |
| `mav_p_dps` | SIMSTATE xgyro | Roll rate (°/s) |
| `mav_q_dps` | SIMSTATE ygyro | Pitch rate (°/s) |
| `mav_r_dps` | SIMSTATE zgyro | Yaw rate (°/s) |
| `mav_airspeed_ms` | VFR_HUD | Airspeed (m/s) |
| `mav_climb_ms` | VFR_HUD | Climb rate (m/s) |
| `mav_alt_m` | GLOBAL_POSITION_INT | Altitude MSL (m) |

---

## Files

| File | Purpose |
|------|---------|
| `sysid_logger.py` | Data collection — bridge + CSV writer |
| `sysid_analyze.py` | Analysis — step detection, curve fitting, param output |
| `instruction_sysid.md` | This file |
