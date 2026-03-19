# Release Notes: fmuv3-hil

**Date:** 2026-03-19
**Target:** ArduPlane on Pixhawk1 (fmuv3) — Hardware-In-the-Loop (HIL) simulation variant
**Context:** OPSI — Kendali Drone Kamikaze, HITL simulation using X-Plane 11

---

## Overview

`fmuv3-hil` is a new ArduPilot firmware target for the Pixhawk1 (fmuv3) board, configured specifically for Hardware-In-the-Loop (HIL) simulation with X-Plane 11 via `mavlink_xplane.py`.

It extends the standard `fmuv3` board definition with the SITL sensor framework enabled at build time, allowing real autopilot firmware to run on physical hardware while receiving simulated sensor data over MAVLink.

---

## New Files

| File | Description |
|------|-------------|
| `libraries/AP_HAL_ChibiOS/hwdef/fmuv3-hil/hwdef.dat` | Board definition: includes `fmuv3/hwdef.dat` and sets `SIM_ENABLED 1` |
| `libraries/AP_HAL_ChibiOS/hwdef/fmuv3-hil/defaults.parm` | Default parameters embedded in firmware for HITL operation |
| `Tools/bootloaders/fmuv3-hil_bl.bin` | Bootloader binary for fmuv3-hil target |

---

## HIL Architecture

```
X-Plane 11
    │
    ▼ (UDP)
mavlink_xplane.py
    │
    ├─► HIL_SENSOR       → AP_InertialSensor_SITL, AP_Baro_SITL
    ├─► GPS_INPUT         → EKF3 position / velocity / yaw
    └─► (VISION disabled) → avoids conflicting velocity fusion
    │
    ▼ (USB/Serial MAVLink)
Pixhawk1 (fmuv3-hil firmware)
    │
    ▼
Servo outputs → ESC / control surfaces
```

---

## Default Parameters (`defaults.parm`)

| Parameter | Value | Reason |
|-----------|-------|--------|
| `AHRS_EKF_TYPE` | `3` | Use EKF3, fusing SITL IMU + GPS_INPUT |
| `EK3_SRC1_YAW` | `2` | GPS yaw from `GPS_INPUT` (not magnetometer) |
| `GPS1_TYPE` | `14` | Accept `GPS_INPUT` MAVLink messages |
| `VISO_TYPE` | `0` | Vision disabled — prevents EKF3 velocity fusion conflict |
| `BRD_SAFETY_DEFLT` | `0` | No safety switch required in simulation |
| `ARMING_SKIPCHK` | `-1` | Skip all pre-arm checks |
| `LOG_BITMASK` | `0` | Logging disabled (no SD card needed) |
| `LOG_BACKEND_TYPE` | `0` | Logging disabled |
| `LOG_DISARM` | `0` | No disarm logging |
| `SCHED_LOOP_RATE` | `100` | Scheduler loop rate 100 Hz |
| `ARSPD_TYPE` | `100` | Airspeed from SITL (simulated) |

---

## Build Instructions

```bash
./waf configure --board fmuv3-hil
./waf plane
```

Flash the resulting binary to a Pixhawk1 via the fmuv3-hil bootloader.

---

## Changes vs. Previous Commit

**Commit `0d91b70`** — *update default.param for fmuv3-hil*
- Added `LOG_DISARM 0` — disable logging on disarm
- Added `SCHED_LOOP_RATE 100` — set scheduler loop to 100 Hz
- Added `ARSPD_TYPE 100` — use SITL airspeed sensor

**Commit `7d9cfc1`** — *initial fmuv3-hil for OPSI HITL simulation*
- Created `hwdef.dat` and `defaults.parm` for the `fmuv3-hil` target
- Added `mavlink_xplane.py` bridge script for X-Plane 11 ↔ MAVLink
- Added `GCS_Common.cpp` HITL sensor handler (`handle_hil_sensor`)
- Added bootloader binary, parameter presets, and documentation

---

## Related Files

- `mavlink_xplane.py` — Python bridge: X-Plane UDP → MAVLink HIL_SENSOR / GPS_INPUT
- `instruction_hitl.md` / `instruction_hitl.pdf` — Setup and wiring guide
- `hitl_diagram.png` / `hitl_setup.png` / `hitl_setup.svg` — Diagrams
- `params/` — Parameter presets for flying wing RC and SITL defaults
