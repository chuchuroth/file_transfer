# QNC Bridge — Deployment Memo

**Target machine:** Raspberry Pi CM5 (QNC)  
**Document ref:** QNC-ICD-001  
**Date:** 2026-03-06

---

## What this folder contains

| File | Purpose |
|------|---------|
| `qnc_bridge.cpp` | Bridge process: DDS ↔ RS485/Modbus RTU |
| `DEPLOY_MEMO.md` | This file |

---

## Prerequisites on the QNC machine

1. **FastDDS 3.x** installed (headers + shared libraries).
   ```
   sudo apt install ros-jazzy-fastrtps   # or build from source
   ```
2. **NeuraSync IDL generated headers** — copy the `neurasync/generated/` folder
   alongside this folder (or install system-wide).
3. **RS485 HAT** wired to `/dev/ttyAMA0` (XY-485 auto-direction or manual DE/RE
   via GPIO).

---

## Build on the QNC machine

```bash
# From the repo root (or a sysroot cross-compile environment):
cmake -S . -B build -DBUILD_WITH_DDS=ON
cmake --build build --target qnc_bridge
```

The resulting binary is `build/qnc_bridge`.

---

## Runtime configuration (environment variables)

| Variable | Default | Description |
|----------|---------|-------------|
| `QNC_SERIAL_PORT` | `/dev/ttyAMA0` | RS485 serial device |
| `QNC_DE_RE_GPIO` | `none` | sysfs GPIO number for DE/RE line; `none` = auto-direction HAT |
| `QNC_DOMAIN_ID` | `0` | FastDDS domain ID (must match robot computer) |
| `QNC_STATS_INTERVAL` | `10` | Publish `BridgeStats` every N write operations |

---

## Starting the bridge

```bash
QNC_SERIAL_PORT=/dev/ttyAMA0 \
QNC_DE_RE_GPIO=none \
QNC_DOMAIN_ID=0 \
./build/qnc_bridge
```

Expected startup output:
```
QNC Bridge
  Serial port  : /dev/ttyAMA0
  DE/RE GPIO   : none
  DDS domain   : 0
  Stats interval: every 10 writes

✓ RS485 open: /dev/ttyAMA0
[DDS] QncBridge active on domain 0
      SUB: qnc/modbus/write_cmd  qnc/modbus/read_cmd
      PUB: qnc/modbus/response   qnc/modbus/stats
[Bridge] Discovery pause 1 s...
[Bridge] Waiting for commands (Ctrl-C to stop)...
```

On clean shutdown (`Ctrl-C`):
```
[Stats/shutdown] tx=0 rx=0 err=0 uptime=3s

[Bridge] Stopped. tx=0 rx=0 err=0
```

The bridge subscribes to `qnc/modbus/write_cmd` and `qnc/modbus/read_cmd`, executes
the corresponding Modbus RTU frames, and publishes results on `qnc/modbus/response`
and health metrics on `qnc/modbus/stats`.

---

## Two-machine test (from the robot computer)

> **Pi CM5 scope:** only `qnc_bridge` runs here.  
> `gripper_control` and the test binary (`gripper_control_tests` / `test_neurasync_idl`)
> live in the **neurasync-robot-client** workspace on the robot computer and are
> not built by this CMakeLists.txt.

Run the test suite with `QNC_REMOTE_GRIPPER=1` **after** `qnc_bridge` is already
running and DDS discovery has completed (≥ 3 seconds):

```bash
# On Pi CM5 (start first):
QNC_SERIAL_PORT=/dev/ttyAMA0 QNC_DE_RE_GPIO=none ./build/qnc_bridge

# On robot computer — neurasync-robot-client workspace (start second):
QNC_REMOTE_GRIPPER=1 \
QNC_GRIPPER_MODEL=AG160-95 \
QNC_DEMO_CYCLES=30 \
./build/test_neurasync_idl
```

In REMOTE mode the test publishes `write_cmd` / `read_cmd` directly over DDS
and asserts that `qnc_bridge` reflects the correct `response` and `stats` counts.
No local serial port is opened on the robot computer side.

---

## Stopping

Send `SIGINT` (`Ctrl-C`).  The bridge publishes a final `BridgeStats` message
before exiting cleanly.

---

## Notes

- A missing RS485 device is **non-fatal**: the bridge continues running and
  returns `EC_DISCONNECTED` in every `Response` so the robot can detect the fault
  over DDS without hanging.
- The DDS domain ID on both machines **must match**.  Default is `0`.
- Firewall / network: FastDDS uses UDP multicast by default.  Ensure multicast
  is not blocked between the robot computer and the QNC on the LAN.
