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

## Two-Machine Demo: Complete Setup Guide

> **Quick reference** — standalone step-by-step instructions for bringing up the
> two-machine demo from scratch.  Skip to
> [Step 5 — Startup Procedure](#step-5--startup-procedure) if both machines are
> already built and configured.

---

### Step 1 — System Architecture Overview

Two physical machines participate in the demo:

```
┌─────────────────────────────┐      LAN (UDP multicast)      ┌───────────────────────────────────────┐
│   Robot Platform Computer   │ ◄── FastDDS domain 0 ──────► │       Raspberry Pi CM5  (QNC)         │
│                             │                               │                                       │
│  gripper_control_tests      │  PUB qnc/modbus/write_cmd ──►│  qnc_bridge                           │
│  (robot-side binary)        │  PUB qnc/modbus/read_cmd  ──►│    subscribes write_cmd / read_cmd    │
│                             │◄── qnc/modbus/response        │    executes Modbus RTU on RS485       │
│                             │◄── qnc/modbus/stats           │                                       │
│  No serial hardware         │                               │  /dev/gripper_ag160 ──► AG-160        │
└─────────────────────────────┘                               │  /dev/gripper_cgc80 ──► CGC-80        │
                                                              │  /dev/gripper_dh56  ──► DH-5-6        │
                                                              └───────────────────────────────────────┘
```

| Machine | Role | Binary | Serial hardware |
|---------|------|--------|-----------------|
| Robot Platform Computer | Generates commands; publishes DDS topics | `gripper_control_tests` | None |
| Raspberry Pi CM5 (QNC) | Bridge — DDS → Modbus RTU → RS485 | `qnc_bridge` | 3 × FTDI USB-RS485 |

**The only interface between the two machines** is the set of FastDDS topics:

| Topic | Direction | Publisher | Subscriber |
|-------|-----------|-----------|------------|
| `qnc/modbus/write_cmd` | Robot → QNC | `gripper_control_tests` | `qnc_bridge` |
| `qnc/modbus/read_cmd`  | Robot → QNC | `gripper_control_tests` | `qnc_bridge` |
| `qnc/modbus/response`  | QNC → Robot | `qnc_bridge` | `gripper_control_tests` |
| `qnc/modbus/stats`     | QNC → Robot | `qnc_bridge` | `gripper_control_tests` |

---

### Step 2 — Network Setup

Both machines must be on the **same LAN segment** (Ethernet or Wi-Fi).

**Requirements:**

1. UDP multicast must **not** be blocked by the switch or firewall.  FastDDS uses
   multicast for automatic peer discovery — no static IPs or DNS required.
2. Verify basic connectivity before starting:

```bash
# From Robot Computer — confirm QNC is reachable
ping <qnc-ip>

# From Pi CM5 — confirm Robot Computer is reachable
ping <robot-computer-ip>
```

**If UDP multicast is blocked** (managed switch, VPN, or Wi-Fi AP isolation) use
a unicast peer config on both machines:

```bash
# 1. Create ~/fastdds_unicast.xml on EACH machine, replacing PEER_IP with the
#    IP address of the OTHER machine.
cat > ~/fastdds_unicast.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <participant profile_name="default_participant" is_default_profile="true">
    <rtps>
      <builtin>
        <discovery_config>
          <discoveryProtocol>SIMPLE</discoveryProtocol>
          <initialPeersList>
            <locator><udpv4><address>PEER_IP</address></udpv4></locator>
          </initialPeersList>
        </discovery_config>
      </builtin>
    </rtps>
  </participant>
</profiles>
EOF

# 2. Export before running any binary on both machines:
export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_unicast.xml
```

---

### Step 3 — Robot Platform Computer: Build and Run

> **Current status (2026-03-12):** The robot-computer role is **not yet fully
> implemented** in `gripper_control_tests`.  Running the binary on the robot
> computer fails with:
>
> ```
> ✗ Error: Failed to open port: /dev/ttyUSB0
> ```
>
> Two blockers prevent running the demo from the robot computer today:
>
> 1. **`BUILD_WITH_NEURASYNC` not set** — `CMakeLists.txt` does not pass this
>    compile definition, so the `GripperDdsBridge` DDS class and all
>    `publish_write_cmd` / `publish_read_cmd` calls are compiled out.
>    Only the `fastdds` / `fastcdr` libraries are linked; no DDS messages are
>    actually sent.
>
> 2. **`demo` command has no DDS publish path** — even when `BUILD_WITH_NEURASYNC`
>    is defined, the `demo` command calls `gripper.demo(cycles)` which drives the
>    gripper directly over the local serial port.  There is no DDS-publishing demo
>    loop wired up yet.  (The individual `init`, `open`, `close`, `position`, and
>    `status` commands do publish DDS messages, but `demo` does not.)
>
> **Today's workaround:** the binary must run on the **Pi CM5** where the FTDI
> USB-RS485 adapters are physically connected.  Build and run it there:
>
> ```bash
> # On Pi CM5
> cmake -S . -B build -DBUILD_WITH_DDS=ON -DBUILD_TESTS=ON
> cmake --build build --target gripper_control_tests
> cd build
> ./gripper_control_tests --gripper ag160 demo 30
> ```
>
> **Future work required** before the robot-computer role is operational:
> - Add `-DBUILD_WITH_NEURASYNC=ON` to the CMake configure step (or wire it
>   through `CMakeLists.txt`).
> - Add a DDS-publishing loop to the `demo` command so it sends over DDS instead
>   of opening a local serial port.

#### 3.1 Prerequisites

| Package | Min. version | Install |
|---------|-------------|---------|
| CMake | 3.16 | `sudo apt install cmake` |
| FastDDS | 3.x | `sudo apt install fastdds` |
| fastcdr | 2.x | `sudo apt install fastcdr` |
| g++ (C++17) | 9 | `sudo apt install build-essential` |

Verify:

```bash
cmake --version           # ≥ 3.16
dpkg -l fastdds fastcdr   # both installed
```

#### 3.2 Build

From the repo root (`neurasync-robot-client/`):

```bash
cmake -S . -B build -DBUILD_WITH_DDS=ON -DBUILD_TESTS=ON
cmake --build build --target neurasync_types gripper_control_tests
```

Expected output:

```
[100%] Built target neurasync_types
[100%] Built target gripper_control_tests
```

Produced binaries:

| File | Description |
|------|-------------|
| `build/gripper_control_tests` | Robot-side demo binary (gripper controller + DDS publisher) |
| `build/libneurasync_types.so` | Shared DDS type library (required at runtime) |

#### 3.3 Environment Variables (Robot Computer)

| Variable | Default | Description |
|----------|---------|-------------|
| `QNC_SERIAL_PORT` | by gripper type | Serial port override (lower priority than `--port`) |
| `QNC_DE_RE_GPIO` | `none` | sysfs GPIO number for RS485 DE/RE line, or `none` |

> On the robot platform computer the serial port is not needed — DDS is the only
> outbound communication path.  Leave `QNC_SERIAL_PORT` unset unless the binary
> is also used for direct local serial access.

#### 3.4 Running the Robot-Side Binary

```bash
cd build

# Single-gripper demo — N open/close cycles published over DDS (default N=30)
./gripper_control_tests --gripper ag160 demo [N]   # AG-160 (slave_id=1)
./gripper_control_tests --gripper cgc80 demo [N]   # CGC-80 (slave_id=2)
./gripper_control_tests --gripper dh56  demo [N]   # DH-5-6 hand (slave_id=1)

# Other commands
./gripper_control_tests --gripper <ag160|cgc80|dh56> init           [device_id]
./gripper_control_tests --gripper <ag160|cgc80>      open           [device_id]
./gripper_control_tests --gripper <ag160|cgc80>      close          [device_id]
./gripper_control_tests --gripper <ag160|cgc80>      position <val> [device_id]
./gripper_control_tests --gripper <ag160|cgc80>      status         [device_id]
```

Each command publishes the corresponding DDS `write_cmd` and `read_cmd` messages
on domain 0.  The `qnc_bridge` on the Pi CM5 receives them and executes the
Modbus RTU frames on the appropriate RS485 port.

---

### Step 4 — Pi CM5 (QNC Bridge): Build and Run

#### 4.1 Prerequisites

1. Same CMake / FastDDS / fastcdr packages as the Robot Computer (Step 3.1).

2. **FTDI USB-RS485 adapters** connected.  Install udev rules for stable device
   symlinks that survive USB port swaps and reboots:

   ```bash
   sudo cp scripts/99-qnc-grippers.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules && sudo udevadm trigger

   # Verify symlinks
   ls -la /dev/gripper_ag160 /dev/gripper_cgc80 /dev/gripper_dh56
   ```

3. Confirm RS485 adapters are visible before building:

   ```bash
   ls /dev/ttyUSB*   # Expected: /dev/ttyUSB0, /dev/ttyUSB1, /dev/ttyUSB2
   ```

#### 4.2 Build

From the repo root:

```bash
cmake -S . -B build -DBUILD_WITH_DDS=ON
cmake --build build --target qnc_bridge
```

Expected output:

```
[100%] Linked CXX executable build/qnc_bridge
```

#### 4.3 Environment Variables (Pi CM5)

| Variable | Default | Description |
|----------|---------|-------------|
| `QNC_SERIAL_PORT` | `/dev/gripper_ag160` | Primary RS485 port (routed to `QNC_SLAVE_ID_1`) |
| `QNC_SERIAL_PORT_2` | `/dev/gripper_cgc80` | Secondary RS485 port (routed to `QNC_SLAVE_ID_2`) |
| `QNC_SLAVE_ID_1` | `1` | DDS `slave_id` value routed to port 1 |
| `QNC_SLAVE_ID_2` | `2` | DDS `slave_id` value routed to port 2 |
| `QNC_MODBUS_ADDR_1` | `1` | Modbus wire address used in frames on port 1 |
| `QNC_MODBUS_ADDR_2` | `1` | Modbus wire address used in frames on port 2 |
| `QNC_DE_RE_GPIO` | `none` | sysfs GPIO for RS485 DE/RE; `none` = auto-direction HAT |
| `QNC_DOMAIN_ID` | `0` | FastDDS domain ID — **must match the robot computer** |
| `QNC_STATS_INTERVAL` | `10` | Publish `stats` topic every N write operations |

#### 4.4 Running the Bridge

```bash
# Minimal run — udev symlinks installed, auto-direction RS485 HAT
QNC_DE_RE_GPIO=none ./build/qnc_bridge

# Explicit port overrides (e.g., without udev rules)
QNC_SERIAL_PORT=/dev/ttyUSB0 QNC_SERIAL_PORT_2=/dev/ttyUSB1 \
  QNC_DE_RE_GPIO=none ./build/qnc_bridge

# Non-default DDS domain (must match QNC_DOMAIN_ID on robot computer)
QNC_DOMAIN_ID=5 QNC_DE_RE_GPIO=none ./build/qnc_bridge
```

Wait for this line before proceeding to Step 5:

```
[Bridge] Waiting for commands (Ctrl-C to stop)...
```

---

### Step 5 — Startup Procedure

> **Order matters:** always start `qnc_bridge` on the Pi CM5 **first**, then
> wait ~1 second for DDS peer discovery before launching the robot-side binary.

#### 5.1 Start the Bridge (Pi CM5)

```bash
# Terminal on Pi CM5
QNC_DE_RE_GPIO=none ./build/qnc_bridge
# Wait for: "[Bridge] Waiting for commands (Ctrl-C to stop)..."
```

#### 5.2 Start the Robot-Side Demo (Robot Computer)

```bash
# Terminal on Robot Platform Computer
cd build

# AG-160 — 30 open/close cycles
./gripper_control_tests --gripper ag160 demo 30

# CGC-80 — 30 open/close cycles
./gripper_control_tests --gripper cgc80 demo 30

# DH-5-6 hand — 30 sequential finger cycles
./gripper_control_tests --gripper dh56 demo 30
```

Multiple grippers can run in parallel (independent RS485 buses, no collision):

```bash
./gripper_control_tests --gripper ag160 demo 30 &
./gripper_control_tests --gripper cgc80 demo 30 &
wait
```

#### 5.3 Verify DDS Discovery

After both processes start, `qnc_bridge` logs arriving commands within 1–2
seconds of the robot-side binary starting.  If no activity appears after 3
seconds, re-check network connectivity (Step 2).

---

### Step 6 — Verification and Test Procedure

#### 6.1 Verify DDS Communication (Pi CM5 console)

On the Pi CM5, `qnc_bridge` should print each arriving command:

```
[Bridge] FC06 slave=1 reg=0x0100 val=1    → /dev/gripper_ag160  (init)
[Bridge] FC06 slave=1 reg=0x0101 val=50   → /dev/gripper_ag160  (force)
[Bridge] FC06 slave=1 reg=0x0102 val=50   → /dev/gripper_ag160  (speed)
[Bridge] FC06 slave=1 reg=0x0103 val=0    → /dev/gripper_ag160  (open)
[Bridge] FC04 slave=1 reg=0x0200 cnt=3    → /dev/gripper_ag160  (read status)
[Bridge] response: 3 regs published to robot computer
```

On the Robot Computer, DDS initialisation is confirmed by:

```
[DDS] Bridge active on domain 0
      topics: qnc/modbus/write_cmd | read_cmd | response | stats
```

#### 6.2 Verify Bridge-to-Gripper Communication (Pi CM5 console)

Each FC06 or FC04 frame should produce a successful RS485 ACK or response line.
A complete single open/close cycle per gripper requires:

- 1 × `FC06` init write
- 3 × `FC06` writes (force, speed, position) per motion step
- 1 × `FC04` read (3 registers: init_state, gripper_state, position)

If a frame times out or errors, the bridge logs `[ERR]` followed by the error
reason.  Common causes: wrong serial port, wrong Modbus slave address, or loose
RS485 wiring (swap A/B if gripper does not respond).

#### 6.3 Full Integration Test (Robot Computer)

Run the built-in test suite, which requires `qnc_bridge` already running on the
Pi CM5 with all three grippers connected:

```bash
# On Robot Computer
cd build
./gripper_control_tests --test
```

Expected result: all suites report `[PASS]`.  Any `[FAIL]` indicates a DDS
connectivity or Modbus communication problem.

#### 6.4 Live DDS Monitoring (optional — either machine)

The NeuraSync Dashboard can visualise all four topics in real time from any
machine on the same LAN:

```bash
neurasync-dashboard   # connects to domain 0 by default
```

---

## Two-Machine Deployment Plan

### System architecture overview

```
┌─────────────────────────────────────┐        LAN (UDP multicast)        ┌──────────────────────────────────────────┐
│        Robot Platform Computer      │ ◄────── FastDDS domain 0 ──────► │         Raspberry Pi CM5  (QNC)          │
│                                     │                                   │                                          │
│  gripper_control  (or future robot  │   PUB  qnc/modbus/write_cmd  ──► │  qnc_bridge                              │
│  controller binary)                 │   PUB  qnc/modbus/read_cmd   ──► │    subscribes to write_cmd / read_cmd    │
│                                     │                                   │    executes Modbus RTU frames on RS485   │
│                                     │  ◄──  qnc/modbus/response        │    publishes response + stats            │
│                                     │  ◄──  qnc/modbus/stats           │                                          │
└─────────────────────────────────────┘                                   │  /dev/ttyUSB0 ──► AG-160  (USB-RS485)   │
                                                                          │  /dev/ttyUSB1 ──► CGC-80  (USB-RS485)   │
                                                                          │  /dev/ttyUSB2 ──► DH-5-6  (USB-RS485)   │
                                                                          └──────────────────────────────────────────┘
```

### Machine responsibilities

| Machine | Role | Binary | Serial hardware |
|---------|------|--------|-----------------|
| Robot Platform Computer | Gripper controller — publishes commands, asserts on responses | `gripper_control` (standalone demo) or future robot controller | None — no serial ports |
| Raspberry Pi CM5 (QNC) | Bridge — translates DDS commands to Modbus RTU on RS485 | `qnc_bridge` | 3 × FTDI USB-RS485 adapters |

> **Future split note:** `gripper_control` currently lives in this repo for
> convenience.  When the codebase is separated, the robot-side code moves to the
> robot platform workspace unchanged — only the Pi CM5 side stays here.  The
> FastDDS topic names and IDL types are the only shared contract between the two.

---

### FastDDS communication design

**Domain:** `0` (default, configurable via `QNC_DOMAIN_ID`).  Both machines must
use the same domain ID.

**Discovery:** standard FastDDS UDP multicast.  Both machines must be on the same
LAN segment (or multicast must be routed between them).

#### Topics

| Topic | Direction | Publisher | Subscriber | QoS |
|-------|-----------|-----------|------------|-----|
| `qnc/modbus/write_cmd` | Robot → QNC | Robot platform | `qnc_bridge` | KEEP_LAST(10) |
| `qnc/modbus/read_cmd` | Robot → QNC | Robot platform | `qnc_bridge` | KEEP_LAST(10) |
| `qnc/modbus/response` | QNC → Robot | `qnc_bridge` | Robot platform | KEEP_LAST(10) |
| `qnc/modbus/stats` | QNC → Robot | `qnc_bridge` | Robot platform | KEEP_LAST(10) |

> **Why KEEP_LAST(10)?** `WriteCommand` uses `@key octet slave_id`.  DDS history
> is per key instance.  With depth=1 only the last of force/speed/position survives
> in the queue; depth=10 accommodates the 3 writes per motion cycle with margin.
> Both publisher and subscriber must declare matching depth (see Bug 5).

#### Message flow — one open/close cycle

```
Robot platform                              QNC bridge (qnc_bridge)
     │                                            │
     │── write_cmd  reg=0x0101 val=50 (force) ──►│
     │── write_cmd  reg=0x0102 val=50 (speed) ──►│
     │── write_cmd  reg=0x0103 val=0  (open)  ──►│── FC06 → /dev/ttyUSB0 (AG-160)
     │                                            │── FC06 → /dev/ttyUSB1 (CGC-80)
     │── read_cmd   reg=0x0200 cnt=3  FC04    ──►│── FC04 → /dev/ttyUSB0
     │                                            │── FC04 → /dev/ttyUSB1
     │◄─ response  (AG-160 status)                │
     │◄─ response  (CGC-80 status)                │
     │                                            │
     │  ... (close cycle, same pattern) ...       │
     │                                            │
     │◄─ stats  (periodic every 10 writes)        │
```

All `write_cmd` and `read_cmd` are broadcast to **all** configured RS485 ports
simultaneously (independent buses, no collision).

---

### Network assumptions

- Both machines connected to the **same LAN** (Ethernet or Wi-Fi).
- **UDP multicast not blocked** by firewall or switch (FastDDS default discovery).
- No static IP or DNS required — FastDDS discovers peers automatically.
- If multicast is unavailable, set `FASTRTPS_DEFAULT_PROFILES_FILE` to a unicast
  peer XML config on both machines.

---

### Build steps

#### Pi CM5 (QNC) — `qnc_bridge`

```bash
# Prerequisites: FastDDS 3.x + NeuraSync IDL headers installed
# From repo root:
cmake -S . -B build -DBUILD_WITH_DDS=ON
cmake --build build --target qnc_bridge
# Binary: build/qnc_bridge
```

#### Robot Platform Computer — `gripper_control` (standalone demo mode)

```bash
# No external dependencies — POSIX only
# From repo root:
g++ -std=c++17 -O2 -Wall -Wextra \
    -o build/gripper_control robot_side/src/gripper_control.cpp
# Binary: build/gripper_control
```

> When the robot-side code is split into its own repo, this build command moves
> with it.  The Pi CM5 side is unaffected.

---

### Run steps

**Start order:** launch `qnc_bridge` first and wait for the discovery pause
(~1 s) before starting anything on the robot platform.

#### Pi CM5 — start bridge

```bash
QNC_DE_RE_GPIO=none ./build/qnc_bridge
# Wait for: "[Bridge] Waiting for commands (Ctrl-C to stop)..."
```

#### Robot Platform — run demo (each gripper independently)

```bash
cd build

# AG-160 — 30 open/close cycles via DDS → qnc_bridge → /dev/ttyUSB0
./gripper_control --gripper ag160 demo 30

# CGC-80 — 30 open/close cycles via DDS → qnc_bridge → /dev/ttyUSB1
./gripper_control --gripper cgc80 demo 30

# DH-5-6 — 30 sequential finger cycles via DDS → qnc_bridge → /dev/ttyUSB2
./gripper_control --gripper dh56 demo 30
```

Each invocation is independent.  Multiple grippers can be run in parallel
(background `&`) because the commands go over separate RS485 buses.

---

### Prerequisites on the QNC machine

1. **FastDDS 3.x** installed (headers + shared libraries).
   ```
   sudo apt install ros-jazzy-fastrtps   # or build from source
   ```
2. **NeuraSync IDL generated headers** — copy the `neurasync/generated/` folder
   alongside this folder (or install system-wide).
3. **Two FTDI USB-RS485 adapters** (one per gripper).  udev rules create stable
   symlinks independent of enumeration order — see [§ udev device symlinks](#udev-device-symlinks) below.

---

## Build on the QNC machine

```bash
# From the repo root (or a sysroot cross-compile environment):
cmake -S . -B build -DBUILD_WITH_DDS=ON
cmake --build build --target qnc_bridge
```

The resulting binary is `build/qnc_bridge`.

---

## udev device symlinks

Symlinks are keyed by FTDI chip serial number so they survive USB port swaps
and reboots.  The rules file lives in `scripts/99-qnc-grippers.rules` and is
installed once:

```bash
sudo cp scripts/99-qnc-grippers.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

| Symlink | FTDI serial | Gripper |
|---------|------------|--------|
| `/dev/gripper_ag160` | `BG00SMUT` | DH AG-160 |
| `/dev/gripper_cgc80` | `B001PRSH` | DH CGC-80 |

Verify after install:
```
ls -l /dev/gripper_*
# lrwxrwxrwx ... /dev/gripper_ag160 -> ttyUSB1
# lrwxrwxrwx ... /dev/gripper_cgc80 -> ttyUSB0
```

---

## Runtime configuration (environment variables)

With udev symlinks installed **no environment variables are required** for normal
operation — the defaults below point directly to the stable symlinks.

| Variable | Default | Description |
|----------|---------|-------------|
| `QNC_SERIAL_PORT` | `/dev/gripper_ag160` | Primary RS485 port (AG-160) |
| `QNC_SERIAL_PORT_2` | `/dev/gripper_cgc80` | Secondary RS485 port (CGC-80) |
| `QNC_SLAVE_ID_1` | `1` | DDS slave_id routed to primary port |
| `QNC_SLAVE_ID_2` | `2` | DDS slave_id routed to secondary port |
| `QNC_MODBUS_ADDR_1` | `1` | Modbus wire address for primary port |
| `QNC_MODBUS_ADDR_2` | `1` | Modbus wire address for secondary port |
| `QNC_DE_RE_GPIO` | `none` | sysfs GPIO for DE/RE line; `none` = auto-direction |
| `QNC_DOMAIN_ID` | `0` | FastDDS domain ID (must match robot computer) |
| `QNC_STATS_INTERVAL` | `10` | Publish `BridgeStats` every N write operations |

> **Broadcast architecture:** every DDS command is forwarded to **all** configured
> ports simultaneously.  Both grippers use Modbus wire address 1 on independent
> RS485 buses, so the robot computer can address all grippers with `slave_id=1`.

---

## Starting the bridge

```bash
QNC_DE_RE_GPIO=none ./build/qnc_bridge
```

No other environment variables are needed when udev symlinks are installed.

Expected startup output:
```
QNC Bridge
  Port 1       : /dev/gripper_ag160  (dds_slave_id=1  modbus_wire_addr=1)
  Port 2       : /dev/gripper_cgc80  (dds_slave_id=2  modbus_wire_addr=1)
  DE/RE GPIO   : none
  DDS domain   : 0
  Stats interval: every 10 writes

✓ RS485 open: /dev/gripper_ag160
✓ RS485 open: /dev/gripper_cgc80
[Probe] /dev/gripper_ag160  (dds_slave_id=1  configured_modbus_addr=1)
  addr=1  RESPOND  init_state=0  gripper_state=1  position=1000
[Probe] /dev/gripper_cgc80  (dds_slave_id=2  configured_modbus_addr=1)
  addr=1  RESPOND  init_state=0  gripper_state=1  position=1000
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
> The test binary (`gripper_control_tests`) lives in the **neurasync-robot-client**
> workspace on the robot computer and is **not** built by this CMakeLists.txt.

**Start order matters** — launch `qnc_bridge` first and wait for its discovery
pause to finish before starting the test on the robot computer.

```bash
# On Pi CM5 (start first):
QNC_DE_RE_GPIO=none ./build/qnc_bridge

# On robot computer — neurasync-robot-client workspace (start second, ≥ 4 s later):
# Run both grippers in parallel — AG-160-95 and CGC-80 simultaneously:
QNC_REMOTE_GRIPPER=1 QNC_GRIPPER_MODEL=AG160-95 QNC_DEMO_CYCLES=30 \
    ./build/gripper_control_tests --test &

QNC_REMOTE_GRIPPER=1 QNC_GRIPPER_MODEL=CGC-80 QNC_DEMO_CYCLES=30 \
    ./build/gripper_control_tests --test &

wait
```

In REMOTE mode the test publishes `write_cmd` / `read_cmd` directly over DDS and
asserts only on `response` and `stats` received from `qnc_bridge`.  No local
serial port is opened and no loopback DataReaders are created for the command topics.

### Expected console output — Pi CM5 (`qnc_bridge`)

```
QNC Bridge
  Port 1       : /dev/gripper_ag160  (dds_slave_id=1  modbus_wire_addr=1)
  Port 2       : /dev/gripper_cgc80  (dds_slave_id=2  modbus_wire_addr=1)
  DE/RE GPIO   : none
  DDS domain   : 0
  Stats interval: every 10 writes

✓ RS485 open: /dev/gripper_ag160
✓ RS485 open: /dev/gripper_cgc80
[Probe] /dev/gripper_ag160  (dds_slave_id=1  configured_modbus_addr=1)
  addr=1  RESPOND  init_state=0  gripper_state=1  position=1000
[Probe] /dev/gripper_cgc80  (dds_slave_id=2  configured_modbus_addr=1)
  addr=1  RESPOND  init_state=0  gripper_state=1  position=1000
[DDS] QncBridge active on domain 0
      SUB: qnc/modbus/write_cmd  qnc/modbus/read_cmd
      PUB: qnc/modbus/response   qnc/modbus/stats
[Bridge] Discovery pause 1 s...
[Bridge] Waiting for commands (Ctrl-C to stop)...
[CMD] write  slave=1 reg=0x0101 val=  50  → /dev/gripper_ag160  modbus_addr=1
[CMD] write  slave=1 reg=0x0101 val=  50  → /dev/gripper_cgc80  modbus_addr=1
[CMD] write  slave=1 reg=0x0102 val=  50  → /dev/gripper_ag160  modbus_addr=1
[CMD] write  slave=1 reg=0x0102 val=  50  → /dev/gripper_cgc80  modbus_addr=1
[CMD] write  slave=1 reg=0x0103 val=   0  → /dev/gripper_ag160  modbus_addr=1
[CMD] write  slave=1 reg=0x0103 val=   0  → /dev/gripper_cgc80  modbus_addr=1
[CMD] read   slave=1 reg=0x0200 cnt=3 FC04 → /dev/gripper_ag160
[CMD] read   slave=1 reg=0x0200 cnt=3 FC04 → /dev/gripper_cgc80
  ... (× 30 cycles — both grippers receive every command) ...
[Stats/periodic] tx=10  rx=2  err=0  uptime=...s
  ...
[Stats/periodic] tx=360 rx=60 err=0  uptime=...s
^C
[Stats/shutdown] tx=360 rx=60 err=0  uptime=...s

[Bridge] Stopped. tx=360 rx=60 err=0
```

### Expected console output — robot computer (`gripper_control_tests --test`)

```
… (suites 1–3: 46 PASS) …

── Suite 5: Physical Gripper 30 Cycles via gripper_control_dds ───
  [PASS] 5.0  DeviceDescriptorParser loaded DH-Robotics_AG160-95_RTU.json
  [PASS] 5.0b–5.0f  register map checks
  [PASS] 5.1  DomainParticipant created on domain 0
  [PASS] 5.2  response/stats DataReaders active on domain 0 (REMOTE)
  Descriptor: DH-Robotics_AG160-95_RTU  model=AG160-95  slave_id=1  ...
  Mode: REMOTE (Pi CM5 qnc_bridge expected on LAN)
  Waiting 3 s for DDS discovery on domain 0...
  [PASS] 5.3  DataWriters for write_cmd/read_cmd active on domain 0 (REMOTE)
  REMOTE: sending 30 open/close cycles to qnc_bridge
  slave=1  force=50%  speed=50%  motion_wait=1500 ms
  [1/30] OPEN → READ → CLOSE → READ
  [TEST][response] msg #1
  [TEST][response] msg #2
  [2/30] OPEN → READ → CLOSE → READ
  [TEST][response] msg #3
  [TEST][response] msg #4
  ...
  [TEST][stats] msg #1
  ...
  [30/30] OPEN → READ → CLOSE → READ
  [TEST][response] msg #59
  [TEST][response] msg #60
  [TEST][stats] msg #3
  Sent:     wc=180  rc=60
  Received: rsp=60  bs=3
  [PASS] 5.4  write_cmd sent >= 180
  [PASS] 5.5  read_cmd  sent == 60
  [PASS] 5.6  response  received >= 60
  [PASS] 5.7  stats     received >= 3

========================================================
 Results: 51 passed, 0 failed
========================================================
```

> **Note:** `[TEST][response]` and `[TEST][stats]` lines are printed by the
> `CountingListener` each time a message arrives from `qnc_bridge`.  In REMOTE
> mode there are **no** `[TEST][write_cmd]` or `[TEST][read_cmd]` lines — those
> DataReaders are not created, eliminating loopback of the test's own publishes.

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

---

## Bug log

### Bug 1 — `DataReaderQos` bare constructor yields null reader (fixed 2026-03-06)

**Symptom:** No commands were received at all — grippers showed zero motion.

**Root cause:** `DataReaderQos cmd_qos;` (default/bare constructor, not
`DATAREADER_QOS_DEFAULT`) produced a qos object that FastDDS rejected silently;
`create_datareader()` returned `nullptr`.  The `drain_write_commands()` /
`drain_read_commands()` functions checked the pointer and exited immediately
every cycle.

**Fix:**
```cpp
// Before:
DataReaderQos cmd_qos;
cmd_qos.history().kind = KEEP_LAST_HISTORY_QOS;
cmd_qos.history().depth = 10;

// After:
DataReaderQos cmd_qos = DATAREADER_QOS_DEFAULT;  // must be initialised this way
cmd_qos.history().kind = KEEP_LAST_HISTORY_QOS;
cmd_qos.history().depth = 10;
```

---

### Bug 2 — Wrong default serial port (fixed 2026-03-06)

**Symptom:** `⚠ No RS485 ports configured` on startup; both grippers silent.

**Root cause:** Default port was `/dev/ttyAMA0` (Pi CM5 built-in UART, nothing
connected).  Grippers are on FTDI USB-RS485 adapters.

**Fix:** Default changed to `/dev/gripper_ag160` + `/dev/gripper_cgc80`
(udev-stable symlinks).  No env vars needed when udev rules are installed.

---

### Bug 3 — Modbus wire address mismatch (fixed 2026-03-06)

**Symptom:** CGC-80 (port 2) initialised and moved; AG-160 (port 1) silent.

**Root cause:** DDS `slave_id` was reused as the Modbus frame address byte.
AG-160 was assigned `slave_id=1` but the robot sent it on the same frame with
address 1.  CGC-80 was `slave_id=2` — a lucky coincidence that its firmware
also replied to address 2.  When routing changed, AG-160 received frames with
address 1 and worked, but only one gripper was ever active at a time.

**Fix:** `QNC_MODBUS_ADDR_N` (wire address in frame, default 1) was decoupled
from `QNC_SLAVE_ID_N` (DDS routing key).  Both grippers use wire address 1 on
independent RS485 buses.

---

### Bug 4 — Single-port routing blocks second gripper (fixed 2026-03-06)

**Symptom:** Only one gripper ever moved regardless of port configuration.

**Root cause:** Robot computer sends all commands with `slave_id=1`.  The bridge
routed `slave_id=1` exclusively to port 1, so port 2 (CGC-80) never received
any command.

**Fix:** Replaced per-slave routing with broadcast (`all_buses()`): every
`write_cmd` / `read_cmd` is forwarded to **all** configured ports simultaneously.
Both grippers receive every command; independent RS485 buses prevent collision.

---

### Bug 5 — DDS KEEP_LAST(1) drops force/speed writes (fixed 2026-03-06)

**Symptom:** Pi CM5 log showed only `reg=0x103` (position) in each cycle — no
`reg=0x101` (force) or `reg=0x102` (speed) lines at all.  Gripper received position
commands but without a valid force/speed profile it did not move.

**Root cause:** `WriteCommand` declares `@key octet slave_id`.  DDS history is
maintained **per key instance**.  With the default `DATAREADER_QOS_DEFAULT` /
`DATAWRITER_QOS_DEFAULT` (history depth = 1), only the newest sample for `slave_id=1`
survived in the queue.  Publishing force, then speed, then position in rapid
succession overwrote each predecessor; by the time `drain_write_commands()` called
`take_next_sample()` only the position sample remained.

**Fix — applied on both sides (QoS must match writer and reader):**

| File | Object | Before | After |
|------|--------|--------|-------|
| `qnc_bridge.cpp` | `wc_reader_`, `rc_reader_` | `DATAREADER_QOS_DEFAULT` (depth=1) | `KEEP_LAST_HISTORY_QOS`, depth=10 |
| `gripper_control_tests.cpp` | `wc_dw`, `rc_dw` (REMOTE branch) | `DATAWRITER_QOS_DEFAULT` (depth=1) | `KEEP_LAST_HISTORY_QOS`, depth=10 |

The depth=10 value matches ICD §6.2 (`KEEP_LAST(10)` for write_cmd / read_cmd) and
accommodates up to 10 in-flight commands per key instance — well above the 3
needed per motion cycle (force + speed + position).

Both sides were rebuilt and the fix is confirmed deployed.

---

## Demo results (2026-03-06)

**Hardware under test:**
- DH AG-160 gripper → `/dev/gripper_ag160` (FTDI `BG00SMUT`, Modbus wire addr 1)
- DH CGC-80 gripper → `/dev/gripper_cgc80` (FTDI `B001PRSH`, Modbus wire addr 1)

**Test procedure:** 30-cycle open/close endurance run via `qnc_bridge` DDS
bridge from robot computer (`gripper_control_tests --test`).

**Result: PASS**
- Both grippers probed as RESPOND on startup
- Both grippers initialised (calibration stroke completed)
- Both grippers executed all 30 open/close cycles
- Error count: 0
- Commands broadcast to both ports per cycle: force → speed → position → status read

```
[Stats/shutdown] tx=360 rx=60 err=0  uptime=~90s
```

| Metric | Value |
|--------|-------|
| Total write_cmd sent (2 ports × 3 regs × 30 cycles×2) | 360 |
| Total read_cmd sent (2 ports × 1 read × 30 cycles×2) | 60 |
| Errors | 0 |

**Run command used:**
```bash
# Pi CM5:
QNC_DE_RE_GPIO=none ./build/qnc_bridge

# Robot computer (both grippers in parallel):
QNC_REMOTE_GRIPPER=1 QNC_GRIPPER_MODEL=AG160-95 QNC_DEMO_CYCLES=30 \
    ./build/gripper_control_tests --test &

QNC_REMOTE_GRIPPER=1 QNC_GRIPPER_MODEL=CGC-80 QNC_DEMO_CYCLES=30 \
    ./build/gripper_control_tests --test &

wait
```

---

## Bug 6 — Only one gripper initialises and moves in dual-gripper run (diagnosed 2026-03-09)

**Symptom:** When both `--test` instances are launched in parallel (AG160-95 and CGC-80), only
one gripper performs its calibration stroke and executes the 30-cycle run.  The second
gripper is silent.

**Root cause — three independent defects across both machines:**

### 6a — `qnc_bridge.cpp` was never updated to the dual-port architecture (Pi CM5) ✓ fixed 2026-03-09

`main()` reads only `QNC_SERIAL_PORT` and opens a single `ModbusRtu` instance.  The
multi-port env vars documented in § Runtime configuration (`QNC_SERIAL_PORT_2`,
`QNC_SLAVE_ID_1`, `QNC_SLAVE_ID_2`, `QNC_MODBUS_ADDR_1`, `QNC_MODBUS_ADDR_2`) are
**not wired up in the source**.  `QncBridge` holds only one `ModbusRtu*` pointer.
The default port is still `/dev/ttyAMA0` (the Pi CM5 built-in UART — nothing
connected), not `/dev/gripper_ag160`.

Evidence — actual `qnc_bridge` startup banner vs. what the memo promises:

| Expected (memo) | Actual |
|-----------------|--------|
| `Port 1 : /dev/gripper_ag160 (dds_slave_id=1 modbus_wire_addr=1)` | `Serial port  : /dev/ttyAMA0` |
| `Port 2 : /dev/gripper_cgc80 (dds_slave_id=2 modbus_wire_addr=1)` | _(absent)_ |

**Fix (applied):** `main()` now reads `QNC_SERIAL_PORT_2` (defaulting to
`/dev/gripper_cgc80`), `QNC_SLAVE_ID_1/2`, and `QNC_MODBUS_ADDR_1/2`; opens two
`ModbusRtu` instances; and passes both to `QncBridge`.  Every write/read command is
forwarded to all configured ports via `all_buses()`.  File header comment updated to
list all env vars.  Binary rebuilt 2026-03-09.

### 6b — `device_id` is not settable via environment variable (robot computer) — **fix pending**

`GripperConfig::from_env()` reads `QNC_GRIPPER_MODEL`, `QNC_DEMO_CYCLES`, and
`QNC_REMOTE_GRIPPER` but **never reads a slave-ID env var**.  `device_id` is
hardcoded to `1` for both instances.  Even if the bridge correctly routed by
`slave_id`, both parallel processes would publish all commands as `slave_id=1`,
so the CGC-80 instance would drive port 1 (AG-160) instead of port 2.

**Fix required on robot computer:** add `QNC_SLAVE_ID` support in
`GripperConfig::from_env()` so the CGC-80 test can be launched with
`QNC_SLAVE_ID=2`.

### 6c — Single `init_wait_active_` flag serialises two calibration windows (Pi CM5) ✓ fixed 2026-03-09

`QncBridge::spin()` has one `init_wait_active_` / `init_wait_until_` pair for the
entire bridge.  When the AG-160 init command (reg=0x0100) arrives, the flag is set
and `spin()` discards all write_cmds for 3500 ms.  The CGC-80 init command that
arrives during this window is consumed-and-discarded — it never reaches RS485, so the
CGC-80 never calibrates and ignores all subsequent motion commands.

**Fix (applied):** replaced the single `init_wait_active_` / `init_wait_until_` pair
with `std::map<ModbusRtu*, time_point> init_wait_map_` — each port tracks its
calibration window independently.  Both grippers can now initialise concurrently.
Binary rebuilt 2026-03-09.

---

**Status:**
- ✓ **Pi CM5** — 6a and 6c fixed; `qnc_bridge` rebuilt and redeployed 2026-03-09.
- ⏳ **Robot computer** — 6b still pending; rebuild `gripper_control_tests` after adding `QNC_SLAVE_ID` support.

---

## Demo results (2026-03-12) — standalone gripper_control, all three grippers

**Hardware under test:**
- DH AG-160 → `/dev/ttyUSB0` (FTDI `BG00SMUT`, Modbus wire addr 1)
- DH CGC-80 → `/dev/ttyUSB1` (FTDI `B001PRSH`, Modbus wire addr 1)
- DH-5-6 dexterous hand → `/dev/ttyUSB2` (FTDI `BG011K9H`, Modbus wire addr 1)

**Test procedure:** 30-cycle endurance run using the local `gripper_control`
binary directly over USB-RS485 (no DDS bridge).  Each gripper was run
independently with its own invocation.  The `gripper_control` source was
refactored so each gripper class owns its `demo()` method — there is no
coordinated sequencing between grippers in `main()`.

**Build:**
```bash
# From repo root:
g++ -std=c++17 -O2 -Wall -Wextra -o build/gripper_control \
    robot_side/src/gripper_control.cpp
```
Zero warnings, zero errors.

**Run commands (each gripper independently):**
```bash
cd build

# AG-160 — 30 open/close cycles
./gripper_control --gripper ag160 demo 30

# CGC-80 — 30 open/close cycles
./gripper_control --gripper cgc80 demo 30

# DH-5-6 — 30 sequential finger cycles
# cycle: HOME(cmd=9) → THUMB OPEN(cmd=14) → MIDDLE(cmd=12) → RING(cmd=49) → HOME(cmd=9)
./gripper_control --gripper dh56 demo 30
```

**Result: PASS — all three grippers, 30/30 cycles each**

| Gripper | Port | Cycles | Result |
|---------|------|--------|--------|
| AG-160 | `/dev/ttyUSB0` | 30/30 | ✓ PASS — pos=1000/1000 on close |
| CGC-80 | `/dev/ttyUSB1` | 30/30 | ✓ PASS — pos=1000/1000 on close |
| DH-5-6 | `/dev/ttyUSB2` | 30/30 | ✓ PASS — all 5 steps per cycle completed |

**DH-5-6 cycle definition (one cycle):**
1. `cmd=9`  — HOME ALL: retracts DOF1-5, closes DOF6 (thumb) → 0
2. `cmd=14` — THUMB OPEN: rotates DOF6 0 → ~870, DOF1 stays idle (no collision)
3. `cmd=12` — MIDDLE OPEN: DOF2 (middle finger) alone, all others idle
4. `cmd=49` — RING OPEN: DOF3 (ring finger) alone, all others idle
5. `cmd=9`  — HOME ALL: returns everything to rest position

**Notes:**
- `--port /dev/ttyUSBx` overrides the default port for any gripper.
- `QNC_SERIAL_PORT` env var also overrides (lower priority than `--port`).
- Run `./gripper_control --gripper <type> init` once before motion commands after power-on.
- DH-5-6 fault state 86 (firmware protection) cannot be cleared by software — requires physical power cycle.

---

## Workspace Layout

Updated 2026-03-12 to reflect the two-machine architecture.

```
qnc-cm5/                          ← Pi CM5 (QNC bridge) workspace root
│
├── src/                          ← Bridge-side source  [Pi CM5]
│   └── qnc_bridge.cpp            ← Bridge firmware: DDS ↔ Modbus RTU
│
├── include/                      ← Bridge-side headers  [Pi CM5]
│   ├── modbus_rtu.h              ← Core Modbus RTU driver
│   ├── protocol_types.h          ← Shared protocol types
│   └── ...                       ← Other bridge headers
│
├── robot_side/                   ← Robot platform workspace  [Robot Computer]
│   ├── src/
│   │   └── gripper_control.cpp   ← Gripper controller (publishes DDS commands)
│   ├── device_descriptors/       ← Gripper device descriptor JSON files
│   └── tests/
│       ├── ModbusRTUBridge.idl   ← IDL type definitions (shared contract)
│       └── generated/            ← fastddsgen output (auto-generated)
│
├── scripts/                      ← Pi CM5 operational scripts  [Pi CM5]
│   ├── 99-qnc-grippers.rules     ← udev rules for stable /dev symlinks
│   ├── qnc.sh                    ← Bridge startup helper
│   └── dh56_*.py                 ← DH-5-6 diagnostic / recovery scripts
│
├── docs/                         ← Documentation (both sides)
│   └── DEPLOY_MEMO.md            ← This file
│
├── CMakeLists.txt                ← Pi CM5 build (qnc_bridge target)
└── iolink/                       ← IO-Link reference material  [Pi CM5]
```

### `robot_side/` — Robot Platform Computer

Everything under `robot_side/` belongs to the **robot computer**.  It contains:

- `src/gripper_control.cpp` — the gripper controller binary.  In the current demo
  this binary runs locally on the Pi CM5 for convenience, but it represents the
  robot-computer role: it generates commands and communicates with the bridge
  exclusively over DDS topics.
- `device_descriptors/` — JSON device descriptor files describing gripper register
  maps.  These are consumed by the robot-side controller when building motion
  commands; the bridge has no awareness of them.
- `tests/` — IDL type definitions and generated DDS types.  These define the
  on-wire format of the DDS topics and are the sole shared contract between the
  two machines.

**Future separation:** the entire `robot_side/` directory can be moved to a separate
repository or machine.  The only coupling to the Pi CM5 side is the IDL file
(`ModbusRTUBridge.idl`) which must be compiled on both sides producing identical
wire types.

### Remaining workspace — Pi CM5 QNC Bridge

Everything outside `robot_side/` belongs to the **Pi CM5**:

| Path | Purpose |
|------|---------|
| `src/qnc_bridge.cpp` | Bridge firmware binary |
| `include/` | Modbus RTU driver and bridge headers |
| `scripts/` | udev rules, bridge startup, gripper diagnostics |
| `iolink/` | IO-Link hardware reference |
| `CMakeLists.txt` | Builds `qnc_bridge` only |

The bridge has **no dependency on `robot_side/src/`**.  It only shares the IDL
types from `robot_side/tests/` to generate matching DDS serialisation code.

### Communication boundary

The **only interface** between the two machines is the set of FastDDS topics:

```
Robot side  PUB → qnc/modbus/write_cmd  ← SUB  Bridge side
Robot side  PUB → qnc/modbus/read_cmd   ← SUB  Bridge side
Robot side  SUB ← qnc/modbus/response   → PUB  Bridge side
Robot side  SUB ← qnc/modbus/stats      → PUB  Bridge side
```

No shared files, no shared memory, no serial ports cross this boundary.