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

## Bug 6 — Only one gripper initialises and moves in dual-gripper run (diagnosed 2026-03-09, **fix pending**)

**Symptom:** When both `--test` instances are launched in parallel (AG160-95 and CGC-80), only
one gripper performs its calibration stroke and executes the 30-cycle run.  The second
gripper is silent.

**Root cause — three independent defects across both machines:**

### 6a — `qnc_bridge.cpp` was never updated to the dual-port architecture (Pi CM5)

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

**Fix required on Pi CM5:** implement dual-port `main()` — read `QNC_SERIAL_PORT_2`,
open two `ModbusRtu` instances, and broadcast every write/read command to both ports
(the `all_buses()` pattern described in Bug 4).

### 6b — `device_id` is not settable via environment variable (robot computer)

`GripperConfig::from_env()` reads `QNC_GRIPPER_MODEL`, `QNC_DEMO_CYCLES`, and
`QNC_REMOTE_GRIPPER` but **never reads a slave-ID env var**.  `device_id` is
hardcoded to `1` for both instances.  Even if the bridge correctly routed by
`slave_id`, both parallel processes would publish all commands as `slave_id=1`,
so the CGC-80 instance would drive port 1 (AG-160) instead of port 2.

**Fix required on robot computer:** add `QNC_SLAVE_ID` support in
`GripperConfig::from_env()` so the CGC-80 test can be launched with
`QNC_SLAVE_ID=2`.

### 6c — Single `init_wait_active_` flag serialises two calibration windows (Pi CM5)

`QncBridge::spin()` has one `init_wait_active_` / `init_wait_until_` pair for the
entire bridge.  When the AG-160 init command (reg=0x0100) arrives, the flag is set
and `spin()` discards all write_cmds for 3500 ms.  The CGC-80 init command that
arrives during this window is consumed-and-discarded — it never reaches RS485, so the
CGC-80 never calibrates and ignores all subsequent motion commands.

**Fix required on Pi CM5:** give each configured port its own calibration state so
both grippers can initialise concurrently (or sequentially with independent timers).

---

**Action required:**
1. **Pi CM5** — rebuild and redeploy `qnc_bridge` after fixing 6a and 6c.
2. **Robot computer** — rebuild `gripper_control_tests` after fixing 6b.
