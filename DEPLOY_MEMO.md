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
> The test binary (`gripper_control_tests`) lives in the **neurasync-robot-client**
> workspace on the robot computer and is **not** built by this CMakeLists.txt.

**Start order matters** — launch `qnc_bridge` first and wait for its discovery
pause to finish before starting the test on the robot computer.

```bash
# On Pi CM5 (start first):
QNC_SERIAL_PORT=/dev/ttyAMA0 QNC_DE_RE_GPIO=none ./build/qnc_bridge

# On robot computer — neurasync-robot-client workspace (start second, ≥ 4 s later):
QNC_REMOTE_GRIPPER=1 \
QNC_GRIPPER_MODEL=AG160-95 \
QNC_DEMO_CYCLES=30 \
./build/gripper_control_tests --test
```

In REMOTE mode the test publishes `write_cmd` / `read_cmd` directly over DDS and
asserts only on `response` and `stats` received from `qnc_bridge`.  No local
serial port is opened and no loopback DataReaders are created for the command topics.

### Expected console output — Pi CM5 (`qnc_bridge`)

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
[CMD] write_cmd slave=1 reg=0x0101 val=50    tag=force
[CMD] write_cmd slave=1 reg=0x0102 val=50    tag=speed
[CMD] write_cmd slave=1 reg=0x0103 val=0     tag=open
[CMD] read_cmd  slave=1 reg=0x0200 cnt=3  FC04  tag=pos_open
[CMD] write_cmd slave=1 reg=0x0101 val=50    tag=force
[CMD] write_cmd slave=1 reg=0x0102 val=50    tag=speed
[CMD] write_cmd slave=1 reg=0x0103 val=1000  tag=close
[CMD] read_cmd  slave=1 reg=0x0200 cnt=3  FC04  tag=pos_close
  ... (× 30 cycles) ...
[Stats/periodic] tx=10  rx=2  err=0  uptime=...s
  ...
[Stats/periodic] tx=180 rx=60 err=0  uptime=...s
^C
[Stats/shutdown] tx=180 rx=60 err=0  uptime=...s

[Bridge] Stopped. tx=180 rx=60 err=0
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

### Bug 1 — RS485 bus collision on back-to-back FC06 writes (fixed 2026-03-06)

**Symptom:** Init command succeeded (gripper made one calibration stroke) but all
subsequent motion commands (open/close) had no effect.

**Root cause:** The original fire-and-forget write path was:
```
write() → tcdrain() → usleep(500 µs) → de_re_rx()
```
At 115200 8N1 each 8-byte Modbus frame takes 694 µs on-wire.  After `tcdrain` the
slave waits ≥3.5 char-times (304 µs) before echoing, and the echo itself takes
another 694 µs — ending ≥998 µs after `tcdrain`.  The `usleep(500)` released DE/RE
at only ~500 µs, and the drain loop started the **next** write ~350 µs later
(~850 µs total) — while the slave's echo was still on the bus.  Both drivers
simultaneously driving the RS485 differential pair corrupted the frame.  The slave
CRC-failed and silently discarded all writes after the first in each back-to-back
burst (force → speed → position).  Init escaped the bug because there was a 4 s
silence after it with nothing to collide.

**Fix — `qnc_bridge.cpp` (already applied, redeploy required on Pi CM5):**

Replace the raw `write→tcdrain→usleep→de_re_rx` path with `transact(frame, 8, 50)`,
which reads and discards the slave's 8-byte echo before returning.  This naturally
paces back-to-back writes to the RS485 round-trip time (~2 ms at 115200 baud):

```cpp
// OLD (collides at ~850 µs when RX echo ends at ≥998 µs):
tcflush(fd_, TCIOFLUSH);
de_re_tx();
::write(fd_, frame.data(), frame.size());
tcdrain(fd_);
usleep(500);
de_re_rx();
return true;

// NEW (paced to round-trip; ~2 ms when RX wired, ≤50 ms worst-case):
std::vector<uint8_t> echo = transact(frame, 8, 50);
(void)echo;
return true;
```

---

### Bug 2 — DDS KEEP_LAST(1) drops force/speed writes (fixed 2026-03-06)

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

**Action required on Pi CM5:** rebuild and redeploy `qnc_bridge` from the updated
source.  The `gripper_control_tests` binary on the robot computer has already been
rebuilt with the matching writer QoS.
