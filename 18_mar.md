Changes made (permanent)
Destructor hold: Reduced from the 10s per-second diagnostic loop to a single sleep_for(3000ms) followed by a one-shot drain — outputs only if samples were received.

Sequence mode: command is now split on commas before dispatch. open,close,open issues all three DDS bursts inside one process invocation (one match handshake, one teardown). Inter-command delay defaults to 2500 ms, overridable with --delay N.

AG-160 (slave=1) output
CGC-80 (slave=2) output
Both grippers received all 9 Modbus writes per sequence, QNC echoed every BridgeStats frame, and match time stayed at ~300 ms.




Definitive pass. QNC received and executed all three Modbus writes:


matched (200 ms)                                  ← DDS discovery OKPublished WriteCommand  slave=1 reg=257/258/259   ← robot → QNC← BridgeStats  device_id=ag95_rs485_1  tx=3      ← QNC confirmed 3 writes dispatched to AG-160 RS-485← Response from QNC  slave=1                      ← AG-160 responded on the busQNC→Robot: bs=2 rsp=1                             ← full round-trip confirmed
The earlier failure was caused by the init commands being run from /build without FASTRTPS_DEFAULT_PROFILES_FILE set — so the unicast peer hint to 172.28.3.48 was absent and QNC never discovered the robot's publishers. The fix is always to export the env var before running:


export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_unicast.xml./build/gripper_control_dds --gripper ag160 open 1
Or add it to ~/.bashrc / ~/.profile on the robot so it's always active.
