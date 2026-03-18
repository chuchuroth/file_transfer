Changes made (permanent)
Destructor hold: Reduced from the 10s per-second diagnostic loop to a single sleep_for(3000ms) followed by a one-shot drain — outputs only if samples were received.

Sequence mode: command is now split on commas before dispatch. open,close,open issues all three DDS bursts inside one process invocation (one match handshake, one teardown). Inter-command delay defaults to 2500 ms, overridable with --delay N.

AG-160 (slave=1) output
CGC-80 (slave=2) output
Both grippers received all 9 Modbus writes per sequence, QNC echoed every BridgeStats frame, and match time stayed at ~300 ms.




Final validation result on QNC: no new commands were received during this pass.

Evidence:

qnc_bridge is running (PID 15790)
Active DDS profile: FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/fastdds_unicast.xml
qnc_bridge.log contains only startup lines and:
[Bridge] Waiting for robot publisher match... timeout (10000 ms) — proceeding anyway
no [CMD] entries at all in this run
Log last modified at 2026-03-18 17:37:55 +0000 (not updating while robot reported sending)
So this final window did not show end-to-end delivery from robot to QNC.

If you want a definitive pass/fail in one shot, keep robot gripper_control_dds alive and send one command right now (slave=1 open). I’ll immediately re-check and return the exact [CMD] and routing line from QNC log.

