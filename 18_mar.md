Changes made (permanent)
Destructor hold: Reduced from the 10s per-second diagnostic loop to a single sleep_for(3000ms) followed by a one-shot drain — outputs only if samples were received.

Sequence mode: command is now split on commas before dispatch. open,close,open issues all three DDS bursts inside one process invocation (one match handshake, one teardown). Inter-command delay defaults to 2500 ms, overridable with --delay N.

AG-160 (slave=1) output
CGC-80 (slave=2) output
Both grippers received all 9 Modbus writes per sequence, QNC echoed every BridgeStats frame, and match time stayed at ~300 ms.




Final validation result on QNC: 






I’ll immediately re-check and return the exact [CMD] and routing line from QNC log.

