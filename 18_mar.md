Every single FC04 read returns err=-2 (timeout — device never answers). The DDS→bridge path is perfect (bridge tx=20, tx=30 confirms all writes received). The AG-160 is silently not responding to anything on the RS485 bus.

This is a physical layer / addressing problem, not a software bug. There are two possible causes:

Diagnostic
Check on the Pi CM5 — what did the bridge startup probe print?

When the bridge starts it runs a probe on each port. It should say either:


addr=1  RESPOND  init_state=...
or:


addr=1  no response (err=...)
If it says no response for port 1, the AG-160 is either:

On a different Modbus address (not 1)
The symlink /dev/gripper_ag160 points to the wrong adapter
Quick address scan — restart the bridge with a different QNC_MODBUS_ADDR_1 and try address 2, then 3:


# On Pi CM5 — kill bridge, restart with addr=2, watch probe outputQNC_MODBUS_ADDR_1=2 ./qnc_bridge
