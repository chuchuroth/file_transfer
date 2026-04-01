Build and run:
Build qnc_bridge.
Start bridge with stable symlinks (/dev/gripper_ag160, /dev/gripper_cgc80, /dev/gripper_dh56) and existing env config.
Run your command publisher scenario (demo_all or equivalent).
Hot-plug test (repeat at least 5 cycles):
Keep bridge running.
Unplug AG160 USB-RS485 adapter.
Wait for disconnect logs.
Replug same adapter.
Confirm logs show:
reconnect detected,
init command sent,
init_state transitions,
init complete only when state 2.
Send AG160 open/close commands and verify success.
Confirm CGC80/DH56 still pass unchanged.
Useful expected logs:
[Reconnect] port1 reconnected, running init sequence
[Reconnect] port1 init_state=1 (reg 0x0200)
[Reconnect] port1 init_state=2 (reg 0x0200)
[Reconnect] port1 init complete
Follow-up recommendations

Make init completion state configurable per port/device (QNC_INIT_COMPLETE_VALUE_1/2/3) to align with mixed vendor semantics.
Move reconnect init parameters into descriptor-driven metadata to remove hardcoded 0x0100/0x0200 assumptions.
Add an integration test with mocked Modbus responses that asserts reconnect does not return success on init_state=1 and only succeeds on 2.
I validated syntax/diagnostics for the edited file (No errors found), but I could not run full hardware-in-loop verification from this environment.
