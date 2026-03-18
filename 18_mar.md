Confirmed from the QNC side: commands for the other two grippers were received and routed.****

And routing lines show they went to the correct ports:
So DDS delivery and bridge routing are not the blocker now.

Most likely remaining issue:

CGC-80 and DH-5-6 are receiving commands but not producing visible motion due to device-side command semantics/state.
DH-5-6 is being driven via reg=0x0100 gesture-like values (9/12/14/49), not the same open/close register pattern as AG-160.
CGC-80 might be at mechanical endpoint already for parts of sequence, so some commands appear as no motion.
Best final verification sequence (robot side):

Send single-gripper-only tests, one at a time, with long delays:
CGC-80: close_position(1000) -> wait 3s -> open_position(0)
DH-5-6: one distinct finger move command only, then opposite command
After each write burst, send read_status for that slave and check position/state changed.
Keep gripper_control_dds process alive through the whole sequence (no restart between commands).
If you run one isolated CGC-80 cycle now, I can immediately confirm exact [CMD] + routing lines from QNC again.
