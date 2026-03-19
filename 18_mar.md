Fresh command verified on the restarted updated bridge.

Observed on QNC in /tmp/qnc_bridge.log:

command stream reached slave=1 on gripper_ag160
first status read failed:
[ReadFail] slave=1 fc=FC03 start=0x0200 cnt=3 port=/dev/gripper_ag160 modbus_addr=1 err=-2
[Response] slave=1 start=0x0200 data=[] err=-2 tag=read_status
subsequent reads succeeded:
[Response] slave=1 start=0x0200 data=[1, 1, 1] err=0 tag=post_cmd_status
[Response] slave=1 start=0x0200 data=[1, 1, 1] err=0 tag=read_status
So the updated bridge is working, and the actual returned registers are now visible on QNC:

data=[1, 1, 1]
Pattern remains:

immediate first read after motion can fail with err=-2
follow-up reads succeed and return stable payload
This is now a confirmed device/read-timing behavior, not a DDS transport issue.



I’m adding a standalone Python helper in scripts that runs a timing-safe AG160 cycle (init -> close -> open) with repeated status reads to absorb first-read failures.



now I see the AG160 is doing open/close physically.

Excellent. That confirms the full chain is now working for AG160:

robot command publish
DDS delivery to QNC
qnc_bridge_updated routing/execution
physical Modbus actuation on hardware
With your current setup, AG160 motion is no longer blocked by the prior read-timing behavior.
