First diagnostic line after the sequence start:

[ReadFail] slave=1 fc=FC03 start=0x0200 cnt=3 port=/dev/gripper_ag160 modbus_addr=1 err=-2

And immediately after it, the first response line is:

[Response] slave=1 start=0x0200 data=[] err=-2 tag=read_status

More importantly, later reads in the same sequence succeed and return actual register data:

[Response] slave=1 start=0x0200 data=[1, 1, 1] err=0 tag=post_cmd_status
[Response] slave=1 start=0x0200 data=[1, 1, 1] err=0 tag=read_status
So the bridge is now giving a clear picture:

the first status read after motion sometimes fails with timeout/error -2
subsequent reads succeed
returned register payload for AG-160 is currently [1, 1, 1]
This is now a usable device-level diagnostic rather than a DDS problem.
