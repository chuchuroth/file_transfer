

This rerun did not give a usable diagnostic block yet.

What the current QNC log shows:

only the beginning of a new command sequence:
write_cmd slave=1 reg=0x0100 val=1 tag=init
routed to gripper_ag160
init wait started
then write_cmd slave=1 reg=0x0101 val=50 tag=force
but the log excerpt ends there
What is not present yet:

no [Response] ... data=[...]
no [ReadFail] ...
no later read_cmd lines in the current log slice
So the rerun definitely reached QNC, but it hasn’t progressed far enough in the log we have to show the read outcome.

Send the sequence once more or wait for the rest of the command burst to complete, and I can extract the first [Response] or [ReadFail] line as soon as it lands in /tmp/qnc_bridge.log.
