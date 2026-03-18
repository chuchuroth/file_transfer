The robot platform's commands never arrived at the bridge.

Log analysis:

Total lines: still 54 (unchanged from 5 minutes ago)
Last [CMD] entry: slave=3 reg=0x0100 val=9 tag=home_all — from the initial inits
No new commands were logged during the entire 5-minute monitoring window
Bridge is running (healthy) but has received nothing since the first sequence
This is a DDS publisher mismatch issue on the robot platform side. The bridge warned you:


[Bridge] Waiting for robot publisher match... timeout (10000 ms) — proceeding anyway
What happened:

✅ gripper_control_dds sent the three inits successfully (they all arrived and routed correctly)
❌ When you sent the follow-up close/open commands, gripper_control_dds was not actively connected to DDS at that moment
Either the process exited after initial inits
Or it restarted but failed to re-join the DDS domain
Diagnosis steps on the robot platform computer:

Is gripper_control_dds still running?


ps aux | grep gripper_control_dds | grep -v grep
If it crashed, why? Check for segfault or error:


dmesg | tail -50
If you restarted it, is DDS finding the bridge's subscriber? Test on robot computer:


export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_unicast.xml  # if needed# Run gripper_control_dds and immediately send an open/close cmd./gripper_control_dds --gripper ag160 open 1  # single command only
Is there a network connectivity issue? Ping the QNC from robot:


ping <qnc-ip>
Once you've confirmed gripper_control_dds is running and actively publishing, resend the commands and I'll watch the bridge log again.
