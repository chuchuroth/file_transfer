
No, still not received.

Current QNC evidence:

Bridge running: qnc_bridge PID 15790
Active profile in process env: FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/fastdds_unicast.xml
QNC profile is WiFi-pinned and peer-pinned (172.28.3.48 whitelist, peer 172.28.3.58): fastdds_unicast.xml
Bridge log qnc_bridge.log shows only startup plus:
[Bridge] Waiting for robot publisher match... timeout (10000 ms) — proceeding anyway
No [CMD] lines after restart
So this attempt did not reach the QNC subscriber.

Next step to close this quickly:

On robot, keep gripper_control_dds process alive for at least 5-10s after publish.
Resend a single command (ag160 open).
I will confirm immediately from qnc_bridge.log (it should show [CMD] write_cmd ... plus → /dev/gripper_ag160).
I’m ready to check as soon as you resend.

