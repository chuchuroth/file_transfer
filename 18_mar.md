
Not yet. I have not received that single ag160 open on QNC after switching to unicast.

Current verified state on QNC:

qnc_bridge is running: PID 14568
Unicast XML exists: fastdds_unicast.xml
Bridge process has env set:
FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/fastdds_unicast.xml
IP connectivity to robot is good (ping 172.28.3.58 succeeds)
Bridge log shows no [CMD] lines after restart, only:
[Bridge] Waiting for robot publisher match... timeout (10000 ms) — proceeding anyway
That means the robot publisher did not match this bridge instance at send time.

Resend the single open now (with robot process kept alive for a few seconds after publish), and I’ll confirm immediately from qnc_bridge.log.
