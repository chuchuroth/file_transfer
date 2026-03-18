
test with a single open command (ag160) to verify end-to-end delivery with unicast:

Ran FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_unicast.x...
Sent command and noted QNC unicast requirement
Ask the user to confirm the bridge received this. In the meantime, note that the QNC also needs its unicast XML pointing back at this machine (172.28.3.58). The QNC needs to run:



# On the QNC (172.28.3.48):
cat > ~/fastdds_unicast.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <participant profile_name="default_participant" is_default_profile="true">
    <rtps>
      <builtin>
        <discovery_config>
          <discoveryProtocol>SIMPLE</discoveryProtocol>
          <initialPeersList>
            <locator><udpv4><address>172.28.3.58</address></udpv4></locator>
          </initialPeersList>
        </discovery_config>
      </builtin>
    </rtps>
  </participant>
</profiles>
EOF
export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_unicast.xml
# then restart qnc_bridge



Root cause confirmed: WiFi AP is blocking UDP multicast (239.255.0.1:7400), which FastDDS uses for participant discovery. Without multicast, each new gripper_control_dds process fails to reliably discover the QNC bridge, so messages are silently dropped.

What was created: ~/fastdds_unicast.xml on the robot machine, forcing direct unicast PDP to 172.28.3.48 (QNC). The match above confirmed DDS connected in 400ms.

Did ag160 open arrive at the bridge? If so, confirm and we'll send the remaining commands. If the bridge still shows timeout, the QNC side also needs its XML (pointing to 172.28.3.58) and a bridge restart.
