Recommended to keep permanently:

WiFi-pinned unicast XML on both machines.
Robot-side response and stats DataReaders enabled.
Post-publish delay (~3s) before DDS teardown.
Single long-lived gripper_control_dds process during tests (avoid rapid restart between commands).

If you want, I can now run one final validation pass by watching QNC while you send open -> close -> open for slave=1 and then slave=2 to confirm motion command cadence is fully stable.
